#!/usr/bin/env python

import redis
import rospy
import math
import tf
import Queue
import time
from collections import deque 
import re
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib
import nav_msgs.msg
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from robot_services.msg import RotateAction, RotateGoal, RotateResult

# threshold to determine if the robot has moved far enough from an expected
# path to warrent user intevention
EXPLORE_TOLERANCE = 0.5

# a Deque of waypoints for the robot to navigate to
waypoints = deque()

# a Deque to store the commands corresponding to the current 
# waypoints in the Deque of waypoints
commands = deque()

# a point corresponding to the last waypoint in the deque or 
# the current position of the robot if there are no 
# waypoints in the deque
last_in = None

# a boolean representing whether or not the robot has
# received a command to stop, and not yet been told 
# to continue
stopped = False

# a boolean representing whether or not the robot should
# cancel its current nav_goal after it has been stopped
cancel = False

# a boolean representing whether or not the robot sould
# cancel all planned nav_goals after it has stopped
cancel_all = False

# a boolean represening whether or not the robot has received
# a command to continue after being stopped 
restart = False

# whether or not the robot has received a command to go back to 
# its previous location after it has moved
go_back = False

# whether or not the robot has received a command to keep going
# after it has explored and been unable to find a path to
# its goal location
keep_going = False

# whether or not the robot has been stopped in the middle of a 
# turn executed with cmd vel
turn_stopped = False

# whether or not the robot is currently patrolling
patrol = False

# the number of degrees completed before this rotation was preempted
degs_completed = 0.0

# the number of radians completed in this turn or 0 if this is not a turn
rot_completed = 0.0

# a regex to grab the signed double or int corresponding to the 
# amount of distance or roatation a given command should be executed
grab_amount = re.compile('-?\d*\.?\d+')
 
# callback function for the odom subscriber
# stores the current location of the robot
def odom_callback(msg):
	global pose 
	pose = msg.pose.pose
	
# callback function for consoleInput passes the 
# input string to the parse function
def input_callback(msg):
	parse(msg.data)
	
# subscriber to odom
odom_sub = rospy.Subscriber('odom', Odometry, odom_callback)

# subscriber to whatever is taking in the commands
input_sub = rospy.Subscriber('redis_cmd_listener', String, input_callback)
#input_sub = rospy.Subscriber('console_input', String, input_callback)

# callback for patrol termination requests
def termination_callback(msg):
	reset_all_control_booleans()
	rospy.loginfo("waiting for commands")

# subscriber to patrol termination requests
termination_sub = rospy.Subscriber('termination_request', String, termination_callback)


# publisher to cmd_vel, used to complete rotations
cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)

# resets the global last_in variable to be equal
# to the current position of the robot
def reset_last_in():
	global last_in
	last_in = get_current()

# creates a new MoveBaseGoal according to the input waypoint
def goal_pose(pose):
	goal_pose = MoveBaseGoal()
	goal_pose.target_pose.header.frame_id = 'map'
	goal_pose.target_pose.pose.position.x = pose[0][0]
	goal_pose.target_pose.pose.position.y = pose[0][1]
	goal_pose.target_pose.pose.position.z = pose[0][2]
	goal_pose.target_pose.pose.orientation.x = pose[1][0]
	goal_pose.target_pose.pose.orientation.y = pose[1][1]
	goal_pose.target_pose.pose.orientation.z = pose[1][2]
	goal_pose.target_pose.pose.orientation.w = pose[1][3]
	return goal_pose

# gets the current pose of the robot in the form of a waypoint
def get_current():
	global pose
	point = [(pose.position.x,pose.position.y,pose.position.z),(pose.orientation.x,pose.orientation.y,pose.orientation.z,pose.orientation.w)]
	return point

# returns the yaw in radians, of the input pose
def get_yaw(pose):
	orientation = (pose[1][0], pose[1][1], pose[1][2], pose[1][3])
	euler = tf.transformations.euler_from_quaternion(orientation) 
	return euler[2]

# takes in an angle of rotation and a pose, returns a quaternion
# corresponding to the new orientation when rotated input degrees
# with respect to the input pose
def rotate(angle, pose):
	orientation = (pose[1][0], pose[1][1], pose[1][2], pose[1][3])
	euler = tf.transformations.euler_from_quaternion(orientation) 
	new_yaw = euler[2] + math.radians(angle)
	return tf.transformations.quaternion_from_euler(euler[0], euler[1], new_yaw)

# adds a waypoint to the front of the deque
def readd_the_goal(goal):
	waypoints.appendleft(goal)

# adds a command to the front of the deque
def readd_the_command(command):
	commands.appendleft(command)

# removes all waypoints from the deque
def clear_queues(): 
	global waypoints, commands
	waypoints.clear()
	commands.clear()

def clear_waypoints():
	global waypoints
	waypoints.clear()

# generates a waypoint based on the input command 
# and the current state of the robot adds it to the deque
def parse(message):
	# global variables that can be updated in parse
	global last_in, stopped, cancel, cancel_all, restart, go_back, keep_going, turn_stopped, degs_completed, rot_completed, patrol

	# the amount(s) associated with the input command
	# could be meters or degrees
	amount = grab_amount.findall(message)

	# boolean representing whether or not the default value should be used
	default = False
	got_coords = False

	# if there was an input value
	if len(amount) == 1: 
		# store it as a float
		amount = float(amount[0])
	# if there are two input values, assume a coordinate, store x, y
	elif len(amount) == 2:
		x = float(amount[0])
		y = float(amount[1])
		got_coords = True
	# otherwise use the default value
	else: 
		default = True

	# if there are items in the deque of waypoints
	# the last_in value should be the rightmost 
	# waypoint in the deque
	if len(waypoints) != 0:
		last_in = waypoints[-1]
	# if there are no waypoints in the deque
	# then last_in is the current position of the robot
	else:
		reset_last_in()
	# if the message starts with 'go to'
	if message[:5] == 'go to' and got_coords:
		# generate a new waypoint at the given x y coord
		generate_coord(x, y)
		# add the message to the deque of commands
		commands.append((message,("go to",x,y)))
	# if the message starts with 'go forward' 
	elif message[:10] == 'go forward':
		# generate a new waypoint X meters ahead of the robot, 
		# and add the corresponding message to the deque of cmds
		if default:
			# default value for forward motion is 1m 
			generate_forward(1.0)
			commands.append((message,("go forward", 1.0)))
		else: 
			generate_forward(amount)
			commands.append((message,("go forward",amount)))
		
	# if the message starts with 'turn left'
	elif message[:9] == 'turn left':
		# generate a new waypoint with the robot rotated X degrees, 
		# and add the corresponding message to the deque of cmds
		if default: 
			# the default value for rotation is 90 degrees
			generate_rotation(90)
			commands.append((message,("turn left",90)))
		else: 
			generate_rotation(amount)
			commands.append((message,("turn left",amount)))
		
	# if the message starts with 'turn right'
	elif message[:10] == 'turn right':
		# generate a new waypoint with the robot rotated -X degrees, 
		# and add the corresponding message to the deque of cmds
		if default: 
			# the default value for rotation is 90 degrees
			generate_rotation(-90)
			commands.append((message,("turn right",90)))
		else: 
			generate_rotation(-amount)
			commands.append((message,("turn right",amount)))
		
	# if the message is 'stop'
	elif message == 'stop':
		# if the robot is currently rotating with cmd_vel
		if rotation_client.get_state() == 1:
			# stop rotation
			rotation_client.cancel_goal()
			# publish feedback stating that rotation has been stopped
			rospy.loginfo("[feedback] paused rotation estimation")
			# update boolean that states a turn has been stopped in the middle of rotation
			turn_stopped = True
			# convert the radians that have been completed so far to degrees
			degs_completed = int(round(math.degrees(rot_completed)))
			# publish an update on how much of the turn was finished before stopping
			rospy.loginfo("[feedback] rotaiton completed: " + str(degs_completed) + " degrees")
		# if the robot is not currently rotating
		else: 	
			# cancel the move_base goal
			client.cancel_goal()
			# publish a message stating that goals have been paused
			rospy.loginfo("[feedback] paused current goal")
		# update the stopped status to note that the robot has stopped
		stopped = True
	# if a message corresponding to a control boolean is received, 
	# update the appropriate control boolean
	elif message == 'cancel': 
		cancel = True
	elif message == 'cancel all':
		cancel_all = True
	elif message == 'continue':
		restart = True
	elif message == 'go back': 
		go_back = True
	elif message == 'keep going':
		keep_going = True
	# if the message is patrol, cancel any goals that are currently being executed
	# clear all queued commands/goals, patrolling will begin. 
	elif message[:6] == 'patrol':
		if rotation_client.get_state() == 1:
			rotation_client.cancel_goal()
		elif client.get_state() == 1:
			client.cancel_goal()
		clear_queues()
		patrol = True
	elif message == 'stop patrol':
		patrol = False

# generates a waypoint the input amount in front of the robot and adds this waypoint to the deque
def generate_forward(amount):
	global waypoints
	# get the current yaw
	yaw = get_yaw(last_in)
	# use yaw to geneate a point amount in front of the robot 
	point = [(last_in[0][0] + amount*math.cos(yaw), last_in[0][1] + amount*math.sin(yaw), 0.0),(last_in[1][0], last_in[1][1], last_in[1][2], last_in[1][3])]
	# add this new waypoint to the back of the deque
	waypoints.append(point)

# generates a waypoint corresponding to the input x,y coord, and adds this waypoint to the back of the deque
def generate_coord(x, y):
	global waypoints
	point = [(x, y, last_in[0][2]),(last_in[1][0], last_in[1][1], last_in[1][2], last_in[1][3])]
	waypoints.append(point)
	
# generates a waypoint the input number of degrees from the current position of the robot and 
# adds this waypoint to the deque
def generate_rotation(amount):
	global waypoints
	# generate quaternion
	quat = rotate(amount, last_in)
	# use it to generate new point with updated orientation
	# a nonsense value will be added for the z location to signify that this is a rotation command
	# the sign of the nonsense value specifies whether it is a left or right turn
	if amount < 0:
		point = [(last_in[0][0], last_in[0][1], -10),(quat[0], quat[1], quat[2], quat[3])]
	else: 
		point = [(last_in[0][0], last_in[0][1], 10),(quat[0], quat[1], quat[2], quat[3])]
	# add the waypoint to the deque
	waypoints.append(point)

# re-adds a new goal that represents the rest of the rotation of paused goal
def readd_partial_rotation():
	# grab the waypoint corresponding to the rotation we had been doing
	goal = waypoints.popleft()
	# grab the command corresponding to what we had been executing
	command = commands.popleft()
	# the amount left to rotate is the original amount minus the degrees completed
	amt = abs(float(command[1][1])) - degs_completed
	# the new command with the updated amount
	cmd = ((command[1][0] + " " + str(amt)),(command[1][0], amt))
	# current position of the robot
	curr = get_current()
	# sign the amount based on the direction of the turn
	if command[1][0] == "turn right":
		quat = rotate(-amt, curr)
	else:
		quat = rotate(amt, curr)
	# generate the point with special value to signify direction of rotation
	if command[1][0] == "turn left":
		point = [(curr[0][0], curr[0][1], 10),(quat[0],quat[1],quat[2],quat[3])]
	else:
		point = [(curr[0][0], curr[0][1], -10),(quat[0],quat[1],quat[2],quat[3])]
	# add the waypoint to the front of the deque	
	waypoints.appendleft(point)
	# add the command to the front of the deque
	commands.appendleft(cmd)

# sets the z component of the position of this pose or point to the current z position of the robot
# this is used primarily to reset the nonsense value used to signify a rotation command
def reset_z(goal_point):
	global pose
	# creates a new point with updated z position. 
	# new z is the current z, becuase z should never change
	point = [(goal_point[0][0], goal_point[0][1], pose.position.z),(goal_point[1][0], goal_point[1][1], goal_point[1][2], goal_point[1][3])]
	return point

# calculates the number of radians that must be rotated to complete this turn
def est_rads(gp, extra_rot):
	global pose
	# convert current pose to a point
	current_pose = get_current()
	# get the current yaw of the robot
	current_yaw = get_yaw(current_pose)
	# the goal yaw based on the input goal pose
	goal_yaw = get_yaw(gp)
	
	# if the nonsense value was negative, this is a right turn
	if gp[0][2] == -10:
		left_turn = False
	# if it was positive, this is a left turn
	else:
		left_turn = True

	# if we are turning right, calculate the number of radians we must 
	# rotate to reach the goal point
	if not left_turn:
		if current_yaw < 0 and goal_yaw > 0:
			rads = (math.pi + current_yaw) + (math.pi - goal_yaw)
		elif current_yaw > 0 and goal_yaw < 0:
			rads = current_yaw + abs(goal_yaw)
		elif current_yaw > 0 and goal_yaw > 0:
			if current_yaw >= goal_yaw:
				rads = current_yaw - goal_yaw
			else:
				rads = 2*math.pi - goal_yaw - current_yaw
		elif current_yaw < 0 and goal_yaw < 0:
			if current_yaw >= goal_yaw:
				rads = current_yaw - goal_yaw
			else: 
				rads = 2*math.pi - current_yaw - goal_yaw
	# if we are turning left, do the same
	else: 
		if current_yaw < 0 and goal_yaw > 0: 
			rads = abs(current_yaw) + goal_yaw
		elif current_yaw > 0 and goal_yaw < 0:
			rads = (math.pi - current_yaw) + (math.pi + goal_yaw)
		elif current_yaw > 0 and goal_yaw > 0:
			if current_yaw >= goal_yaw:
				rads = 2*math.pi - (current_yaw - goal_yaw)
			else:
				rads = goal_yaw - current_yaw
		elif current_yaw < 0 and goal_yaw < 0:
			if current_yaw < goal_yaw:
				rads = goal_yaw - current_yaw
			else: 
				rads = 2*math.pi - (goal_yaw - current_yaw)
	
	# add any additional full turns
	if extra_rot < 0:
		rads = 2*math.pi * abs(extra_rot)
	else:
		rads += extra_rot * 2*math.pi
	
	# if it's a right turn, rads are negative
	if not left_turn:
		rads = -(rads)
	return rads
	
# adds the input waypoint to the front of the deque and 
# adds a corresponding command 
# the input waypoint is usually the location of the robot before
# it started whatever goal it was most recently pursuing
def undo(pt):
	waypoints.appendleft(pt)
	commands.appendleft(("go back",("go back", pt[0][0], pt[0][1])))	

# checks whether the robot has moved beyond a certain threshold by 
# comparing the input waypoint, representing the starting location
# to the robot's current location
def check_explored(start):
	curr = get_current()
	return (abs(curr[0][0] - start[0][0]) >= EXPLORE_TOLERANCE) or (abs(curr[0][1] - start[0][1]) >= EXPLORE_TOLERANCE)

# generates a message signifying successful completion of the given goal
def generate_success(goal, cmd):
	# go forward success message
	if cmd[1][0] == "go forward":
		return "[feedback] successfully went forward " + str(cmd[1][1]) + "m to (" + str(goal[0][0]) + ", " + str(goal[0][1]) + ")"
	# go to success message
	elif cmd[1][0] == "go to": 
		return "[feedback] successfully navigated to (" + str(goal[0][0]) + ", " + str(goal[0][1]) + ")"
	# turn success message	
	elif cmd[1][0] == "turn left" or cmd[1][0] == "turn right":
		return "[feedback] rotation verified"
	# go back success message	
	elif cmd[1][0] == "go back": 
		return "[feedback] returned to previous location"
	else: 
		return "" 

# generates current status message based on the input goal and command
def currently_doing(goal, cmd):
	# go forward current status
	if cmd[1][0] == "go forward":
		return "[feedback] currently looking for a path forward " + str(cmd[1][1]) + "m to (" + str(goal[0][0]) + ", " + str(goal[0][1]) + ")"
	# go to current status	
	elif cmd[1][0] == "go to":
		return "[feedback] currently looking for a path to (" + str(goal[0][0]) + ", " + str(goal[0][1]) + ")"
	# turn current status
	elif cmd[1][0] == "turn left" or cmd[1][0] == "turn right":
		return "[feedback] verifying rotation"
	# go back current status
	elif cmd[1][0] == "go back":
		return "[feedback] previous location (" + str(goal[0][0]) + ", " + str(goal[0][1]) + ")"
	else: 
		return ""
	
# resets all control booleans to false
def reset_all_control_booleans():
	global degs_completed, rot_completed, cancel, cancel_all, restart, stopped, go_back, keep_going, turn_stopped, patrol
	cancel = False
	cancel_all = False
	restart = False
	stopped = False
	go_back = False
	keep_going = False
	turn_stopped = False
	degs_completed = 0.0
	rot_completed = 0.0
	patrol = False

# stores the feedback from the rotation action
def rotate_feedback(feedback):
	global rot_completed
	rot_completed = feedback.rotation_completed

if __name__ == '__main__':
	
	
	rospy.init_node('commands_with_move_base')
	rate = rospy.Rate(10)

	# represents the current goal the robot is processing
	current_goal = None
	# represents the current command the robot is processing
	current_command = None
	# the starting point of the first goal is the initial location of the robot
	starting_point = [(0.0,0.0,0.0),(0.0,0.0,0.0,0.0)]
	
	
	# simple action client to handle move_base cmds
	client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
	# wait for an action to be sent to the server
	client.wait_for_server()

	# simple action client to handle rotation
	rotation_client = actionlib.SimpleActionClient('rotate', RotateAction)
	# wait for an action to be sent
	rotation_client.wait_for_server()
	
	# initial reference point should be robots initial location
	reset_last_in()

	while not rospy.is_shutdown():
		while not patrol:
			# handles cases where there are no waypoints in the deque
			# if the robot is stopped
			if stopped: 
				if go_back and current_goal != None:
					clear_queues()
					undo(starting_point)
					reset_all_control_booleans()
					rospy.loginfo("[feedback] returning to previous location")
				# if it receives a command to cancel all or to cancel, nothing needs to be done
				# because there are already 0 nav goals in the deque, so wait for a continue 
				# command and then restart with an empty deque
				elif (cancel_all or cancel) and restart: 
					# after restart command, reset all control booleans and continue
					clear_queues()
					reset_all_control_booleans()
					rospy.loginfo("[feedback] cancelled goal")
					rospy.loginfo("[feedback] waiting for commands")
				# if a continue command is received without a cancel command then we 
				elif restart: 
					# if we have taken in a command, and it wasn't rotation
					if not current_goal == None and not turn_stopped:
						# re-add the goal we had been processing before being stopped to the 
						# front of the deque 
						readd_the_goal(current_goal)
						# re-add the corresponding command to the front of the command deque for parity
						readd_the_command(current_command)
						# reset all control booleans and continue
						reset_all_control_booleans()
						# publish feedback 
						rospy.loginfo("[feedback] restarting current goal")
					# if it was rotation that we paused, 
					elif turn_stopped:
						# re-add an updated version of the rotation goal
						readd_partial_rotation()
						# reset the control booleans
						reset_all_control_booleans()
						# publish feedback
						rospy.loginfo("[feedback] restarting current goal")
					# if we actually haven't received any commands
					else:
						# reset the control booleans
						reset_all_control_booleans()
						# publish feedback that we are waiting for commands
						rospy.loginfo("[feedback] waiting for commands")
			# handles cases where there are waypoints in the deque
			while not len(waypoints) == 0:
				# if a stop command has been received
				if stopped:
					if go_back and current_goal != None:
						clear_queues()
						undo(starting_point)
						reset_all_control_booleans()
						rospy.loginfo("[feedback] returning to previous location")
					# if a cancel all command is received
					elif cancel_all:
						# remove all planned waypoints from the deque
						
						# if a continue command is received
						if restart: 
							clear_queues()
							# reset all control booleans and continue
							reset_all_control_booleans()
							# publish feedback
							rospy.loginfo("[feedback] cancelled all planned goals")
							rospy.loginfo("[feedback] waiting for commands")
					# if a cancel command is received
					elif cancel:
						# if a continue command is received
						if restart:
							# if we had been turning, we haven't actually removed the current
							# goal from the deques yet, so we do that
							if turn_stopped:
								waypoints.popleft()
								commands.popleft() 
							# new frame of reference should be the current position of the stopped robot
							reset_last_in()
							# clear all existing waypoints
							clear_waypoints()
							# reparse all unexecuted commands from new reference frame
							for i in range(0,len(commands)):
								parse(commands[i][0])
							# reset all control booleans and continue
							reset_all_control_booleans()
							# publish feedback
							rospy.loginfo("[feedback] cancelled current goal")
							rospy.loginfo("[feedback] continuing execution of planned goals")
					# if a restart command is received without any cancel commands
					elif restart: 
						# if we were doing something other than rotating
						if not turn_stopped:
							# re-add the paused goal and command to the fronts of the deques of waypoints
							readd_the_goal(current_goal)
							readd_the_command(current_command)
						# if we were rotating
						else:
							# add a modified rotation goal
							readd_partial_rotation()
						# reset all control booleans and publish feedback
						reset_all_control_booleans()
						rospy.loginfo("[feedback] restarting current goal")
				# if a stop command hasn't been received 
				else: 
					# the next goal point is the point at the front of the waypoint deque
					goal_point = waypoints[0]
					# init a boolean to track whether this is rotation
					is_rotation = False
					# if we encounter the nonsense value, it is rotation
					if goal_point[0][2] == -10 or goal_point[0][2] == 10:
						is_rotation = True
					# if it's rotation, we complete most of the turn with an action
					# using cmd_vel, and then adjust for errors with a move base goal
					if is_rotation:
						# the number of degrees to be rotated according to the input cmd
						deg = grab_amount.findall(commands[0][0])
						# if no number was input, the default is 90 degrees
						if len(deg) == 0:
							degree = 90
						else:
							degree = deg[0]
						# publish feedback (to left or right)
						if goal_point[0][2] == -10:
							rospy.loginfo("[feedback] estimating a " + str(degree) + " degree turn to the right")
						else: 
							rospy.loginfo("[feedback] estimating a " + str(degree) + " degree turn to the left")
						# calculate number of full rotations completed in this cmd					
						if (float(degree) % 360) == 0:	
							extra_rot = -(float(degree)//360)
						else: 
							extra_rot = float(degree)//360
						# the number of radians either + or - that need to be traversed to complete this rotation
						rads = est_rads(goal_point, extra_rot)
						# replace the nonsense value with a non-nonsense value
						goal_point = reset_z(goal_point)
					# create a moveBaseGoal from the waypoint
					goal = goal_pose(goal_point)
					# store the goal point in the current goal field
					current_goal = goal_point
					# store the command that the current goal came from
					current_command = commands[0]
					# stores the current position of the robot
					starting_point = get_current()
					# if the current command is rotation
					if is_rotation:
						# create a new Rotation goal
						r_goal = RotateGoal()
						# input the number of radians to turn
						r_goal.rads_to_turn = rads
						# send the goal to the rotation client
						rotation_client.send_goal(r_goal, feedback_cb=rotate_feedback)
						# wait for a result
						rotation_client.wait_for_result()
						# if the rotation succeeded
						if rotation_client.get_state() == 3:
							rospy.loginfo("[feedback] successfully estimated rotation")
							# reset the amount of rotation completed
							reset_all_control_booleans()
					# as long as we haven't paused
					if not turn_stopped and not patrol: 				
						# send the goal to the move base client
						client.send_goal(goal)
						# publish feedback on current status
						rospy.loginfo(currently_doing(current_goal, current_command))
						# wait for the result of the move base action
						client.wait_for_result()
						# remove the completed goal from the deque of waypoints
						waypoints.popleft()
						commands.popleft()
						# if it was successful, log successful completion
						if client.get_state() == 3: 
							rospy.loginfo(generate_success(current_goal, current_command))
						# if it was aborted
						elif client.get_state() == 4:
							# whether or not it's moved beyond the explore threshold
							moved = check_explored(starting_point)
							# if it has moved, publish feedback and wait for user input
							if moved: 
								rospy.loginfo("[feedback] moved from expected path and failed to reach goal")
								rospy.loginfo("[feedback] user input is required: keep going OR go back")
								while moved:
									# if user says to continue, proceed with queued goals
									if keep_going:
										rospy.loginfo("[feedback] continuing from new location")
										reset_all_control_booleans()
										moved = False
									# if user says to go back to prior location
									elif go_back:
										# queue a goal of the prior location and continue
										undo(starting_point)
										rospy.loginfo("[feedback] returning to previous location")
										reset_all_control_booleans()
										moved = False
							# if the robot did not move, publish failure status
							else:
								rospy.loginfo("[feedback] unable to complete goal")
		rate.sleep()
























