#!/usr/bin/env python

import rospy
import math
import tf
import Queue
from collections import deque 
import re
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib
import nav_msgs.msg
from nav_msgs.msg import Odometry
from std_msgs.msg import String

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

# a regex to grab the double or int corresponding to the 
# amount of distance or roatation a given command should be executed
grab_amount = re.compile('\d*\.?\d+')
 
# callback function for the odom subscriber
def odom_callback(msg):
	global pose 
	pose = msg.pose.pose
	
# subscriber to odom
odom_sub = rospy.Subscriber('odom', Odometry, odom_callback)

# callback function for consoleInput passes the 
# input string to the parse function
def input_callback(msg):
	parse(msg.data)
	
# subscriber to consoleInput
input_sub = rospy.Subscriber('console_input', String, input_callback)

# resets the global last_in variable to be equal
# to the current position of the robot
def reset_last_in():
	global last_in
	last_in = [(pose.position.x, pose.position.y, pose.position.z),(pose.orientation.x,pose.orientation.y,pose.orientation.z, pose.orientation.w)]

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

# updates the stopped boolean to the input boolean 
def set_stopped(boolean):
	global stopped
	stopped = boolean 

# updates the cancel boolean to the input boolean
def set_cancel(boolean):
	global cancel
	cancel = boolean 

# updates the cancel_all boolean to the input boolean
def set_cancel_all(boolean):
	global cancel_all
	cancel_all = boolean 

# updates the restart boolean to the input boolean
def set_restart(boolean):
	global restart
	restart = boolean

# returns the yaw in radians, of the input pose
def get_yaw(pose):
	# the orientation 
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
def clear_waypoints(): 
	global waypoints
	waypoints.clear()

# removes all commands from the deque
def clear_commands():
	global commands
	commands.clear()

# generates a waypoint based on the input command 
# and the current state of the robot adds it to the deque
def parse(message):
	
	# global variables that can be update in parse
	global last_in
	global stopped
	global cancel
	global cancel_all
	global restart

	# the amount associated with the input command
	# could be meters or degrees
	amount = grab_amount.search(message)

	# boolean representing whether or not the default value should be used
	default = False

	# if there was an input value
	if amount: 
		# store it as a float
		amount = float(amount.group())
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

	# if the message starts with 'go forward' 
	if message[:10] == 'go forward':
		# generate a new waypoint X meters ahead of the robot
		if default: 
			generate_forward(1.0)
		else: 
			generate_forward(amount)
		# add the message to the deque of commands
		commands.append(message)
	# if the message starts with 'turn left'
	elif message[:9] == 'turn left':
		# generate a new waypoint with the robot rotated X degrees
		if default: 
			generate_rotation(90)
		else: 
			generate_rotation(amount)
		# add the message to the deque of commands
		commands.append(message)
	# if the message starts with 'turn right'
	elif message[:10] == 'turn right':
		# generate a new waypoint with the robot rotated -X degrees
		if default: 
			generate_rotation(-90)
		else: 
			generate_rotation(-amount)
		# add the message to the deque of commands
		commands.append(message)
	# if the message is 'stop'
	elif message == 'stop':
		# abort the current goal
		client.cancel_goal()
		# update the stopped status to note that the robot has stopped
		stopped = True
	# if the message is 'cancel'
	elif message == 'cancel':
		# update the cancel status 
		cancel = True
	# if the message is 'cancel all'
	elif message == 'cancel all':
		# update the cancel all status
		cancel_all = True
	# if the message is 'continue'
	elif message == 'continue':
		# update the restart status
		restart = True

# generates a waypoint the input amount in front of the robot and adds this waypoint to the deque
def generate_forward(amount):
	global waypoints
	# get the current yaw
	yaw = get_yaw(last_in)
	# use yaw to geneate a point amount in front of the robot 
	point = [(last_in[0][0] + amount*math.cos(yaw), last_in[0][1] + amount*math.sin(yaw), 0.0),(last_in[1][0], last_in[1][1], last_in[1][2], last_in[1][3])]
	# add this new waypoint to the back of the deque
	waypoints.append(point)
	
# generates a waypoint the input number of degrees from the current position of the robot and 
# adds this waypoint to the deque
def generate_rotation(amount):
	global waypoints
	# generate quaternion
	quat = rotate(amount, last_in)
	# use it to generate new point with updated orientation
	point = [(last_in[0][0], last_in[0][1], last_in[0][2]),(quat[0], quat[1], quat[2], quat[3])]
	# add the waypoint to the deque
	waypoints.append(point)

def reset_all_control_booleans():
	set_cancel(False)
	set_cancel_all(False)
	set_restart(False)
	set_stopped(False)

if __name__ == '__main__':
	
	rospy.init_node('commands_with_move_base')
	rate = rospy.Rate(10)

	# represents the current goal the robot is processing
	global current_goal
	# represents the current command the robot is processing
	global curent_command
	
	# simple action client
	client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
	# wait for an action to be sent to the server
	client.wait_for_server()
	
	# initial reference point should be robots initial location
	reset_last_in()
	while not rospy.is_shutdown():
		# handles cases where there are no waypoints in the deque
		# if the robot is stopped
		if stopped: 
			# if it receives a command to cancel all or to cancel, nothing needs to be done
			# because there are already 0 nav goals in the deque, so wait for a continue 
			# command and then restart with an empty deque
			if (cancel_all or cancel) and restart: 
				# after restart command, reset all control booleans and continue
				reset_all_control_booleans()
			# if a continue command is received without a cancel command then we 
			elif restart: 
				# re-add the goal we had been processing before being stopped to the 
				# front of the deque 
				readd_the_goal(current_goal)
				# re-add the corresponding command to the front of the command deque for parity
				readd_the_command(current_command)
				# reset all control booleans and continue
				reset_all_control_booleans()

		# handles cases where there are waypoints in the deque
		while not len(waypoints) == 0:
			# if a stop command has been received
			if stopped: 
				# if a cancel all command is received
				if cancel_all:
					# remove all planned waypoints from the deque
					clear_waypoints()
					# if a continue command is received
					if restart: 
						# reset all control booleans and continue
						reset_all_control_booleans()
				# if a cancel command is received
				elif cancel:
					# if a continue command is received
					if restart: 
						# new frame of reference should be the current position of the stopped robot
						reset_last_in()
						# clear all existing waypoints
						clear_waypoints()
						# reparse all unexecuted commands from new reference frame
						for i in range(0,len(commands)):
							parse(commands[i])
						# reset all control booleans and continue
						reset_all_control_booleans()
				# if a restart command is received without any cancel commands
				elif restart: 
					# re-add the goal that has been stopped to the front of the deque of waypoints
					readd_the_goal(current_goal)
					# reset all control booleans and continue
					reset_all_control_booleans()
			# if a stop command hasn't been received 
			else: 
				# the next goal point is the point at the front of the waypoint deque
				goal_point = waypoints[0]
				# create a moveBaseGoal from that waypoint
				goal = goal_pose(goal_point)
				# store the goal point in the current goal field
				current_goal = goal_point
				# store the command that the current goal came from
				current_command = commands.popleft()
				# send the goal to the move base client
				client.send_goal(goal)
				# wait for the result of the move base action
				client.wait_for_result()
				# remove the completed goal from the deque of waypoints
				waypoints.popleft()
		rate.sleep()
























