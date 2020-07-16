#! /usr/bin/env python

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

numbers = re.compile('-?\d*\.?\d+')

# if the robot moves from a starting position in an 
# attempt to reach a goal, but ultimately cannot reach
# that goal, it will request user input if it has moved
# beyond EXPLORE_TOLERANCE of its original location
EXPLORE_TOLERANCE = 0.5

# booleans that help to keep track of the state of the robot
patrolling = False
stopped = False
go_back = False
cancel = False
cancel_all = False
restart = False
keep_going = False

# commands the robot will execute
commands = []

# resets all control booleans
def reset():
	global patrolling, stopped, go_back, cancel, cancel_all, restart, keep_going
	patrolling = False
	stopped = False
	go_back = False
	cancel = False
	cancel_all = False
	restart = False
	keep_going = False

# callback for odom
def odom_callback(msg):
	global pose
	pose = msg.pose.pose

# subscriber to odom
odom_sub = rospy.Subscriber('odom', Odometry, odom_callback)

# callback for the input (usually redis)
def input_callback(msg):
	parse(msg.data)

# subscriber to node that listens to redis
input_sub = rospy.Subscriber('redis_cmd_listener', String, input_callback)
#input_sub = rospy.Subscriber('console_input', String, input_callback)

# converts an input string into some executable for the robot or 
# updates a boolean affecting the state of the robot
def parse(msg):
	global patrolling, keep_going, go_back, cancel, cancel_all, restart, commands, stopped
	# gets all numbers from input string
	metrics = numbers.findall(msg)
	# if the robot is paused it will not take in any new goals until it is unpaused
	if stopped and msg != 'continue' and msg != 'cancel' and msg != 'cancel all' and msg != 'go back' and msg != 'queue':
		rospy.loginfo("[kirby_feedback invalid] invalid command, robot is paused")
	# if the robot is patrolling it will not take in any new goals until it has stopped patrolling
	elif msg != 'stop patrol' and msg != 'start_patrol' and patrolling:
		rospy.loginfo("[kirby_feedback invalid] invalid command, robot is patrolling")
	# adds go to commands to the execution queue
	elif len(msg) >= 5 and msg[:5] == 'go to' and len(metrics) == 2:
		commands.append(Navigate(msg, "go to", float(metrics[0]), float(metrics[1])))
	# adds go forward commands to the execution queue
	elif len(msg) >= 10 and msg[:10] == 'go forward':
		if len(metrics) == 0:
			commands.append(Navigate(msg, "go forward", 1.0))
		elif len(metrics) == 1:
			commands.append(Navigate(msg, "go forward", float(metrics[0])))
		else: 
			rospy.loginfo("[kirby_feedback invalid] go forward commands only take one argument")
	# adds turn left commands to the execution queue
	elif len(msg) >= 9 and msg[:9] == 'turn left':
		if len(metrics) == 0:
			commands.append(Rotate(msg, "turn left", 90.0))
		elif len(metrics) == 1 and metrics[0] < 0:
			rospy.loginfo("[kirby_feedback invalid] angle must be positive")
		elif len(metrics) == 1:
			commands.append(Rotate(msg, "turn left", float(metrics[0])))
		else: 
			rospy.loginfo("[kirby_feedback invalid] rotation commands only take one argument")
	# adds turn right commands to the execution queue
	elif len(msg) >= 10 and msg[:10] == 'turn right':
		if len(metrics) == 0:
			commands.append(Rotate(msg, "turn right", -90.0))
		elif len(metrics) == 1 and metrics[0] < 0:
			rospy.loginfo("[kirby_feedback invalid] angle must be positive")
		elif len(metrics) == 1:
			commands.append(Rotate(msg, "turn right", -float(metrics[0])))
		else: 
			rospy.loginfo("[kirby_feedback invalid] rotation commands only take one argument")
	# adds patrol commands to the execution queue 
	elif len(msg) >= 6 and msg[:6] == 'patrol':
		if len(metrics) == 0:
			commands.append(Patrol(msg, 8, 1.0, 1.0))
		elif len(metrics) == 3:
			commands.append(Patrol(msg, int(metrics[0]), float(metrics[1]), float(metrics[2])))
		else: 
			rospy.loginfo("[kirby_feedback invalid] patrol must take 3 arguments or none at all")
	# a special key that signals that the robot has begun patrolling. This key is published
	# by this script when a patrol command begins to be executed. The kirby_patrol script 
	# listens for this key, and takes over controlling the robot's actions. 
	elif len(msg) >= 12 and msg[:12] == 'start_patrol':
		patrolling = True
	# the robot is no longer patrolling	
	elif msg == 'stop patrol':
		if patrolling:
			patrolling = False
		else:
			rospy.loginfo("[kirby_feedback invalid] robot is not patrolling")
	# the robot should cancel the first goal in its queue
	elif msg == 'cancel':
		if stopped: 
			cancel = True
		else:
			rospy.loginfo("[kirby_feedback invalid] robot must be stopped before a goal can be cancelled")
	# the robot should cancel all goals in its queue
	elif msg == 'cancel all':
		if stopped:
			cancel_all = True
		else:
			rospy.loginfo("[kirby_feedback invalid] robot must be stopped before goals can be cancelled")
	# the robot should unpause 
	elif msg == 'continue':
		if stopped:
			restart = True
		else:
			rospy.loginfo("[kirby_feedback invalid] robot must be stopped before it can continue")
	# the robot should cancel all goals and return to previous location
	# if the robot has asked for input, it will go back to the previous location but cancel pending goals
	elif msg == 'go back':
		go_back = True
	# the robot should continue with planned goals from where it is. This is a special input
	# that should only be given in response to robot request for user input
	elif msg == 'keep going':
		keep_going = True
	# the robot should pause its current action
	elif msg == 'stop':
		reset()
		# handles pausing of rotation action
		if rotation_client.get_state() == 1:
			rotation_client.cancel_goal()
			stopped = True
			rospy.loginfo("[kirby_feedback paused] paused rotation estimation")
			rotation_completed = int(round(math.degrees(float(rot_completed))))
			current = commands[0]
			to_do = current.degrees - rotation_completed
			current.update_goal(to_do)
		# handles pausing of move_base action	
		elif move_base_client.get_state() == 1:
			move_base_client.cancel_goal()
			stopped = True
			rospy.loginfo("[kirby_feedback paused] paused current goal") 	
		else: 
			rospy.loginfo("[kirby_feedback ready] no pending actions") 
	# robot will publish feedback containing its current list of pending goals
	elif msg == 'queue':
		log_queue()
		#rospy.loginfo("[kirby_feedback debug] i heard the queue")
	# if the message is anything else it is invalid
	elif msg != '':
		rospy.loginfo("[kirby_feedback invalid] invalid command")
				
# callback for patrol termination requests
def termination_callback(msg):
	reset()

# callback for rotation action client
def rotate_feedback(feedback):
	global rot_completed
	rot_completed = feedback.rotation_completed

# subscriber to patrol termination requests
termination_sub = rospy.Subscriber('termination_request', String, termination_callback)

# generates string representation of current pending goals
def log_queue():
	global commands
	if len(commands) == 0:
		rospy.loginfo("[kirby_feedback queue] nothing pending")
		#rospy.loginfo("[kirby_feedback debug] queue")
	else:
		q = "pending actions: "
		for command in commands[:len(commands) - 1]:
			#rospy.loginfo("[kirby_feedback queue] " + str(q))
			q += str(command.original) + ", "
			#rospy.loginfo("[kirby_feedback queue] " + str(q))
		q += str(commands[-1].original)
		rospy.loginfo("[kirby_feedback queue] " + q)

# returns waypoint representation of robot's current position
def current_robot_location():
	return [(pose.position.x, pose.position.y, pose.position.z),(pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w)]

# returns the yaw, in radians, of the input pose
def get_yaw(position):
	orientation = position[1]
	euler = tf.transformations.euler_from_quaternion(orientation) 
	return euler[2]

# creates a move_base goal out of the input waypoint
def create_goal(pose):
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

# checks whether the robot has moved beyond EXPLORE_TOLERANCE
# used whenever a move_base goal fails, to check if the robot is in an 
# unexpected location and alert the user and ask for input if so
def check_movement(start):
	curr = current_robot_location()
	return (abs(curr[0][0] - start[0][0]) >= EXPLORE_TOLERANCE) or (abs(curr[0][1] - start[0][1]) >= EXPLORE_TOLERANCE)

# generates a feedback message for success of a goal
def success_message(command):
	# go forward success message
	if command.command == "go forward":
		return "[kirby_feedback success_forward] went forward " + str(round(command.x,1)) + " m to (" + str(round(command.waypoint[0][0],1)) + ", " + str(round(command.waypoint[0][1],1)) + ")"
	# go to success message
	elif command.command == "go to": 
		return "[kirby_feedback success_go_to] navigated to (" + str(round(command.waypoint[0][0],1)) + ", " + str(round(command.waypoint[0][1],1)) + ")"
	# turn success message	
	elif command.command == "turn left" or command.command == "turn right":
		return "[kirby_feedback success_verify_rotation] rotation verified"
	# go back success message	
	elif command.command == "go back": 
		return "[kirby_feedback success_go_back] returned to previous location"
	else: 
		return "" 

# generates a feedback message for a goal in progress
def in_progress_message(command):
	# go forward current status
	if command.command == "go forward":
		return "[kirby_feedback forward] going forward " + str(round(command.x,1)) + " m to (" + str(round(command.waypoint[0][0],1)) + ", " + str(round(command.waypoint[0][1],1)) + ")"
	# go to current status	
	elif command.command == "go to":
		return "[kirby_feedback go_to] going to (" + str(round(command.waypoint[0][0],1)) + ", " + str(round(command.waypoint[0][1],1)) + ")"
	# turn current status
	elif command.command == "turn left" or command.command == "turn right":
		return "[kirby_feedback verify_rotation] verifying rotation"
	# go back current status
	elif command.command == "go back":
		return "[kirby_feedback go_back] returning to previous location (" + str(round(command.waypoint[0][0],1)) + ", " + str(round(command.waypoint[0][1],1)) + ")"
	else: 
		return ""


# object representing a command given to the robot
#   - command: the type of command e.g. "turn left", "go forward", "patrol"
#   - original: the raw string input command
#   - waypoint: the goal location for the robot after this command has been completed 
class Command(object):

	def __init__(self, command, original):
		self.command = command
		self.original = original
		self.waypoint = self.generate_waypoint()

	def generate_waypoint(self):
		pass


# object representing a command that moves the robot to a new location
#   - x: the x coordinate of the goal location
#   - y: the y coordinate of the goal location
class Navigate(Command):

	def __init__(self, msg, command, x, y=None):
		self.x = x
		self.y = y 
		super(Navigate, self).__init__(command, msg)

	# generate the goal location based on current location, pending actions, and input command
	def generate_waypoint(self):
		global commands
		start = current_robot_location()
		# if there are pending actions, assume that the starting location for this action
		# will be the resulting location of the final pending action.  
		if len(commands) != 0: 
			start = commands[-1].waypoint
		# if no y coordinate is given, this is a go forward command, and the x and y coordinates will be calculated
		if self.y == None: 
			yaw = get_yaw(start)
			goal = [(start[0][0] + self.x*math.cos(yaw), start[0][1] + self.x*math.sin(yaw), start[0][2]), start[1]]
		# otherwise the input x and y coordinates can be used directly 
		else:
			goal = [(self.x, self.y, start[0][2]), start[1]]
		return goal


# object representing a command that rotates the robot in place
#   - degrees: the number of degrees the robot should turn. 
class Rotate(Command):
	
	def __init__(self, msg, command, degrees):
		self.degrees = degrees
		super(Rotate, self).__init__(command, msg)

	# a waypoint is generated for error correction after the majority of the rotation
	# is completed with the rotation action
	def generate_waypoint(self):
		global commands
		start = current_robot_location()
		if len(commands) != 0:
			start = commands[-1].waypoint	
		quaternion = self.final_orientation(start)
		return [(start[0][0], start[0][1], start[0][2]), (quaternion[0], quaternion[1], quaternion[2], quaternion[3])]

	# calculates final position after completing desired turn
	def final_orientation(self, position):
		orientation = (position[1][0], position[1][1], position[1][2], position[1][3])
		euler = tf.transformations.euler_from_quaternion(orientation)
		new_yaw = euler[2] + math.radians(float(self.degrees))
		return tf.transformations.quaternion_from_euler(euler[0],euler[1],new_yaw)

	def update_goal(self, degrees):
		self.degrees = degrees
		self.original = self.command + " " + str(degrees)


# object representing a command that tells the robot to patrol
#
# the patrol algorithm has the robot explore its environment in concentric polygons
# it generates a list of waypoints for each polygon, and explores each in turn 
# once all waypoints have been explored, it creates another, larger polygon
# this repeats until none of the generated waypoints can be explored because they are
#    all in unreachable locations 
#
#   - points: the number of waypoints to explore in each polygon
#   - radius: the initial radius of the circle to be explored
#   - increment: the number of meters the radius of each subsequent circle increases
class Patrol(Command):

	def __init__(self, msg, points, radius, increment):
		self.points = points
		self.radius = radius
		self.increment = increment
		super(Patrol, self).__init__("patrol", msg)
	
	def generate_waypoint(self):
		return [(.0,0.0,0.0),(0.0,0.0,0.0,0.0)]


if __name__ == '__main__':

	rospy.init_node('commands_with_move_base')
	rate = rospy.Rate(10)
	# publisher for starting patrol
	patrol_pub = rospy.Publisher('redis_cmd_listener', String, queue_size=10)
	# each goal that is executed has a starting location that can be returned to
	current_goal = None
	starting_point = [(.0,0.0,0.0),(0.0,0.0,0.0,0.0)]
	# move_base action client
	move_base_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
	move_base_client.wait_for_server()
	# rotation action client
	rotation_client = actionlib.SimpleActionClient('rotate', RotateAction)
	rotation_client.wait_for_server()

	while not rospy.is_shutdown():
		while not patrolling:
			while len(commands) != 0:
				while stopped:
					# cancels all pending actions and sends robot back to where it started this action
					if go_back and current_goal != None: 
						commands = []
						commands.append(Navigate("go back", "go back", starting_point[0][0], starting_point[0][1]))
						reset()
					# cancels all pending actions
					elif cancel_all:
						commands = []
						reset()
						rospy.loginfo("[kirby_feedback cancelled_all] cancelled all planned goals")
					# cancels the paused action 
					elif cancel:
						commands.pop(0)
						pending = commands
						commands = []
						reset()
						# re-parse all actions from new reference point
						for i in range(0, len(pending)):
							parse(pending[i].original)
						rospy.loginfo("[kirby_feedback cancelled_goal] cancelled current goal")
						rospy.loginfo("[kirby_feedback restarting] continuing execution of planned goals")
					# restarts the paused action
					elif restart:
						reset()
				if len(commands) == 0:
					break
				current_goal = commands[0]
				starting_point = current_robot_location()
				# handles patrol actions
				if isinstance(current_goal, Patrol):
					patrol_pub.publish("start_patrol " + str(current_goal.points) + " " + str(current_goal.radius) + " " + str(current_goal.increment))
					patrolling = True					
					commands = []
					break
				# handles rotation actions
				elif isinstance(current_goal, Rotate):
					d = math.radians(float(current_goal.degrees))
					rotate_goal = RotateGoal()
					rotate_goal.rads_to_turn = d
					rotation_client.send_goal(rotate_goal, feedback_cb=rotate_feedback)
					rospy.loginfo("[kirby_feedback estimate_rotation] estimating a " + str(d) + " degree turn")
					rotation_client.wait_for_result()
					if rotation_client.get_state() == 3:
						rospy.loginfo("[kirby_feedback success_estimate_rotation] successfully estimated rotation")
					elif stopped:
						break
				# handles actions sent to move_base					
				move_base_client.send_goal(create_goal(current_goal.waypoint))
				rospy.loginfo(in_progress_message(current_goal))
				move_base_client.wait_for_result()
				if stopped:
					break
				commands.pop(0)
				if move_base_client.get_state() == 3:
					rospy.loginfo(success_message(current_goal))
				elif move_base_client.get_state() == 4:
					moved = check_movement(starting_point)
					if moved:
						rospy.loginfo("[kirby_feedback strayed] moved from expected path and failed to reach goal")
						rospy.loginfo("[kirby_feedback help] user input is required: 'keep going' OR 'go back'")
						while moved:
							if keep_going:
								rospy.loginfo("[kirby_feedback restarting] continuing from new location")
								reset()
								moved = False
							elif go_back:
								commands.insert(0, Navigate("go back", starting_point[0][0], starting_point[0][1]))
								rospy.loginfo("[kirby_feedback go_back] returning to previous location")
								reset()
								moved = False
					else: 
						rospy.loginfo("[kirby_feedback unreachable] unable to complete goal")
				rate.sleep()
