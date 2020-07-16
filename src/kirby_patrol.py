#!/usr/bin/env python

import rospy
import math
import re
from std_msgs.msg import String
from smach import State,StateMachine
from smach_ros import SimpleActionState
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

failure = False
patrolling = False
preempted = False 

# the inital radius to be explored 
radius = 0.0

# number of points per circle 
POLYGON = 8
# radius of first circle
INITIAL_RADIUS = 1.0
# the amount the radius should be incremented for 
# each concentric circle explored
INC_RADIUS = 1.0

# robot position when patrolling is started
anchor = [(0.0,0.0,0.0),(0.0,0.0,0.0,0.0)]

# each move base command should have this orientation pose
orientation = (0.0, 0.0, 0.0, 1.0)

arguments = re.compile('-?\d*\.?\d+')

# callback for odom
def odom_callback(msg):
	global pose
	pose = msg.pose.pose
# subscriber to odom
odom_sub = rospy.Subscriber('odom', Odometry, odom_callback)

# resets the anchor
def reset_anchor():
	global anchor
	anchor = [(0.0,0.0,0.0),(0.0,0.0,0.0,0.0)]

# callback for the input
def input_callback(msg):
	global POLYGON, INITIAL_RADIUS, INC_RADIUS, patrolling, failure, radius, preempted
	# starts patrolling
	if len(msg.data) >= 12 and msg.data[:12] == 'start_patrol':
		params = arguments.findall(msg.data)
		POLYGON = int(params[0])
		INITIAL_RADIUS = float(params[1])
		INC_RADIUS = float(params[2])
		radius = INITIAL_RADIUS
		patrolling = True
		failure = False
		preempted = False
		rospy.loginfo("[kirby_feedback patrol] patrolling")
    	# cancels the patrolling
	elif len(msg.data) >= 11 and msg.data[:11] == 'stop patrol' and patrolling:
		patrolling = False
		preempted = True
		reset_anchor()
		patrol.request_preempt()
	
# subscriber to input
input_sub = rospy.Subscriber('redis_cmd_listener', String, input_callback)
#input_sub = rospy.Subscriber('console_input', String, input_callback)


'''creates a nuber of points according to the POLYGON size
around a circle of input radius for the robot to explore
and returns the list of move_base waypoints'''
def create_waypoints(radius):
	global anchor, orientation
	if anchor == [(0.0,0.0,0.0),(0.0,0.0,0.0,0.0)]:
		anchor = [(pose.position.x, pose.position.y, pose.position.z), orientation]
	increment = (2*math.pi)/POLYGON
	waypoints = []
	for i in range(0, POLYGON):
		waypoints.append([str(i), ((math.cos(i*increment)*radius)+anchor[0][0],(math.sin(i*increment)*radius)+anchor[0][1]), orientation])
	return waypoints

def set_failure(boolean):
	global failure 
	failure = boolean

'''class representing a failure state in the FSM used in 
this patroling algorithm. This state will only be reached
if all preceeding states in the FSM have also failed.'''
class Failure(State):
	def __init__(self):
		State.__init__(self, outcomes=['failure'])

	def execute(self, userdata):
		return 'failure'


if __name__=='__main__':
	rospy.init_node('patrol')
	# publish termination request
	termination = rospy.Publisher('termination_request', String, queue_size=10)
	while not rospy.is_shutdown():
		while patrolling and not failure:
			waypoints = create_waypoints(radius)
			rospy.loginfo("[kirby_feedback plan_loop] creating " + str(POLYGON) + " waypoints around a circle with radius " + str(radius))
			patrol = StateMachine(['succeeded','preempted','aborted','failure'])
			# initialize all states
			with patrol:
				if POLYGON > 1:
					# deal with initial state
					goal_pose = MoveBaseGoal()
					goal_pose.target_pose.header.frame_id = 'map'
					goal_pose.target_pose.pose.position.x = waypoints[0][1][0]
					goal_pose.target_pose.pose.position.y = waypoints[0][1][1]
					goal_pose.target_pose.pose.position.z = 0.0
					goal_pose.target_pose.pose.orientation.x = waypoints[0][2][0]
					goal_pose.target_pose.pose.orientation.y = waypoints[0][2][1]
					goal_pose.target_pose.pose.orientation.z = waypoints[0][2][2]
					goal_pose.target_pose.pose.orientation.w = waypoints[0][2][3]
					# add a single initial state representing this move base goal to the machine
					# if it succeeds or is preempted, it will move to the second state
					# if it fails, it will move to the second state on the fail path
					StateMachine.add(waypoints[0][0],SimpleActionState('move_base', MoveBaseAction, goal=goal_pose), transitions={'succeeded':waypoints[1][0], 'preempted':'FAIL', 'aborted':waypoints[1][0]+'_FAIL'})
					# for the rest of the waypoints except the last
					for i in range(1, len(waypoints)-1):
						# create a move base goal
						goal_pose = MoveBaseGoal()
						goal_pose.target_pose.header.frame_id = 'map'
						goal_pose.target_pose.pose.position.x = waypoints[i][1][0]
						goal_pose.target_pose.pose.position.y = waypoints[i][1][1]
						goal_pose.target_pose.pose.position.z = 0.0
						goal_pose.target_pose.pose.orientation.x = waypoints[i][2][0]
						goal_pose.target_pose.pose.orientation.y = waypoints[i][2][1]
						goal_pose.target_pose.pose.orientation.z = waypoints[i][2][2]
						goal_pose.target_pose.pose.orientation.w = waypoints[i][2][3]
						# add a state with this action to the success path
						# add a state with this action to the fail path
						StateMachine.add(waypoints[i][0],SimpleActionState('move_base', MoveBaseAction, goal=goal_pose), transitions={'succeeded':waypoints[(i+1)][0], 'preempted':'FAIL','aborted':waypoints[(i+1)][0]})
						StateMachine.add(waypoints[i][0]+'_FAIL',SimpleActionState('move_base', MoveBaseAction, goal=goal_pose), transitions={'succeeded':waypoints[(i+1)][0], 'preempted':'FAIL', 'aborted':waypoints[(i+1)][0]+'_FAIL'})	
					# create the move base goal for the final state
					goal_pose = MoveBaseGoal()
					goal_pose.target_pose.header.frame_id = 'map'
					goal_pose.target_pose.pose.position.x = waypoints[len(waypoints)-1][1][0]
					goal_pose.target_pose.pose.position.y = waypoints[len(waypoints)-1][1][1]
					goal_pose.target_pose.pose.position.z = 0.0
					goal_pose.target_pose.pose.orientation.x = waypoints[len(waypoints)-1][2][0]
					goal_pose.target_pose.pose.orientation.y = waypoints[len(waypoints)-1][2][1]
					goal_pose.target_pose.pose.orientation.z = waypoints[len(waypoints)-1][2][2]
					goal_pose.target_pose.pose.orientation.w = waypoints[len(waypoints)-1][2][3]
					# any outcome at final state on success path moves to succeeded
					# failure at final state on failure path moves to FAIL State
					StateMachine.add(waypoints[len(waypoints)-1][0],SimpleActionState('move_base', MoveBaseAction, goal=goal_pose), transitions={'succeeded':'succeeded','aborted':'succeeded','preempted':'FAIL'})
					StateMachine.add(waypoints[len(waypoints)-1][0]+'_FAIL',SimpleActionState('move_base', MoveBaseAction, goal=goal_pose), transitions={'succeeded':'succeeded','aborted':'FAIL', 'preempted':'FAIL'})
					# final state representing a failure
					# can only be reached if each previous state failed 
					StateMachine.add('FAIL',Failure(),transitions={'failure':'failure'})
				
				
				# will return 'failure' if FAIL state has been reached	
				outcome = patrol.execute()
				# if FAIL state was reached, loop should terminate
				if outcome == 'failure':
					if preempted:
						rospy.loginfo("[kirby_feedback stop_patrol] patroling cancelled")
					else:
						rospy.loginfo("[kirby_feedback finish_patrol] no more explorable waypoints")
					set_failure(True)
					reset_anchor()
					termination.publish("terminating")
				# if loop was successful then create a new larger one
				else: 
					radius += INC_RADIUS
					rospy.loginfo("[kirby_feedback completed_loop] completed an exploration loop")
		
