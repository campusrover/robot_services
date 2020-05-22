#!/usr/bin/env python

import rospy
import math
import re
from std_msgs.msg import String
from smach import State,StateMachine
from smach_ros import SimpleActionState
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Twist

# boolean representing whether or not all possible 
# waypoints have been explored
failure = False

# boolean representing whether or not Kirby 
# is currently patroling 
patroling = False

# boolean representing whether or not 
# this patrol has been preempted 
preempted = False 

# the inital radius to be explored 
radius = 0.0

# number of points per circle 
POLYGON = 16

# radius of first circle
INITIAL_RADIUS = 1.5

# the amount the radius should be incremented for 
# each concentric circle explored
INC_RADIUS = 1.0

# each move base command should have this orientation pose
orientation = (0.0, 0.0, 0.0, 1.0)

# a regex to grab the signed double or int corresponding to the 
# amount of distance or roatation a given command should be executed
grab_amount = re.compile('-?\d*\.?\d+')

def input_callback(msg):
	global POLYGON, INITIAL_RADIUS, INC_RADIUS, patroling, failure, radius, preempted
	if len(msg.data) > 5 and msg.data[:6] == 'patrol':
		params = grab_amount.findall(msg.data)
		if len(params) > 0 and (len(params) != 3 or int(params[0]) < 2):
			rospy.loginfo("invalid input parameters")
			rospy.loginfo("terminating")
			termination.publish("terminating")
		else: 
			if len(params) == 3:
				POLYGON = int(params[0])
				INITIAL_RADIUS = float(params[1])
				INC_RADIUS = float(params[2])
			radius = INITIAL_RADIUS
			patroling = True
			failure = False
			preempted = False
	elif len(msg.data) == 11 and msg.data[:11] == 'stop patrol':
		patroling = False
		preempted = True
		patrol.request_preempt()
	
# subscriber to consoleInput
input_sub = rospy.Subscriber('redis_cmd_listener', String, input_callback)
#input_sub = rospy.Subscriber('console_input', String, input_callback)


'''creates a nuber of points according to the POLYGON size
around a circle of input radius for the robot to explore
and returns the list of move_base waypoints'''
def create_waypoints(radius):
	increment = (2*math.pi)/POLYGON
	waypoints = []
	for i in range(0, POLYGON):
		waypoints.append([str(i), (math.cos(i*increment)*radius,math.sin(i*increment)*radius), orientation])
	return waypoints


def set_failure(boolean)
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


	termination = rospy.Publisher('termination_request', String, queue_size=10)

	while not rospy.is_shutdown():
		
		# if we've received a patrol command and not every generated waypoint is inaccessible
		while patroling and not failure:
			
			# create a list of waypoints according to the current 
			# radius and polygon number
			waypoints = create_waypoints(radius)
			
			rospy.loginfo("creating " + str(POLYGON) + " waypoints around a circle with radius " + str(radius))
			# create new state machine to hold all waypoints
			patrol = StateMachine(['succeeded','preempted','aborted','failure'])

			# init all the states
			with patrol:
				
				if POLYGON > 1:

					# init new move base goal for the first state
					goal_pose = MoveBaseGoal()
					goal_pose.target_pose.header.frame_id = 'map'
					
					# set x y z coords based on first waypoint
					goal_pose.target_pose.pose.position.x = waypoints[0][1][0]
					goal_pose.target_pose.pose.position.y = waypoints[0][1][1]
					goal_pose.target_pose.pose.position.z = 0.0

					# set orienation based on first waypoint
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
				
				rospy.loginfo("patrolling")
				# will return 'failure' if FAIL state has been reached	
				outcome = patrol.execute()

				# if FAIL state was reached, loop should terminate
				if outcome == 'failure':
					if preempted:
						rospy.loginfo("patrolling cancelled")
					else:
						rospy.loginfo("no more explorable waypoints")
					set_failure(True)
					termination.publish("terminating")
					rospy.loginfo("terminating")
				else: 
					radius += INC_RADIUS
					rospy.loginfo("completed an exploration loop")
		
