#! /usr/bin/env python

import rospy
import time
import actionlib
from robot_services.msg import RotateAction, RotateGoal, RotateResult, RotateFeedback
from geometry_msgs.msg import Twist

# rotates by publishing to cmd vel based on the given goal
def estimate_rotation(goal):
	# new twist command
	motion = Twist()
	# set appropriate angular velocity
	if goal.rads_to_turn < 0:
		motion.angular.z = -1.0
	else: 
		motion.angular.z = 1.0
	# the time at which we should stop rotating
	rotate_time = rospy.Time.now() + rospy.Duration(abs(goal.rads_to_turn))
	# start time
	start = rospy.Time.now()
	# while we have not yet reached the time where we stop rotating
	while rospy.Time.now() < rotate_time:
		# publish the rotation
		cmd_vel_pub.publish(motion)
		# if the action is preempted
		if server.is_preempt_requested():
			# stop rotating
			motion.angular.z = 0.0
			cmd_vel_pub.publish(motion)
			# record the time at which it stopped
			stop = rospy.Time.now()
			result = RotateResult()
			result.time_taken = (stop-start).to_sec()
			# the status is preempted, with the unfinished result
			server.set_preempted(result, 'goal cancelled')
			return
		# feedback about how much rotation has been completed at this point
		feedback = RotateFeedback()
		feedback.rotation_completed = (rospy.Time.now() - start).to_sec()
		server.publish_feedback(feedback)
		# publish feedbakc every tenth of a second
		time.sleep(0.1)	
	# once we have rotated for the appropriate amount of time
	# store the stop time	
	stop = rospy.Time.now()
	result = RotateResult()
	result.time_taken = (stop - start).to_sec()
	# status is success, with given result
	server.set_succeeded(result)


rospy.init_node('rotation_action_server')
# publisher to cmd vel
cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
# rotation action server
server = actionlib.SimpleActionServer('rotate', RotateAction, estimate_rotation, False)
# start the server and wait for actions
server.start()
rospy.spin()
