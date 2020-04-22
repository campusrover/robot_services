#!/usr/bin/env python

import sys 
import rospy
from std_msgs.msg import String

if __name__ == '__main__':
	pub = rospy.Publisher('console_input', String, queue_size=10)
	rospy.init_node("console_input")
	rate = rospy.Rate(5)

	while not rospy.is_shutdown():
		input = raw_input("Motion Command: ")
		pub.publish(input)
		rate.sleep()
