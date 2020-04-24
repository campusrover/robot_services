#!/usr/bin/env python


import redis
import rospy
from std_msgs.msg import String

if __name__ == '__main__':
	r = redis.Redis(host='redis-10221.c10.us-east-1-3.ec2.cloud.redislabs.com', port=10221, password='ROSlab134')

	pub = rospy.Publisher('redis_cmd_listener', String, queue_size=10)
	rospy.init_node('redis_cmd_listener')
	rate = rospy.Rate(5)
	prev_cmd = ""
	print prev_cmd
	while not rospy.is_shutdown():
		
		cmd = r.get('cmd')
		
		if (cmd != prev_cmd):
			print cmd[1:len(cmd)-1]
			pub.publish(cmd[1:len(cmd)-1])
			prev_cmd = cmd
		rate.sleep()

