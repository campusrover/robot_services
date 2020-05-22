#!/usr/bin/env python


import redis
import rospy
from std_msgs.msg import String

if __name__ == '__main__':
	r = redis.Redis(host='redis-10221.c10.us-east-1-3.ec2.cloud.redislabs.com', port=10221, password='ROSlab134')

	pub = rospy.Publisher('redis_cmd_listener', String, queue_size=10)
	rospy.init_node('redis_cmd_listener')
	rate = rospy.Rate(10)

	while (r.llen('Cmd') > 0):
		r.lpop('Cmd')
	while not rospy.is_shutdown():
		
		if (r.llen('Cmd')) > 0:
			cmd = r.lpop('Cmd')
			pub.publish(cmd)
			rate.sleep()
		

