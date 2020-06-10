#!/usr/bin/env python


import redis
import rospy
from std_msgs.msg import String

if __name__ == '__main__':
	r = redis.Redis(host='redis-10221.c10.us-east-1-3.ec2.cloud.redislabs.com', port=10221, password='ROSlab134')

	pub = rospy.Publisher('redis_cmd_listener', String, queue_size=10)
	rospy.init_node('redis_cmd_listener')

	redis_key = "Cmd"
	r_namespace = rospy.get_param("redis_ns", "")
	if r_namespace:
		redis_key = r_namespace + "/" + redis_key
	rate = rospy.Rate(10)

	while (r.llen(redis_key) > 0):
		r.lpop(redis_key)
	
	while not rospy.is_shutdown():

		if (r.llen(redis_key)) > 0:
			cmd = r.lpop(redis_key).decode('utf8')
			rospy.loginfo("Publishing command: " + cmd)
			pub.publish(cmd)
			rate.sleep()
		

