#! /usr/bin/python
"""
this script will:
* publish odom to redis
* receive cmd_vel from redis
"""
import rospy
import redis
import json
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion

odom_channel = "Odom"
cmd_channel = "cmd"
redis = redis.Redis()
rospy.init_node("move_bridge")

def odom_cb(msg):
    p = msg.pose.pose
    m = msg.twist.twist
    o = euler_from_quaternion([p.orientation.x, p.orientaion.y, p.orientation.z, p.orientation.w])
    package = json.dumps({
        "location": [p.position.x, p.position.y, p.position.z],
        "orientation": o,
        "linearvelocity": m.linear.x,
        "angularvelocity": m.angular.z,
        "odom_id": msg.header.seq,
        "time": msg.header.stamp.sec

    })

    redis.set(odom_channel, str(package))


odom_sub = rospy.Subscriber("/odom", Odometry, odom_cb)

rospy.spin()