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
from geometry_msgs.msg import Pose
from tf.transformations import euler_from_quaternion
from map_bridge import get_nearby_file

odom_channel = "Odom"
cmd_channel = "cmd"
redis = redis.Redis()
rospy.init_node("move_bridge")

def odom_cb(msg):
    p = msg.pose.pose
    print(p)
    m = msg.twist.twist
    o = euler_from_quaternion([p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w])
    package = json.dumps({
        "location": [p.position.x, p.position.y, p.position.z],
        "orientation": o,
        "linearvelocity": round(m.linear.x, 3),
        "angularvelocity": round(m.angular.z, 3),
        "odom_id": msg.header.seq,
        "time": msg.header.stamp.secs

    })

    redis.set(odom_channel, str(package))
    with open(get_nearby_file("odomdump.json"), 'w') as f:
        f.write(str(package))


odom_sub = rospy.Subscriber("/odom", Odometry, odom_cb)

rospy.spin()