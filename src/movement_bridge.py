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


def odom_cb(msg):
    p = msg.pose.pose
    m = msg.twist.twist
    o = euler_from_quaternion([p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w])
    package = json.dumps({
        "location": [round(x, 3) for x in [p.position.x, p.position.y, p.position.z]],
        "orientation": [round(x, 3) for x in o],
        "linearvelocity": round(m.linear.x, 3),
        "angularvelocity": round(m.angular.z, 3),
        "odom_id": msg.header.seq,
        "time": msg.header.stamp.secs

    })

    redis.set(odom_channel, str(package))
    with open(get_nearby_file("odomdump.json"), 'w') as f:
        f.write(str(package))

rospy.init_node("move_bridge")
r_serv = rospy.get_param("redis_server", "")
r_port = rospy.get_param("redis_port", "")
r_pass = rospy.get_param("redis_pw", "")
if r_serv and r_port and r_pass:
    redis = redis.Redis(host=r_serv, port=int(r_port), password=r_pass)
else:
    redis = redis.Redis()

odom_channel = "Odom"
odom_sub = rospy.Subscriber("/odom", Odometry, odom_cb)

rospy.spin()
