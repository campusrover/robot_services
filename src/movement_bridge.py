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
    global px, py, pz
    p = msg.pose.pose
    m = msg.twist.twist
    o = [round(x, 3) for x in euler_from_quaternion([p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w])]
    l = round(m.linear.x, 3)
    a = round(m.angular.z, 3)
    lo = [round(x, 3) for x in [p.position.x, p.position.y, p.position.z]]
    package = json.dumps({
        "location": lo,
        "orientation":  o,
        "linearvelocity": l,
        "angularvelocity": a,
        "odom_id": msg.header.seq,
        "time": msg.header.stamp.secs

    })
    changes = [abs(i - j) for i, j in zip([px, py, pz], [lo[0], lo[1], o[2]])]
    # only post updates if enough has changed since the last send
    if sum(changes) > send_thresh:
        redis.set(odom_channel, str(package))
        px, py, pz = lo[0], lo[1], o[2]
    with open(get_nearby_file("odomdump.json"), 'w') as f:
        json.dump(json.loads(package), f)

    

rospy.init_node("move_bridge")
r_serv = rospy.get_param("redis_server", "")
r_port = rospy.get_param("redis_port", "")
r_pass = rospy.get_param("redis_pw", "")
if r_serv and r_port and r_pass:
    redis = redis.Redis(host=r_serv, port=int(r_port), password=r_pass)
else:
    redis = redis.Redis()

odom_channel = "Odom"
send_thresh = float(rospy.get_param("odom_thresh", 0))
px, py, pz = -1, -1, -1
odom_sub = rospy.Subscriber("/odom", Odometry, odom_cb)

rospy.spin()