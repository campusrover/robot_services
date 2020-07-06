#! /usr/bin/python
"""
this script will:
* publish odom to redis
"""
import rospy
import redis
import json
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose, Twist
from tf.transformations import euler_from_quaternion
from collections import OrderedDict
import bridge_tools as bt

def odom_cb(msg):
    global px, py, pz, pose, moving, prev_moving, id_num
    pose = msg.pose.pose
    m = msg.twist.twist
    o = [round(x, 3) for x in euler_from_quaternion([pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w])]
    l = round(m.linear.x, 3)
    a = round(m.angular.z, 3)
    lo = [round(x, 3) for x in [pose.position.x, pose.position.y, pose.position.z]]
    package = json.dumps(OrderedDict([
        ("odom_id", id_num),
        ("real_odom_id", msg.header.seq),
        ("location", lo),
        ("orientation",  o),
        ("linearvelocity", l),
        ("angularvelocity", a),
        ("time", msg.header.stamp.secs)
    ]))
    id_num += 1
    changes = [abs(i - j) for i, j in zip([px, py, pz], [lo[0], lo[1], o[2]])]
    moving = l != 0 or a != 0
    # only post updates if enough has changed since the last send or if the robot is changing from motionless to in motion (or vice versa)
    if sum(changes) > send_thresh or moving != prev_moving:
        redis.set(redis_key, str(package))
        px, py, pz = lo[0], lo[1], o[2]  # update previous values
    bt.save_json_file("odomdump.json", json.loads(package))
    prev_moving = moving

def odom_reset():
    global moving, prev_moving, px, py, pz, id_num, pose
    px, py, pz = -1, -1, -1  # default previous values
    id_num = 0
    pose = None
    moving, prev_moving = False, False

if __name__ == '__main__':
    rospy.init_node("odom_bridge")
    redis = bt.redis_client()
    redis_key = bt.namespace_key("Odom")
    
    send_thresh = float(rospy.get_param("pose_update_thresh", 0))
    px, py, pz = -1, -1, -1  # default previous values
    id_num = 0
    pose = None
    moving, prev_moving = False, False
    odom_sub = rospy.Subscriber("/odom", Odometry, odom_cb)
    bt.establish_reset(redis, redis_key, bt.reset, odom_reset, "odom_id", "real_odom_id", "location", "orientation", "time", "linearvelocity", "angularvelocity")
    bt.establish_pulse()
    rospy.spin()
