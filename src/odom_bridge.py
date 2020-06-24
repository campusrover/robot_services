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
from std_msgs.msg import Empty, String
from tf.transformations import euler_from_quaternion
from collections import OrderedDict
import bridge_tools as bt

def pulse_cb(msg):
    echo_pub.publish(rospy.get_name())

def odom_cb(msg):
    global px, py, pz, pose
    pose = msg.pose.pose
    m = msg.twist.twist
    o = [round(x, 3) for x in euler_from_quaternion([pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w])]
    l = round(m.linear.x, 3)
    a = round(m.angular.z, 3)
    lo = [round(x, 3) for x in [pose.position.x, pose.position.y, pose.position.z]]
    package = json.dumps(OrderedDict([
        ("odom_id", msg.header.seq),
        ("location", lo),
        ("orientation",  o),
        ("linearvelocity", l),
        ("angularvelocity", a),
        ("time", msg.header.stamp.secs)
    ]))
    changes = [abs(i - j) for i, j in zip([px, py, pz], [lo[0], lo[1], o[2]])]
    # only post updates if enough has changed since the last send
    if sum(changes) > send_thresh:
        redis.set(redis_key, str(package))
        px, py, pz = lo[0], lo[1], o[2]  # update previous values
    bt.save_json_file("odomdump.json", json.loads(package))

def reset_cb(msg):
    package = json.dumps(OrderedDict([
        ("odom_id", 0),
        ("location", [0,0,0]),
        ("orientation",  [0,0,0]),
        ("linearvelocity", 0),
        ("angularvelocity", 0),
        ("time", 0)
    ]))
    redis.set(redis_key, str(package))

if __name__ == '__main__':
    rospy.init_node("odom_bridge")
    redis = bt.redis_client()
    redis_key = bt.namespace_key("Odom")
    
    send_thresh = float(rospy.get_param("pose_update_thresh", 0))
    px, py, pz = -1, -1, -1  # default previous values
    pose = None
    odom_sub = rospy.Subscriber("/odom", Odometry, odom_cb)
    reset_sub = rospy.Subscriber("/reset", Empty, reset_cb)
    pulse_sub = rospy.Subscriber("/pulse", Empty, pulse_cb)
    echo_pub = rospy.Publisher("/pulse_echo", String, queue_size=3)

    rospy.spin()
