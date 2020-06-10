#! /usr/bin/python
import rospy, redis, json
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Empty
from collections import OrderedDict

def scan_cb(msg):
    global slices
    slice_size = int(round(len(msg.ranges) / slices))
    first_slice = msg.ranges[len(msg.ranges) - slice_size/2:] + msg.ranges[:slice_size/2]
    ranges = [min(first_slice)]
    s = slice_size/2
    while s+slice_size < len(msg.ranges):
        ranges.append(min(msg.ranges[s:s+slice_size]))
        s+=slice_size

    package = json.dumps(OrderedDict([
        ("max_range", msg.range_max),
        ("min_range", msg.range_min),
        ("slices", slices),
        ("fov", msg.angle_increment * len(msg.ranges)), # field of view
        ("data", ranges)
    ]))

    redis.set(redis_key, str(package))
    rospy.Rate(1).sleep()

def reset_cb(msg):
    package = json.dumps(OrderedDict([
        ("max_range", 0),
        ("min_range", 0),
        ("slices", 0),
        ("fov", 0), # field of view
        ("data", [])
    ]))

    redis.set(redis_key, str(package))


if __name__ == '__main__':
    rospy.init_node("lidar_bridge")
    r_serv = rospy.get_param("redis_server", "")
    r_port = rospy.get_param("redis_port", "")
    r_pass = rospy.get_param("redis_pw", "")
    if r_serv and r_port and r_pass:
        redis = redis.Redis(host=r_serv, port=int(r_port), password=r_pass)
    else:
        redis = redis.Redis()

    redis_key = "Lidar"
    r_namespace = rospy.get_param("redis_ns", "")
    if r_namespace:
        redis_key = r_namespace + "/" + redis_key
    slices = min(int(rospy.get_param("lidar_slices", 4)), 360)
    scan_sub = rospy.Subscriber("/scan", LaserScan, scan_cb)
    reset_sub = rospy.Subscriber("/reset", Empty, reset_cb)

    rospy.spin()