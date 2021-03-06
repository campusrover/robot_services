#! /usr/bin/python
"""
ROS map to line segment JSON converter
originally made for voxelworld Kirby
this node: 
* subscribes to /map
* converts the 1D map array into a 2D numpy uint8 array
* passes that numpy array through cv2  HoughP to get a list of lines that mark tha walls of the map
* converts that list of lines back to a JSON serializable python list
* pushes the JSON string to a redis list with the key "Map"
* saves both the JSON and image locally

Dependancies: numpy, python-opencv, redis-py, Pillow (for debug), rospy
"""
import rospy
import tf
import json
import redis
import cv2
from tf.transformations import euler_from_quaternion
import numpy as np 
from nav_msgs.msg import OccupancyGrid
from tavares_padilha_line_merge import Point, Line_Segment, merge_lines, convert_coords
import bridge_tools as bt
from math import pi
from collections import OrderedDict

def brute_force_consolodation(lines, distance_diff, angle_diff, min_len):
    ls = sorted([Line_Segment(l, i+1) for l, i in zip(lines, range(len(lines))) if Line_Segment(l).length > 0], key=lambda x: x.length, reverse=True)
    ls_set = set(ls)
    did_a_merge = True
    while did_a_merge:
        did_a_merge = False
        for l1 in ls:
            for l2 in ls:
                if l1 in ls_set and l2 in ls_set and l1.distance(l2) < distance_diff and l1.angle_diff(l2) < angle_diff and l1 != l2:
                    ls_set.remove(l1)
                    ls_set.remove(l2)
                    ls_set.add(merge_lines(l1, l2))
                    did_a_merge = True
        ls = sorted(list(ls_set), key=lambda x: x.length, reverse=True)   
    return sorted([l.fourtuple() for l in ls if l.length >= min_len], key=lambda x: (x[0], x[1], x[2], x[3]))


def map_cb(msg):
    global map_shift, map_rot
    grid = msg.data
    width = msg.info.width
    height = msg.info.height
    res = msg.info.resolution
    origin = msg.info.origin
    map_id = msg.header.seq

    # convert the grid to a numpy array - also flip valence of pixels
    # values in grid are from -1 to 100, probability of obstacle in that cell (-1 is unknown)
    # ros maps use black (100%) do depict obstacles and white (0%) to denote open space, but opencv hough lines uses white to detect lines and ignores black. 
    g_img = np.zeros([height, width], dtype=np.uint8)  # start with an initially black image with dimensions of the image
    for y in xrange(height):
        for x in xrange(width):
            ind = (y * width) + x
            val = max(0, min(100, grid[ind]))  # clamp value between 0 and 100, remove extraneous values and treat -1 as 0
            g_img[height - 1 - y, x] = np.uint8(255/100 * val)  # pixel value is occupied % of 255. inserted with y coord height - 1 - y because a vertical flip is required from the raw grid. this does it in one pass w/o needing another operation (e.g. numpy.flip)
          
 
    if width + height < 1500:  # arbitrary value picked, dialation slows down hough on large images
        g_img = cv2.dilate(g_img, None)

    # set limits to hough parameters
    max_dist = 7

    # generate hough paramters
    min_line_length = round(max(width, height) / 20)
    intersections = int(2 * min_line_length)
    dist = min(int(round(min_line_length / 3)), max_dist)
    
    lines = cv2.HoughLinesP(g_img, 5, np.pi/180, intersections, min_line_length, dist) # started with  rho = 5, minint = 45, minlen = 30, dist=0
    
    if lines.size != 0:
        # convert lines list into better list where coords are in meters and have same origin as odom
        walls = []
        for l in lines: 
            walls.append([int(l[0,0]), int(l[0,1]), int(l[0,2]), int(l[0,3])])
        walls = brute_force_consolodation(walls, dist, pi / 12, 4)
        walls = convert_coords(walls, (origin.position.x, origin.position.y), (width, height), res, (map_shift, map_rot))
        
        package = json.dumps(OrderedDict([
            ("id", map_id), 
            ("line_count", len(walls)),
            ("width", round(width * res, 3)),
            ("height", round(height * res, 3)),
            ("origin", (origin.position.x * res + map_shift[0], origin.position.y * res + map_shift[1])),
            ("data", walls)
        ]))
        # save most recent copy of JSON to a local file
        bt.save_json_file("walldump.json", json.loads(package))

        # push to redis
        redis.rpush(redis_key, str(package))
        # save what the map looks like to ros to a local file
        cv2.imwrite(bt.get_nearby_file('bw_map_img.jpg'), g_img)
    
    
if __name__ == "__main__":
    # init node and get redis info from rosparam. use local redis if not all params provided
    rospy.init_node("map_bridge")
    redis = bt.redis_client()
    redis_key = bt.namespace_key("Map")

    queue_size = rospy.get_param("redis_qs", 5)
    map_sub = rospy.Subscriber("/map", OccupancyGrid, map_cb)
    bt.establish_reset(redis, redis_key, bt.reset_queue, None, "data", "id", "Line_count", "width", "height")  
    bt.establish_pulse()
    map_shift = [0,0,0]
    map_rot = [0,0,0]
    listener = tf.TransformListener()
    tf_present = False
    rate = rospy.Rate(5)
    # main loop: get tf from odom to map
    while not rospy.is_shutdown():
        try:
            map_shift, map_rot = listener.lookupTransform("odom", "map", rospy.Time(0))  # gives us the tf from odom to map. values inverted for tf from map to odom. 
            map_rot = euler_from_quaternion(map_rot)
            if not tf_present:
                rospy.loginfo_once("MAP_BRIDGE: Odom to map tf found")
                tf_present = True
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            if tf_present:
                rospy.loginfo_once("MAP_BRIDGE: no transform from odom to map")
                tf_present = False
            continue
        # trim queue size
        while redis.llen(redis_key) > queue_size:
            redis.lpop(redis_key)
        rate.sleep()
       
