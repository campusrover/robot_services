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
from std_msgs.msg import Empty
from os.path import dirname, realpath
from os import sep
import sys
from tavares_padilha_line_merge import Point, Line_Segment, merge_lines, convert_coords
from math import pi

def find_close_lines(l, close_lines, line_hash, distance_diff, angle_diff, og_angle):
    nearby = set(l.nearby_points(distance_diff)).intersection(line_hash.keys()) # the set of nearby endpoints of other lines to line l
    new_close_lines = set()
    cl_set = set(close_lines)
    for n in nearby:
        for cl in line_hash[n]:
            # get the angle difference
            larger_theta = max(cl.theta, og_angle)
            smaller_theta = min(cl.theta, og_angle)
            alt_theta = smaller_theta + pi
            theta_diff = min(larger_theta - smaller_theta, alt_theta - larger_theta)
            # if the angle difference is small enough comapred to the original angle, and cl is not already in the list of close lines, then add it
            if  theta_diff < angle_diff and cl != l and cl not in cl_set and cl not in new_close_lines: 
                new_close_lines.add(cl)

    return list(new_close_lines)


def consolodate_lines(lines, distance_diff, angle_diff, min_len):
    # input: list of 4-tuples representing lines, how close lines need to be together and how similar their angles need to be to be merged
    # steps:

    # 1. convert all line tuples in Line_Segment objects. remove any points that are masquerading as lines
    ls = sorted([Line_Segment(l, i+1) for l, i in zip(lines, range(len(lines))) if Line_Segment(l).length > 0], key=lambda x: x.length, reverse=True)
    next_id = len(lines) + 1
    line_hash = {}
    # 2. hash every line with their endpoints being the keys. basically a bucket hash map using python lists inside dicts
    for l in ls:
        line_hash[l.point1] = line_hash.get(l.point1, []) + [l]
        line_hash[l.point2] = line_hash.get(l.point2, []) + [l]
    # 3. iterate through all the line_segments.  
    keep_merging = True
    merge_queue = []
    while keep_merging:
        i = 0
        while i < len(ls):
            l = ls[i]
            close_lines = [l]
            # TODO: put close lines in here, and find close lines of close lines too
            close_lines += find_close_lines(l, close_lines, line_hash, distance_diff, angle_diff, l.theta)
            close_lines.remove(l)
            if close_lines:
                # find the close lines of the close lines to l
                for cl in close_lines:
                    close_lines += find_close_lines(cl, close_lines, line_hash, distance_diff, angle_diff, l.theta)
                # sort the close lines based on angle difference, then length
                close_lines = sorted(close_lines, key=lambda x: (abs(x.theta - l.theta), -x.length))
                merge_queue.append((l, close_lines))
                if l in close_lines:
                    close_lines.remove(l)
                # remove l from hash and list
                line_hash[l.point1].remove(l)
                line_hash[l.point2].remove(l)
                ls.pop(i)
                # remove all the lines l will be merged with from the hash and list
                for merge_pal in close_lines:
                    line_hash[merge_pal.point1].remove(merge_pal)
                    line_hash[merge_pal.point2].remove(merge_pal)
                    ls.pop(ls.index(merge_pal))
            else:
                # i only increments if l[i] is not merged, becuase if it is merged then l[i] gets popped and l[i+1] becomes l[i]
                i += 1
    # 4. go through the queue of pairs to be merged and merge them. put the resulting line segments back into hash and list. 
        if merge_queue:
            for pair in merge_queue:
                m = pair[0]
                for l2 in pair[1]:
                    m = merge_lines(m, l2)
                merged = m
                merged.id = next_id
                next_id += 1
                # add new line to the data structures
                ls.append(merged)
                ls = sorted(ls, key=lambda x: x.length, reverse=True)
                line_hash[merged.point1] = line_hash.get(merged.point1, []) + [merged]
                line_hash[merged.point2] = line_hash.get(merged.point2, []) + [merged]
            # empty the queue
            merge_queue = []
    # 5. repeat 3 and 4 until an iteration yeilds no pairs to be merged. 
        else:
            keep_merging = False

    # 6. convert all Line_Segments to 4-tuples and return that list
    return sorted([l.fourtuple() for l in ls if l.length >= min_len], key=lambda x: (x[0], x[1], x[2], x[3]))


def get_nearby_file(filename):
    return dirname(realpath(sys.argv[0])) + sep + filename


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
    
    # convert lines list into better list where coords are in meters and have same origin as odom
    walls = []
    for l in lines: 
        walls.append([int(l[0,0]), int(l[0,1]), int(l[0,2]), int(l[0,3])])
    walls = consolodate_lines(walls, dist, pi / 12, 4)
    walls = convert_coords(walls, (origin.position.x, origin.position.y), (width, height), res, (map_shift, map_rot))
    
    package = json.dumps({ 
        "width": round(width * res, 3),
        "height": round(height * res, 3),
        "data": walls,
        "id": map_id, 
        "line_count": len(walls)
    })
    # save most recent copy of JSON to a local file
    with open(get_nearby_file("walldump.json"), 'w') as f:
        json.dump(json.loads(package), f)

    # push to redis
    redis.rpush(redis_key, str(package))
    # save what the map looks like to ros to a local file
    cv2.imwrite(get_nearby_file('bw_map_img.jpg'), g_img)

def reset_cb(msg):
    # empty the list 
    while int(redis.llen(redis_key)) > 0:
        redis.lpop(redis_key)
    # push empty JSON
    package = json.dumps({ 
        "width": 0,
        "height": 0,
        "data": [],
        "id": 0, 
        "line_count": 0
    })
    redis.rpush(redis_key, str(package))

    
    
if __name__ == "__main__":
    # init node and get redis info from rosparam. use local redis if not all params provided
    rospy.init_node("map_bridge")
    r_serv = rospy.get_param("redis_server", "")
    r_port = rospy.get_param("redis_port", "")
    r_pass = rospy.get_param("redis_pw", "")
    if r_serv and r_port and r_pass:
        redis = redis.Redis(host=r_serv, port=int(r_port), password=r_pass)
    else:
        redis = redis.Redis()
    redis_key = "Map"
    r_namespace = rospy.get_param("redis_ns", "")
    if r_namespace:
        redis_key = r_namespace + "/" + redis_key
    map_sub = rospy.Subscriber("/map", OccupancyGrid, map_cb)
    reset_sub = rospy.Subscriber("/reset", Empty, reset_cb)
    map_shift = [0,0,0]
    map_rot = [0,0,0]
    listener = tf.TransformListener()
    rate = rospy.Rate(5)
    # main loop: get tf from odom to map
    while not rospy.is_shutdown():
        try:
            map_shift, map_rot = listener.lookupTransform("odom", "map", rospy.Time(0))  # gives us the tf from odom to map. values inverted for tf from map to odom. 
            map_rot = euler_from_quaternion(map_rot)
            rospy.loginfo("Odom to map tf found")
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            rospy.loginfo("no transform from odom to map")
            continue
        rate.sleep()
       
