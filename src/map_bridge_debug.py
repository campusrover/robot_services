#! /usr/bin/python
"""
robby ros/unity bridge via redis
this node: 
* subscribes to /map
* converts the 1D map array into a 2D numpy uint8 array
* passes that numpy array through cv2 Canny and HoughP to get a list of lines that mark tha walls of the map
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
from os.path import dirname, realpath
from os import sep
import sys
from PIL import Image, ImageDraw
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


def consolodate_lines2(lines, distance_diff, angle_diff, min_len):
    # using some new tools in the Point and Line_Segment objects, improve upon the old algorithm
    ls = sorted([Line_Segment(l, i+1) for l, i in zip(lines, range(len(lines))) if Line_Segment(l).length > 0], key=lambda x: x.length, reverse=True)
    
    # n^2 dict builder of lines that are close to each other
    close_lines = {}
    for l1 in ls:
        close_lines[l1] = set()
        for l2 in ls:
            if l2 != l1 and l1.distance(l2) < distance_diff:
                close_lines[l1].add(l2)
                print(l1.distance(l2), l1, l2)
    # build merge groups
    merge_groups = []
    merge_memberships = {}  # stores the indexes of the merge groups each line belongs to
    for l1 in ls:
        for l2 in close_lines[l1]:
            if l1.angle_diff(l2) < angle_diff:
                groups = set(merge_memberships.get(l1, [-1]) + merge_memberships.get(l2, [-1]))
                for group in groups:
                    if group >= 0:
                        merged = merge_groups[group]
                        if merged.angle_diff(l1) < angle_diff and merged.distance(l1) < distance_diff:
                            merged = merge_lines(merged, l1)
                            merge_memberships[l1] = merge_memberships.get(l1, []) + [group]
                        if merged.angle_diff(l2) < angle_diff and merged.distance(l2) < distance_diff:
                            merged = merge_lines(merged, l2)
                            merge_memberships[l2] = merge_memberships.get(l2, []) + [group]
                        merge_groups[group] = merged
                    else:
                        merge_groups.append(merge_lines(l1, l2))
                        merge_memberships[l1] = [len(merge_groups) - 1]
                        merge_memberships[l2] = [len(merge_groups) - 1]

    # remove lines staged to be merged from list
    for l in merge_memberships.keys():
        ls.remove(l)
    
    print("{} lines did not get merged, {} new lines created".format(len(ls), len(merge_groups)))
    ls += merge_groups
    
    return sorted([l.fourtuple() for l in ls if l.length >= min_len], key=lambda x: (x[0], x[1], x[2], x[3]))

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
                    l3 = merge_lines(l1, l2)
                    ls_set.add(l3)
                    if l3.length > l2.length + l1.length:
                        print(l1, l2)
                        print(distance_diff, l1.distance(l2), angle_diff, l1.angle_diff(l2), l1.length, l2.length, l3.length)
                        print(l3)
                    did_a_merge = True
        print("complete loop {}->{}".format(len(ls), len(ls_set)))
        ls = sorted(list(ls_set), key=lambda x: x.length, reverse=True)   
    return sorted([l.fourtuple() for l in ls if l.length >= min_len], key=lambda x: (x[0], x[1], x[2], x[3]))


def get_nearby_file(filename):
    return dirname(realpath(sys.argv[0])) + sep + filename

def draw_map(lines, w, h, map_num=2, fp=None):
    im = Image.new("RGB", (w, h), color="black")
    d = ImageDraw.Draw(im)
    i=0
    for l in lines:
        try:
            x1, y1, x2, y2 = l[0,0], l[0,1], l[0,2], l[0,3]
        except:
            x1, y1, x2, y2 = l[0], l[1], l[2], l[3]
        d.line((x1, y1, x2, y2), fill="white", width=1)
        
        d.text(((x1 + x2)/2, (y1 + y2)/2), "{}".format(i), fill="green")
        i+=1
    d.text((20,20), str(len(lines)), fill="green")
    print(len(lines))
    if not fp:
        #im.show()
        im.save(get_nearby_file("hough_line_map{}.png".format(map_num)))
    else:
        im.save(fp)



def map_cb(msg):
    global map_id, map_shift, map_rot
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
            g_img[height - 1 - y, x] = np.uint8(255/100 * val)  # pixel value is occupied % of 255
          
 
    # perform hough transform to get the lines
    if width + height < 1500:  # arbitrary value picked, dialation slows down hough on large images
        g_img = cv2.dilate(g_img, None)
    """
    g_img = cv2.Canny(g_img, 100, 200)
    g_img = cv2.dilate(g_img, None)
    cv2.imshow("dialtedCanny", g_img)
    cv2.waitKey()
    cv2.destroyAllWindows()
    cv2.imwrite(get_nearby_file('hough_line_map0.png'), g_img)
    """
    # set limits to hough parameters
    max_dist = 7

    # generate hough paramters
    min_line_length = round(max(width, height) / 5)
    intersections = int(2 * min_line_length)
    dist = min(int(round(min_line_length / 3)), max_dist)
    
    lines = cv2.HoughLinesP(g_img, 8, np.pi/180, intersections, min_line_length, dist) # started with  rho = 5, minint = 45, minlen = 30, dist=0
    # print(lines[0])
    # exit()
    # convert lines list into better list
    walls = []
    for l in lines: 
        walls.append([int(l[0,0]), int(l[0,1]), int(l[0,2]), int(l[0,3])])

    draw_map(walls, width, height, map_num=1)  # this creates hough_line_map1
    package = json.dumps({ 
        "width": round(width * res, 3),
        "height": round(height * res, 3),
        "data": walls,
        "id": "raw hough lines", 
        "line_count": len(walls)
    })
    with open(get_nearby_file("walldumpprelim.json"), 'w') as f:
        f.write(str(package))
    print("---")
    walls2 = [w for w in walls]
    walls = consolodate_lines(walls, dist, pi / 12, 4)
    pixel_walls = [[round(x) for x in l] for l in walls]
    draw_map(pixel_walls, width, height, map_num=2)  # TODO remove this once this is stable
    walls2 = brute_force_consolodation(walls2, dist, pi / 12, 4)
    pixel_walls = [[round(x) for x in l] for l in walls2]
    draw_map(pixel_walls, width, height, map_num=3)
    walls = convert_coords(walls, (origin.position.x, origin.position.y), (width, height), res, (map_shift, map_rot))
    
    
    

    package = json.dumps({ 
        "width": round(width * res, 3),
        "height": round(height * res, 3),
        "data": walls,
        "id": map_id, 
        "line_count": len(walls)
    })

    with open(get_nearby_file("walldump.json"), 'w') as f:
        f.write(str(package))

    # push to redis
    redis.rpush(redis_key, str(package))
    # save 
    cv2.imwrite(get_nearby_file('bw_map_img.jpg'), g_img)
    
if __name__ == "__main__":
    redis = redis.Redis()
    redis_key = "Map"
    rospy.init_node("map_bridge")
    map_id = 0
    map_sub = rospy.Subscriber("/map", OccupancyGrid, map_cb)
    map_shift = [0,0,0]
    map_rot = [0,0,0,0]
    listener = tf.TransformListener()
    while not rospy.is_shutdown():
        try:
            map_shift, map_rot = listener.lookupTransform("odom", "map", rospy.Time(0))  # gives us the tf from odom to map. values inverted for tf from map to odom. 
            map_rot = euler_from_quaternion(map_rot)
            #print("MAP TF ", map_shift, map_rot)
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue

       