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
import json
import redis
import cv2
import numpy as np 
from nav_msgs.msg import OccupancyGrid
from os.path import dirname, realpath
from os import sep
import sys
from PIL import Image, ImageDraw

redis = redis.Redis()
redis_key = "Map"
rospy.init_node("map_bridge")
map_id = 0

def consolodate_lines(lines, distance_diff, angle_diff):
    # input: list of 4-tuples representing lines
    # steps:
    # 1. convert all line tuples in Line_Segment objects. remove any points that are masquerading as lines
    # 2. hash every line with their endpoints being the keys
    # 3. iterate through all the line_segments. if there is another segment with a similar enough slope that is close enough to the line, then pair them together and put them in a queue to be merged. remove both lines from data structures as well
    # 4. go through the queue of pairs to be merged and merge them. put the resulting line segments back into hash and list. 
    # 5. repeat 3 and 4 until an iteration yeilds no pairs to be merged. 
    # 6. convert all Line_Segments to 4-tuples and return that list


def get_nearby_file(filename):
    return dirname(realpath(sys.argv[0])) + sep + filename

def draw_map(lines, fp=None):
    im = Image.new("1", (384, 384), color=0)
    d = ImageDraw.Draw(im)

    for l in lines:
        try:
            x1, y1, x2, y2 = l[0,0], l[0,1], l[0,2], l[0,3]
        except:
            x1, y1, x2, y2 = l[0], l[1], l[2], l[3]
        d.line((x1, y1, x2, y2), fill=1, width=1)

    print(len(lines))
    if not fp:
        im.show()
        im.save(get_nearby_file("hough_line_map2.png"))
    else:
        im.save(fp)

def map_cb(msg):
    global map_id
    grid = msg.data
    width = msg.info.width
    height = msg.info.height
    res = msg.info.resolution

    # convert the grid to a numpy array - also flip valence of pixels
    # ros maps use black do depict obstacles and white to denote open space, but opencv hough lines uses white to detect lines and ignores black. 
    img = np.zeros([height, width, 3], dtype=np.uint8)  # start with an initially black image with dimensions of the image
    for y in xrange(height):
        for x in xrange(width):
            ind = (y * width) + x
            if grid[ind] > 0:  # if there is an obstacle at that point, add it to the image as a white pixel
                img[y, x, :] = np.uint8(255)

    img = np.flip(img, 0) # vertical flip is required of the data. 
 
    # perform hough transform to get the lines
    g_img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    # generate 4500 images to try to find the best paramters (most accurate image, fewest lines)
    for rho in range(1, 5):
        for intersect in range(20, 50):
            for minlength in range(10, 40):
                lines = cv2.HoughLinesP(g_img, rho, np.pi/180, intersect, minlength, 0)
                path = get_nearby_file('') + sep + 'testdata' + sep + '{}-{}-{}-{}.png'.format(len(lines), rho, intersect, minlength)
                draw_map(lines, fp=path)

   
    lines = cv2.HoughLinesP(g_img, 5, np.pi/180, 45, 30, 0)
    
    # convert lines list into better list
    walls = []
    for l in lines: 
        walls.append([int(l[0,0]), int(l[0,1]), int(l[0,2]), int(l[0,3])])
    
    print("---")
    draw_map(walls)  # TODO remove this once this is stable

    package = json.dumps({
        "resolution": res, 
        "width": width,
        "height": height,
        "data": walls,
        "id": map_id, 
        "line_count": len(walls)
    })
    map_id += 1

    with open(get_nearby_file("walldump.json"), 'w') as f:
        f.write(str(package))

    # push to redis
    redis.rpush(redis_key, str(package))
    # save 
    cv2.imwrite(get_nearby_file('bw_map_img_flip_v.jpg'), img)
    

map_sub = rospy.Subscriber("/map", OccupancyGrid, map_cb)
rospy.spin()