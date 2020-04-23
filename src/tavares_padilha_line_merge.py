#! /usr/bin/python
import math 

class Point():
    def __init__(self, point_2tuple):
        self.x = round(point_2tuple[0])
        self.y = round(point_2tuple[1])

    def distance(self, other_point):
        # returns the distance between this point and the given point
        return math.sqrt(math.pow(self.x - other_point.x, 2) + math.pow(self.y - other_point.y, 2))

    def angle(self, other_point):
        # gives the angle that this point and the other point create, in radians, between 0 and pi
        run = self.x - other_point.x
        rise = self.y - other_point.y
        return math.atan2(rise, run) % math.pi

    def copy(self):
        # creates a copy of this point
        return Point((self.x, self.y))

    def __hash__(self):
        return (self.x, self.y).__hash__()

    def __eq__(self, other):
        return self.x == other.x and self.y == other.y

    def __repr__(self):
        # print the point as "(x, y)"
        return '({}, {})'.format(self.x, self.y)


class Line_Segment():
    def __init__(self, line_4tuple, lid=None):
        self.id = lid
        self.point1 = Point((line_4tuple[0], line_4tuple[1]))
        self.point2 = Point((line_4tuple[2], line_4tuple[3]))
        self.length = self.point1.distance(self.point2)
        self.theta = self.point1.angle(self.point2)
        if self.theta == 0.0:
            self.theta = math.pi
        self.perpindicular = (self.theta + math.pi / 2) % math.pi

    def fourtuple(self):
        # returns the 4-tuple that was passed into to creat the object. 
        return [self.point1.x, self.point1.y, self.point2.x, self.point2.y]

    def nearby_points(self, radius, res=8):
        # finds all points close to this line segment
        # params: radius is the distane from the line to search within
        # determine which point to start at. the slop is always between 0-pi, so the lower one should be the starter
        traverse_len = int(self.length) # + 2*radius)
        if self.point2.y < self.point1.y:
            start = self.point2.copy()
            end = self.point1.copy()
        else:
            start = self.point1.copy()
            end = self.point2.copy()
        try:
            direction = (end.x - start.x) / abs(end.x - start.x)  # either 1 or -1
        except:
            direction = 1  # in case divide by 0
        #start = Point((start.x + math.cos(self.theta) * -direction * radius, start.y + math.sin(self.theta) * -radius))
        #end = Point((end.x + math.cos(self.theta) * direction * radius, end.y + math.sin(self.theta) * radius))
        points = set()
        for l in range(traverse_len):
            for r in range(-radius, radius + 1):
                x = start.x + math.cos(self.perpindicular) * r
                y = start.y + math.sin(self.perpindicular) * r
                points.add((int(x), int(y)))
            start.x += math.cos(self.theta) * -direction 
            start.y += math.sin(self.theta)

        # get nearby points around the endpoints too
        for p in [self.point1, self.point2]:
            for a in range(res):
                t = math.pi / (res/2) * a
                for r in range(radius + 1):
                    x = round(p.x + math.cos(t) * r)
                    y = round(p.y + math.sin(t) * r)
                    points.add((x, y))

        return [Point((x, y)) for x,y in points]


    def __eq__(self, other):
        return self.id == other.id and self.point1 == other.point1 and self.point2 == other.point2

    def __repr__(self):
        # prints as "[(x1, y1), (x2, y2)]"
        if not self.id:
            return '[{}, {}]'.format(self.point1, self.point2)
        else:
            return '(#{})[{}, {}]'.format(self.id, self.point1, self.point2)



def merge_lines(l1, l2):
    # define a centroid, g, formed by the 4 given endpoints and weighted by line lengths
    gx = (l1.length * (l1.point1.x + l1.point2.x) + l2.length * (l2.point1.x + l2.point2.x)) / (2 * (l1.length + l2.length))
    gy = (l1.length * (l1.point1.y + l1.point2.y) + l2.length * (l2.point1.y + l2.point2.y)) / (2 * (l1.length + l2.length))
    centroid = Point((gx, gy))
    # define the orientation of the new merges line
    if abs(l1.theta - l2.theta) <= math.pi / 2:
        theta = (l1.length * l1.theta + l2.length * l2.theta) / (l1.length + l2.length)
    else:
        try:
            theta = (l1.length * l1.theta + l2.length * (l2.theta - math.pi * (l2.theta / abs(l2.theta)))) / (l1.length + l2.length)  # add immensely small number to l2.theta in case l2.theta=0
        except:
            print(l2.theta, l2.id, l1.length + l2.length)
            exit()
    # convert all coordinates to a new frame centered around the cnetroid at angle theta
    # a and b belong to l1, c and d belong to l2.
    # only x converrstions are required 
    axg = (l1.point1.y - gy) * math.sin(theta) + (l1.point1.x - gx) * math.cos(theta)
    bxg = (l1.point2.y - gy) * math.sin(theta) + (l1.point2.x - gx) * math.cos(theta)
    cxg = (l2.point1.y - gy) * math.sin(theta) + (l2.point1.x - gx) * math.cos(theta)
    dxg = (l2.point2.y - gy) * math.sin(theta) + (l2.point2.x - gx) * math.cos(theta)
    new_coords = [axg, bxg, cxg, dxg]
    # the xmin and xmax of the new endpoints of the merged line
    new_end1 = min(new_coords)
    new_end2 = max(new_coords)
    # convert back to normal coordinates
    x1 = gx + (math.cos(theta) * new_end1)
    x2 = gx + (math.cos(theta) * new_end2)
    y1 = gy + (math.sin(theta) * new_end1)
    y2 = gy + (math.sin(theta) * new_end2)
    newline = [x1, y1, x2, y2]
    newline = [round(i) for i in newline]

    return Line_Segment(newline)


def convert_coords(lines, origin, size, res, tf):
    # input: lines in a list of 4-tuples, origin (coordinate in meters of the lower left corner, straight from the /map topic), map size (2-tuple width and height in pixels), map resolution (in meters/pixel), tf (the transfrom from odom to map, a tuple of (trnas, rot))
    # converts pixel coordinates from pixel units anchored to the upper-right of an image to meter units anchored to the point designated by the /map topic
    pix_x = -origin[0] / res
    pix_y = size[1] + (origin[1] / res)
    o = (pix_x, pix_y) # the origin of the map in image pixel coordinates
    trans = tf[0]
    rot = tf[1]
    print(o)
    new_lines = []
    for l in lines:
        new_l = [0,0,0,0]
        for i in range(4):
            new_l[i] = (l[i] - o[i%2]) * res * (-1 if i%2==1 else 1) + trans[i%2] # the pixel coordinate, shifted by the map origin, scaled to meters by resolution, fliped +/- if a y coordinate, and shifted by tf from odom to map]
            # TODO: rotation

        new_lines.append([round(x, 3) for x in new_l])

    return new_lines


if __name__ == '__main__':
    # scrap script space to test that parts are working
    
    a = Line_Segment([156,128,161,56])
    b = Line_Segment([162,54,160,91])
    c = Line_Segment([159,73,159,62])
    d = Line_Segment([50, 50, 100, 50])
    e = Line_Segment([183,67,197,66])
    print(math.pi / 6)

    for l in [a,b,c,d,e]:
        print(l.theta)
        l.nearby_points(3)

    #print(sorted(a.nearby_points(3), key=lambda x: (x.x, x.y)))
    print(merge_lines(a, b))
    
    print(5 * -True)
    print(9 * False)
    print(convert_coords([x.fourtuple() for x in [a,b,c,d,e]], (-10,-10), (384,384), 0.05))
    """
    l = Line_Segment([151, 173, 151, 167])
    print(sorted(l.nearby_points(2), key=lambda x: (x.x, x.y)))
    
    lines = {}
    lines[l.point1] = l
    print(lines)
    lines.pop(l.point1)
    print(lines)
    llist = [l]
    print(llist.index(l))
    st = [1, 2, 3]
    st += [4]
    st.remove(2)
    print(st)
    print(l == l)
    """
    """
    print("l length: ", l.point1.distance(l.point2))
    t = l.point2.angle(l.point1)
    print(t)
    l2 = Line_Segment([3, 1,4,5])

    print("lengths: ", l.length, l2.length)
    merged = merge_lines(l, l2)
    print(merged)
    """
