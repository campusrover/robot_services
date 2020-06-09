#! /usr/bin/python
import math 

class Point():
    def __init__(self, point_2tuple):
        self.x = point_2tuple[0]
        self.y = point_2tuple[1]

    def distance(self, other_point):
        # returns the distance between this point and the given point
        return math.sqrt(math.pow(self.x - other_point.x, 2) + math.pow(self.y - other_point.y, 2))
    
    def distanceL(self, line):
        # returns the distance from this point to a line segment
        fake_l1 = Line_Segment([self.x, self.y, line.point1.x, line.point1.y])
        fake_l2 = Line_Segment([self.x, self.y, line.point2.x, line.point2.y])
        a1 = fake_l1.angle_diff(fake_l2)  # this is the only angle that's allowed to be obtuse, because it will be if the point is very close to the line segment
        a2 = min(line.angle_diff(fake_l1), line.angle_diff(fake_l2))
        a3 = math.pi - (a1 + a2)
        if max(a2, a3) > math.pi / 2:
            # if an angle larger than pi/2 exists between the given line and either of the fake lines, then the triangle formed by the three points is obtuse, and therefore the distance to the line from the point is best expressed as the distance from the point to the nearest endpoint of the line segment
            return min(self.distance(line.point1), self.distance(line.point2))
        else:
            # this only works if the three points (the free point and the two line segment points) create a triangle where the thrid point is above/below the line segment. The article below doesn't say that. I discovered it.  
            # https://en.wikipedia.org/wiki/Distance_from_a_point_to_a_line#Line_defined_by_two_points
            x1, y1, x2, y2 = line.fourtuple()
            x0, y0 = self.twotuple()
            return abs((y2 - y1)*x0 - (x2 - x1)*y0 + x2*y1 - y2*x1) / math.sqrt((y2-y1)**2 + (x2-x1)**2)

    def angle(self, other_point):
        # gives the angle that a line betweeen point and the other point create, in radians, between 0 and pi
        run = self.x - other_point.x
        rise = self.y - other_point.y
        return math.atan2(rise, run) % math.pi

    def copy(self):
        # creates a copy of this point
        return Point((self.x, self.y))

    def twotuple(self):
        return (round(self.x, 3), round(self.y, 3))

    def __hash__(self):
        return (round(self.x), round(self.y)).__hash__()

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

    def fourtuple(self, rounded=False):
        # returns the 4-tuple that was passed into to creat the object. 
        ft = [self.point1.x, self.point1.y, self.point2.x, self.point2.y]
        if rounded:
            ft = [[round(x) for x in l] for l in ft]
        return ft

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

    def angle_diff(self, other_line):
        mint, maxt = min(self.theta, other_line.theta), max(self.theta, other_line.theta)
        alt = (maxt - math.pi)
        return min(abs(maxt - mint), abs(mint - alt))

    def distance(self, other_line):
        # returns the distance between this line and another line
        if self.intersects_with(other_line):
            return 0
        else:
            ds = [self.point1.distanceL(other_line), self.point2.distanceL(other_line), other_line.point1.distanceL(self), other_line.point2.distanceL(self)]
            return min(ds)

    def distanceP(self, point):
        # returns the distance between this line and a point
        return point.distanceL(self)

    def intersects_with(self, other_line):
        # returns a boolean beased on if the two lines intersect each other
        # https://www.geeksforgeeks.org/check-if-two-given-line-segments-intersect/
        def orientation(p, q, r): 
            val = (float(q.y - p.y) * (r.x - q.x)) - (float(q.x - p.x) * (r.y - q.y)) 
            if (val > 0): 
                return 1  # Clockwise orientation 
            elif (val < 0): 
                return 2  # Counterclockwise orientation 
            else: 
                return 0  # Colinear orientation
        def onSegment(p, q, r): 
            if ( (q.x <= max(p.x, r.x)) and (q.x >= min(p.x, r.x)) and 
                (q.y <= max(p.y, r.y)) and (q.y >= min(p.y, r.y))): 
                return True
            return False
        p1, q1 = self.point1, self.point2
        p2, q2 = other_line.point1, other_line.point2
        o1, o2, o3, o4 = orientation(p1, q1, p2), orientation(p1, q1, q2), orientation(p2, q2, p1), orientation(p2, q2, q1)
        # General case 
        if ((o1 != o2) and (o3 != o4)): 
            return True
        # Special Cases
        if ((o1 == 0) and onSegment(p1, p2, q1)):  # p1 , q1 and p2 are colinear and p2 lies on segment p1q1 
            return True
        if ((o2 == 0) and onSegment(p1, q2, q1)):  # p1 , q1 and q2 are colinear and q2 lies on segment p1q1 
            return True
        if ((o3 == 0) and onSegment(p2, p1, q2)):  # p2 , q2 and p1 are colinear and p1 lies on segment p2q2
            return True
        if ((o4 == 0) and onSegment(p2, q1, q2)):  # p2 , q2 and q1 are colinear and q1 lies on segment p2q2
            return True
        return False  # If none of the cases 


    def __eq__(self, other):
        return self.id == other.id and self.point1 == other.point1 and self.point2 == other.point2

    def __hash__(self):
        return tuple([round(x) for x in [self.point1.x, self.point1.y, self.point2.x, self.point2.y]]).__hash__()

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
    # define the orientation of the new merged line
    # this section of the algorithm does it slightly differently than exactly how T+P's paper describes, but the result is the same - a weighted average of the line's angles based on length
    # improvements include no skewing when two lines are similar in angle but nominally different (e.g. 3.14 and 0.001) and no divide by 0 error possibility (a fault of T+P's alg if l2.theta = 0)
    t1 = l1.theta
    t2 = l2.theta
    if abs(l1.theta - l2.theta) > math.pi / 2:
        if l2.theta > l1.theta:
            t2 = (l2.theta - math.pi)
        else:
            t1 = (l1.theta - math.pi)
    theta = (l1.length * t1 + l2.length * t2) / (l1.length + l2.length)
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
    #newline = [round(i) for i in newline]

    return Line_Segment(newline)


def convert_coords(lines, origin, size, res, tf):
    # input: lines in a list of 4-tuples, origin (coordinate in meters of the lower left corner, straight from the /map topic), map size (2-tuple width and height in pixels), map resolution (in meters/pixel), tf (the transfrom from odom to map, a tuple of (trnas, rot). rot must be euler)
    # converts pixel coordinates from pixel units anchored to the upper-right of an image to meter units anchored to the point designated by the /map topic
    pix_x = -origin[0] / res
    pix_y = size[1] + (origin[1] / res)
    o = (pix_x, pix_y) # the origin of the map in image pixel coordinates
    trans = tf[0]
    rot = tf[1]
    new_lines = []
    for l in lines:
        new_l = [0,0,0,0]
        for i in range(4):
            new_l[i] = (l[i] - o[i%2]) * res * (-1 if i%2==1 else 1) + trans[i%2] # the pixel coordinate, shifted by the map origin, scaled to meters by resolution, fliped +/- if a y coordinate, and shifted by tf from odom to map]
        new_lines.append([round(x, 3) for x in new_l])
    # rotate all points about the origin based on the tf
    theta = rot[2]  # z in euler 
    rot_lines = []
    for l in new_lines:
        new_rot_line = []
        for p in [(l[0], l[1]), (l[2], l[3])]:
            x = p[0] * math.cos(theta) - p[1] * math.sin(theta)
            y = p[1] * math.cos(theta) + p[0] * math.sin(theta)
            new_rot_line.append(x)
            new_rot_line.append(y)
        rot_lines.append(new_rot_line)


    return rot_lines


def TP_angle_test(t1, t2):
    # small test-case of lines 108-117
    # lengths assumes to be = 1 for both lines
    if abs(t1 - t2) <= math.pi / 2:
        t = t1 + t2 / 2
        print('A')
    else:
        t2 += 0.0000000000000000000000000001
        t2 = (t2 - math.pi * (t2 / abs(t2))) % math.pi
        t = t1 + t2 / 2
        print('B')
        print(t2)
    return t

if __name__ == '__main__':
    # scrap script space to test that parts are working
    l1 = Line_Segment([250, 200, 252, 189])
    l2 = Line_Segment([245, 208, 243, 215])
    l3 = Line_Segment([147, 194, 167, 224])
    l6 = Line_Segment([147, 224, 133, 224])
    l4 = Line_Segment([1, 1, 5, 1])
    l5 = Line_Segment([10, 2,  20, 2])
    print(l2.intersects_with(l1))
    print("l2 to l1:", l2.distance(l1))
    print(l2.angle_diff(l1))
    
    #print(Point((252, 189)).distanceL(l2))
    print("l4 to l5: ", l4.distance(l5))
    print("l3 to l6", l3.distance(l6))
    """
    print(l2.intersects_with(l3))
    print(l2.distance(l3))
    print(l2.angle_diff(l3))
    print(l1.intersects_with(l3))
    print(l1.distance(l3))
    print(l1.angle_diff(l3))
    """
