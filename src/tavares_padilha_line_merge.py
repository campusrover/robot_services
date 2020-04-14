import math 

class Point():
    def __init__(self, point_2tuple):
        self.x = int(point_2tuple[0])
        self.y = int(point_2tuple[1])

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
        self.perpindicular = (self.theta + math.pi / 2) % math.pi

    def fourtuple(self):
        # returns the 4-tuple that was passed into to creat the object. 
        return [self.point1.x, self.point1.y, self.point2.x, self.point2.y]

    def nearby_points(self, radius, res=8):
        # finds all points close to this line segment
        # params: radius is the distane from the line to search within
        # res is the resolution of the search at the ends. should probably be an even number between 6 and 12, but could be even higher with higher radiuses. so maybe between 3-6 times the radius
        # determine which point to start at. the slop is always between 0-pi, so the lower one should be the starter
        if self.point1.y <= self.point2.y:
            startpoint= self.point1.copy()
            endpoint = self.point2
        else:
            startpoint = self.point2.copy()
            endpoint = self.point1
        # initialize a set of points near the line
        points = set()
        startpoint.x += math.cos(self.theta)
        startpoint.y += math.sin(self.theta)
        # crawl up the line, moving the "cursor" 1 unit each iteration. 
        while startpoint.x < endpoint.x and startpoint.y < endpoint.y:
            # crawl along the line perpindicular to the given line. 
            for r in range(-radius, radius + 1):
                x = round(startpoint.x + math.cos(self.perpindicular) * r)
                y = round(startpoint.y + math.sin(self.perpindicular) * r)
                points.add((x, y))
            # increment to next point on the line, 1 unit from the previous point
            startpoint.x += math.cos(self.theta)
            startpoint.y += math.sin(self.theta)

        # get nearby points around the endpoints too
        for p in [self.point1, self.point2]:
            for a in range(res):
                t = math.pi / (res/2) * a
                for r in range(radius + 1):
                    x = round(p.x + math.cos(t) * r)
                    y = round(p.y + math.sin(t) * r)
                    points.add((x, y))

        return [Point(p) for p in points]

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
        theta = (l1.length * l1.theta + l2.length * (l2.theta - math.pi * (l2.theta / abs(l2.theta)))) / (l1.length + l2.length)
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

    


if __name__ == '__main__':
    # scrap script space to test that parts are working
    l = Line_Segment([0,0,2,2])
    print(l.nearby_points(2))
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
    """
    print("l length: ", l.point1.distance(l.point2))
    t = l.point2.angle(l.point1)
    print(t)
    l2 = Line_Segment([3, 1,4,5])

    print("lengths: ", l.length, l2.length)
    merged = merge_lines(l, l2)
    print(merged)
    """
