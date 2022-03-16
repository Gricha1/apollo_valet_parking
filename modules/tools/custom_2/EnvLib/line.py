from modules.tools.custom_2.EnvLib.Vec2d import Vec2d
from math import cos, sin, tan

class Point:
    def __init__(self, x, y):
        self.x = x
        self.y = y
 
# Given three collinear points p, q, r, the function checks if
# point q lies on line segment 'pr'
def onSegment(p, q, r):
    if ( (q.x <= max(p.x, r.x)) and (q.x >= min(p.x, r.x)) and
           (q.y <= max(p.y, r.y)) and (q.y >= min(p.y, r.y))):
        return True
    return False
 
def orientation(p, q, r):
    # to find the orientation of an ordered triplet (p,q,r)
    # function returns the following values:
    # 0 : Collinear points
    # 1 : Clockwise points
    # 2 : Counterclockwise
     
    # See https://www.geeksforgeeks.org/orientation-3-ordered-points/amp/
    # for details of below formula.
     
    val = ((q.y - p.y) * (r.x - q.x)) - ((q.x - p.x) * (r.y - q.y))
    if (val > 0): 
        # Clockwise orientation
        return 1
    elif (val < 0):
        # Counterclockwise orientation
        return 2
    else:
        # Collinear orientation
        return 0
 
# The main function that returns true if
# the line segment 'p1q1' and 'p2q2' intersect.
def doIntersect(p1,q1,p2,q2):
     
    # Find the 4 orientations required for
    # the general and special cases
    o1 = orientation(p1, q1, p2)
    o2 = orientation(p1, q1, q2)
    o3 = orientation(p2, q2, p1)
    o4 = orientation(p2, q2, q1)
 
    # General case
    if ((o1 != o2) and (o3 != o4)):
        return True
 
    # Special Cases
 
    # p1 , q1 and p2 are collinear and p2 lies on segment p1q1
    if ((o1 == 0) and onSegment(p1, p2, q1)):
        return True
 
    # p1 , q1 and q2 are collinear and q2 lies on segment p1q1
    if ((o2 == 0) and onSegment(p1, q2, q1)):
        return True
 
    # p2 , q2 and p1 are collinear and p1 lies on segment p2q2
    if ((o3 == 0) and onSegment(p2, p1, q2)):
        return True
 
    # p2 , q2 and q1 are collinear and q1 lies on segment p2q2
    if ((o4 == 0) and onSegment(p2, q1, q2)):
        return True
 
    # If none of the cases
    return False

class Line:
    def __init__(self, start, end):
        self.start = start
        self.end = end
        delta_x = end.x - start.x
        delta_y = end.y - start.y
        
        if delta_x == 0:
            b = 0
            a = 1
        elif delta_y == 0:
            a = 0
            b = 1
        else:
            b = 1
            a = -delta_y / delta_x
        c = - a * start.x - b * start.y
        
        self.a = a
        self.b = b
        self.c = c
        
    def isIntersect(self, other):
        det = self.a * other.b - self.b * other.a
        if det == 0:
            #exception
            return Point(other.start.x, other.start.y)
        
        det1 = -self.c * other.b + self.b * other.c
        det2 = -self.a * other.c + self.c * other.a
        x = det1 / det
        y = det2 / det

        return Point(x, y)
    
    # def getProj(self, point):
    #     normal = Line(point, point + self.getNorm())
    #     return normal.isIntersect(self)
    
    # def getNorm(self):
    #     return Vec2d(self.a, self.b)

def rotate(vector, angle):
    x = cos(angle)*vector.x - sin(angle)*vector.y
    y = sin(angle)*vector.x + cos(angle)*vector.y
    return Vec2d(x, y)
    

class Ray(Line):
    def __init__(self, point, heading):
        super().__init__(point, point + heading)
        self.begin = point
        self.heading = heading.normalize()
        
    @classmethod
    def fromPhi(cls, x, y, theta):
        return Ray(Vec2d(x,y), Vec2d(cos(theta), sin(theta)))
        
    def rotate(self, angle):
        return Ray(self.begin, rotate(self.heading, angle))
    
    def isIntersect(self, segment):
        det = self.a * segment.b - self.b * segment.a
        if det == 0:
            return float('inf'), None
        
        det1 = -self.c * segment.b + self.b * segment.c
        det2 = -self.a * segment.c + self.c * segment.a
        x = det1 / det
        y = det2 / det
        point = Vec2d(x,y)
        
        if self.heading.innerProd(point - self.begin) < 0 or not segment.hasIn(point):
            return float('inf'), None
        return (point - self.begin).length, point
    
    def __str__(self):
        return f'Ray(Point:{self.begin}, heading: {self.heading})'
    
    def __repr__(self):
        return str(self)
           
        
class Segment(Ray):
    def __init__(self, start, end):
        super().__init__(start, end - start)
        self.start = start
        self.end = end
        self.length = (end - start).length
        
    def hasIn(self, point):
        vec = point - self.start
        proj = vec.innerProd(self.heading)
        prod = vec.outProd(self.heading)
        return prod == 0 and 0 <= proj <= self.length
    
    def __str__(self):
        return f'Segment(start:{self.start}, end: {self.end})'
    
    def __repr__(self):
        return str(self)
    
    def rotate(self, rotation_point, angle):
        return Segment(rotate(self.begin - rotation_point, angle) + rotation_point, 
                       rotate(self.end - rotation_point, angle) + rotation_point)
    
    def isIntersect(self, other):
        det = (self.begin.x - self.end.x)*(other.begin.y - other.end.y) - (self.begin.y - self.end.y)*(other.begin.x - other.end.x)
        det1 = (self.begin.x - self.end.x)*(other.begin.y - self.end.y) - (self.begin.y - self.end.y)*(other.begin.x - self.end.x)
        det2 = (other.begin.x - self.end.x)*(other.begin.y - other.end.y) - (other.begin.y - self.end.y)*(other.begin.x - other.end.x)
        
        if det == 0:
            return False
        
        t = det1 / det
        s = det2 / det
        if 0 <= t <= 1 and 0 <= s <= 1:
            return True
        return False
    
    def distance(self, other):
        if self.isIntersect(other):
            return 0
        
        min_dist = float('inf')
        
        for point in (other.begin, other.end):
            proj = self.getProj(point)
            if self.hasIn(proj):
                min_dist = min(min_dist, (point-proj).length)
                
        for point in (self.begin, self.end):
            proj = other.getProj(point)
            if other.hasIn(proj):
                min_dist = min(min_dist, (point-proj).length)
        
        
        min_dist = min(min_dist, (self.begin-other.begin).length)
        min_dist = min(min_dist, (self.begin-other.end).length)        
        min_dist = min(min_dist, (self.end-other.begin).length)        
        min_dist = min(min_dist, (self.end-other.end).length)
        return min_dist