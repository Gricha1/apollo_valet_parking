kMathEpsilon = 1e-6

class Vec2d:
    def __init__(self, x, y):
        if abs(x) < kMathEpsilon:
            x = 0
        if abs(y) < kMathEpsilon:
            y = 0 
        self.x = x
        self.y = y
        self.length = (x**2 + y**2)**(1/2)
        
    @classmethod
    def fromVec(cls, vec):
        return cls(vec.x, vec.y)
    
    def innerProd(self, other):
        res = self.x * other.x + self.y * other.y
        if abs(res) < kMathEpsilon:
            res = 0
        return res
    
    def outProd(self, other):
        res = self.x * other.y - self.y * other.x
        if abs(res) < kMathEpsilon:
            res = 0
        return res
    
    def __add__(self, other):
        return Vec2d(self.x + other.x, self.y + other.y)
        
    def __neg__(self):
        return Vec2d(-self.x, -self.y)
    
    def __sub__(self, other):
        return self + (-other)
    
    def __str__(self):
        return f'({self.x}, {self.y})'
    
    def __repr__(self):
        return f'Vec2d({self.x}, {self.y})'
    
    def __mul__(self, num):
        return Vec2d(num * self.x, num * self.y)
    
    def __truediv__(self, num):
        if num != 0:
            return Vec2d(self.x / num, self.y / num)
    
    def __rmul__(self, num):
        return Vec2d.fromVec(self) * num
    
    def normalize(self):
        return Vec2d.fromVec(self) / self.length