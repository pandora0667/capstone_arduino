
from math import sqrt, pi, sin, cos, tan, atan, atan2

## 3차원 벡터 클래스. 딱 필요한 기능만 구현.
## x is longitugal(front/back), y is latheral(left/right), z is vertical(up/down)
class Vector3(object) :
    def __init__(self, x, y, z) :
        self.x = x
        self.y = y
        self.z = z
    def size(self) :
        return sqrt(self.x**2 + self.y**2 + self.z**2)
    
    def size_horizental(self) :
        return sqrt(self.x**2 + self.y**2)
    
    def __str__(self) :
        return "x : " + str(self.x) + ", y : " + str(self.y) + ", z : " + str(self.z)
    
    def __repr__(self) :
        return str(self)