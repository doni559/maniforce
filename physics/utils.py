from math import sqrt, sin, cos
from pygame import draw

from settings import HEIGHT

class Vector():
    def __init__(self, x, y):
        self.x = x
        self.y = y

    def get_length(self):
        return sqrt(self.x**2 + self.y**2)

    def normalise(self):
        try:
            new_x = self.x / sqrt(self.x**2 + self.y**2)
            if self.y < 0:
                new_y = -sqrt(1-new_x**2)
            else:
                new_y = sqrt(1-new_x**2)
            return Vector(new_x, new_y)
        except ZeroDivisionError:
            return Vector(0, 0)

    def scalar_multiply(self, other_vector) -> float:
        return self.x * other_vector.x + self.y * other_vector.y
    
    def vector_multiply(self, other_vector) -> float:
        return self.x * other_vector.y - self.y * other_vector.x

    def rotate(self, angle):
        return Vector(self.x*cos(angle) - self.y*sin(angle), self.x*sin(angle) + self.y*cos(angle))


    def __add__(self, other):
        if isinstance(other, Vector):
            return Vector(self.x + other.x, self.y + other.y)
        return NotImplemented

    def __sub__(self, other):
        if isinstance(other, Vector):
            return Vector(self.x - other.x, self.y - other.y)
        return NotImplemented

    def __mul__(self, other):
        if isinstance(other, (int, float)):
            return Vector(self.x * other, self.y * other)

        return NotImplemented

    def __truediv__(self, other):
        if isinstance(other, (int, float)):
            if other == 0:
                raise ZeroDivisionError
            return Vector(self.x / other, self.y / other)
    
        return NotImplemented
    
    def __str__(self):
        return f"Vector({self.x}, {self.y})"

    def draw(self, screen, start_pos, color):
        draw.line(screen, color=color, start_pos=(start_pos.x, HEIGHT-start_pos.y), end_pos=(start_pos.x+self.x, HEIGHT-(start_pos.y+self.y)), width=4)

def clamp(x, left, right) -> float:
    return max(left, min(x, right))

