from math import pi, sqrt, sin, cos
from pygame import draw

from typing import Tuple, List

from configs.settings import HEIGHT
from abc import ABC, abstractmethod
from dataclasses import dataclass

class Vector():
    def __init__(self, x, y):
        self.x = x
        self.y = y

    def get_length(self):
        return sqrt(self.x**2 + self.y**2)

    def normalise(self):
        L=self.get_length()
        if L == 0:
            return Vector(0, 0)
        return Vector(self.x/L, self.y/L)

    def scalar_multiply(self, other_vector) -> float:
        return self.x * other_vector.x + self.y * other_vector.y
    
    def vector_multiply(self, other_vector) -> float:
        return self.x * other_vector.y - self.y * other_vector.x

    def rotate(self, angle):
        return Vector(self.x*cos(angle) - self.y*sin(angle), self.x*sin(angle) + self.y*cos(angle))

    def as_tuple(self) -> Tuple[float, float]:
        return (self.x, self.y)

    def convert_to_screen_cords(self, camera_pos: List[float], camera_zoom:float):
        result_vector=Vector(self.x-camera_pos[0], HEIGHT+camera_pos[1]-self.y)
        result_vector*=camera_zoom
        return result_vector.as_tuple()

    def draw(self, screen, start_pos, color, camera_pos: List[float], camera_zoom:float, width = 4):
        start_pos : Vector = start_pos
        end_pos : Vector = (start_pos+self).convert_to_screen_cords(camera_pos, camera_zoom)
        start_pos = start_pos.convert_to_screen_cords(camera_pos, camera_zoom)

        draw.line(screen, color=color, start_pos=start_pos, end_pos=end_pos, width=int(width*camera_zoom))

    def __repr__(self):
        return self.__str__()

    def __add__(self, other):
        if isinstance(other, Vector):
            return Vector(self.x + other.x, self.y + other.y)
        return NotImplemented

    def __radd__(self, other):
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
        if isinstance(other, Vector):
            return self.scalar_multiply(other)
        return NotImplemented
    def __rmul__(self, other):
        if isinstance(other, (int, float)):
            return Vector(self.x*other, self.y*other)
        if isinstance(other, Vector):
            return self.scalar_multiply(other)
        return NotImplemented

    def __truediv__(self, other):
        if isinstance(other, (int, float)):
            if other == 0:
                raise ZeroDivisionError
            return Vector(self.x / other, self.y / other)
    
        return NotImplemented

    def __neg__(self):
        return Vector(-self.x, -self.y)

    def __eq__(self, value):
        if not isinstance(value, Vector):
            return NotImplemented
        value : Vector = value
        if self.x == value.x and self.y == value.y:
            return True
        return False

    

    
    def __str__(self):
        return f"Vector({self.x:.4f}, {self.y:.4f})"

    
def clamp(x, left, right) -> float:
    return max(left, min(x, right))

@dataclass
class PhysicalObjectState():
    pos: Vector
    velocity: Vector
    resultant_force: Vector
    acceleration: Vector
    mass: float
    moment_of_inertia: float

    resultant_torque: float
    angle: float
    angular_velocity: float
    angular_acceleration: float

@dataclass
class JointNodeState():
    pos: Vector
    velocity: Vector
    resultant_force: Vector
    acceleration: Vector
    mass: float

class Integrator(ABC):
    def __init__(self, pre_integrations: int):
        self.pre_integrations = pre_integrations

    @abstractmethod
    def integrate(self, state: PhysicalObjectState, prev_acceleration: Vector|None, prev_angular_acceleration: float | None,  dt: float):...

    @abstractmethod
    def pre_integrate(self, state: PhysicalObjectState | JointNodeState, dt: float):...

    def __str__(self):
        return "Integrator"


class EulerIntegrator(Integrator):
    def __init__(self):
        super().__init__(0)
    def pre_integrate(self, state, dt):
        return None

    def integrate(self, state: PhysicalObjectState | JointNodeState,prev_acceleration: Vector|None, prev_angular_acceleration: float | None, dt: float):
        state.acceleration = state.resultant_force / state.mass

        state.velocity += state.acceleration * dt
        state.pos += state.velocity * dt

        if isinstance(state, PhysicalObjectState):
            state.angular_acceleration = state.resultant_torque / state.moment_of_inertia
            state.angular_velocity += state.angular_acceleration * dt
            state.angle += state.angular_velocity * dt

            #check if angle is out of bounds
            if state.angle // (2*pi) !=0 and state.angle != (2*pi) :
                state.angle = state.angle - (state.angle//(2*pi)) *2*pi
        return state
    
    def __str__(self):
        return "EulerIntegrator"

class VerletIntegrator(Integrator):
    def __init__(self):
        super().__init__(1)

    def pre_integrate(self, state: PhysicalObjectState | JointNodeState, dt: float):
        state.acceleration = state.resultant_force / state.mass
        state.pos += state.velocity * dt + 0.5 * state.acceleration * dt**2

        if isinstance(state, PhysicalObjectState):
            state.angular_acceleration = state.resultant_torque / state.moment_of_inertia
            state.angle += state.angular_velocity * dt + 0.5 * state.angular_acceleration * dt**2
        return state
    def integrate(self, state: PhysicalObjectState | JointNodeState, prev_acceleration: Vector, prev_angular_acceleration: float | None, dt: float):
        state.acceleration = state.resultant_force / state.mass
        state.velocity+=1/2*(prev_acceleration + state.acceleration)*dt
        if isinstance(state, PhysicalObjectState):
            state.angular_acceleration = state.resultant_torque / state.moment_of_inertia
            state.angular_velocity+=1/2*(prev_angular_acceleration + state.angular_acceleration)*dt

            #check if angle is out of bounds
            if state.angle // (2*pi) !=0 and state.angle != (2*pi) :
                state.angle = state.angle - (state.angle//(2*pi)) *2*pi
        return state

    def __str__(self):
        return "VerletIntegrator"

    def __repr__(self):
        return self.__str__()