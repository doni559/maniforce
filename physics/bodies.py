from pygame import sprite

from dataclasses import dataclass
from typing import List, Tuple
from math import radians, pi

from .collider import Collider

from .utils import Vector
from settings import SUBSTEPS, GRAV_CONST

@dataclass(frozen=True)
class ObjectConfig():
    gravity: bool
    draw_trajectory: bool
    stiffnes_cf : float
    friction_cf: float
    restitution : float
    density: float

    radius : float | None = None
    width: float | None = None
    height : float | None = None
    corners: List[Tuple[float]] | None = None


    rope_stiffness_cf : float | None = None
    rope_dampfing_cf : float | None = None
    rope_force_limit : float | None = None

    rope_exists : bool | None = None
    is_rope_breakable : bool | None = None
    pendulum_friction : bool | None = None



class PhysicalObject(sprite.Sprite):
    def __init__(self, config: ObjectConfig, start_pos = Vector(0,0), start_velocity= Vector(0,0), start_angle=0, start_angular_velocity=0, collider_type="Circle" , name="Object0"):
        sprite.Sprite.__init__(self)
        self.type="PhysicalObject"
        self.name = name
        self.config= config
        
        self.velocity = start_velocity
        self.acceleration = Vector(0,0)
        self.pos = start_pos

        self.angle = radians(start_angle)
        self.angular_velocity=radians(start_angular_velocity)
        self.angular_acceleration=0

        self.resultant_force = Vector(0,0)
        self.resultant_torque = 0

        self.stiffness_cf = config.stiffnes_cf
        self.friction_cf=config.friction_cf
        self.restitution=config.restitution

        #Other things
        self.trajectory_arr = [Vector(self.pos.x, self.pos.y)]
    
        #Constants
        self.density =config.density
        self.calibrating_length=10**-6
                
        ##System Flags
        self.gravity=config.gravity
        self.draw_trajectory=config.draw_trajectory

        if collider_type == "Circle":
            self.collider = Collider(collider_type,center=start_pos, radius=config.radius)
            self.radius = config.radius
            self.volume = self.radius**3 * pi * 4/3 /4187666
            self.mass=self.density*self.volume
            self.moment_of_inertia=1/2*self.mass*self.radius**2
    
        if collider_type in ("Box", "Polygon"):
            total_volume = 0
            total_moment_of_inertia=0
            center_mass=Vector(0,0)
            for index in range(0, len(config.corners)):
                corners = [Vector(corner[0], corner[1]) for corner in config.corners]
                point = corners[index]

                next_point = corners[(index+1) % len(corners)]
    
                sector_volume = abs(point.vector_multiply(next_point))/2 * self.calibrating_length
                total_volume+= sector_volume
    
                sector_center = (next_point+point) * (1/3)
                sector_mass = self.density*sector_volume
                total_moment_of_inertia += sector_mass / 6 * (
                    point.x**2
                    + point.y**2
                    + point.x * next_point.x
                    + point.y * next_point.y
                    + next_point.x**2
                    + next_point.y**2
                )
                center_mass+= sector_center* sector_mass
            
            self.volume = total_volume
            self.mass = self.volume*self.density

            center_mass /= self.mass
            self.pos += center_mass.rotate(self.angle)

            #steiner theorem
            self.moment_of_inertia=total_moment_of_inertia - self.mass* center_mass.get_length()**2
            self.collider = Collider(collider_type,
                        center=self.pos,
                        corners = config.corners)
            origin_offset=start_pos-self.pos
            self.origin_offset_local = origin_offset.rotate(-self.angle)

    def calc_position(self, dt):
        self.acceleration = self.resultant_force / self.mass
        self.velocity += self.acceleration * dt   
        self.pos += self.velocity*dt

        self.angular_acceleration =self.resultant_torque / self.moment_of_inertia
        self.angular_velocity += self.angular_acceleration *dt
        self.angle += self.angular_velocity* dt

        #check if angle is out of bounds
        if self.angle // (2*pi) !=0 and self.angle != (2*pi) :
            self.angle = self.angle - (self.angle//(2*pi)) *2*pi

        if (self.collider.type in ("Box", "Polygon")):
            #changing coordinates from origin to center of mass
            offset = self.origin_offset_local
            relative_coords : List[Vector] = [coord + offset for coord in self.collider.relative_corners ]
            #rotating on angle
            rotated_coords = [coord.rotate(self.angle) for coord in relative_coords]
            #changing back to origin
            new_corners= [ 
                self.pos + rotated_coords[i] for i in range(0, len(rotated_coords))
            ]
            self.collider.corners=new_corners
        self.collider.center=self.pos 

        self.resultant_force=Vector(0,0)
        self.resultant_torque=0
    
    def calc_forces(self):
        if self.gravity:
            gravity_force = Vector(0, -GRAV_CONST * self.mass)
            self.resultant_force+=gravity_force

    def update(self, dt):
        self.calc_forces()
        self.calc_position(dt/SUBSTEPS)


        
#WILL BE REMOVED
class Pendulum(PhysicalObject):
    def __init__(self, config : ObjectConfig, start_pos = Vector(0,0), start_velocity= Vector(0,0), start_angle =0,collider_type="Circle", suspension_point = Vector(0,0), name= "Pendulum0"):
        super().__init__(config, start_pos, start_velocity, start_angle,collider_type, name)
        self.type = "Pendulum"
        #Other things
        self.elongation=0

        #Constants
        self.rope_stiffness_cf= config.rope_stiffness_cf
        self.rope_dampfing_cf = config.rope_dampfing_cf
        self.rope_force_limit=config.rope_force_limit
        self.rope_length=150
        self.suspension_point=Vector(start_pos.x, start_pos.y+self.rope_length)


        ##System Flags
        self.rope_exists= config.rope_exists
        self.is_rope_breakable=config.is_rope_breakable
        self.pendulum_friction=config.pendulum_friction

    def calc_forces(self):
        super().calc_forces()
        radius_vector = Vector(self.pos.x-self.suspension_point.x, self.pos.y-self.suspension_point.y)
        radius = radius_vector.get_length()
        if self.rope_exists:
            self.elongation=self.rope_length-radius

            rope_friction_force = Vector(0,0)
            rope_stiffness_force : Vector = radius_vector.normalise() * self.rope_stiffness_cf * self.elongation
            try:
                radial_velocity = radius_vector.scalar_multiply(self.velocity)/radius
            except ZeroDivisionError:
                radial_velocity =0
            if self.pendulum_friction:
                rope_friction_force : Vector = radius_vector.normalise() * self.rope_dampfing_cf * radial_velocity * -1
            rope_force = rope_stiffness_force + rope_friction_force
            
            if rope_force.scalar_multiply(radius_vector) > 0 or radius < self.rope_length:
                rope_force*=0
            if rope_force.get_length() > self.rope_force_limit and self.is_rope_breakable: 
                self.rope_exists = False
            self.forces.append(rope_force)   
    def update(self, dt):
        super().update(dt)





class Obstacle():
    def __init__(self, x0,x1, y0,y1):
        #x0,x1 - corners' xs, y0,y1 - corners' ys
        self.x0=x0
        self.x1=x1
        self.y0=y0
        self.y1=y1
        self.center = Vector(x0+(x1-x0)/2, y0+(y1-y0)/2)
        self.width=x1-x0
        self.height = y1-y0

        self.collider= Collider("Polygon", center= self.center, corners =[
            (-self.width/2, -self.height/2),
            (+self.width/2, -self.height/2),
            (+self.width/2, +self.height/2),
            (-self.width/2, +self.height/2),
        ])

        self.fill=True
        self.color = (0, 0, 0)