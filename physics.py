from math import sqrt, log, pi, sin, cos, radians, degrees
from math import e as euler
from typing import List, Tuple

from dataclasses import dataclass
from pygame import *
from utils import *

from settings import SUBSTEPS, GRAV_CONST, EPS


@dataclass(frozen=True)
class ObjectConfig():
    name: str

    gravity: bool
    draw_trajectory: bool
    stiffnes_cf : float
    friction_cf: float
    restitution : float
    density: float

    radius : float | None = None
    width: float | None = None
    height : float | None = None
    corners: List[Vector] | None = None


    rope_stiffness_cf : float | None = None
    rope_dampfing_cf : float | None = None
    rope_force_limit : float | None = None

    rope_exists : bool | None = None
    is_rope_breakable : bool | None = None
    pendulum_friction : bool | None = None

class Collider():
    def __init__(self, collider_type : str , center : Vector,**kwargs):
        possible_types = [
            "Box",
            "Circle",
            "Polygon"
        ]

        if not collider_type.capitalize() in possible_types:
            raise ValueError("Collider type is unknown") 
        self.type = collider_type.capitalize()
        self.center = center
        if self.type == "Circle":
            self.radius = kwargs["radius"]
        if self.type in ["Box", "Polygon"]:
            self.relative_corners = kwargs["corners"]
            world_corners = [Vector(center.x+point.x, center.y+point.y) for point in kwargs["corners"]]
            self.corners = world_corners
        
    def draw(self, screen, color, center : Tuple[int] | None = None):
        if (self.type == "Circle"):
            draw.circle(screen, color,center, radius=self.radius)
        if (self.type == "Box" or self.type == "Polygon"):
            draw.polygon(screen,color, [(point.x, HEIGHT-point.y) for point in self.corners])

    def calculate_deformation(self, another_collider, object , another_object_pos) -> Tuple[Vector, List[dict]]:
        another_collider : Collider = another_collider
        if (self.type == "Circle" and another_collider.type == "Circle"):
            collision_distance = sum([self.radius, another_collider.radius])
            distance : Vector= (another_object_pos - object.pos)
            deformation= collision_distance-distance.get_length()
            normal = distance.normalise()
            if distance.scalar_multiply(normal) >0:
                normal*= -1
            contact_points=[]
            if deformation > 0:
                contact_points.append({
                    "pos":object.pos+normal*self.radius,
                    "deformation":deformation
                })
            return normal, contact_points
        if (self.type == "Circle" and another_collider.type in ["Polygon", "Box"]):
            normals : List[Vector]=[]
            penetrations : List[float] = []
            for i in range(0, len(another_collider.corners)):
                point = another_collider.corners[i]
                if i == (len(another_collider.corners)-1):
                    next_point= another_collider.corners[0]
                else:
                    next_point= another_collider.corners[i+1]
                side= next_point-point
            
                normal = Vector(side.y, -side.x).normalise()
                normals.append(normal)
            for i in range(0, len(another_collider.corners)):
                normal= (another_collider.corners[i]-object.pos).normalise()
                normals.append(normal)
            
            #projecting objects on normals and writing intervals
            for normal in normals:
                projections : List[Vector]=[]
                center= self.center.scalar_multiply(normal)
                t_interval=[
                    center-self.radius,
                    center+self.radius
                ]
                projections.clear()
                for point in another_collider.corners:
                    projected_point=point.scalar_multiply(normal)
                    projections.append(projected_point)
                a_interval=[
                    min(projections),
                    max(projections)
                ]
                penetration=min(a_interval[1]-t_interval[0], t_interval[1]-a_interval[0])
                #check if projections are overlapping
                if penetration < 0:
                    #then no collision, return normal=0 (no need), deformation=0
                    return Vector(0,0), [] 
                penetrations.append(penetration)  
            deformation=min(penetrations)
            index=penetrations.index(deformation)
            normal = Vector(normals[index].x, normals[index].y)
            contact_points=[]
            distance : Vector = another_object_pos - object.pos
            if distance.scalar_multiply(normal) > 0:
                normal*=-1
            if deformation > 0:
                contact_points.append(
                    {
                        "pos":object.pos-normal*self.radius,
                        "deformation":deformation
                    }
                )
            return normal, contact_points
        if (self.type in ["Box", "Polygon"] and another_collider.type == "Circle"):
            normals : List[Vector]=[]
            penetrations : List[float] = []
            for i in range(0, len(self.corners)):
                point = self.corners[i]
                if i == (len(self.corners)-1):
                    next_point= self.corners[0]
                else:
                    next_point= self.corners[i+1]
                side= next_point-point
        
                normal = Vector(side.y, -side.x).normalise()
                normals.append(normal)
            for i in range(0, len(self.corners)):
                normal= (self.corners[i]-another_object_pos).normalise()
                normals.append(normal)
            #projecting objects on normals and writing intervals
            for normal in normals:
                projections : List[Vector]=[]

                center= another_collider.center.scalar_multiply(normal)
                t_interval=[
                    center-another_collider.radius,
                    center+another_collider.radius
                ]
                projections.clear()
                for point in self.corners:
                    projected_point=point.scalar_multiply(normal)
                    projections.append(projected_point)
                a_interval=[
                    min(projections),
                    max(projections)
                ]
                penetration=min(a_interval[1]-t_interval[0], t_interval[1]-a_interval[0])
                #check if projections are overlapping
                if penetration < 0:
                    #then no collision, return normal=0 (no need), deformation=0
                    return Vector(0,0), [] 
                penetrations.append(penetration)  
            deformation=min(penetrations)
            index=penetrations.index(deformation)

            normal = Vector(normals[index].x, normals[index].y)
            contact_points=[]
            distance : Vector = another_object_pos- object.pos
            if distance.scalar_multiply(normal) > 0:
                normal*=-1
            if deformation > 0:
                contact_points.append(
                    {
                        "pos":another_object_pos-normal*another_collider.radius,
                        "deformation":deformation
                    }
                )       
            return normal, contact_points

        if (self.type in ["Box", "Polygon"] and another_collider.type in ["Box", "Polygon"]):
            #SAT
            #finding normals
            normals : List[Vector]=[]
            penetrations : List[float] = []
            for i in range(0, len(another_collider.corners)):
                point = another_collider.corners[i]
                if i == (len(another_collider.corners)-1):
                    next_point= another_collider.corners[0]
                else:
                    next_point= another_collider.corners[i+1]
                side= next_point-point

                normal = Vector(side.y, -side.x).normalise()
                normals.append(normal)

            for i in range(0, len(self.corners)):
                point = self.corners[i]
                if i == (len(self.corners)-1):
                    next_point= self.corners[0]
                else:
                    next_point= self.corners[i+1]

                side=  next_point-point
                normal = Vector(side.y, -side.x).normalise()
                normals.append(normal)
            
            #projecting objects on normals and writing intervals
            for normal in normals:
                projections : List[Vector]=[]
                for point in self.corners:
                    projected_point=point.scalar_multiply(normal)
                    projections.append(projected_point)
                t_interval=[
                    min(projections),
                    max(projections)
                ]
                projections.clear()
                for point in another_collider.corners:
                    projected_point=point.scalar_multiply(normal)
                    projections.append(projected_point)
                a_interval=[
                    min(projections),
                    max(projections)
                ]
                penetration=min(a_interval[1]-t_interval[0], t_interval[1]-a_interval[0])
                #check if projections are overlapping
                if penetration < 0:
                    #then no collision, return normal=0 (no need), deformation=0
                    return Vector(0,0), [] 
                penetrations.append(penetration)  
            deformation=min(penetrations)
            
            index=penetrations.index(deformation)
            normal = Vector(normals[index].x, normals[index].y)
            
            distance : Vector = another_object_pos - object.pos
            if index in range(0 , len(another_collider.corners)):
                reference_index = index
                reference = another_collider
                incident = self
                
                max_scalar_normal=float("-inf")
                if distance.scalar_multiply(normal) > 0:
                    normal *= -1

                for i in range(0, len(reference.corners)):
                    
                    if normals[i].scalar_multiply(normal) > max_scalar_normal:
                        max_scalar_normal = normals[i].scalar_multiply(normal)
                        reference_index=i
                
            else:
                reference_index=index-len(another_collider.corners)
                reference = self
                incident = another_collider
                
                max_scalar_normal=float("-inf")
                if distance.scalar_multiply(normal) < 0:
                    normal *= -1
                
                for i in range(len(another_collider.corners), len(another_collider.corners)+len(self.corners)):
                    if normals[i].scalar_multiply(normal) > max_scalar_normal:
                        max_scalar_normal = normals[i].scalar_multiply(normal)
                        reference_index=i-len(another_collider.corners)
    
            reference_p0=reference.corners[reference_index]
            reference_p1=reference.corners[(reference_index + 1) % len(reference.corners)]

            min_scalar_normal = float("inf")
            incident_index = 0
            if reference is self:
                for k in range(0, len(another_collider.corners)):
                    n1=normals[k]
                    scalar=n1.scalar_multiply(normal)
                    if scalar < min_scalar_normal:
                        min_scalar_normal = scalar
                        incident_index= k
            else:
                for k in range(0, len(self.corners)):
                    n1=normals[len(another_collider.corners)+k]
                    scalar=n1.scalar_multiply(normal)
                    if scalar < min_scalar_normal:
                        min_scalar_normal = scalar
                        incident_index= k
            incident_p0=incident.corners[incident_index]
            incident_p1=incident.corners[(incident_index + 1) % len(incident.corners)]

            tangent_L=(reference_p1-reference_p0).get_length()
            tangent : Vector = (reference_p1-reference_p0).normalise()

            s0=(incident_p0-reference_p0).scalar_multiply(tangent)
            s1=(incident_p1-reference_p0).scalar_multiply(tangent)

            if abs(s1 - s0) < EPS:
                clipped_p0, clipped_p1 = incident_p0, incident_p1
            else:
                left_border = -s0/(s1-s0)
                right_border = (tangent_L-s0)/(s1-s0)
                if (s0 < 0 and s1 < 0) or (s0 > tangent_L and s1 > tangent_L):
                    return Vector(0,0), []
                if s0 < 0:
                    clipped_p0= incident_p0+(incident_p1-incident_p0)*left_border
                elif s0 > tangent_L:
                    clipped_p0= incident_p0+(incident_p1-incident_p0)*right_border
                else:
                    clipped_p0=incident_p0
                if s1 <0:
                    clipped_p1 = incident_p0 +(incident_p1-incident_p0) *  left_border
                elif s1 > tangent_L:
                    clipped_p1 = incident_p0 +(incident_p1-incident_p0) *  right_border

                else:
                    clipped_p1 = incident_p1

            deformation_p0 = -(clipped_p0-reference_p0).scalar_multiply(normal)
            deformation_p1 = -(clipped_p1-reference_p0).scalar_multiply(normal)
            contact_points =[]
            if deformation_p0 >= 0:
                contact_points.append({
                    "pos": clipped_p0,
                    "deformation":deformation_p0
                })
            if deformation_p1 >= 0:
                
                contact_points.append({
                    "pos": clipped_p1,
                    "deformation":deformation_p1
                })
            if distance.scalar_multiply(normal) > 0:
                normal *= -1
            return normal, contact_points

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

        self.torques = []
        self.forces = []
        self.frame=1

        self.stiffness_cf = config.stiffnes_cf
        self.friction_cf=config.friction_cf
        self.restitution=config.restitution

        #Other things
        self.trajectory_arr = [Vector(self.pos.x, self.pos.y)]
        self.colliding_force = Vector(0,0)
        self.colliding_torque = 0
    
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
                point = config.corners[index]
                next_point = config.corners[(index+1) % len(config.corners)]
    
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
            self.pos += center_mass /self.mass
            #steiner theorem
            self.moment_of_inertia=total_moment_of_inertia - self.mass* center_mass.get_length()**2
            self.collider = Collider(collider_type,
                        center=self.pos,
                        corners = config.corners)
            origin_offset=start_pos-self.pos
            self.origin_offset_local = origin_offset.rotate(-self.angle)

    def calc_position(self, dt):
        self.acceleration = self.resultant_force *(1/ self.mass)
        self.velocity += self.acceleration * dt   
        self.pos += self.velocity*dt

        self.angular_acceleration =self.resultant_torque/self.moment_of_inertia
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
    
    def calc_forces(self):
        self.forces.clear()
        self.torques.clear()
        if self.gravity:
            gravity_force = Vector(0, -GRAV_CONST * self.mass)
            self.forces.append(gravity_force)
        self.forces.append(self.colliding_force)
        self.colliding_force=Vector(0,0)
        self.torques.append(self.colliding_torque)
        # resultant_force = Vector(0, 0)
        # for force in self.forces:
        #     resultant_force.x += force.x
        #     resultant_force.y += force.y
        # self.resultant_force = resultant_force
    
        # resultant_torque = sum(self.torques)
        # self.resultant_torque = resultant_torque

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
            Vector(-self.width/2, -self.height/2),
            Vector(+self.width/2, -self.height/2),
            Vector(+self.width/2, +self.height/2),
            Vector(-self.width/2, +self.height/2),
        ])

        self.fill=True
        self.color = (0, 0, 0)

class CollisionCalculator():
    def __init__(self, all_sprites : sprite.Group, all_obstacles: List[Obstacle]):
        self.colliders = all_obstacles
        self.sprites= all_sprites.sprites()

    def calculate_collisions_penalty(self):
        sprites: List[PhysicalObject]=self.sprites
        resulting_forces={}
        resulting_torques={}

        for i in range(0, len(sprites)):
            target_sprite : PhysicalObject=sprites[i]
            try:
                resulting_forces[i]+=Vector(0,0)
                resulting_torques[i]+=0
            except:
                resulting_forces[i]=Vector(0,0)
                resulting_torques[i]=0

            for j in range(i+1, len(sprites)):
                try:
                    resulting_forces[j]+=Vector(0,0)
                    resulting_torques[j]+=0
                except:
                    resulting_forces[j]=Vector(0,0)
                    resulting_torques[j]=0
                another_sprite : PhysicalObject =sprites[j]
                
                normal, contact_points = target_sprite.collider.calculate_deformation(another_sprite.collider, target_sprite, another_sprite.pos)

                for contact_point in contact_points:
                    position=contact_point["pos"]
                    target_point_leverarm : Vector=position-target_sprite.pos
                    another_point_leverarm : Vector=position-another_sprite.pos
                    deformation=contact_point["deformation"]
                    
                    #collision_force calc
                    stiffnes_cf1=target_sprite.stiffness_cf
                    stiffnes_cf2=another_sprite.stiffness_cf
                    summary_stiffness_cf = (stiffnes_cf1*stiffnes_cf2)/(stiffnes_cf1+stiffnes_cf2)
                    spring_force= normal * summary_stiffness_cf * deformation

                    mass_efficient = (target_sprite.mass * another_sprite.mass)/(target_sprite.mass+another_sprite.mass)
                    mixed_restitution= sqrt(target_sprite.restitution * another_sprite.restitution)
                    damping_ratio = - log(mixed_restitution, euler)/ (sqrt(pi**2 + (log(mixed_restitution, euler))**2 ))
                    damping_cf= 2*damping_ratio*sqrt(summary_stiffness_cf*mass_efficient)

                    target_rotational_velocity = Vector(
                                            -target_sprite.angular_velocity * target_point_leverarm.y,
                                            target_sprite.angular_velocity * target_point_leverarm.x
                                        )
                    target_dot_contact_velocity = target_sprite.velocity+target_rotational_velocity
                    
                    another_rotational_velocity=Vector(
                                            -another_sprite.angular_velocity * another_point_leverarm.y,
                                            another_sprite.angular_velocity * another_point_leverarm.x
                                        )
                    another_dot_contact_velocity= another_sprite.velocity+another_rotational_velocity

                    relative_velocity=target_dot_contact_velocity-another_dot_contact_velocity
                    radial_velocity : Vector = normal * (relative_velocity.scalar_multiply(normal))

                    damping_force = radial_velocity.normalise() *(-1) * damping_cf*radial_velocity.get_length()
                    contact_force= spring_force+damping_force

                    #friction_force calc
                    friction_force = relative_velocity.normalise() * target_sprite.friction_cf * contact_force.get_length() *(-1)
                    collision_force = contact_force+friction_force
                    collision_torque_target = target_point_leverarm.vector_multiply(collision_force)
                    collision_torque_another = another_point_leverarm.vector_multiply(collision_force)

                    resulting_forces[i]+=collision_force
                    resulting_forces[j]+=collision_force*(-1)
                    resulting_torques[i]+=collision_torque_target
                    resulting_torques[j]+=collision_torque_another*(-1)


            for k in range(0, len(self.colliders)):
                collider = self.colliders[k]
                normal, contact_points = target_sprite.collider.calculate_deformation(collider.collider, target_sprite, collider.center)
                for contact_point in contact_points:
                    position=contact_point["pos"]
                    deformation=contact_point["deformation"]
                    torque_leverarm : Vector= position-target_sprite.pos
                
                    spring_force : Vector= normal * target_sprite.stiffness_cf * abs(deformation)
                    dot_rotational_velocity = Vector(
                                            -target_sprite.angular_velocity * torque_leverarm.y,
                                            target_sprite.angular_velocity * torque_leverarm.x
                                        )
                    contact_velocity = target_sprite.velocity+dot_rotational_velocity
                    radial_velocity : Vector = normal * (contact_velocity.scalar_multiply(normal))

                    damping_ratio = - log(target_sprite.restitution, euler)/ (sqrt(pi**2 + (log(target_sprite.restitution, euler))**2 ))
                    damping_cf= 2*damping_ratio*sqrt(target_sprite.stiffness_cf*target_sprite.mass)
                    damping_force : Vector = radial_velocity.normalise()*(-1) * damping_cf * radial_velocity.get_length()

                    contact_force=spring_force+damping_force
                    
                    #friction force
                    friction_force = contact_velocity.normalise()*(-1) * target_sprite.friction_cf * contact_force.get_length()

                    collision_force=contact_force+friction_force
                    collision_torque = torque_leverarm.vector_multiply(collision_force)


                    resulting_forces[i]+=collision_force
                    resulting_torques[i]+=collision_torque

        for key, value in resulting_forces.items():
            sprites[key].colliding_force=value
        for key, value in resulting_torques.items():
            sprites[key].colliding_torque=value