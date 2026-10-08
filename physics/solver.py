from typing import List
from math import sqrt, log, pi

from .bodies import PhysicalObject, Obstacle
from .connections import Joint, JointNode

from .utils import Vector
from configs.settings import EPS

class CollisionCalculator():
    def __init__(self, all_sprites : List[PhysicalObject], all_obstacles: List[Obstacle], all_joints: List[Joint]):
        self.obstacles= all_obstacles
        self.sprites= all_sprites
        self.joints = all_joints

    def calculate_collision_forces_penalty(self, stiffnes_cf: float, normal: Vector, deformation : float, damping_cf: float, radial_velocity: Vector, tangential_velocity: Vector, friction_cf:float):
        spring_force= normal * stiffnes_cf * deformation
        damping_force = radial_velocity.normalise() *(-1) * damping_cf*radial_velocity.get_length()
        contact_force= spring_force+damping_force
        
        friction_force = tangential_velocity.normalise() *(-1) * friction_cf * contact_force.get_length()
        collision_force = contact_force+friction_force

        return collision_force

    def calculate_collision_objects_penalty(self, target: PhysicalObject, another: PhysicalObject):
        normal, contact_points = target.collider.calculate_deformation(another.collider, target.pos, another.pos)
        if normal != Vector(0,0):
           for contact_point in contact_points:
               position=contact_point["pos"]
               target_point_leverarm : Vector=position-target.pos
               another_point_leverarm : Vector=position-another.pos
               deformation=contact_point["deformation"]
               
               #collision_force calc
               stiffnes_cf1=target.stiffness_cf
               stiffnes_cf2=another.stiffness_cf
               summary_stiffness_cf = (stiffnes_cf1*stiffnes_cf2)/(stiffnes_cf1+stiffnes_cf2)
        
               mass_efficient = (target.mass * another.mass)/(target.mass+another.mass)
                   
               mixed_restitution= sqrt(target.restitution * another.restitution)
               if mixed_restitution == 0:
                   mixed_restitution=EPS
               damping_ratio = - log(mixed_restitution)/ (sqrt(pi**2 + (log(mixed_restitution))**2 ))
               damping_cf= 2*damping_ratio*sqrt(summary_stiffness_cf*mass_efficient)
        
               target_rotational_velocity = Vector(
                                       -target.angular_velocity * target_point_leverarm.y,
                                       target.angular_velocity * target_point_leverarm.x
                                   )
               target_dot_contact_velocity = target.velocity+target_rotational_velocity
               
               another_rotational_velocity=Vector(
                                       -another.angular_velocity * another_point_leverarm.y,
                                       another.angular_velocity * another_point_leverarm.x
                                   )
               another_dot_contact_velocity= another.velocity+another_rotational_velocity
        
               relative_velocity=target_dot_contact_velocity-another_dot_contact_velocity
               radial_velocity : Vector = normal * (relative_velocity.scalar_multiply(normal))
        
               tangential_velocity = relative_velocity-radial_velocity
        
               #cf must be evaluated due to material data. materials WYP so it will be changed later
               common_friction_cf = sqrt(target.friction_cf*another.friction_cf)

               collision_force = self.calculate_collision_forces_penalty(
                   stiffnes_cf=summary_stiffness_cf,
                   normal=normal,
                   deformation=deformation,
                   damping_cf=damping_cf,
                   radial_velocity=radial_velocity,
                   tangential_velocity=tangential_velocity,
                   friction_cf=common_friction_cf
               )
        
               target.apply_force(collision_force, position)
               another.apply_force(-collision_force, position)  
    def calculate_collision_obstacles_penalty(self, target: PhysicalObject, obstacle: Obstacle):
        normal, contact_points = target.collider.calculate_deformation(obstacle.collider, target.pos, obstacle.center)
        if normal != Vector(0,0):
            for contact_point in contact_points:
                position=contact_point["pos"]
                deformation=contact_point["deformation"]
                torque_leverarm : Vector= position-target.pos
                dot_rotational_velocity = Vector(
                                        -target.angular_velocity * torque_leverarm.y,
                                        target.angular_velocity * torque_leverarm.x
                                    )
                contact_velocity = target.velocity+dot_rotational_velocity
                radial_velocity : Vector = normal * (contact_velocity.scalar_multiply(normal))
                tangential_velocity = contact_velocity-radial_velocity
                restitution=target.restitution
                if restitution == 0:
                    restitution=EPS
                damping_ratio = - log(restitution)/ (sqrt(pi**2 + (log(restitution))**2 ))
                damping_cf= 2*damping_ratio*sqrt(target.stiffness_cf*target.mass)

                collision_force = self.calculate_collision_forces_penalty(
                                   stiffnes_cf=target.stiffness_cf,
                                   normal=normal,
                                   deformation=deformation,
                                   damping_cf=damping_cf,
                                   radial_velocity=radial_velocity,
                                   tangential_velocity=tangential_velocity,
                                   friction_cf=target.friction_cf
                )
         
                target.apply_force(collision_force, position)

    def calculate_collision_joint_obstacle_penalty(self, joint: Joint):
        for endpoint in joint.endpoints:
            if isinstance(endpoint, JointNode):
                for obstacle in self.obstacles:
                    normal, contact_points = endpoint.collider.calculate_deformation(obstacle.collider, endpoint.pos, obstacle.center)
                    if normal != Vector(0,0):
                        for contact_point in contact_points:
                            position=contact_point["pos"]
                            deformation=contact_point["deformation"]
                            #temprorarily
                            stiffnes_cf=10
                            restitution=0.2
                            
                            contact_velocity = endpoint.velocity
                            radial_velocity : Vector = normal * (contact_velocity.scalar_multiply(normal))
                            tangential_velocity = contact_velocity-radial_velocity
                            damping_ratio = - log(restitution)/ (sqrt(pi**2 + (log(restitution))**2 ))
                            damping_cf= 2*damping_ratio*sqrt(stiffnes_cf*endpoint.mass)

                            collision_force = self.calculate_collision_forces_penalty(
                                stiffnes_cf=stiffnes_cf,
                                normal=normal,
                                deformation=deformation,
                                damping_cf=damping_cf,
                                radial_velocity=radial_velocity,
                                tangential_velocity=tangential_velocity,
                                friction_cf=endpoint.friction_cf
                            )
                           
                            endpoint.apply_force(collision_force)

    def calculate_collisions_penalty(self):
        objects: List[PhysicalObject]=self.sprites
        joints: List[Joint] = self.joints
        for i, target in enumerate(objects):
            for another in objects[i+1:]:                
                self.calculate_collision_objects_penalty(target, another)
            for obstacle in self.obstacles:
                self.calculate_collision_obstacles_penalty(target, obstacle)
        for joint in joints:
            self.calculate_collision_joint_obstacle_penalty(joint)
                