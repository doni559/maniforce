from pygame import sprite

from typing import List
from math import sqrt, log, e as euler, pi

from .bodies import PhysicalObject, Obstacle
from .connections import Joint, JointNode

from .utils import Vector
from configs.settings import EPS

class CollisionCalculator():
    def __init__(self, all_sprites : sprite.Group, all_obstacles: List[Obstacle], all_joints: List[Joint]):
        self.colliders = all_obstacles
        self.sprites= all_sprites.sprites()
        self.joints = all_joints

    def calculate_collisions_penalty(self):
        objects: List[PhysicalObject]=self.sprites
        joints: List[Joint] = self.joints
        for i, target in enumerate(objects):
            for another in objects[i+1:]:                
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
                        spring_force= normal * summary_stiffness_cf * deformation

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

                        damping_force = radial_velocity.normalise() *(-1) * damping_cf*radial_velocity.get_length()
                        contact_force= spring_force+damping_force

                        #friction_force calc

                        #cf must be evaluated due to material data. materials WYP so it will be changed later
                        common_friction_cf = sqrt(target.friction_cf*another.friction_cf)
                        friction_force = tangential_velocity.normalise() *(-1) * common_friction_cf * contact_force.get_length()
                        collision_force = contact_force+friction_force

                        target.apply_force(collision_force, position)
                        another.apply_force(-collision_force, position)  

            for collider in self.colliders:
                normal, contact_points = target.collider.calculate_deformation(collider.collider, target.pos, collider.center)
                if normal != Vector(0,0):
                    for contact_point in contact_points:
                        position=contact_point["pos"]
                        deformation=contact_point["deformation"]
                        torque_leverarm : Vector= position-target.pos
                    
                        spring_force : Vector= normal * target.stiffness_cf * abs(deformation)
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
                        damping_force : Vector = radial_velocity.normalise()*(-1) * damping_cf * radial_velocity.get_length()

                        contact_force=spring_force+damping_force
                        
                        #friction force
                        friction_force = tangential_velocity.normalise()*(-1) * target.friction_cf * contact_force.get_length()

                        collision_force=contact_force+friction_force
                        target.apply_force(collision_force, position)
        for joint in joints:
            for endpoint in joint.endpoints:
                if isinstance(endpoint, JointNode):
                    for k in range(0, len(self.colliders)):
                        collider = self.colliders[k]
                        normal, contact_points = endpoint.collider.calculate_deformation(collider.collider, endpoint.pos, collider.center)
                        if normal != Vector(0,0):
                            for contact_point in contact_points:
                                position=contact_point["pos"]
                                deformation=contact_point["deformation"]
                                stiffnes_cf=10
                                spring_force : Vector= normal * stiffnes_cf * abs(deformation)
                                
                                contact_velocity = endpoint.velocity
                                radial_velocity : Vector = normal * (contact_velocity.scalar_multiply(normal))

                                tangential_velocity = contact_velocity-radial_velocity

                                restitution=0.2
                                damping_ratio = - log(restitution)/ (sqrt(pi**2 + (log(restitution))**2 ))
                                damping_cf= 2*damping_ratio*sqrt(stiffnes_cf*endpoint.mass)
                                damping_force : Vector = radial_velocity.normalise()*(-1) * damping_cf * radial_velocity.get_length()

                                contact_force=spring_force+damping_force
                                #friction force
                                friction_force = tangential_velocity.normalise()*(-1) * endpoint.friction_cf * contact_force.get_length()
                                collision_force=contact_force+friction_force
                                endpoint.apply_force(collision_force)
                