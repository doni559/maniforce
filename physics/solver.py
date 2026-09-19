from pygame import sprite

from typing import List
from math import sqrt, log, e as euler, pi

from .bodies import PhysicalObject, Obstacle
from .connections import Joint, JointNode

from .utils import Vector
from settings import EPS

class CollisionCalculator():
    def __init__(self, all_sprites : sprite.Group, all_obstacles: List[Obstacle], all_joints: List[Joint]):
        self.colliders = all_obstacles
        self.sprites= all_sprites.sprites()
        self.joints = all_joints

    def calculate_collisions_penalty(self):
        sprites: List[PhysicalObject]=self.sprites
        joint: List[Joint] = self.joints
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
                
                normal, contact_points = target_sprite.collider.calculate_deformation(another_sprite.collider, target_sprite.pos, another_sprite.pos)

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
                    if mixed_restitution == 0:
                        mixed_restitution=EPS
                    damping_ratio = - log(mixed_restitution)/ (sqrt(pi**2 + (log(mixed_restitution))**2 ))
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

                    tangential_velocity = relative_velocity-radial_velocity

                    damping_force = radial_velocity.normalise() *(-1) * damping_cf*radial_velocity.get_length()
                    contact_force= spring_force+damping_force

                    #friction_force calc

                    #cf must be evaluated due to material data. materials WYP so it will be changed later
                    common_friction_cf = sqrt(target_sprite.friction_cf*another_sprite.friction_cf)

                    friction_force = tangential_velocity.normalise() *(-1) * common_friction_cf * contact_force.get_length()
                    collision_force = contact_force+friction_force
                    collision_torque_target = target_point_leverarm.vector_multiply(collision_force)
                    collision_torque_another = another_point_leverarm.vector_multiply(collision_force)

                    resulting_forces[i]+=collision_force
                    resulting_forces[j]+=collision_force*(-1)
                    resulting_torques[i]+=collision_torque_target
                    resulting_torques[j]+=collision_torque_another*(-1)


            for k in range(0, len(self.colliders)):
                collider = self.colliders[k]
                normal, contact_points = target_sprite.collider.calculate_deformation(collider.collider, target_sprite.pos, collider.center)
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

                    tangential_velocity = contact_velocity-radial_velocity

                    restitution=target_sprite.restitution
                    if restitution == 0:
                        restitution=EPS
                    damping_ratio = - log(restitution)/ (sqrt(pi**2 + (log(restitution))**2 ))
                    damping_cf= 2*damping_ratio*sqrt(target_sprite.stiffness_cf*target_sprite.mass)
                    damping_force : Vector = radial_velocity.normalise()*(-1) * damping_cf * radial_velocity.get_length()

                    contact_force=spring_force+damping_force
                    
                    #friction force
                    friction_force = tangential_velocity.normalise()*(-1) * target_sprite.friction_cf * contact_force.get_length()

                    collision_force=contact_force+friction_force
                    collision_torque = torque_leverarm.vector_multiply(collision_force)


                    resulting_forces[i]+=collision_force
                    resulting_torques[i]+=collision_torque

        for key, value in resulting_forces.items():
            sprites[key].resultant_force+=value
        for key, value in resulting_torques.items():
            sprites[key].resultant_torque+=value

        joint_resulting_forces = {}

        for i in range (0, len(self.joints)):
            joint : Joint=self.joints[i]
            joint_resulting_forces[i]={}
            for j in range(0, len(joint.endpoints)):
                endpoint= joint.endpoints[j]
                if isinstance(endpoint, JointNode):
                    joint_resulting_forces[i][j]=Vector(0,0)
                    for k in range(0, len(self.colliders)):
                        collider = self.colliders[k]
                        normal, contact_points = endpoint.collider.calculate_deformation(collider.collider, endpoint.pos, collider.center)
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

                            joint_resulting_forces[i][j]+=collision_force
        for key, value in joint_resulting_forces.items():
            joint = self.joints[key]
            for k,v  in joint_resulting_forces[key].items():
                node : JointNode =joint.endpoints[k]
                node.resultant_force+=v
            
                