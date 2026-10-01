from pygame import *
from pygame.sprite import Group

from physics.utils import Vector, Integrator, PhysicalObjectState, EulerIntegrator, VerletIntegrator
from physics.bodies import PhysicalObject, Obstacle, ObjectConfig, PhysicalObjectInitFields
from physics.solver import CollisionCalculator
from physics.connections import Joint, JointInitFields, JointNode


from configs.settings import WIDTH, HEIGHT, SUBSTEPS

from typing import List

from pathlib import Path
import json
from dataclasses import asdict

from configs.materials import ball, box, polygon, rope

class Scene():
    def __init__(self, load:bool, name:str,used_integrator : Integrator, objects: List[PhysicalObject]=[], obstacles: List[Obstacle]=[], joints : List[Joint] = [], substeps: int = SUBSTEPS):
        if load == False:
            self.name= name
            self.used_integrator = used_integrator


            self.sprites= Group(objects)

            self.objects = objects
            self.obstacles = obstacles
            self.joints = joints
            self.substeps = substeps

            self.collisions=CollisionCalculator(self.sprites, self.obstacles, self.joints)
        else:
            self.__init__(**load_scene(name, substeps=substeps, used_integrator=used_integrator).get_fields())

    def step(self, dt: float):
        joint_to_destroy = []
        for _ in range(0,self.substeps):
            if isinstance(self.used_integrator, EulerIntegrator):
                for joint in self.joints:
                    joint.calculate()
                    if joint.to_destroy == True:
                        joint_to_destroy.append(joint)
                self.collisions.calculate_collisions_penalty()
                for sprite in self.sprites:
                    sprite: PhysicalObject = sprite
                    sprite.calc_forces()

                    state = sprite.get_state()
                    new_state = self.used_integrator.integrate(state, dt/self.substeps)
                    sprite.apply_state(new_state)                    

                for joint in self.joints:
                    for endpoint in joint.endpoints:
                        if isinstance(endpoint, JointNode):
                            state = endpoint.get_state()
                            new_state = self.used_integrator.integrate(state, dt/self.substeps)
                            endpoint.apply_state(new_state)
                
            elif isinstance(self.used_integrator, VerletIntegrator):
                for joint in self.joints:
                    joint.calculate()
                    if joint.to_destroy == True:
                        joint_to_destroy.append(joint)
                for sprite in self.sprites:
                    sprite: PhysicalObject = sprite
                    sprite.calc_forces()
                    state = sprite.get_state()

                    new_state = self.used_integrator.pre_integrate(state, dt/self.substeps)
                    sprite.apply_state(new_state)
                for joint in self.joints:
                    for endpoint in joint.endpoints:
                        if isinstance(endpoint, JointNode):
                            state = endpoint.get_state()
                            new_state = self.used_integrator.pre_integrate(state, dt/self.substeps)
                            endpoint.apply_state(new_state)
                for joint in self.joints:
                    joint.calculate()
                    if joint.to_destroy == True:
                        joint_to_destroy.append(joint)
                self.collisions.calculate_collisions_penalty()

                for sprite in self.sprites:
                    sprite.calc_forces()
                    state = sprite.get_state()
                    old_acceleration = state.acceleration
                    old_angular_acceleration = state.angular_acceleration
                    new_state = self.used_integrator.integrate(state, old_acceleration, old_angular_acceleration, dt/self.substeps)
                    sprite.apply_state(new_state)
                    
                for joint in self.joints:
                    for endpoint in joint.endpoints:
                        if isinstance(endpoint, JointNode):
                            state = endpoint.get_state()
                            old_acceleration = state.acceleration
                            new_state = self.used_integrator.integrate(state, old_acceleration, None, dt/self.substeps)
                            endpoint.apply_state(new_state)
                self.collisions.calculate_collisions_penalty()
        for joint in joint_to_destroy:
            self.remove_joint(joint)
        
    
    def render(self, screen:Surface, camera_pos: List[float], camera_zoom: float):
        screen.fill((255, 255, 255))
        for joint in self.joints:
            joint.draw_joint(screen, camera_pos, camera_zoom)
        for sprite in self.sprites.sprites():
            sprite : PhysicalObject = sprite
            #Collider draw
            sprite.collider.draw(screen, (0,255,0), sprite.pos.convert_to_screen_cords(camera_pos, camera_zoom), camera_pos, camera_zoom)
            #Draw origin. TODO: Add system flag
            try:
                origin_center = sprite.pos + sprite.origin_offset_local.rotate(sprite.angle)
                draw.circle(screen, (255, 255, 0), origin_center.convert_to_screen_cords(camera_pos, camera_zoom),3*camera_zoom)
            except:
                pass
            #Draw COM. TODO: Add system flag
            draw.circle(screen, (255, 0, 255), sprite.pos.convert_to_screen_cords(camera_pos, camera_zoom),3*camera_zoom)

            #Trajectory render
            if sprite.draw_trajectory:
                for dot_index in range(1,len(sprite.trajectory_arr)):
                    dot=sprite.trajectory_arr[dot_index]
                    prev_dot = sprite.trajectory_arr[dot_index-1]
                    draw.line(screen, (255, 0, 255), prev_dot.convert_to_screen_cords(camera_pos, camera_zoom), dot.convert_to_screen_cords(camera_pos, camera_zoom), width=3)
                    
                sprite.trajectory_arr.append(Vector(sprite.pos.x, sprite.pos.y))
                if len(sprite.trajectory_arr) > 1000:
                    sprite.trajectory_arr = sprite.trajectory_arr[-1000:-1]
        #Obstacles render
        for obstacle in self.obstacles:
            obstacle.collider.draw(screen, (0,0,0), camera_pos=camera_pos, camera_zoom=camera_zoom)

    def remove_joint(self, target_joint: Joint):
        for i in range(0, len(self.joints)):
            if self.joints[i] is target_joint:
                self.joints.pop(i)

    def add_object(self, object: PhysicalObject, **kwargs):
        init_fields=object.get_fields()
        init_fields.__dict__.update(kwargs)

        new_object = PhysicalObject(init_fields)

        self.sprites.add(new_object)
        self.objects.append(new_object)
        self.collisions = CollisionCalculator(self.sprites, self.obstacles, self.joints)
        return new_object

    def add_obstacle(self, obstacle: Obstacle):
        self.obstacles.append(obstacle)

        self.collisions = CollisionCalculator(self.sprites, self.obstacles, self.joints)

    def add_joint(self, joint: Joint, **kwargs):
        init_fields=joint.get_fields()
        init_fields.__dict__.update(kwargs, scene=self)

    
        new_joint = Joint(init_fields)
    
        self.joints.append(new_joint)
        return new_joint

    def restart(self):
        return load_scene(self.name, substeps=self.substeps)
    
    def save_scene(self,path: str = "./scenes/"):
        objects = []
        object_names=[]
        for object in self.objects:
            object_names.append(object.name)
            obj_data = asdict(object.get_fields())
            for key, value in obj_data.items():
                if isinstance(value, Vector):
                    obj_data[key]=value.as_tuple()
            if obj_data["corners"] is not None:
                obj_data["corners"]=[corner.as_tuple() for corner in obj_data["corners"]]
            objects.append(obj_data)
        obstacles_data=[]
        for obstacle in self.obstacles:
            obstacles_data.append(
                {
                    "x0":obstacle.x0,
                    "x1":obstacle.x1,
                    "y0":obstacle.y0,
                    "y1":obstacle.y1
                }
            )
        joints_data=[]
        for joint in self.joints:
            anchor0_name=joint.anchor_0.name
            anchor1_name=None
            if isinstance(joint.anchor_1, PhysicalObject):
                anchor1_name=joint.anchor_1.name

            if (object_names.count(anchor0_name) > 1 or object_names.count(anchor1_name) > 1):
                raise ValueError("Objects names are not unique")
            joint_data= asdict(joint.get_fields())
            joint_data["anchor_0"]=anchor0_name
            if anchor1_name is not None:
                joint_data["anchor_1"]=anchor1_name
            else:
                joint_data["anchor_1"]=joint_data["anchor_1"].as_tuple()
            joints_data.append(joint_data)
        data={
            "obstacles":obstacles_data,
            "objects":objects,
            "joints": joints_data
        }
        path= Path(path+self.name+".json")
        with open(path, "w") as f:
            json.dump(data, f, indent=1)
    def get_fields(self):
        return {
            "load":False,
            "objects": self.objects,
            "obstacles": self.obstacles,
            "name": self.name,
            "joints": self.joints,
            "used_integrator": self.used_integrator,
            "substeps": self.substeps
        }

def load_scene(name, used_integrator: Integrator, path: str = "./scenes/", substeps: int = SUBSTEPS):
    path_to_scene=path+name+".json"
    objects_list : List[PhysicalObject] =[]
    obstacles_list : List[Obstacle] =[]
    joints_list : List[Joint] = []
    with open(path_to_scene, "r") as f:
        scene_json=json.load(f)
    for object in scene_json["objects"]:
        object : dict= object

        object["config"] = ObjectConfig(**object["config"])
        if object["corners"] is not None:
            object["corners"]=[Vector(corner[0], corner[1]) for corner in object["corners"]]
        object["start_pos"]=Vector(object["start_pos"][0], object["start_pos"][1])
        object["start_velocity"]=Vector(object["start_velocity"][0], object["start_velocity"][1])

        init_fields = PhysicalObjectInitFields(**object)

        objects_list.append(PhysicalObject(init_fields))
    for obstacle in scene_json["obstacles"]:
        obstacle_instance=Obstacle(obstacle["x0"],obstacle["x1"],obstacle["y0"],obstacle["y1"])
        obstacles_list.append(obstacle_instance)
    for joint in scene_json["joints"]:
        joint:dict = joint
        anchor_0_name=joint["anchor_0"]
        anchor_1_name=None
        if isinstance(joint["anchor_1"], str):
            anchor_1_name=joint["anchor_1"]
        else:
            anchor_1=Vector(joint["anchor_1"][0],joint["anchor_1"][1])
        for obj in objects_list:
            if anchor_0_name == obj.name:
                anchor_0=obj
            if (anchor_1_name is not None) and anchor_1_name == obj.name:
                anchor_1=obj
        joint["anchor_0"]=anchor_0
        joint["anchor_1"]=anchor_1
        init_fields = JointInitFields(**joint)
        joint_instance = Joint(init_fields)
        joints_list.append(joint_instance)
    scene = Scene(load=False, name=name, objects=objects_list, obstacles=obstacles_list, joints=joints_list, substeps=substeps, used_integrator=used_integrator)
    return scene

screen_borders = [
    Obstacle(0,WIDTH, HEIGHT, HEIGHT+100),
    Obstacle(0,WIDTH, -100, 0),
    Obstacle(-100, 0, 0, HEIGHT),
    Obstacle(WIDTH, WIDTH+100, 0, HEIGHT),
]

scene_to_load = "stiff_rope_stability_test"
scene = Scene(load=False, name=scene_to_load, obstacles=screen_borders, substeps=SUBSTEPS, used_integrator=VerletIntegrator())
scene.add_object(ball, 
                 start_pos=Vector(1000,400),
                 start_velocity=Vector(-1000,0),
                 radius=50
                 )

scene.add_joint(
    rope,
    anchor_0 = scene.objects[0],
    anchor_1 = Vector(1000, 1000),
    stiffnes_cf = 10000,
    nodes_count = 10,
    damping_cf = 0.6,
    joint_mass=0.4,
)
scene.save_scene()
