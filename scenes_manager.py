from pygame import *
from pygame.sprite import Group

from physics.utils import Vector
from physics.bodies import PhysicalObject, Obstacle, ObjectConfig, PhysicalObjectInitFields
from physics.solver import CollisionCalculator
from physics.connections import Joint, JointInitFields

from settings import WIDTH, HEIGHT, SUBSTEPS

from typing import List

from pathlib import Path
import json
from dataclasses import asdict

from materials import ball, box, polygon, rope

class Scene():
    def __init__(self, objects: List[PhysicalObject], obstacles: List[Obstacle], name:str, joints : List[Joint] = []):
        self.name= name
        self.objects = objects
        self.sprites= Group(objects)
        self.obstacles = obstacles
        self.joints = joints

        self.collisions=CollisionCalculator(self.sprites, self.obstacles, self.joints)
    
    def render_scene(self, screen:Surface, dt: float):
        screen.fill((255, 255, 255))
        for _ in range(0,SUBSTEPS):
            self.sprites.update(dt)
            
            self.collisions.calculate_collisions_penalty()
        for joint in self.joints:
            joint.update(dt)
            if joint.to_destroy == True:
                self.remove_joint(joint)
        for joint in self.joints:

            joint.draw_joint(screen)

        for sprite in self.sprites.sprites():
            sprite : PhysicalObject = sprite
            sprite.collider.draw(screen, (0,255,0), sprite.pos.convert_to_screen_cords())
            # sprite.velocity.draw(screen, sprite.pos, (0,0,255))
            try:
                origin_center = sprite.pos + sprite.origin_offset_local.rotate(sprite.angle)
                draw.circle(screen, (255, 255, 0), origin_center.convert_to_screen_cords(),3)

            except:
                pass
            draw.circle(screen, (255, 0, 255), sprite.pos.convert_to_screen_cords(),3)
        

            if sprite.draw_trajectory:
                for dot_index in range(1,len(sprite.trajectory_arr)):
                    dot=sprite.trajectory_arr[dot_index]
                    prev_dot = sprite.trajectory_arr[dot_index-1]
                    draw.line(screen, (255, 0, 255), prev_dot.convert_to_screen_cords(), dot.convert_to_screen_cords(), width=3)
                    
                sprite.trajectory_arr.append(Vector(sprite.pos.x, sprite.pos.y))
                if len(sprite.trajectory_arr) > 1000:
                    sprite.trajectory_arr = sprite.trajectory_arr[-1000:-1]
        for obstacle in self.obstacles:
            obstacle.collider.draw(screen, (0,0,0))

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
        return load_scene(self.name)
    
    def save_scene(self):
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
        path= Path("./scenes/"+self.name+".json")
        with open(path, "w") as f:
            json.dump(data, f, indent=1)

def load_scene(name):
    path_to_scene="./scenes/"+name+".json"
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
    scene = Scene(objects_list, obstacles_list, name=name, joints=joints_list)
    return scene

screen_borders = [
    Obstacle(0,WIDTH, HEIGHT, HEIGHT+100),
    Obstacle(0,WIDTH, -100, 0),
    Obstacle(-100, 0, 0, HEIGHT),
    Obstacle(WIDTH, WIDTH+100, 0, HEIGHT),
]

scene_to_load = ""
scene = Scene([], obstacles=screen_borders, name=scene_to_load)