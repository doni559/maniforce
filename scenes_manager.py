from pygame import *
from pygame.sprite import Group
from pygame.time import Clock

from physics.utils import Vector
from physics.bodies import PhysicalObject, Obstacle, ObjectConfig, PhysicalObjectInitFields
from physics.solver import CollisionCalculator
from physics.connections import Joint

from settings import WIDTH, HEIGHT, SUBSTEPS

from typing import List

from pathlib import Path
import json
from dataclasses import asdict



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
        for object in self.objects:
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
        data={
            "obstacles":obstacles_data,
            "objects":objects
        }
        path= Path("./scenes/"+self.name+".json")
        with open(path, "w") as f:
            json.dump(data, f, indent=1)

def load_scene(name):
    path_to_scene="./scenes/"+name+".json"
    objects_list : List[PhysicalObject] =[]
    obstacles_list : List[Obstacle] =[]
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
    scene = Scene(objects_list, obstacles_list, name=name)
    return scene

screen_borders = [
    Obstacle(0,WIDTH, HEIGHT, HEIGHT+100),
    Obstacle(0,WIDTH, -100, 0),
    Obstacle(-100, 0, 0, HEIGHT),
    Obstacle(WIDTH, WIDTH+100, 0, HEIGHT),
]

polygon_cfg = ObjectConfig(
    stiffnes_cf=1000,
    density=1,
    friction_cf=0.3,
    restitution=0.4,
    gravity=True,
    draw_trajectory=False
)
box_cfg = ObjectConfig(
    stiffnes_cf=250,
    density=1,
    friction_cf=0.2,
    restitution=0.2,
    gravity=True,
    draw_trajectory=False
)
rubber_ball = ObjectConfig(

    stiffnes_cf=1000,
    density=1,
    friction_cf=0.6,
    restitution=0.2,
    gravity=True,
    draw_trajectory=False
)

polygon = PhysicalObject(init_fields=None, config=polygon_cfg,
    corners = 
    [
        Vector(-150, -20),
        Vector(150, -20),
        Vector(150, 20),
        Vector(-300, 20),
    ], start_pos=Vector(0,0), start_velocity=Vector(0,0), start_angle=90, collider_type="Polygon", name="Polygon0")
box = PhysicalObject(init_fields=None, config= box_cfg,corners = 
    [
        Vector(-50, -50),
        Vector(100, -50),
        Vector(100, 50),
        Vector(-50, 50),
    ], start_pos=Vector(0,0), start_velocity=Vector(0,0), start_angle=90, collider_type="Polygon", name="Box0")
ball= PhysicalObject(init_fields=None, config=rubber_ball,    radius=25, start_pos=Vector(0,0), start_velocity=Vector(0,0), collider_type="Circle", name="Ball0")
rope = Joint(anchor_0 = ball, anchor_1=Vector(ball.pos.x+100, ball.pos.y+100), stiffness_cf =50, nodes_count= 5, joint_mass=0.2, damping_cf=1, friction_cf = 0.001)

scene_to_load = "pendulum_test"
scene = Scene([], obstacles=screen_borders, name=scene_to_load)

# wall=Obstacle(100,200,0,500)
# wall1=Obstacle(800,900,0,500)
# scene.add_obstacle(wall)
# scene.add_obstacle(wall1)
ball1=scene.add_object(ball, name="Ball0",    radius=25, start_pos=Vector(270,345), start_velocity=Vector(100,0), start_angle = 0, start_angular_velocity=0)

scene.add_object(ball, start_pos = Vector(150, 25), radius=25)
scene.add_object(ball, start_pos = Vector(350, 25), radius=25)
scene.add_object(box, corners=[
    Vector(-200, -25),
    Vector(200, -25),
    Vector(200, 25),
    Vector(-200, 25)
], start_pos = Vector(250, 100), start_angle=0)
box1=scene.add_object(box, corners=[
    Vector(-200, -25),
    Vector(200, -25),
    Vector(200, 25),
    Vector(-200, 25)
], start_pos = Vector(250, 455), start_angle=0)

scene.add_object(box, corners=[
    Vector(-150, -25),
    Vector(150, -25),
    Vector(150, 25),
    Vector(-150, 25)
], start_pos = Vector(100, 275), start_angle=90)
scene.add_object(box, corners=[
    Vector(-150, -25),
    Vector(150, -25),
    Vector(150, 25),
    Vector(-150, 25)
], start_pos = Vector(350, 275), start_angle=90)

# ball1=scene.add_object(ball, name="Ball1",    radius=25, start_pos=Vector(600,25), start_velocity=Vector(1000,0), start_angle = 0, start_angular_velocity=-1000)
rope=scene.add_joint(rope, anchor_0 = ball1, anchor_1=box1, stiffness_cf = 100, force_limit = 5000)
# scene.save_scene()

# scene = load_scene(scene_to_load)