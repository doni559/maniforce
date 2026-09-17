from pygame import *
from pygame.sprite import Group
from pygame.time import Clock

from physics.utils import Vector
from physics.bodies import PhysicalObject, Obstacle, Pendulum, ObjectConfig
from physics.solver import CollisionCalculator

from settings import WIDTH, HEIGHT, SUBSTEPS

from typing import List


from pathlib import Path
import json
from dataclasses import asdict



class Scene():
    def __init__(self, objects: List[PhysicalObject], obstacles: List[Obstacle], name:str):
        self.name= name
        self.objects = objects
        self.sprites= Group(objects)
        self.obsctacles = obstacles
        self.collisions=CollisionCalculator(self.sprites, obstacles)
    
    def render_scene(self, screen:Surface, clock: Clock, dt: float):
        screen.fill((255, 255, 255))
        for _ in range(0,SUBSTEPS):
            self.sprites.update(dt)
            self.collisions.calculate_collisions_penalty()

        for sprite in self.sprites.sprites():
            if isinstance(sprite, Pendulum) and sprite.rope_exists:
                sprite : Pendulum= sprite
                draw.line(screen, (255, 0, 0), (sprite.suspension_point.x, HEIGHT-sprite.suspension_point.y), (sprite.pos.x, HEIGHT-sprite.pos.y), 3)
            else:
                sprite : PhysicalObject = sprite

            sprite : PhysicalObject = sprite
            sprite.collider.draw(screen, (0,255,0), (sprite.pos.x, HEIGHT-sprite.pos.y))
            sprite.velocity.draw(screen, sprite.pos, (0,0,255))
            try:
                origin_center = sprite.pos + sprite.origin_offset_local.rotate(sprite.angle)
                draw.circle(screen, (255, 255, 0), (origin_center.x, HEIGHT-origin_center.y),3)

            except:
                pass
            draw.circle(screen, (255, 0, 255), (sprite.pos.x, HEIGHT-sprite.pos.y),3)
        

            if sprite.draw_trajectory:
                for dot_index in range(1,len(sprite.trajectory_arr)):
                    dot=sprite.trajectory_arr[dot_index]
                    prev_dot = sprite.trajectory_arr[dot_index-1]
                    draw.line(screen, (255, 0, 255), (prev_dot.x, HEIGHT-prev_dot.y), (dot.x, HEIGHT-dot.y), width=3)
                    
                sprite.trajectory_arr.append(Vector(sprite.pos.x, sprite.pos.y))
                if len(sprite.trajectory_arr) > 1000:
                    sprite.trajectory_arr = sprite.trajectory_arr[-1000:-1]
        for obstacle in self.obsctacles:
            obstacle.collider.draw(screen, (0,0,0))
    def add_object(self, object: PhysicalObject):
        self.sprites.add(object)
        self.objects.append(object)

        self.collisions = CollisionCalculator(self.sprites, self.obsctacles)

    def add_obstacle(self, obstacle: Obstacle):
        self.obsctacles.append(obstacle)

        self.collisions = CollisionCalculator(self.sprites, self.obsctacles)

    def restart(self):
        return load_scene(self.name)
    
    def save_scene(self):
        objects = []
        for object in self.objects:
            obj_data= asdict(object.config)
            obj_data["name"] = object.name
            obj_data["type"] = object.type
            obj_data["start_pos"]=(object.pos.x, object.pos.y)
            obj_data["start_velocity"]=(object.velocity.x, object.velocity.y)
            obj_data["collider_type"]=object.collider.type
            obj_data["start_angle"]=object.angle
            obj_data["start_angular_velocity"]=object.angular_velocity
            if object.type == "Pendulum":
                obj_data["suspension_point"]=(object.suspension_point.x, object.suspension_point.y)
            objects.append(obj_data)
        obstacles_data=[]
        for obstacle in self.obsctacles:
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
        config = ObjectConfig(**{k: v for k, v in object.items() if k in ObjectConfig.__dataclass_fields__})
        start_pos=Vector(object["start_pos"][0] , object["start_pos"][1])
        start_velocity=Vector(object["start_velocity"][0], object["start_velocity"][1])
        start_angle=object["start_angle"]
        start_angular_velocity=object["start_angular_velocity"]
        collider_type = object["collider_type"]
            
        if object["type"] == "Pendulum":
            suspension_point=Vector(object["suspension_point"][0], object["suspension_point"][1])
            objects_list.append(Pendulum(config, start_pos, start_velocity, collider_type, suspension_point, object["name"]))
        else:
            objects_list.append(PhysicalObject(config, start_pos, start_velocity, start_angle, start_angular_velocity, collider_type, object["name"]))
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
box_cfg = ObjectConfig(
     corners = [
        (-150, -20),
        (150, -20),
        (150, 20),
        (-150, 20),
    ],
    stiffnes_cf=1000,
    density=1,
    friction_cf=0.1,
    restitution=0.2,
    gravity=True,
    draw_trajectory=False
)
rubber_ball = ObjectConfig(
    radius=25,
    stiffnes_cf=1000,
    density=1,
    friction_cf=0.1,
    restitution=0.4,
    gravity=True,
    draw_trajectory=False
)
rubber_ball1 = ObjectConfig(
    radius=50,
    stiffnes_cf=1000,
    density=1,
    friction_cf=0.1,
    restitution=0.4,
    gravity=True,
    draw_trajectory=False
)
rubber_ball_pendulum = ObjectConfig(
    density=1,
    radius=50,
    gravity=True,
    draw_trajectory=False,
    rope_stiffness_cf=2000,
    restitution=1,
    stiffnes_cf=1000,
    friction_cf=0,
    rope_dampfing_cf=200,
    rope_force_limit=100000,
    rope_exists=True,
    is_rope_breakable=True,
    pendulum_friction= False
)

polygon = PhysicalObject(box_cfg, start_pos=Vector(400,25+50), start_velocity=Vector(0,0), start_angular_velocity=0, collider_type="Polygon", name="Box1")
ball= PhysicalObject(rubber_ball, start_pos=Vector(450,25), start_velocity=Vector(0,0), collider_type="Circle", name="Ball1")
ball1= PhysicalObject(rubber_ball, start_pos=Vector(350,25), start_velocity=Vector(0,0), collider_type="Circle", name="Ball2")
ball2= PhysicalObject(rubber_ball1, start_pos=Vector(450,25+50+70), start_velocity=Vector(-100,0), collider_type="Circle", name="Ball3")

scene = Scene([polygon, ball, ball1, ball2], obstacles=screen_borders, name="ball on cart")
scene.save_scene()


scene = load_scene("ball on cart")