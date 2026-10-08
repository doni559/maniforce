from physics.utils import Vector, Integrator, EulerIntegrator, VerletIntegrator
from physics.bodies import PhysicalObject, Obstacle, ObjectConfig, PhysicalObjectInitFields
from physics.solver import CollisionCalculator
from physics.connections import Joint, JointInitFields, JointNode


from configs.settings import SUBSTEPS

from typing import List

from pathlib import Path
import json
from dataclasses import asdict, dataclass

@dataclass(frozen=True, slots=True)
class ObjectSnapshot():
    id = int

    type: str
    form: str
    points: List[Vector]
    width : float | None = None
    radius: float | None = None
    angle : float | None = None


@dataclass(frozen=True, slots=True)
class SceneSnapshot():
    objects: List[ObjectSnapshot]   

class Scene():
    def __init__(self, load:bool, name:str,used_integrator : Integrator, objects: List[PhysicalObject] | None = None, obstacles: List[Obstacle] | None= None, joints : List[Joint] | None = None, substeps: int = SUBSTEPS):
        if load == False:
            self.name= name
            self.used_integrator = used_integrator

            if objects is not None:
                self.objects = objects
            else:
                self.objects = []
            if obstacles is not None:
                self.obstacles = obstacles
            else:
                self.obstacles = []
            if joints is not None:
                self.joints = joints
            else:
                self.joints = []
            self.substeps = substeps

            self.collisions=CollisionCalculator(self.objects, self.obstacles, self.joints)
        else:
            self.__init__(**load_scene(name, substeps=substeps, used_integrator=used_integrator).get_fields())

    def step(self, dt: float):
        joint_to_destroy = []
        for _ in range(0,self.substeps):
            for joint in self.joints:
                joint.calculate()
                if joint.to_destroy == True:
                    joint_to_destroy.append(joint)
            for i in range(self.used_integrator.pre_integrations):
                for sprite in self.objects:
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
            for sprite in self.objects:
                sprite.calc_forces()
                state = sprite.get_state()
                old_acceleration = state.acceleration
                old_angular_acceleration = state.angular_acceleration
                new_state = self.used_integrator.integrate(state, old_acceleration, old_angular_acceleration, dt/self.substeps )
                sprite.apply_state(new_state)
                
            for joint in self.joints:
                for endpoint in joint.endpoints:
                    if isinstance(endpoint, JointNode):
                        state = endpoint.get_state()
                        old_acceleration = state.acceleration
                        new_state = self.used_integrator.integrate(state, old_acceleration, None,dt/self.substeps)
                        endpoint.apply_state(new_state)
            self.collisions.calculate_collisions_penalty()
        for joint in joint_to_destroy:
            self.remove_joint(joint)
        
    def make_snapshot(self):
        objects_snapshots: List[ObjectSnapshot]=[]
        for obj in self.objects:
            collider = obj.collider
            angle = obj.angle

            if collider.type == "Circle":
                radius = obj.radius
                center = collider.center
                form = "Circle"
                snapshot = ObjectSnapshot(
                    type="object",
                    form = form,
                    points=[center],
                    radius=radius,
                    angle=angle
                )
            elif collider.type == "Polygon":
                points = collider.get_world_corners(obj)
                form = "Polygon"
                snapshot=ObjectSnapshot(
                    type="object",
                    form=form,
                    points=points,
                    angle=angle
                )
            objects_snapshots.append(snapshot)
            
            if obj.draw_trajectory == True:
                for trajectory_point in obj.trajectory_arr:
                    snapshot=ObjectSnapshot(
                        type="trajectory",
                        form="Circle",
                        points=[trajectory_point],
                        radius=3
                    )
                    objects_snapshots.append(snapshot)

        for obstacle in self.obstacles:
            form = "Polygon"
            collider=obstacle.collider
            points=collider.corners

            snapshot=ObjectSnapshot(
                type="obstacle",
                form=form,
                points=points
            )
            objects_snapshots.append(snapshot)
        for joint in self.joints:
            form = "Line"
            width = 4
            lines=joint.get_lines_to_draw()
            for line in lines:
                points = line
                snapshot = ObjectSnapshot(
                    type="joint",
                    form=form,
                    points=points,
                    width=width
                )
                objects_snapshots.append(snapshot)
        scene_snapshot= SceneSnapshot(objects_snapshots)
        return scene_snapshot

    def remove_joint(self, target_joint: Joint):
        for i in range(0, len(self.joints)):
            if self.joints[i] is target_joint:
                self.joints.pop(i)

    def add_object(self, object: PhysicalObject, **kwargs):
        init_fields=object.get_fields()
        init_fields.__dict__.update(kwargs)

        new_object = PhysicalObject(init_fields)
        self.objects.append(new_object)
        self.collisions = CollisionCalculator(self.objects, self.obstacles, self.joints)
        return new_object

    def add_obstacle(self, obstacle: Obstacle):
        self.obstacles.append(obstacle)

        self.collisions = CollisionCalculator(self.objects, self.obstacles, self.joints)

    def add_joint(self, joint: Joint, **kwargs):
        init_fields=joint.get_fields()
        init_fields.__dict__.update(kwargs, scene=self)

    
        new_joint = Joint(init_fields)
    
        self.joints.append(new_joint)
        return new_joint

    def restart(self):
        return load_scene(self.name, substeps=self.substeps, used_integrator=self.used_integrator)
    
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



