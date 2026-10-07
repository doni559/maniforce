from .bodies import PhysicalObject
from .utils import Vector, JointNodeState
from .collider import Collider

from configs.settings import GRAV_CONST, SECTOR_LENGTH

from dataclasses import asdict, dataclass, field
from typing import List
from abc import ABC,abstractmethod 

from math import ceil, pi, sqrt

class JointEndpoint(ABC):
    @abstractmethod
    def get_pos(self)->Vector:...

    @abstractmethod
    def get_velocity(self)->Vector:...

    @abstractmethod
    def apply_force(self, force: Vector):...

@dataclass
class BodyAnchor(JointEndpoint):
    object: PhysicalObject

    def get_pos(self):
        return self.object.pos

    def get_velocity(self):
        return self.object.velocity

    def apply_force(self, force):
        self.object.apply_force(force)


@dataclass
class WorldAnchor(JointEndpoint):
    pos: Vector

    def get_pos(self):
        return self.pos

    def get_velocity(self):
        return Vector(0,0)

    def apply_force(self, force):
        pass

@dataclass(slots=True)
class JointNode(JointEndpoint):
    pos: Vector
    mass: float
    friction_cf: float
    acceleration: Vector = field(
        default_factory=lambda: Vector(0, 0)
    )
    
    velocity: Vector = field(
        default_factory=lambda: Vector(0, 0)
    )

    resultant_force: Vector = field(
        default_factory=lambda: Vector(0, 0)
    )
    def __post_init__(self):
        self.collider = Collider(
            "Circle",
            center=self.pos,
            radius=2
        )
    
    def get_pos(self):
        return self.pos

    def get_velocity(self):
        return self.velocity

    def apply_force(self, force):
        self.resultant_force+=force

    def get_state(self) -> JointNodeState:
        state = JointNodeState(**{k: v for k, v in asdict(self).items() if k in JointNodeState.__dataclass_fields__})
        return state

    def apply_state(self, state: JointNodeState):
        self.velocity=state.velocity
        self.pos=state.pos
        self.acceleration=state.acceleration 
        self.resultant_force=Vector(0,-GRAV_CONST*self.mass)

@dataclass
class JointSector():
    node_a: JointEndpoint
    node_b: JointEndpoint
    stiffnes_cf: float
    rest_length: float
    damping_cf : float
    force_limit: float
    elong : float = 0

    to_destroy : bool = False

    def calculate(self):

        delta = self.node_b.get_pos()-self.node_a.get_pos()
        length = delta.get_length()
        normal = delta.normalise()

        elong = length-self.rest_length
        self.elong = elong

        spring_force = normal * elong * self.stiffnes_cf

        relative_velocity = (self.node_b.get_velocity()-self.node_a.get_velocity())
        damping_force = normal * self.damping_cf * relative_velocity.scalar_multiply(normal)

        resultant_force = spring_force+ damping_force

        if resultant_force.get_length() > self.force_limit:
            self.to_destroy= True

        self.node_a.apply_force(resultant_force)
        self.node_b.apply_force(-resultant_force)


@dataclass
class JointInitFields():
    anchor_0: PhysicalObject
    anchor_1: PhysicalObject | Vector

    is_flexible: bool

    damping_ratio: float = 0.2 
    friction_cf: float = 0.01

    youngs_modulus: float = 1
    density: float = 1
    force_limit: float | None = None



class Joint():
    def __init__(self, init_fields: JointInitFields | None = None, **kwargs):
        if init_fields is None:
            init_fields = JointInitFields(**kwargs)

        self.anchor_0 = init_fields.anchor_0
        self.anchor_1 = init_fields.anchor_1

        youngs_modulus = init_fields.youngs_modulus
        density = init_fields.density*10**(-6)
        damping_ratio = init_fields.damping_ratio

        self.friction_cf=init_fields.friction_cf

        self.is_flexible = init_fields.is_flexible
        self.to_destroy = False

        self.sectors : List[JointSector] = []
        self.endpoints : List[JointEndpoint] = []

        if init_fields.force_limit is not None:
            self.force_limit = init_fields.force_limit
        else:
            self.force_limit =  float("inf")

        
        if isinstance(self.anchor_1, PhysicalObject):
            self.rope_normal = (self.anchor_1.pos-self.anchor_0.pos).normalise()
            self.rope_length= (self.anchor_1.pos-self.anchor_0.pos).get_length()
            last_node = BodyAnchor(self.anchor_1)
        else:
            self.rope_normal= (self.anchor_1-self.anchor_0.pos).normalise()
            self.rope_length = (self.anchor_1-self.anchor_0.pos).get_length()
            last_node = WorldAnchor(self.anchor_1)

        self.stiffness_cf=(pi*youngs_modulus)/self.rope_length
        mass = density*pi*self.rope_length

        if self.is_flexible == True:
            self.nodes_count=ceil(self.rope_length/SECTOR_LENGTH)-1
            self.node_mass=mass/self.nodes_count
            self.sector_len=SECTOR_LENGTH
        else:
            self.nodes_count=0
            self.node_mass=mass
            self.sector_len=self.rope_length
        self.damping_cf = 2*sqrt(self.stiffness_cf*self.node_mass/2) * damping_ratio
        self.endpoints.append(BodyAnchor(self.anchor_0))
        for N in range(0, self.nodes_count):
            node_pos = self.anchor_0.pos+self.rope_normal * (self.sector_len*(N+1))
            node = JointNode(node_pos, self.node_mass, friction_cf=self.friction_cf)

            self.endpoints.append(node)

        self.endpoints.append(last_node)
        for N in range(0, (len(self.endpoints)-1)):
            node_a = self.endpoints[N]
            node_b = self.endpoints[N+1]

            sector_len=(node_b.get_pos()-node_a.get_pos()).get_length()
            stiffness_cf = self.stiffness_cf
            damping_cf = self.damping_cf

            sector = JointSector(node_a, node_b, stiffness_cf, sector_len, damping_cf, self.force_limit)
            self.sectors.append(sector)

    def get_fields(self):
        return JointInitFields(**{k: v for k, v in self.__dict__.items() if k in JointInitFields.__dataclass_fields__})

    def calculate(self):
        for sector in self.sectors:
            sector.calculate()
            if sector.to_destroy == True:
                self.destroy_joint()
    
    def destroy_joint(self):
        self.to_destroy=True

    def get_lines_to_draw(self) -> List[List[Vector]]:
        sectors = []
        for sector in self.sectors:
            start_point = sector.node_a.get_pos()
            end_point = sector.node_b.get_pos()
        
            vector : Vector= end_point-start_point
            sectors.append(
                [start_point, vector]
            )
        return sectors

    

    



        