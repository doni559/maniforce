from physics.bodies import ObjectConfig, PhysicalObject
from physics.utils import Vector

import pytest

rubber_ball = ObjectConfig(

    stiffnes_cf=1000,
    density=1,
    friction_cf=0.6,
    restitution=0.2,
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

@pytest.fixture
def rectangular_body() -> PhysicalObject:
    return PhysicalObject(
        config=box_cfg,
        corners = [
            Vector(-100, -50),
            Vector(100, -50),
            Vector(100, 50),
            Vector(-100, 50)
        ],
        start_pos = Vector(100, 500),
        start_velocity = Vector(100, -500),
        start_angle=90,
        start_angular_velocity = -90,
        collider_type="Polygon",
        name = "Rectangle0"
    )

@pytest.fixture
def circle_body() -> PhysicalObject:
    return PhysicalObject(
        name = "Circle0",
        config = rubber_ball,
        collider_type = "Circle",
        radius = 50,
        start_pos = Vector(100,-500),
        start_velocity= Vector(-500, 100),
        start_angle = 90,
        start_angular_velocity=-90
    )