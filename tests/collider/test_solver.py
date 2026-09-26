from .test_colliders import bodies
from physics.utils import Vector
from physics.bodies import PhysicalObject
from physics.solver import CollisionCalculator
from . import box_cfg, rubber_ball

import pytest
from pygame.sprite import Group



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

@pytest.fixture
def bodies(rectangular_body, circle_body):
    rect_0 = PhysicalObject(rectangular_body.get_fields())
    rect_1 = PhysicalObject(rectangular_body.get_fields())

    circle_0 = PhysicalObject(circle_body.get_fields())
    circle_1 = PhysicalObject(circle_body.get_fields())

    rect_0.update(0)
    rect_1.update(0)
    circle_0.update(0)
    circle_1.update(0)

    collision_solver = CollisionCalculator(Group([rect_0, rect_1, circle_0, circle_1]),[],[])

    return rect_0, rect_1, circle_0, circle_1, collision_solver

def do_step(*bodies):
    for body in bodies:
        body.update(0)
        body.velocity=Vector(0,0)
        body.angular_velocity=0

def test_solver_force_apply(bodies):
    rect_0, rect_1, circle_0, circle_1, collision_solver = bodies
    rect_0.pos=Vector(0, 500)
    rect_1.pos=Vector(99,450)

    circle_0.pos= Vector(99, 649)
    circle_1.pos= Vector(0, 649)


    do_step(rect_0, rect_1, circle_0, circle_1)
    collision_solver.calculate_collisions_penalty()
    assert rect_0.resultant_force != Vector(0,0)
    assert rect_0.resultant_torque != 0
    assert rect_1.resultant_force != Vector(0,0)
    assert rect_1.resultant_torque != 0

    assert circle_0.resultant_force != Vector(0,0)
    assert circle_0.resultant_torque == 0
    assert circle_1.resultant_force != Vector(0,0)
    assert circle_1.resultant_torque == 0

def test_solver_central_case(bodies):
    rect_0, rect_1, circle_0, circle_1, collision_solver = bodies
    rect_0.pos=Vector(0, 500)
    rect_1.pos=Vector(99,500)

    circle_0.pos= Vector(99, 649)
    circle_1.pos= Vector(0, 649)

    do_step(rect_0, rect_1, circle_0, circle_1)
    collision_solver = CollisionCalculator(Group([rect_0, rect_1]), [], [])
    collision_solver.calculate_collisions_penalty()

    collision_solver = CollisionCalculator(Group([circle_0, circle_1]), [], [])
    collision_solver.calculate_collisions_penalty()

    assert rect_0.resultant_force != Vector(pytest.approx(0),pytest.approx(0))
    assert rect_0.resultant_torque == pytest.approx(0)
    assert rect_1.resultant_force != Vector(pytest.approx(0),pytest.approx(0))
    assert rect_1.resultant_torque == pytest.approx(0)

    assert circle_0.resultant_force != Vector(pytest.approx(0),pytest.approx(0))
    assert circle_0.resultant_torque == pytest.approx(0)
    assert circle_1.resultant_force != Vector(pytest.approx(0),pytest.approx(0))
    assert circle_1.resultant_torque == pytest.approx(0)





    
    