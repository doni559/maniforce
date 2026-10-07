from physics.utils import Vector, EulerIntegrator
from physics.bodies import PhysicalObject

from configs.scenes_manager import Scene

from . import box_cfg, rubber_ball

import pytest



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
        start_velocity = Vector(0, 0),
        start_angle=90,
        start_angular_velocity = 0,
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
        start_velocity= Vector(0, 0),
        start_angle = 90,
        start_angular_velocity=0
    )

@pytest.fixture
def bodies(rectangular_body, circle_body):
    rect_0 = PhysicalObject(rectangular_body.get_fields())
    rect_1 = PhysicalObject(rectangular_body.get_fields())

    circle_0 = PhysicalObject(circle_body.get_fields())
    circle_1 = PhysicalObject(circle_body.get_fields())

    scene = Scene(load=False, name="tests", used_integrator=EulerIntegrator(), objects=[rect_0,rect_1,circle_0,circle_1])
    scene.step(0)

    return *scene.objects, scene


def test_solver_force_apply(bodies):
    rect_0, rect_1, circle_0, circle_1, scene = bodies

    rect_0.pos=Vector(0, 500)
    rect_1.pos=Vector(99,500)

    circle_0.pos= Vector(99, 1649)
    circle_1.pos= Vector(0, 1649)

    scene.step(0)

    assert rect_0.resultant_force != Vector(0,0)
    assert rect_0.resultant_torque == 0
    assert rect_1.resultant_force != Vector(0,0)
    assert rect_1.resultant_torque == 0

    assert circle_0.resultant_force != Vector(0,0)
    assert circle_0.resultant_torque == 0
    assert circle_1.resultant_force != Vector(0,0)
    assert circle_1.resultant_torque == 0

def test_solver_central_case(bodies):
    rect_0, rect_1, circle_0, circle_1, scene = bodies

    rect_0.pos=Vector(0, 500)
    rect_1.pos=Vector(99,500)

    circle_0.pos= Vector(99, 1649)
    circle_1.pos= Vector(0, 1649)

    scene.step(0)

    assert rect_0.resultant_force != Vector(pytest.approx(0),pytest.approx(0))
    assert rect_0.resultant_torque == pytest.approx(0)
    assert rect_1.resultant_force != Vector(pytest.approx(0),pytest.approx(0))
    assert rect_1.resultant_torque == pytest.approx(0)

    assert circle_0.resultant_force != Vector(pytest.approx(0),pytest.approx(0))
    assert circle_0.resultant_torque == pytest.approx(0)
    assert circle_1.resultant_force != Vector(pytest.approx(0),pytest.approx(0))
    assert circle_1.resultant_torque == pytest.approx(0)





    
    