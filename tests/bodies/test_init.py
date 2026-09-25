from physics.bodies import PhysicalObject, PhysicalObjectInitFields
from physics.utils import Vector

import pytest
from math import radians, pi

from . import rubber_ball, box_cfg, rectangular_body, circle_body


def test_init_polygon(rectangular_body):
    rectangle = rectangular_body
    init_fields = PhysicalObjectInitFields(
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

    rectangle_through_init_fields=PhysicalObject(init_fields)

    init_fields_from_rectangle=rectangle.get_fields()

    assert rectangle.config == box_cfg == rectangle_through_init_fields.config
    assert rectangle.relative_corners== [
            Vector(-100, -50),
            Vector(100, -50),
            Vector(100, 50),
            Vector(-100, 50)
    ]
    assert rectangle.pos == Vector(100, 500) == rectangle_through_init_fields.pos
    assert rectangle.velocity == Vector(100, -500) == rectangle_through_init_fields.velocity
    assert rectangle.angle == radians(90) == rectangle_through_init_fields.angle
    assert rectangle.angular_velocity == radians(-90) == rectangle_through_init_fields.angular_velocity
    assert rectangle.collider_type == rectangle.collider.type == "Polygon" == rectangle_through_init_fields.collider_type
    assert rectangle.name == "Rectangle0" == rectangle_through_init_fields.name
    with pytest.raises(AttributeError):
        rectangle.radius

    assert init_fields.config == init_fields_from_rectangle.config
    assert init_fields.corners == init_fields_from_rectangle.corners
    assert init_fields.start_pos == init_fields_from_rectangle.start_pos
    assert init_fields.start_velocity == init_fields_from_rectangle.start_velocity
    assert init_fields.start_angle == init_fields_from_rectangle.start_angle
    assert init_fields.start_angular_velocity == init_fields_from_rectangle.start_angular_velocity
    assert init_fields.collider_type == init_fields_from_rectangle.collider_type
    assert init_fields.name == init_fields_from_rectangle.name

def test_init_circle(circle_body):
    circle=circle_body
    init_fields = PhysicalObjectInitFields(
        name = "Circle0",
        config = rubber_ball,
        collider_type = "Circle",
        radius = 50,
        start_pos = Vector(100,-500),
        start_velocity= Vector(-500, 100),
        start_angle = 90,
        start_angular_velocity=-90
    )
    
    circle_through_init_fields=PhysicalObject(init_fields)
    
    init_fields_from_circle=circle.get_fields()
    
    assert circle.config == rubber_ball == circle_through_init_fields.config
    assert circle.pos == Vector(100, -500) == circle_through_init_fields.pos
    assert circle.velocity == Vector(-500, 100) == circle_through_init_fields.velocity
    assert circle.angle == radians(90) == circle_through_init_fields.angle
    assert circle.angular_velocity == radians(-90) == circle_through_init_fields.angular_velocity
    assert circle.collider_type == circle.collider.type == "Circle" == circle_through_init_fields.collider_type
    assert circle.name == "Circle0" == circle_through_init_fields.name
    assert circle.radius == 50
    assert circle.relative_corners is None

    assert init_fields.config == init_fields_from_circle.config
    assert init_fields.radius == init_fields_from_circle.radius
    assert init_fields.start_pos == init_fields_from_circle.start_pos
    assert init_fields.start_velocity == init_fields_from_circle.start_velocity
    assert init_fields.start_angle == init_fields_from_circle.start_angle
    assert init_fields.start_angular_velocity == init_fields_from_circle.start_angular_velocity
    assert init_fields.collider_type == init_fields_from_circle.collider_type
    assert init_fields.name == init_fields_from_circle.name

def test_mass_calculation(circle_body, rectangular_body):
    expected_circle_volume = 0.03623*pi
    expected_circle_mass =  0.03623*pi
    expected_circle_moment_of_inertia= 142.125

    expected_rect_volume=0.02
    expected_rect_mass=0.02
    expected_rect_moment_of_inertia=83.33
    
    init_fields = rectangular_body.get_fields()
    init_fields.__dict__.update(corners=[
        Vector(0, 0),
        Vector(200, 0),
        Vector(200, 100),
        Vector(0,100)
    ])
    assymetric_rectangle = PhysicalObject(init_fields)

    assert circle_body.mass == pytest.approx(expected_circle_mass, rel=10**(-3))
    assert rectangular_body.mass == pytest.approx(expected_rect_mass, rel=10**(-3)) == assymetric_rectangle.mass

    assert circle_body.volume == pytest.approx(expected_circle_volume, rel=10**(-3))
    assert rectangular_body.volume == pytest.approx(expected_rect_volume, rel=10**(-3)) == assymetric_rectangle.volume

    assert circle_body.moment_of_inertia == pytest.approx(expected_circle_moment_of_inertia, rel=2*10**(-3))
    assert rectangular_body.moment_of_inertia == pytest.approx(expected_rect_moment_of_inertia, rel=2*10**(-3)) == assymetric_rectangle.moment_of_inertia

    #testing COM calculation for both assymetric and symmetric polygons + circle
    assert circle_body.pos.x == pytest.approx(100) and circle_body.pos.y == pytest.approx(-500)
    assert rectangular_body.pos.x == pytest.approx(100) ==assymetric_rectangle.pos.x and rectangular_body.pos.y == pytest.approx(500) == assymetric_rectangle.pos.y
    #testing explicitly if corners world coords is exactly equals for both assymetric and symmetric polygon
    expected_corners = [
        Vector(50, 600),
        Vector(150, 600),
        Vector(150, 400),
        Vector(50, 400)
    ]
    #getting world corners
    for polygon in [rectangular_body, assymetric_rectangle]:
        new_corners= polygon.collider.get_world_corners(polygon)
        for expected_corner in expected_corners:
            found=False

            for new_corner in new_corners:
                if expected_corner.x == pytest.approx(new_corner.x) and expected_corner.y == pytest.approx(new_corner.y):
                    found = True
            assert found 
        
        