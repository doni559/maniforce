from ..bodies import rectangular_body, circle_body
from physics.bodies import PhysicalObject
from physics.utils import Vector

import pytest
from typing import List

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


    return rect_0, rect_1, circle_0, circle_1

def do_collisions(rect_0 : PhysicalObject, rect_1: PhysicalObject, circle_0:PhysicalObject, circle_1: PhysicalObject):
    rect_0.update(0)
    rect_1.update(0)
    circle_0.update(0)
    circle_1.update(0)

    collision_0 = rect_0.collider.calculate_deformation(rect_1.collider, rect_0.pos, rect_1.pos)
    _collision_0 = rect_1.collider.calculate_deformation(rect_0.collider, rect_1.pos, rect_0.pos)
    collision_1 = rect_0.collider.calculate_deformation(circle_1.collider, rect_0.pos, circle_1.pos)
    _collision_1 = circle_1.collider.calculate_deformation(rect_0.collider, circle_1.pos, rect_0.pos)
    collision_2 = circle_0.collider.calculate_deformation(circle_1.collider, circle_0.pos, circle_1.pos)
    _collision_2 = circle_1.collider.calculate_deformation(circle_0.collider, circle_1.pos, circle_0.pos)

    return collision_0, collision_1, collision_2, _collision_0, _collision_1, _collision_2

def get_contact_manifolds(normal: Vector,contact_points_0 :List[Vector],contact_points_1 : List[Vector] , deformations: List[float]):
    return_array_0=[]
    return_array_1=[]
    for contact_point, deformation in zip(contact_points_0, deformations):
        return_array_0.append(
            {
            "pos": Vector(pytest.approx(contact_point.x), pytest.approx(contact_point.y)),
            "deformation": pytest.approx(deformation)
            })
    for contact_point, deformation in zip(contact_points_1, deformations):
        return_array_1.append(
            {
            "pos": Vector(pytest.approx(contact_point.x), pytest.approx(contact_point.y)),
            "deformation": pytest.approx(deformation)
            })
    return (Vector(pytest.approx(normal.x), pytest.approx(normal.y)),return_array_0), (Vector(pytest.approx(-normal.x), pytest.approx(-normal.y)),return_array_1),


def test_no_collsion(bodies):
    rect_0, rect_1, circle_0, circle_1 = bodies
    none_set = (Vector(0,0), [])
    rect_0.pos=Vector(0, 500)
    rect_1.pos=Vector(300,500)

    circle_0.pos= Vector(200, 500)
    circle_1.pos= Vector(400, 500)


    collision_0, collision_1, collision_2, _collision_0, _collision_1, _collision_2 = do_collisions(rect_0, rect_1, circle_0, circle_1)
    
    
    assert collision_0 == _collision_0 == none_set
    assert collision_1 == _collision_1 == none_set
    assert collision_2 == _collision_2 == none_set

def test_edge_collision(bodies):
    rect_0, rect_1, circle_0, circle_1 = bodies
    none_set = (Vector(0,0), [])

    rect_0.pos=Vector(0, 500)
    rect_1.pos=Vector(100,500)

    circle_0.pos= Vector(100, 650)
    circle_1.pos= Vector(0, 650)


    collision_0, collision_1, collision_2, _collision_0, _collision_1, _collision_2 = do_collisions(rect_0, rect_1, circle_0, circle_1)


    assert collision_0 == _collision_0 == none_set
    assert collision_1 == _collision_1 == none_set
    assert collision_2 == _collision_2 == none_set

def test_collision(bodies):
    rect_0, rect_1, circle_0, circle_1 = bodies
    none_set = (Vector(0,0), [])

    rect_0.pos=Vector(0, 500)
    rect_1.pos=Vector(99,500)

    circle_0.pos= Vector(99, 649)
    circle_1.pos= Vector(0, 649)


    collision_0, collision_1, collision_2, _collision_0, _collision_1, _collision_2 = do_collisions(rect_0, rect_1, circle_0, circle_1)

    collision_0_manifold, _collision_0_manifold = get_contact_manifolds(Vector(-1, 0), [Vector(50, 600), Vector(50, 400)], [Vector(49,400), Vector(49, 600)], [1, 1])
    collision_1_manifold, _collision_1_manifold = get_contact_manifolds(Vector(0, -1), [Vector(0, 600)], [Vector(0, 599)], [1])
    collision_2_manifold, _collision_2_manifold = get_contact_manifolds(Vector(1, 0), [Vector(49, 649)], [Vector(50, 649)], [1])

    assert collision_0[0] == collision_0_manifold[0]
    for actual in collision_0[1]:
        assert any(expected == actual for expected in collision_0_manifold[1])

    assert _collision_0[0] == _collision_0_manifold[0]
    for actual in _collision_0[1]:
        assert any(expected == actual for expected in _collision_0_manifold[1])

    assert collision_1[0] == collision_1_manifold[0]
    for actual in collision_1[1]:
        assert any(expected == actual for expected in collision_1_manifold[1])

    assert _collision_1[0] == _collision_1_manifold[0]
    for actual in _collision_1[1]:
        assert any(expected == actual for expected in _collision_1_manifold[1])
    print(_collision_2_manifold, _collision_2)

    assert collision_2[0] == collision_2_manifold[0]
    for actual in collision_2[1]:
        assert any(expected == actual for expected in collision_2_manifold[1])

    assert _collision_2[0] == _collision_2_manifold[0]
    for actual in _collision_2[1]:
        assert any(expected == actual for expected in _collision_2_manifold[1])
