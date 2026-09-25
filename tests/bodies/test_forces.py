from . import rectangular_body
from physics.utils import Vector

import pytest


def test_apply_force_at_com(rectangular_body):
    force = Vector(100,0)
    point = Vector(100,500)

    rectangular_body.apply_force(force, point)

    assert rectangular_body.resultant_force == force
    assert rectangular_body.resultant_torque == pytest.approx(0)

    rectangular_body.clear_forces()

    assert rectangular_body.resultant_torque == 0
    assert rectangular_body.resultant_force == Vector(0,0)

    rectangular_body.apply_force(Vector(1,3), point)
    rectangular_body.apply_force(Vector(4,5), point)
    rectangular_body.apply_force(Vector(-2,3), point)

    assert rectangular_body.resultant_force == Vector(3, 11)
    assert rectangular_body.resultant_torque ==0

def test_apply_force_not_at_com(rectangular_body):
    force_1 = Vector(100,0)
    point_1=Vector(200,500)
    rectangular_body.apply_force(force_1, point_1)

    assert rectangular_body.resultant_torque == 0

def test_zero_forces_positive_torque(rectangular_body):
    force_2 = Vector(0, 100)
    force_3 = Vector(0, -100)
    point_2 = Vector(200, 500)
    point_3 = Vector(0, 500)

    rectangular_body.apply_force(force_2, point_2)
    rectangular_body.apply_force(force_3, point_3)

    assert rectangular_body.resultant_force == Vector(0,0)
    assert rectangular_body.resultant_torque == pytest.approx(20000)

def test_zero_forces_negative_torque(rectangular_body):
    force_1 = Vector(0, -100)
    force_2 = Vector(0, 100)
    point_1 = Vector(200, 500)
    point_2 = Vector(0, 500)

    rectangular_body.apply_force(force_1, point_1)
    rectangular_body.apply_force(force_2, point_2)

    assert rectangular_body.resultant_force == Vector(0,0)
    assert rectangular_body.resultant_torque == pytest.approx(-20000)

def test_invariant(rectangular_body):
    force_1 = Vector(0, -100)
    point_1 = Vector(200, 500)

    rectangular_body.apply_force(force_1, point_1)
    rectangular_body.apply_force(-force_1, point_1)
    
    assert rectangular_body.resultant_force == Vector(0,0)
    assert rectangular_body.resultant_torque == 0



