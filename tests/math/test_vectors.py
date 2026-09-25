from physics.utils import Vector
from math import radians, sqrt
import pytest

@pytest.fixture
def vectors():
    a = Vector(1, 5)
    b = Vector(-2, 9)
    zero = Vector(0, 0)

    return a, b, zero


def test_basic_operands(vectors):
    a,b,zero =vectors
    c = Vector(3,6)

    assert a + b == Vector(-1, 14)
    assert a + b == b + a
    assert (a + b) + c == a + (b + c)

    assert a - b  == Vector(3, -4)
    assert a - b == a + (-b)

    assert a + zero == a
    assert zero + a == a
    assert a - a == zero
    assert -(-a) == a

    assert a.x == 1 and a.y == 5

def test_multiplication_and_division(vectors):
    a,b, zero = vectors
    constant_a = -5
    constant_b = 5


    assert constant_a*a == Vector(-5, -25)
    assert constant_b*a == Vector(5, 25)

    assert a/constant_a == Vector(-0.2, -1)
    assert a/constant_b == Vector(0.2, 1)

    assert a*b == b*a == a.scalar_multiply(b) == b.scalar_multiply(a)
    assert a*1 == a
    assert a*zero == 0
    assert a*0 == zero

    with pytest.raises(ZeroDivisionError):
        a/0

def test_products(vectors):
    a,b,zero = vectors

    assert a.scalar_multiply(b) == 43
    assert a.vector_multiply(b) == 19

    assert a.scalar_multiply(zero) == 0
    assert a.vector_multiply(zero) == 0

def test_length(vectors):
    a,b, zero  = vectors

    assert a.get_length() == pytest.approx(sqrt(26))
    assert a.normalise() == Vector(1/sqrt(26), 5/sqrt(26))
    assert a.normalise().get_length() == pytest.approx(1)

    assert zero.normalise() == zero

def test_rotation(vectors):
    a,b, zero= vectors

    rotated_vector = a.rotate(radians(90))
    invarinat_vector = a.rotate(radians(90)).rotate(radians(-90))

    assert rotated_vector.x == pytest.approx(-5) and rotated_vector.y == pytest.approx(1)
    assert invarinat_vector.x == pytest.approx(1) and invarinat_vector.y == pytest.approx(5)
    assert rotated_vector.get_length() == invarinat_vector.get_length() == a.get_length()
    assert rotated_vector*a == pytest.approx(0)