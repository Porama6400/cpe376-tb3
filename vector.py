import math
from unittest import TestCase


class Vector(object):
    PRECISION = 0.001

    def __init__(self, x: float, y: float):
        self.x = x
        self.y = y

    def __sub__(self, other):
        return Vector(self.x - other.x, self.y - other.y)

    def __add__(self, other):
        return Vector(self.x + other.x, self.y + other.y)

    def __mul__(self, other: float):
        return Vector(self.x * other, self.y * other)

    def __eq__(self, other):
        return self.manhattan_distance(other) < Vector.PRECISION

    def __str__(self):
        return str(self.x) + "," + str(self.y)

    def manhattan_distance(self, other):
        return abs(self.x - other.x) + abs(self.y - other.y)

    def euclidean_distance(self, other):
        return math.sqrt(math.pow(other.x - self.x, 2) + math.pow(other.y - self.y, 2))

    def angle_toward(self, other):
        return math.atan2(other.y - self.y, other.x - self.x)

    def rotate(self, angle: float):
        x_new = self.x * math.cos(angle) - self.y * math.sin(angle)
        y_new = self.x * math.sin(angle) + self.y * math.cos(angle)
        self.x = x_new
        self.y = y_new

class TestPosition(TestCase):
    def test_add(self):
        added = Vector(1, 3) + Vector(5, 8)
        self.assertTrue(added == Vector(6, 11))

    def test_sub(self):
        subtracted = Vector(1, 3) - Vector(5, 8)
        self.assertTrue(subtracted == Vector(-4, -5))

    def test_sub(self):
        vector = Vector(10, 0)
        vector.pivot_origin(Vector(9, 0), math.pi)
        print(vector)
