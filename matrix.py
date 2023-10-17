import math
from unittest import TestCase


class Matrix(object):
    PRECISION = 0.001

    def __init__(self, height: int, width: int):
        self.height = height
        self.width = width
        self.data = [0.0] * height * width

    def get(self, y: int, x: int):
        if x < 0 or y < 0 or x >= self.width or y >= self.height:
            raise Exception("Invalid position ", x, y, " in matrix ", self.width, self.height)

        return self.data[y * self.width + x]

    def set(self, y: int, x: int, value: float):
        if x < 0 or y < 0 or x >= self.width or y >= self.height:
            raise Exception("Invalid position ", x, y, " in matrix ", self.width, self.height)

        self.data[(y * self.width) + x] = value

    def clone(self):
        cloned = Matrix(self.width, self.height)
        for i in range(0, self.width, self.height):
            cloned.data[i] = self.data[i]
        return cloned

    def add(self, other):
        if self.width != other.width or self.height != other.height:
            raise Exception("adding different sized matrix")
        for i in range(0, self.width * self.height):
            self.data[i] += other.data[i]

    def print(self):
        print("=== MATRIX ", self.height, "x", self.width, "===")
        for row in range(0, self.height):
            print(self.data[row * self.width:(row + 1) * self.width])

    def __eq__(self, other):
        if self.width != other.width or self.height != other.height:
            return False

        for i in range(0, self.width * self.height):
            if self.data[i] < other.data[i] - Matrix.PRECISION:
                return False

            if self.data[i] > other.data[i] + Matrix.PRECISION:
                return False

        return True


class TestPosition(TestCase):
    def test_add(self):
        matrix = Matrix(2, 2)
        matrix.set(0, 0, 1.23)
        matrix.set(1, 0, 23.4)
        matrix.set(0, 1, 3.45)
        matrix.set(1, 1, 45.6)
        matrix.print()
