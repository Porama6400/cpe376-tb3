import math
import unittest

from vector import Vector


class MaysonController(object):
    def __init__(self):
        self.zero_origin: Vector = Vector(0, 0)
        self.zero_angle: float = 0.0

        self.actual_position: Vector = Vector(0, 0)
        self.actual_heading: float = 0

        self.target_position: Vector = Vector(0, 0)

    def tare(self, zero_origin: Vector, zero_angle: float):
        self.zero_origin = zero_origin
        self.zero_angle = zero_angle

    def normalize_position(self, ros_pos: Vector):
        morm_pos = ros_pos - self.zero_origin
        morm_pos.rotate(-self.zero_angle)
        return morm_pos

    def update_current_position(self, ros_pos: Vector, heading: float):
        self.actual_position = self.normalize_position(ros_pos)
        self.actual_heading = heading - self.zero_angle

    def tick(self):
        delta = self.target_position - self.actual_position


class TestController(unittest.TestCase):
    def test(self):
        controller = MaysonController()
        controller.tare(Vector(1, 1), math.pi / 4)
        transform = controller.normalize_position(Vector(1, 3))
        print(transform)
