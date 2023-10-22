import math
import unittest

from angle_util import angle_calculate_delta
from vector import Vector


# flak export


class MaysonController(object):
    MODE_POSITION = 0
    MODE_HEADING = 1

    def __init__(self):
        self.mode = MaysonController.MODE_POSITION

        self.zero_origin: Vector = Vector(0, 0)
        self.zero_angle: float = 0.0

        self.actual_position: Vector = Vector(0, 0)
        self.actual_heading: float = 0

        self.target_position: Vector = Vector(0, 0)

        self.delta_distance: Vector
        self.delta_angle: float

    def set_zero(self, zero_origin: Vector, zero_angle: float):
        self.zero_origin = zero_origin
        self.zero_angle = zero_angle

    def normalize_position(self, ros_pos: Vector):
        morm_pos = ros_pos - self.zero_origin
        morm_pos.rotate(-self.zero_angle)
        return morm_pos

    def update_current_position(self, ros_pos: Vector, heading: float):
        self.actual_position = self.normalize_position(ros_pos)
        self.actual_heading = heading - self.zero_angle

    def target_distance(self):
        return self.actual_position.euclidean_distance(self.target_position)

    def tick(self):
        self.delta_distance = self.actual_position.euclidean_distance(self.target_position)
        self.target_angle = self.actual_position.angle_toward(self.target_position)
        self.delta_angle = angle_calculate_delta(self.actual_heading, self.target_angle)

        if abs(self.delta_angle) > 0.2 or self.mode == MaysonController.MODE_HEADING:
            self.delta_distance = 0.0

    def set_target(self, target):
        self.target_position = target


# flak noexport
class TestController(unittest.TestCase):
    def test(self):
        controller = MaysonController()
        controller.set_zero(Vector(1, 1), math.pi / 4)
        transform = controller.normalize_position(Vector(1, 3))
        print(transform)
