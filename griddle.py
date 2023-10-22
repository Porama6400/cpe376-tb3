import unittest

from angle_util import angle_distance
from mayson_controller import MaysonController


# flak export

class Griddle(object):
    def __init__(self, controller: MaysonController):
        self.controller: MaysonController = controller
        self.x: int = 0
        self.y: int = 0
        self.heading: int = 0
        self.near_threshold = 0.2

    def turn_absolute(self, angle: int):
        self.heading = angle % 4

    def turn_relative(self, angle: int):
        self.heading = (self.heading + angle) % 4

    def align(self, sensor: list):
        distance_front = angle_distance(sensor, -15, 15)
        distance_left = angle_distance(sensor, 75, 105)
        distance_back = angle_distance(sensor, 165, 195)
        distance_right = angle_distance(sensor, 255, 285)

        near_front = distance_front < self.near_threshold
        near_left = distance_left < self.near_threshold
        near_back = distance_back < self.near_threshold
        near_right = distance_right < self.near_threshold



    def move_forward(self, distance: int):
        self.heading %= 4
        if self.heading == 0:
            self.x += distance
        elif self.heading == 1:
            self.y += distance
        elif self.heading == 2:
            self.x -= distance
        elif self.heading == 3:
            self.y -= distance


# flak noexport

class TestGriddle(unittest.TestCase):

    def test_normalize_angle(self):
        print(Griddle.normalize_angle(0))


if __name__ == '__main__':
    unittest.main()
