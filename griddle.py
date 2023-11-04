import math
import unittest

from angle_util import angle_distance, angle_correction_min
from mayson_controller import MaysonController
from vector import Vector
from stab_checker import StabChecker

# flak export

GRIDDLE_NOP = 0x00
GRIDDLE_ALIGN = 0x01

GRIDDLE_L = 0x10
GRIDDLE_R = 0x13

GRIDDLE_F1 = 0x20
GRIDDLE_F2 = 0x21
GRIDDLE_F3 = 0x22
GRIDDLE_F4 = 0x23
GRIDDLE_F5 = 0x24


class Griddle(object):
    def __init__(self, controller: MaysonController):
        self.controller: MaysonController = controller
        self.position: Vector = Vector(0, 0)
        self.heading: int = 0
        self.near_threshold = 0.2
        self.expected_wall_distance = 0.15
        self.grid_size = 0.3

        self.current_instruction: int = 0
        self.queue: list = []

    def enqueue(self, inst: int):
        self.queue.append(inst)

    def tick(self, ros_pos: Vector, ros_angle: float, sensor: list):
        if self.current_instruction == GRIDDLE_NOP or (self.controller.calculate_target_distance() < 0.01):
            self.current_instruction = self.queue.pop()
            self.align(ros_pos, ros_angle, sensor)

            if self.current_instruction == GRIDDLE_F1:
                self.controller.set_target(Vector(self.grid_size, 0))

    def align(self, ros_pos: Vector, ros_angle: float, sensor: list):
        distance_front = angle_distance(sensor, 0 - 30, 0 + 30)
        distance_left = angle_distance(sensor, 90 - 30, 90 + 30)
        distance_right = angle_distance(sensor, 270 - 30, 270 + 30)

        print("lfr", distance_left, distance_front, distance_right)

        angle_offset = 0.0
        angle_offset_counter = 0

        pos_offset = Vector(0.0, 0.0)

        near_front = distance_front < self.near_threshold
        near_left = distance_left < self.near_threshold
        near_right = distance_right < self.near_threshold

        if not near_front and not near_left and not near_right:
            self.controller.set_zero(ros_pos, ros_angle)
            return

        if near_front:
            angle_offset += angle_correction_min(sensor, 0, 30) / 180 * math.pi
            pos_offset.x += distance_front - self.expected_wall_distance
            angle_offset_counter += 1
            print("align front")
        if near_left:
            angle_offset += angle_correction_min(sensor, 90, 30) / 180 * math.pi
            pos_offset.y += distance_front - self.expected_wall_distance
            angle_offset_counter += 1
            print("align left")
        if near_right:
            angle_offset += angle_correction_min(sensor, 270, 30) / 180 * math.pi
            pos_offset.y -= distance_front - self.expected_wall_distance
            angle_offset_counter += 1
            print("align right")

        if near_left and near_right:
            pos_offset.y /= 2.0  # this will average out in the case if both side has a wall
        if angle_offset_counter > 0:
            angle_offset /= angle_offset_counter

        print("align linear offset", pos_offset)
        print("align angular offset", angle_offset)
        pos_offset.rotate(ros_angle)

        self.controller.set_zero(ros_pos + pos_offset, ros_angle + angle_offset)

        print("align completed")


# flak noexport

class TestGriddle(unittest.TestCase):

    def test_normalize_angle(self):
        print(Griddle.normalize_angle(0))


if __name__ == '__main__':
    unittest.main()
