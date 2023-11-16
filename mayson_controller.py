import math
import unittest

from angle_util import *
from vector import Vector
from stab_checker import StabChecker

# flak export

MC_NOP = 0x00
MC_ZERO = 0x01

MC_FWD = 0x10

MC_TURN_LEFT = 0x20
MC_TURN_RIGHT = 0x21
MC_TURN_U = 0x22


class MaysonController(object):
    MODE_POSITION = 0
    MODE_HEADING = 1

    def __init__(self):
        self.near_threshold = 0.25
        self.expected_wall_distance = 0.12
        self.grid_size = 0.28

        self.mode = MaysonController.MODE_POSITION
        self.queue = []
        self.current_inst = 0
        self.current_inst_stack = 0
        self.angular_stab_checker = StabChecker(10, 0.1)

        self.zero_origin: Vector = Vector(0, 0)
        self.zero_angle: float = 0.0

        self.actual_position: Vector = Vector(0, 0)
        self.actual_heading: float = 0

        self.inst_start_pos = Vector(0, 0)
        self.inst_start_angle = 0

        self.delta_distance: float = 0
        self.delta_angle: float = 0

    def set_zero(self, zero_origin: Vector, zero_angle: float):
        self.zero_origin = zero_origin
        self.zero_angle = zero_angle

    def normalize_position(self, ros_pos: Vector):
        morm_pos = ros_pos - self.zero_origin
        morm_pos.rotate(-self.zero_angle)
        return morm_pos

    def tick(self, ros_pos: Vector, heading: float, distance_data: list):
        self.actual_position = self.normalize_position(ros_pos)
        self.actual_heading = heading - self.zero_angle

        if self.current_inst == MC_NOP:
            if len(self.queue) == 0:
                raise Exception("no further instruction")
            self.current_inst = self.queue[0]
            self.current_inst_stack = 1
            self.queue = self.queue[1:]
            self.inst_start_pos = self.actual_position.clone()
            self.inst_start_angle = self.actual_heading
            print("next inst ", self.current_inst)

        inst_delta_dist = self.inst_start_pos.euclidean_distance(self.actual_position)
        inst_delta_angle = angle_calculate_delta(self.inst_start_angle, self.actual_heading)
        print("inst delta", inst_delta_dist, inst_delta_angle)

        distance_left = angle_distance(distance_data, 90 - 5, 90 + 5)
        distance_right = angle_distance(distance_data, 270 - 5, 270 + 5)

        if self.current_inst == MC_ZERO:
            self.set_zero(ros_pos, heading)
            self.current_inst = MC_NOP

        elif self.current_inst == MC_FWD:
            if len(self.queue) > 0 and self.queue[0] == MC_FWD:
                self.queue = self.queue[1:]
                self.current_inst_stack += 1

            target_distance = self.grid_size * self.current_inst_stack
            if inst_delta_dist >= target_distance:
                self.delta_distance = 0
                self.delta_angle = 0
                self.current_inst = MC_NOP
            else:
                self.delta_distance = target_distance - inst_delta_dist
                self.delta_angle = 0

                if distance_left < self.near_threshold:
                    nearest_left = angle_nearest_range_relative(distance_data, 77, 15)
                    distance_angle_offset = (distance_left - self.expected_wall_distance) * 0.7
                    self.delta_angle = (nearest_left + distance_angle_offset) / 180 * math.pi
                    print("fwd turn", nearest_left, distance_left)

                elif distance_right < self.near_threshold:
                    nearest_right = angle_nearest_range_relative(distance_data, 257, 15)
                    distance_angle_offset = (distance_left - self.expected_wall_distance) * 0.7
                    self.delta_angle = (nearest_right + distance_angle_offset) / 180 * math.pi
                    print("fwd turn", nearest_right, distance_left)



        elif self.current_inst == MC_TURN_LEFT:
            self.delta_distance = 0.0
            self.delta_angle = angle_calculate_delta(inst_delta_angle, math.pi / 2)
            if self.angular_stab_checker.tick(self.delta_angle):
                self.current_inst = MC_NOP

        elif self.current_inst == MC_TURN_RIGHT:
            self.delta_distance = 0.0
            self.delta_angle = angle_calculate_delta(inst_delta_angle, -math.pi / 2)
            if self.angular_stab_checker.tick(self.delta_angle):
                self.current_inst = MC_NOP

        elif self.current_inst == MC_TURN_U:
            self.delta_distance = 0.0
            self.delta_angle = angle_calculate_delta(inst_delta_angle, math.pi)
            if self.angular_stab_checker.tick(self.delta_angle):
                self.current_inst = MC_NOP

    def enqueue(self, instruction: int):
        self.queue.append(instruction)


# flak noexport
class TestController(unittest.TestCase):
    def test(self):
        controller = MaysonController()
        controller.set_zero(Vector(1, 1), math.pi / 4)
        transform = controller.normalize_position(Vector(1, 3))
        print(transform)
