import math
from unittest import TestCase

from vector import Vector
from angle_util import angle_normalize


class WaypointController(object):
    def __init__(self):
        self.desired_position: Vector | None = None
        self.current_position: Vector | None = None
        self.offset_angle: float = 0
        self.waypoint: Vector | None = None
        self.waypoint_list: list[Vector] = []

        self.delta_distance: float = 0
        self.delta_angle: float = 0
        self.time_counter: int = 0

        self.builder_current_angle: float = 0

    def turn_to(self, angle: int):
        self.builder_current_angle = angle_normalize(self.builder_current_angle + (angle / 1800 * math.pi))

    def go_to(self, distance_mm: int):
        distance = distance_mm / 1000
        real_angle = self.builder_current_angle - self.offset_angle
        vec = Vector(math.cos(real_angle) * distance, math.sin(real_angle) * distance)
        self.waypoint_list.append(vec)

    def tick(self, current_position: Vector, current_angle: float):
        self.current_position = current_position
        self.time_counter += 1
        if self.time_counter <= 10:
            self.desired_position = current_position
            self.offset_angle = current_angle
            return

        if self.waypoint is None and len(self.waypoint_list) > 0 and self.time_counter > 10:
            self.waypoint = self.waypoint_list.pop(0)
            self.desired_position += self.waypoint

        if self.waypoint is None:
            self.delta_distance = 0
            self.delta_angle = 0
            return

        self.delta_distance = current_position.euclidean_distance(self.desired_position)
        self.delta_angle = current_position.angle_toward(self.desired_position) - current_angle

        if self.delta_distance < 0.01:
            self.waypoint = None


class TestWaypointController(TestCase):
    def test_calibration(self):
        controller = WaypointController()
        position: Vector
        angle: float = math.pi / 2
        for i in range(0, 15):
            position = Vector(1, i)
            controller.tick(position, angle)
            print(controller.desired_position, controller.offset_angle)
        self.assertEqual(controller.desired_position.y, 9, "calibration failed")
        # 1, 9

        controller.go_to(1000)
        controller.turn_to(900)
        controller.go_to(1000)
        controller.turn_to(900)
        controller.go_to(1000)
        controller.turn_to(900)
        controller.go_to(1000)

        for i in range(0, 4):
            print("====================")
            controller.tick(controller.current_position, angle)
            print(controller.desired_position)
            print(controller.delta_distance)
            print(controller.delta_angle)
            print("--------")
            controller.current_position = controller.desired_position
            angle += controller.delta_angle
            controller.tick(controller.current_position, angle)
            print(controller.desired_position)
            print(controller.delta_distance)
            print(controller.delta_angle)

        self.assertEqual(controller.current_position, Vector(1, 9), "end position mismatch")
