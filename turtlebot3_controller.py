# Authors : Nopphakorn Subs. Niwatchai Wang. Supasate Wor. Narith Tha. Tawan Thaep. Napatharak Muan.
from dis import dis
from socket import TIPC_SUBSCR_TIMEOUT
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import LaserScan, BatteryState
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
# from std_msgs.msg import String

import math


class Turtlebot3Controller(Node):

    def __init__(self):
        super().__init__('turtlebot3_controller')  # node name
        self.cmdVelPublisher = self.create_publisher(Twist, 'cmd_vel', 1)
        self.scanSubscriber = self.create_subscription(LaserScan, 'scan', self.scanCallback,
                                                       qos_profile=qos_profile_sensor_data)
        self.batteryStateSubscriber = self.create_subscription(BatteryState, 'battery_state', self.batteryStateCallback,
                                                               1)
        self.odomSubscriber = self.create_subscription(Odometry, 'odom', self.odomCallback, 1)

        self.valueLaserRanges = [0.0] * 360
        self.valueBatteryState = None
        self.valuePosition = None
        self.valueOrientation = None
        self.valueRotation = 0

        self.state = 0
        # Use this timer for the job that should be looping until interrupted
        self.timer = self.create_timer(0.1, self.timerCallback)

        self.pid_linear = PidController()
        self.pid_linear.output_cap = 0.1
        self.pid_linear.proportional_gain = 0.8
        self.pid_linear.integral_gain = 0.5
        self.pid_linear.integral_cap = 0.03
        self.pid_linear.integral_time = 8

        self.pid_angular = PidController()
        self.pid_angular.proportional_gain = 1
        self.pid_angular.integral_gain = 0.5
        self.pid_angular.integral_time = 8
        self.pid_linear.integral_cap = 0.1
        self.pid_angular.output_cap = 2

        self.controller = MaysonController()
        self.tick_counter = 0

        self.array_average = ArrayAverage()
        self.griddle = Griddle(self.controller)
        self.griddle.enqueue(GRIDDLE_F1)
        self.griddle.enqueue(GRIDDLE_F1)

        self.sensor_angle_offset = -18

    def publishVelocityCommand(self, linearVelocity, angularVelocity):
        msg = Twist()
        msg.linear.x = linearVelocity
        msg.angular.z = angularVelocity
        self.cmdVelPublisher.publish(msg)
        # self.get_logger().info('Publishing cmd_vel: "%s", "%s"' % linearVelocity, angularVelocity)

    def scanCallback(self, msg):
        self.valueLaserRanges = list(msg.ranges)

    def batteryStateCallback(self, msg):
        self.valueBatteryState = msg

    def odomCallback(self, msg):
        self.valuePosition = msg.pose.pose.position
        self.valueOrientation = msg.pose.pose.orientation

        siny_cosp = 2 * (
                self.valueOrientation.w * self.valueOrientation.z + self.valueOrientation.x * self.valueOrientation.y
        )
        cosy_cosp = 1 - 2 * (
                self.valueOrientation.y * self.valueOrientation.y + self.valueOrientation.z * self.valueOrientation.z
        )
        self.valueRotation = math.atan2(siny_cosp, cosy_cosp)

    def timerCallback(self):
        self.tick_counter += 1
        ros_pos = Vector(self.valuePosition.x, self.valuePosition.y)
        self.controller.update_current_position(ros_pos, self.valueRotation)
        self.controller.tick()


        print("tick")
        self.griddle.tick(ros_pos, self.valueRotation, self.valueLaserRanges)
        self.controller.tick()
        speed = self.pid_linear.tick(self.controller.delta_distance)
        angle = self.pid_angular.tick(self.controller.delta_angle)

        print("==========")
        print(self.controller.actual_position, self.controller.actual_heading)
        print(self.controller.target_position)
        print(speed, angle)
        self.publishVelocityCommand(float(speed), float(angle))


def robotStop():
    node = rclpy.create_node('tb3Stop')
    publisher = node.create_publisher(Twist, 'cmd_vel', 1)
    msg = Twist()
    msg.linear.x = 0.0
    msg.angular.z = 0.0
    publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    tb3ControllerNode = Turtlebot3Controller()
    print('tb3ControllerNode created')
    try:
        rclpy.spin(tb3ControllerNode)
    except KeyboardInterrupt as ki:
        print("KI")
    except Exception as ex:
        print(ex)
    print('Done')

    tb3ControllerNode.publishVelocityCommand(0.0, 0.0)
    tb3ControllerNode.destroy_node()
    robotStop()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

# flak: vector.py
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


# flak end: vector.py
# flak: angle_util.py

PI2 = math.pi * 2


def angle_normalize(rad: float) -> float:
    while rad < 0:
        rad += PI2
    while rad > PI2:
        rad -= PI2
    return rad


def angle_calculate_delta(rad_a: float, rad_b: float):
    delta = rad_b - rad_a
    if abs(delta) <= math.pi:
        return delta
    elif rad_a < rad_b:
        return rad_b - (rad_a + PI2)
    else:
        return (rad_b + PI2) - rad_a


def angle_nearest_range(data: list, deg_a: int, deg_b: int) -> int:
    min_value = 1000000
    min_index = -1
    for i in range(deg_a, deg_b):
        val = data[i]
        if 0 < val < min_value:
            min_value = val
            min_index = i

    return min_index


def angle_distance(data: list, a: int, b: int) -> float:
    last = 1000000
    r = range(a, b)
    for index in r:
        data_len = len(data)
        if index < 0:
            index += data_len
        if last > data[index] > 0.01:
            last = data[index]
    return last


def angle_correction_min(data: list, center: int, margin: int):
    min_value = 1000000
    min_index = -1
    for index in range(center - margin, center + margin):
        list_index = index
        if list_index < 0:
            list_index += len(data)
        value = data[list_index]
        if value < 0.01:
            continue
        if value > 100000:
            continue
        if value < min_value:
            min_value = value
            min_index = index
    return min_index - center


# flak end: angle_util.py
# flak: pid_controller.py
class PidController(object):
    def __init__(self):
        self.proportional_gain: float = 0.5
        self.integral_gain: float = 0.3
        self.integral_time: int = 10
        self.derivative_gain: float = 0
        self.derivative_time: int = 15

        self.output_cap: float = 0.1
        self.output_multi: float = 1
        self.time_delta: float = 0.1

        self.history_array: list = []

        self.last_input: float = 0
        self.last_output: float = 0

    def reset(self):
        self.history_array = [0] * max(self.derivative_time, self.integral_time)

    def calculate_integral(self) -> float:
        if self.integral_gain == 0:
            return 0

        if len(self.history_array) < self.integral_time:
            return 0

        accu = 0.0
        for e in range(self.integral_time):
            accu += self.history_array[e] * self.time_delta

        return accu * self.integral_gain

    def calculate_derivative(self) -> float:
        if self.derivative_gain == 0:
            return 0

        if len(self.history_array) < self.derivative_time:
            return 0

        derivative = (self.history_array[0] - self.history_array[self.derivative_time - 1]) / self.derivative_time
        return derivative * self.derivative_gain

    def tick(self, value: float):
        # append history
        self.history_array.append(value)
        if len(self.history_array) > max(self.integral_time, self.derivative_time):
            self.history_array.pop(0)

        self.last_input = value
        self.last_output = self.output_multi * (
                value * self.proportional_gain +
                self.calculate_integral() +
                self.calculate_derivative()
        )

        # capping output amplitude
        if self.last_output > self.output_cap:
            return self.output_cap
        if self.last_output < -self.output_cap:
            return -self.output_cap
        return self.last_output
# flak end: pid_controller.py
# flak: mayson_controller.py


class MaysonController(object):
    MODE_POSITION = 0
    MODE_HEADING = 1

    def __init__(self):
        self.mode = MaysonController.MODE_POSITION
        self.angular_stab_checker = StabChecker(10, 0.05)

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

    def calculate_target_distance(self):
        return self.actual_position.euclidean_distance(self.target_position)

    def tick(self):
        self.delta_distance = self.actual_position.euclidean_distance(self.target_position)
        self.target_angle = self.actual_position.angle_toward(self.target_position)
        self.delta_angle = angle_calculate_delta(self.actual_heading, self.target_angle)

        if not self.angular_stab_checker.tick(self.delta_angle) or self.mode == MaysonController.MODE_HEADING:
            self.delta_distance = 0.0

    def set_target(self, target):
        self.target_position = target

    def check_finished(self):
        pass


# flak end: mayson_controller.py
# flak: array_average.py
class ArrayAverage(object):
    def __init__(self):
        self.len: int = 0
        self.accumulator_list: list = []
        self.count_list: list = []

    def tick(self, data: list):
        if self.len == 0:
            self.len = len(data)
            self.accumulator_list = [0] * self.len
            self.count_list = [0] * self.len
        elif self.len != len(data):
            raise Exception("ArrayAverage length mismatch")

        for i in range(0, self.len):
            d = data[i]
            if d != 0:
                self.accumulator_list[i] += d
                self.count_list[i] += 1

    def average(self) -> list:
        result = [0] * self.len
        for i, accumulated in enumerate(self.accumulator_list):
            if self.count_list[i] == 0:
                result[i] = 1000000
            else:
                result[i] = accumulated / self.count_list[i]
        return result

    def min_index(self) -> int:
        avg = self.average()
        min_value = 1000000
        min_index = -1
        for i, e in enumerate(avg):
            if self.count_list[i] == 0:
                continue
            if e < min_value:
                min_value = e
                min_index = i
        return min_index

    def max_index(self) -> int:
        avg = self.average()
        max_value = 0
        max_index = -1
        for i, e in enumerate(avg):
            if self.count_list[i] == 0:
                continue
            if e > max_value:
                max_value = e
                max_index = i
        return max_index
# flak end: array_average.py
# flak: griddle.py

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


# flak end: griddle.py
# flak: stab_checker.py
class StabChecker(object):
    def __init__(self, size: int, threshold: float):
        self.size = size
        self.threshold = threshold
        self.values: list[float] = []

    def tick(self, value: float) -> bool:
        value = abs(value)
        self.values.append(value)
        if len(self.values) > self.size:
            self.values.pop(0)

            for i in range(0, self.size):
                if self.values[i] > self.threshold:
                    return False

            return True
        else:
            return False
# flak end: stab_checker.py
