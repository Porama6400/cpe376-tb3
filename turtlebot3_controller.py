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

        self.sensor_angle_offset = -12

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

        if self.tick_counter < 10:
            self.array_average.tick(self.valueLaserRanges)

        if self.tick_counter == 10:
            averaged_data = self.array_average.average()
            offset_angle = angle_correction_min(averaged_data, 90 + self.sensor_angle_offset, 30)

            self.controller.set_zero(ros_pos, self.valueRotation + (offset_angle / 180 * math.pi))
            self.controller.set_target(Vector(1.0, 0.0))
            print("calibrated")

        print("tick")
        speed = self.pid_linear.tick(self.controller.delta_distance)
        angle = self.pid_angular.tick(self.controller.delta_angle)
        if self.tick_counter > 10:
            if self.controller.target_distance() > 0.01:
                self.publishVelocityCommand(float(speed), float(angle))
            else:
                self.publishVelocityCommand(float(0), float(0))
        else:
            self.publishVelocityCommand(float(0), float(0))

        print("==========")
        print(self.controller.actual_position, self.controller.actual_heading)
        print(self.controller.target_position)
        print(speed, angle)


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
    def __init__(self):
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

        if abs(self.delta_angle) > 0.2:
            self.delta_distance = 0.0

    def set_target(self, target):
        self.target_position = target


# flak end: mayson_controller.py
# flak: arrayaverage.py
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
# flak end: arrayaverage.py
