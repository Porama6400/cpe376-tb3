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
        self.pid_linear.output_cap = 0.2
        self.pid_linear.proportional_gain = 1
        self.pid_linear.integral_gain = 0.3

        self.pid_angular = PidController()
        self.pid_angular.proportional_gain = 2
        self.pid_angular.integral_gain = 0.3
        self.pid_angular.output_cap = 2

        self.stab_angular = StabChecker(5, 0.005)
        self.stab_linear = StabChecker(5, 0.005)
        self.state = 0

        self.last_delta_distance: float = 0
        self.waypoint_controller = WaypointController()
        self.waypoint_controller.go_to(300)

        self.waypoint_controller.turn_to(900)
        self.waypoint_controller.go_to(300)

        self.waypoint_controller.turn_to(-900)
        self.waypoint_controller.go_to(300)

        self.waypoint_controller.turn_to(900)
        self.waypoint_controller.go_to(300)

        self.waypoint_controller.turn_to(1800)
        self.waypoint_controller.go_to(300)

        self.waypoint_controller.turn_to(-900)
        self.waypoint_controller.go_to(300)

        self.waypoint_controller.turn_to(900)
        self.waypoint_controller.go_to(300)

        self.waypoint_controller.turn_to(-900)
        self.waypoint_controller.go_to(300)

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
        print("tick")
        vec_pos = Vector(self.valuePosition.x, self.valuePosition.y)
        print("pos", vec_pos, self.valueRotation)

        self.waypoint_controller.tick(vec_pos, self.valueRotation)

        delta_distance = self.waypoint_controller.delta_distance_ndir
        delta_angle = self.waypoint_controller.delta_angle
        print("delta", delta_distance, delta_angle)

        if self.state == 0:
            delta_distance = 0
            if self.stab_angular.tick(self.waypoint_controller.delta_angle):
                self.state = 1
        elif self.state == 1:
            delta_angle = 0
            if self.stab_linear.tick(self.waypoint_controller.delta_distance_ndir):
                self.waypoint_controller.next()
                print("next")
                self.state = 0

        print("state", self.state)

        self.last_delta_distance = delta_distance

        # if self.waypoint_controller.delta_distance < 0.03:
        #     delta_angle = 0

        print("desired", self.waypoint_controller.desired_position)

        speed_linear = self.pid_linear.tick(delta_distance)
        speed_angular = self.pid_angular.tick(delta_angle)
        print("speed", speed_linear, speed_angular)
        self.publishVelocityCommand(float(speed_linear), float(speed_angular))


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
    except:
        KeyboardInterrupt
    print('Done')

    tb3ControllerNode.publishVelocityCommand(0.0, 0.0)
    tb3ControllerNode.destroy_node()
    robotStop()
    rclpy.shutdown()


if __name__ == '__main__':
    main()


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

    def length(self):
        return self.euclidean_distance(Vector(0, 0))

    def angle_toward(self, other):
        return math.atan2(other.y - self.y, other.x - self.x)

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


class WaypointController(object):
    def __init__(self):
        self.desired_position_previous: Vector | None = None
        self.desired_position: Vector | None = None
        self.current_position: Vector | None = None
        self.offset_angle: float = 0
        self.waypoint: Vector | None = None
        self.waypoint_list: list[Vector] = []
        self.delta_distance_ndir: float = 0

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

    def next(self):
        self.waypoint = self.waypoint_list.pop(0)
        self.desired_position_previous = self.desired_position
        self.desired_position += self.waypoint
        # NPE is intentionally left here to terminate the program cuz it's garbage

    def tick(self, current_position: Vector, current_angle: float):
        self.current_position = current_position
        self.time_counter += 1

        if self.time_counter <= 10:
            self.desired_position = current_position
            self.offset_angle = current_angle
            return

        if self.time_counter == 10:
            self.next()

        if self.waypoint is None:
            self.delta_distance = 0
            self.delta_angle = 0
            return

        self.delta_distance = current_position.euclidean_distance(self.desired_position)
        self.delta_angle = angle_calculate_delta(current_angle, current_position.angle_toward(self.desired_position))
        if self.waypoint is not None:
            self.delta_distance_ndir = (
                    self.waypoint.length() -
                    self.current_position.euclidean_distance(self.desired_position_previous)
            )
        else:
            self.delta_distance_ndir = 0


class StabChecker(object):
    def __init__(self, size: int, threshold: float):
        self.size = size
        self.threshold = threshold
        self.values: list[float] = []

    def tick(self, value: float) -> bool:
        self.values.append(value)
        if len(self.values) > self.size:
            self.values.pop(0)

            for i in range(0, self.size):
                if self.values[i] > self.threshold or self.values[i] < -self.threshold:
                    return False

            return True
        else:
            return False


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


def angle_distance(data: list, a: int, b: int) -> float:
    last = 1000000
    r = range(a, b)
    for index in r:
        if index < 0:
            index += 360
        if last > data[index] > 0.01:
            last = data[index]
    return last
