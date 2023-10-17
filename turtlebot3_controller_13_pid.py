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
        dist_right = angle_distance(self.valueLaserRanges, 210, 330)
        dist_front = angle_distance(self.valueLaserRanges, -15, 15)
        dist_left = angle_distance(self.valueLaserRanges, 75, 105)

        print("detect", dist_right, dist_front)
        speed_linear = self.pid_linear.tick(dist_front - 0.3)
        speed_angular = self.pid_angular.tick((-dist_right + 0.2) * 5)
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
        self.integral_cap: float = 0.1
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

        accu *= self.integral_gain
        if accu > self.integral_cap:
            accu = self.integral_cap
        elif accu < -self.integral_cap:
            accu = -self.integral_cap

        return accu

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
