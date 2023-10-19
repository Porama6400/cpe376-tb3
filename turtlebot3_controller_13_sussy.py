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

        dist_back_left = angle_distance(self.valueLaserRanges, 120, 150)
        dist_left = angle_distance(self.valueLaserRanges, 75, 105)
        dist_front_left = angle_distance(self.valueLaserRanges, 30, 60)
        dist_front = angle_distance(self.valueLaserRanges, -15, 15)
        dist_front_right = angle_distance(self.valueLaserRanges, 300, 330)
        dist_right = angle_distance(self.valueLaserRanges, 255, 285)
        dist_back_right = angle_distance(self.valueLaserRanges, 210, 240)

        near_dist = 0.125
        far_dist = 0.25
        far_back_left = sussy_activation_linear(dist_back_left, near_dist, far_dist)
        near_back_left = 1 - far_back_left
        far_left = sussy_activation_linear(dist_left, near_dist, far_dist)
        near_left = 1 - far_left
        far_front_left = sussy_activation_linear(dist_front_left, near_dist, far_dist)
        near_front_left = 1 - far_front_left
        far_front = sussy_activation_linear(dist_front, near_dist, far_dist)
        near_front = 1 - far_front
        far_front_right = sussy_activation_linear(dist_front_right, near_dist, far_dist)
        near_front_right = 1 - far_front_right
        far_right = sussy_activation_linear(dist_right, near_dist, far_dist)
        near_right = 1 - far_right
        far_back_right = sussy_activation_linear(dist_back_right, near_dist, far_dist)
        near_back_right = 1 - far_back_right

        result = [0.0, 0.0]
        maximumSpeed = [0.2, 1.0]

        sussy_add(result, [0, 1], near_left * near_front_left)
        sussy_add(result, [0, -1], near_right * near_front_right)
        sussy_add(result, [1, 0], far_front_right * far_front * far_front_left)
        sussy_add(result, [0, 2], near_front_right * near_front * near_front_left)
        sussy_add(result, [0, 1], far_front_right * near_front)
        sussy_add(result, [0, -1], far_front_left * near_front)
        sussy_multiply(result, maximumSpeed)
        sussy_cap_abs(result, maximumSpeed)

        print(dist_back_left, dist_left, dist_front_left, dist_front, dist_front_right, dist_right, dist_back_right)
        print(near_back_left, near_left, near_front_left, near_front, near_front_right, near_right, near_back_right)
        print(result)
        self.publishVelocityCommand(result[0], -result[1])


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
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(e)
    print('Done')

    tb3ControllerNode.publishVelocityCommand(0.0, 0.0)
    tb3ControllerNode.destroy_node()
    robotStop()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

# flak emplacement angle_util

# flak emplacement sussy
