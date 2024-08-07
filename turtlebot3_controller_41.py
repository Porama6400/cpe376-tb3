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

# flak omit
from angle_util import angle_correction_min
from array_average import ArrayAverage
from pid_controller import PidController
from mayson_controller import *
from vector import Vector
from planner import *
from planner_superposition import *


# flak unomit

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
        self.pid_linear.output_cap = 0.15
        self.pid_linear.proportional_gain = 0.8
        self.pid_linear.integral_gain = 0.5
        self.pid_linear.integral_cap = 0.03
        self.pid_linear.integral_time = 8

        self.pid_angular = PidController()
        self.pid_angular.proportional_gain = 1.5
        self.pid_angular.integral_gain = 0.2
        self.pid_angular.integral_time = 8
        self.pid_linear.integral_cap = 0.01
        self.pid_angular.output_cap = 2

        self.controller = MaysonController()
        self.tick_counter = 0
        self.near_threshold = 0.23
        self.laser_distance_available = False

    def publishVelocityCommand(self, linearVelocity, angularVelocity):
        msg = Twist()
        msg.linear.x = linearVelocity
        msg.angular.z = angularVelocity
        self.cmdVelPublisher.publish(msg)
        # self.get_logger().info('Publishing cmd_vel: "%s", "%s"' % linearVelocity, angularVelocity)

    def scanCallback(self, msg):
        self.laser_distance_available = True
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
        if not self.laser_distance_available:
            return

        self.tick_counter += 1
        ros_pos = Vector(self.valuePosition.x, self.valuePosition.y)
        distance_left = angle_distance(self.valueLaserRanges, 90 - 5, 90 + 5)
        distance_front = angle_distance(self.valueLaserRanges, 0 - 5, 0 + 5)
        distance_back = angle_distance(self.valueLaserRanges, 180 - 5, 180 + 5)
        distance_right = angle_distance(self.valueLaserRanges, 270 - 5, 270 + 5)
        near_left = distance_left < self.near_threshold
        near_front = distance_front < self.near_threshold
        near_back = distance_back < self.near_threshold
        near_right = distance_right < self.near_threshold

        try:
            self.controller.tick(ros_pos, self.valueRotation, self.valueLaserRanges)
        except Exception as ex:
            print(ex)
            print("sensor", distance_left, distance_front, distance_back, distance_right)
            print("near", near_left, near_front, near_back, near_right)

            if near_left and near_front and near_right and not near_back:
                # except back
                raise Exception("done")

            elif near_left and not near_front and near_right:
                # left and right
                self.controller.enqueue(MC_FWD)

            elif near_left and not near_front and not near_right and near_back:
                # left and back
                self.controller.enqueue(MC_FWD)

            elif not near_left and not near_front and near_right and near_back:
                # right and back
                self.controller.enqueue(MC_FWD)

            elif near_left and near_front and not near_right:
                # left and front
                self.controller.enqueue(MC_TURN_RIGHT)
                self.controller.enqueue(MC_TURN_RIGHT)

            elif not near_left and near_front and near_right:
                # right and front
                self.controller.enqueue(MC_TURN_LEFT)

            elif not near_left and not near_front and not near_right and near_back:
                # back only
                self.controller.enqueue(MC_FWD)

            elif near_left and not near_front and not near_right and not near_back:
                # left only
                self.controller.enqueue(MC_FWD)

            elif not near_left and near_front and not near_right and not near_back:
                # front only
                self.controller.enqueue(MC_TURN_RIGHT)

            elif not near_left and not near_front and near_right and not near_back:
                # right only
                self.controller.enqueue(MC_TURN_LEFT)

            elif near_back and not near_front:
                self.controller.enqueue(MC_FWD)

            elif not near_front:
                self.controller.enqueue(MC_FWD)

            elif not near_left:
                self.controller.enqueue(MC_TURN_LEFT)

            else:
                self.controller.enqueue(MC_TURN_RIGHT)

        speed = self.pid_linear.tick(self.controller.delta_distance)
        angle = self.pid_angular.tick(self.controller.delta_angle)

        print("==========")
        print(self.controller.actual_position, self.controller.actual_heading)
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

# flak emplacement vector
# flak emplacement angle_util
# flak emplacement pid_controller
# flak emplacement mayson_controller
# flak emplacement array_average
# flak emplacement stab_checker
# flak emplacement planner
# flak emplacement planner_position
# flak emplacement planner_superposition
