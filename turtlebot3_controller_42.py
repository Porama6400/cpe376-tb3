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

        self.collapsed = False
        self.world = PlannerWorld(5, 3)
        self.world.set_wall(0, 0, SIDE_RIGHT, False)
        self.world.set_wall(0, 0, SIDE_BOTTOM, False)
        self.world.set_wall(1, 0, SIDE_RIGHT, False)
        self.world.set_wall(2, 0, SIDE_RIGHT, False)
        self.world.set_wall(2, 0, SIDE_BOTTOM, False)
        self.world.set_wall(3, 0, SIDE_RIGHT, False)
        self.world.set_wall(3, 0, SIDE_BOTTOM, False)
        self.world.set_wall(4, 0, SIDE_BOTTOM, False)

        self.world.set_wall(0, 1, SIDE_RIGHT, False)
        self.world.set_wall(0, 1, SIDE_BOTTOM, False)
        self.world.set_wall(1, 1, SIDE_RIGHT, False)
        self.world.set_wall(1, 1, SIDE_BOTTOM, False)
        self.world.set_wall(2, 1, SIDE_BOTTOM, False)
        self.world.set_wall(4, 1, SIDE_BOTTOM, False)

        self.world.set_wall(0, 2, SIDE_RIGHT, False)
        self.world.set_wall(2, 2, SIDE_RIGHT, False)
        self.world.set_wall(3, 2, SIDE_RIGHT, False)

        self.planner = PlannerSuperPosition(self.world)
        self.planner.populate_all()

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
        distance_top = angle_distance(self.valueLaserRanges, 0 - 5, 0 + 5)
        distance_right = angle_distance(self.valueLaserRanges, 270 - 5, 270 + 5)
        near_left = distance_left < self.near_threshold
        near_top = distance_top < self.near_threshold
        near_right = distance_right < self.near_threshold

        if not self.collapsed:
            try:
                self.controller.tick(ros_pos, self.valueRotation, self.valueLaserRanges)
            except Exception as ex:
                print(ex)
                print("sensor", distance_left, distance_top, distance_right)
                print("near", near_left, near_top, near_right)
                self.planner.validate(near_left, near_top, near_right)

                if self.planner.is_collapsed():
                    self.collapsed = True

                    position: PlannerPosition = self.planner.get_certain_position()
                    print("collapsed!")
                    print("certain position known:", position.x, position.y)
                    print("start position known:", position.start_x, position.start_y, "@", position.start_direction)
                    plan = position.generate(self.world, 3, 1)
                    for instruction in plan:
                        self.controller.enqueue(instruction)

                elif not near_top:
                    self.controller.enqueue(MC_FWD)
                    self.planner.step()
                elif not near_left:
                    self.controller.enqueue(MC_TURN_LEFT)
                    self.planner.turn(1)
                elif not near_right:
                    self.controller.enqueue(MC_TURN_RIGHT)
                    self.planner.turn(-1)
                else:
                    self.controller.enqueue(MC_TURN_U)
                    self.planner.turn(2)

        else:
            self.controller.tick(ros_pos, self.valueRotation, self.valueLaserRanges)

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
