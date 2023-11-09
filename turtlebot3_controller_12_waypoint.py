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

        self.waypoint_controller.validate(vec_pos, self.valueRotation)

        delta_distance = self.waypoint_controller.delta_distance_ndir
        delta_angle = self.waypoint_controller.delta_angle
        print("delta", delta_distance, delta_angle)

        if self.state == 0:
            delta_distance = 0
            if self.stab_angular.validate(self.waypoint_controller.delta_angle):
                self.state = 1
        elif self.state == 1:
            delta_angle = 0
            if self.stab_linear.validate(self.waypoint_controller.delta_distance_ndir):
                self.waypoint_controller.next()
                print("next")
                self.state = 0

        print("state", self.state)

        self.last_delta_distance = delta_distance

        # if self.waypoint_controller.delta_distance < 0.03:
        #     delta_angle = 0

        print("desired", self.waypoint_controller.desired_position)

        speed_linear = self.pid_linear.validate(delta_distance)
        speed_angular = self.pid_angular.validate(delta_angle)
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

# flak emplacement pid_controller
# flak emplacement vector
# flak emplacement waypoint
# flak emplacement stab_checker
# flak emplacement angle_util
