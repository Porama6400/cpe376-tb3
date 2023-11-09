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

        self.world = PlannerWorld(5, 3)
        self.world.set_wall(0, 0, SIDE_RIGHT, False)
        self.world.set_wall(1, 0, SIDE_RIGHT, False)
        self.world.set_wall(2, 0, SIDE_RIGHT, False)
        self.world.set_wall(2, 0, SIDE_BOTTOM, False)
        self.world.set_wall(3, 0, SIDE_RIGHT, False)
        self.world.set_wall(4, 0, SIDE_BOTTOM, False)

        self.world.set_wall(0, 1, SIDE_RIGHT, False)
        self.world.set_wall(0, 1, SIDE_BOTTOM, False)
        self.world.set_wall(2, 1, SIDE_BOTTOM, False)

        self.world.set_wall(0, 2, SIDE_RIGHT, False)
        self.world.set_wall(1, 2, SIDE_RIGHT, False)
        self.world.set_wall(2, 2, SIDE_RIGHT, False)
        self.world.set_wall(3, 2, SIDE_RIGHT, False)

        self.start_position = PlannerPosition(4, 0, SIDE_LEFT)
        plan = self.start_position.generate(self.world, 1, 1)
        for instruction in plan:
            self.controller.enqueue(instruction)

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

    def clone(self):
        return Vector(self.x, self.y)


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


def angle_nearest_range_relative(data: list, deg_center: int, deg_arc: int) -> int:
    val = angle_nearest_range(data, deg_center - deg_arc, deg_center + deg_arc)
    return val - deg_center


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
        self.grid_size = 0.3

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
                    nearest_left = angle_nearest_range_relative(distance_data, 80, 15)
                    distance_angle_offset = (distance_left - self.expected_wall_distance) * 0.7
                    self.delta_angle = (nearest_left + distance_angle_offset) / 180 * math.pi
                    print("fwd turn", nearest_left, distance_left)



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
# flak: planner.py
SIDE_RIGHT = 0x00
SIDE_TOP = 0x01
SIDE_LEFT = 0x02
SIDE_BOTTOM = 0x03
SIDE_NULL = 0x10


class PlannerWorld(object):
    def __init__(self, width: int, height: int):
        self.width = width
        self.height = height

        self.data = [True] * self.width * (self.height * 2)
        self.cost_map = [0] * self.width * self.height

    # index -1 is perma wall
    def calculate_index(self, x: int, y: int, side: int):
        if x == 0 and side == SIDE_LEFT:
            return -1
        elif y == 0 and side == SIDE_TOP:
            return -1
        elif x == self.width - 1 and side == SIDE_RIGHT:
            return -1
        elif y == self.height - 1 and side == SIDE_BOTTOM:
            return -1
        elif side == SIDE_BOTTOM:
            return x + (y * 2) * self.width
        elif side == SIDE_RIGHT:
            return x + (y * 2 + 1) * self.width
        elif side == SIDE_LEFT:
            return (x - 1) + (y * 2 + 1) * self.width
        elif side == SIDE_TOP:
            return x + (y - 1) * 2 * self.width

    def set_wall(self, x: int, y: int, side: int, value: bool):
        index = self.calculate_index(x, y, side)
        # print("write index", x, y, index, value)
        if index < 0:
            raise Exception("tried to set perma wall")
        self.data[index] = value

    def get_wall(self, x: int, y: int, side: int) -> bool:
        index = self.calculate_index(x, y, side)
        if index < 0:
            # print("read index", x, y, index, "perma true")
            return True
        # print("read index", x, y, index, self.data[index])
        return self.data[index]

    def set_cost(self, x: int, y: int, cost: int):
        self.cost_map[x + y * self.width] = cost

    def get_cost(self, x: int, y: int) -> int:
        return self.cost_map[x + y * self.width]

    def solve_cost(self, bx: int, by: int):
        self.cost_map = [1000] * self.width * self.height
        queue = [(bx, by)]
        self.set_cost(bx, by, 0)
        while len(queue) > 0:
            (x, y) = queue[0]
            queue = queue[1:]

            new_cost = self.get_cost(x, y) + 1
            if not self.get_wall(x, y, SIDE_LEFT):
                current_cost = self.get_cost(x - 1, y)
                if new_cost < current_cost:
                    self.set_cost(x - 1, y, new_cost)
                    queue.append((x - 1, y))
            if not self.get_wall(x, y, SIDE_RIGHT):
                current_cost = self.get_cost(x + 1, y)
                if new_cost < current_cost:
                    self.set_cost(x + 1, y, new_cost)
                    queue.append((x + 1, y))
            if not self.get_wall(x, y, SIDE_TOP):
                current_cost = self.get_cost(x, y - 1)
                if new_cost < current_cost:
                    self.set_cost(x, y - 1, new_cost)
                    queue.append((x, y - 1))
            if not self.get_wall(x, y, SIDE_BOTTOM):
                current_cost = self.get_cost(x, y + 1)
                if new_cost < current_cost:
                    self.set_cost(x, y + 1, new_cost)
                    queue.append((x, y + 1))

    def get_next_move(self, x: int, y: int) -> int:
        min_value = 1000
        min_side = SIDE_NULL

        if not self.get_wall(x, y, SIDE_TOP):
            value = self.get_cost(x, y - 1)
            if value < min_value:
                min_value = value
                min_side = SIDE_TOP

        if not self.get_wall(x, y, SIDE_BOTTOM):
            value = self.get_cost(x, y + 1)
            if value < min_value:
                min_value = value
                min_side = SIDE_BOTTOM

        if not self.get_wall(x, y, SIDE_LEFT):
            value = self.get_cost(x - 1, y)
            if value < min_value:
                min_value = value
                min_side = SIDE_LEFT

        if not self.get_wall(x, y, SIDE_RIGHT):
            value = self.get_cost(x + 1, y)
            if value < min_value:
                min_value = value
                min_side = SIDE_RIGHT

        return min_side


def planner_delta_angle(fr: int, to: int) -> int:
    delta = (to - fr) % 4
    if delta == 3:
        delta = -1
    return delta


# flak end: planner.py
# flak: planner_position.py

class PlannerPosition(object):
    def __init__(self, start_x: int, start_y: int, start_direction: int):
        self.start_x = start_x
        self.start_y = start_y
        self.start_direction = start_direction
        self.x = start_x
        self.y = start_y
        self.direction = start_direction
        self.viable: bool = True

    def is_valid(self):
        return self.viable

    def drop(self):
        self.viable = False
        print(self.start_x, self.start_y, "@", self.start_direction, "start position dropped out")

    def step(self, world: PlannerWorld):
        if not self.viable:
            return

        has_wall = world.get_wall(self.x, self.y, self.direction)
        if has_wall:
            self.drop()
            return

        if self.direction == SIDE_TOP:
            self.y -= 1
        elif self.direction == SIDE_BOTTOM:
            self.y += 1
        elif self.direction == SIDE_LEFT:
            self.x -= 1
        elif self.direction == SIDE_RIGHT:
            self.x += 1

    def validate(self, world: PlannerWorld, left: bool, front: bool, right: bool):
        if not self.viable:
            return
        front_side_id = self.direction
        left_side_id = (self.direction + 1) % 4
        right_side_id = (self.direction - 1) % 4

        front_wall = world.get_wall(self.x, self.y, front_side_id)
        left_wall = world.get_wall(self.x, self.y, left_side_id)
        right_wall = world.get_wall(self.x, self.y, right_side_id)
        print("val", )
        print("pos", self.x, self.y, "@", self.direction)
        print("sid", front_side_id, left_side_id, right_side_id)
        print("expt", front_wall, left_wall, right_wall)
        print("found", front, left, right)

        if front_wall != front or left_wall != left or right_wall != right:
            self.drop()

    def turn(self, delta):
        self.direction = (self.direction + delta) % 4

    def generate(self, world: PlannerWorld, target_x: int, target_y: int):
        current_x = self.x
        current_y = self.y
        current_direction = self.direction
        inst_queue = []

        world.solve_cost(target_x, target_y)

        while current_x != target_x or current_y != target_y:
            next_move = world.get_next_move(current_x, current_y)
            print("nm", next_move)
            if next_move == SIDE_TOP:
                current_y -= 1
            elif next_move == SIDE_BOTTOM:
                current_y += 1
            elif next_move == SIDE_LEFT:
                current_x -= 1
            elif next_move == SIDE_RIGHT:
                current_x += 1

            delta_angle = planner_delta_angle(current_direction, next_move)
            current_direction = next_move
            print("delta", delta_angle)
            if delta_angle == 1:
                inst_queue.append(MC_TURN_LEFT)
            if delta_angle == 2:
                inst_queue.append(MC_TURN_U)
            if delta_angle == -1:
                inst_queue.append(MC_TURN_RIGHT)
            inst_queue.append(MC_FWD)
        return inst_queue
# flak end: planner_position.py
