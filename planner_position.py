from planner import SIDE_LEFT, SIDE_RIGHT, SIDE_BOTTOM, SIDE_TOP, SIDE_NULL, PlannerWorld, planner_delta_angle
from mayson_controller import MC_NOP, MC_FWD, MC_ZERO, MC_TURN_LEFT, MC_TURN_RIGHT, MC_TURN_U


# flak export

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
