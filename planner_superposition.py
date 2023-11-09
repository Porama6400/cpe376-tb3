from planner import SIDE_LEFT, SIDE_RIGHT, SIDE_BOTTOM, SIDE_TOP, SIDE_NULL, PlannerWorld
from mayson_controller import MC_NOP, MC_FWD, MC_ZERO, MC_TURN_LEFT, MC_TURN_RIGHT


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
        print("val", )
        print("pos", self.x, self.y , "@", self.direction)
        print("sid", front_side_id, left_side_id, right_side_id)
        print("expt", front_wall, left_wall, right_wall)
        print("found", front, left, right)

        if front_wall != front or left_wall != left or right_wall != right:
            self.drop()

    def turn(self, delta):
        self.direction = (self.direction + delta) % 4


class PlannerSuperPosition(object):
    def __init__(self, world: PlannerWorld):
        self.possible_positions: list = []
        self.certain_position = None
        self.world = world

    def add(self, x: int, y: int, direction: int):
        wrapped = PlannerPosition(x, y, direction)
        self.possible_positions.append(wrapped)

    def step(self):
        for position in self.possible_positions:
            planner_position: PlannerPosition = position
            planner_position.step(self.world)
        self.prune()

    def turn(self, delta: int):
        for position in self.possible_positions:
            planner_position: PlannerPosition = position
            planner_position.turn(delta)

    def validate(self, left: bool, top: bool, right: bool):
        for position in self.possible_positions:
            planner_position: PlannerPosition = position
            planner_position.validate(self.world, left, top, right)
        self.prune()

    def prune(self):
        temp_list = []
        for position in self.possible_positions:
            planner_position: PlannerPosition = position
            if planner_position.is_valid():
                temp_list.append(planner_position)

        self.possible_positions = temp_list

    def print(self):
        print("All possible positions:")
        for position in self.possible_positions:
            planner_position: PlannerPosition = position
            print(planner_position.x, planner_position.y, "@", planner_position.direction, "( start positon ",
                  planner_position.start_x, planner_position.start_y, "@", planner_position.start_direction, ")")


# flak noexport

world = PlannerWorld(3, 3)
world.set_wall(0, 0, SIDE_RIGHT, False)
world.set_wall(1, 0, SIDE_RIGHT, False)
world.set_wall(0, 0, SIDE_BOTTOM, False)
world.set_wall(0, 1, SIDE_RIGHT, False)
psp = PlannerSuperPosition(world)
psp.add(0, 0, SIDE_RIGHT)
psp.add(0, 0, SIDE_TOP)
psp.add(0, 0, SIDE_LEFT)
psp.add(0, 0, SIDE_BOTTOM)

print("t0")
psp.step()
psp.print()
print("t1")
psp.validate(True, False, True)
psp.print()
print("t2")
psp.step()
psp.print()
print("t3")
psp.validate(True, True, True)
psp.print()
print("tn", psp.possible_positions)
