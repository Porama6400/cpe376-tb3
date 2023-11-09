from planner import SIDE_LEFT, SIDE_RIGHT, SIDE_BOTTOM, SIDE_TOP, SIDE_NULL, PlannerWorld, planner_delta_angle
from planner_position import PlannerPosition


# flak export

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

world = PlannerWorld(5, 3)
world.set_wall(0, 0, SIDE_RIGHT, False)
world.set_wall(1, 0, SIDE_RIGHT, False)
world.set_wall(2, 0, SIDE_RIGHT, False)
world.set_wall(2, 0, SIDE_BOTTOM, False)
world.set_wall(3, 0, SIDE_RIGHT, False)
world.set_wall(4, 0, SIDE_BOTTOM, False)

world.set_wall(0, 1, SIDE_RIGHT, False)
world.set_wall(0, 1, SIDE_BOTTOM, False)
world.set_wall(2, 1, SIDE_BOTTOM, False)

world.set_wall(0, 2, SIDE_RIGHT, False)
world.set_wall(1, 2, SIDE_RIGHT, False)
world.set_wall(2, 2, SIDE_RIGHT, False)
world.set_wall(3, 2, SIDE_RIGHT, False)

pos = PlannerPosition(0, 0, SIDE_TOP)
generated = pos.generate(world, 1, 1)
print(generated)
