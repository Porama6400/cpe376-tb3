from typing import Any

from planner import SIDE_LEFT, SIDE_RIGHT, SIDE_BOTTOM, SIDE_TOP, SIDE_NULL, PlannerWorld, planner_delta_angle
from planner_position import PlannerPosition


# flak export

class PlannerSuperPosition(object):
    def __init__(self, world: PlannerWorld):
        self.possible_positions: list = []
        self.world = world

    def add(self, pos: PlannerPosition):
        self.possible_positions.append(pos)

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

    def populate_all(self):
        for i in range(self.world.width):
            for j in range(self.world.height):
                self.add(PlannerPosition(i, j, SIDE_RIGHT))
                self.add(PlannerPosition(i, j, SIDE_TOP))
                self.add(PlannerPosition(i, j, SIDE_LEFT))
                self.add(PlannerPosition(i, j, SIDE_BOTTOM))

    def print(self):
        print("All possible positions:", self.count_possible_positions())
        for position in self.possible_positions:
            planner_position: PlannerPosition = position
            print(planner_position.x, planner_position.y, "@", planner_position.direction, "( start positon ",
                  planner_position.start_x, planner_position.start_y, "@", planner_position.start_direction, ")")

    def count_possible_positions(self) -> int:
        return len(self.possible_positions)

    def is_collapsed(self):
        return self.count_possible_positions() <= 1

    def get_certain_position(self) -> Any | None:
        count = self.count_possible_positions()
        if count == 0:
            raise Exception("unable to solve for position: no possible valid position")
        elif count == 1:
            return self.possible_positions[0]
        else:
            return None



# flak noexport

world = PlannerWorld(5, 3)
world.set_wall(0, 0, SIDE_RIGHT, False)
world.set_wall(0, 0, SIDE_BOTTOM, False)
world.set_wall(1, 0, SIDE_RIGHT, False)
world.set_wall(2, 0, SIDE_RIGHT, False)
world.set_wall(2, 0, SIDE_BOTTOM, False)
world.set_wall(3, 0, SIDE_RIGHT, False)
world.set_wall(3, 0, SIDE_BOTTOM, False)
world.set_wall(4, 0, SIDE_BOTTOM, False)

world.set_wall(0, 1, SIDE_RIGHT, False)
world.set_wall(0, 1, SIDE_BOTTOM, False)
world.set_wall(1, 1, SIDE_RIGHT, False)
world.set_wall(1, 1, SIDE_BOTTOM, False)
world.set_wall(2, 1, SIDE_BOTTOM, False)
world.set_wall(4, 1, SIDE_BOTTOM, False)

world.set_wall(0, 2, SIDE_RIGHT, False)
world.set_wall(2, 2, SIDE_RIGHT, False)
world.set_wall(3, 2, SIDE_RIGHT, False)

psp = PlannerSuperPosition(world)
psp.populate_all()
psp.step()
psp.validate(True, False, True)
psp.step()
psp.validate(True, False, False)
psp.step()
psp.validate(True, False, False)
psp.step()
psp.validate(True, True, False)
psp.print()
print(psp.get_certain_position())

# generated = pos.generate(world, 3, 1)
# print(generated)
