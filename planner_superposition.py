from planner import SIDE_LEFT, SIDE_RIGHT, SIDE_BOTTOM, SIDE_TOP, SIDE_NULL, PlannerWorld
from mayson_controller import MC_NOP, MC_FWD, MC_ZERO, MC_TURN_LEFT, MC_TURN_RIGHT


# flak export

class PlannerPosition(object):
    def __init__(self, world: PlannerWorld):
        self.x: int = 0
        self.y: int = 0
        self.direction: int = SIDE_NULL

    def move(self, world: PlannerWorld):
        has_wall = world.get_wall(self.x, self.y, self.direction)
        if has_wall:
            raise Exception("unable to move")
        if self.direction == SIDE_TOP:
            self.y -= 1
        elif self.direction == SIDE_BOTTOM:
            self.y += 1
        elif self.direction == SIDE_LEFT:
            self.x -= 1
        elif self.direction == SIDE_RIGHT:
            self.x += 1


class PlannerSuperPosition(object):
    def __init__(self):
        self.possible_positions: list = []
        self.certain_position = None

    def tick(self, mc_inst: int, near_left: bool, near_front: bool, near_right: bool):
        if mc_inst == MC_FWD
