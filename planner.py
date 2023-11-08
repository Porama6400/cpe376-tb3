from vector import Vector

# flak export
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


# flak noexport

world = PlannerWorld(4, 3)
world.set_wall(0, 0, SIDE_BOTTOM, False)
world.set_wall(0, 1, SIDE_BOTTOM, False)
world.set_wall(0, 2, SIDE_RIGHT, False)
world.set_wall(1, 2, SIDE_TOP, False)
world.set_wall(1, 1, SIDE_TOP, False)
print(world.get_wall(1, 1, SIDE_RIGHT))
print(world.get_wall(1, 1, SIDE_TOP))
print(world.get_wall(1, 1, SIDE_LEFT))
print(world.get_wall(1, 1, SIDE_BOTTOM))

world.solve_cost(1, 1)
print(world.get_cost(0, 0), world.get_cost(1, 0), world.get_cost(2, 0))
print(world.get_cost(0, 1), world.get_cost(1, 1), world.get_cost(2, 1))
print(world.get_cost(0, 2), world.get_cost(1, 2), world.get_cost(2, 2))

print(world.get_next_move(0, 0))
print(world.get_next_move(0, 1))
print(world.get_next_move(0, 2))
print(world.get_next_move(1, 2))
