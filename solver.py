SIDE_RIGHT = 0
SIDE_TOP = 1
SIDE_LEFT = 2
SIDE_BOTTOM = 3


class SolverWorld(object):
    def __init__(self, width: int, height: int):
        self.width = width
        self.height = height

        self.data = [True] * (self.width + 1) * 2 * (self.height + 1)

    def calculate_index_raw(self, x: int, y: int, top: bool):
        data_index = x + y * (self.width + 1)
        data_index *= 2
        if top:
            data_index += 1
        return data_index

    def calculate_index(self, x: int, y: int, side: int):
        top = False
        if side == SIDE_LEFT:
            top = False
        elif side == SIDE_TOP:
            top = True
        elif side == SIDE_RIGHT:
            top = False
            x += 1
        elif side == SIDE_BOTTOM:
            top = True
            y += 1
        return self.calculate_index_raw(x, y, top)

    def set(self, x: int, y: int, side: int, value: bool):
        self.data[self.calculate_index(x, y, side)] = value

    def get(self, x: int, y: int, side: int):
        return self.data[self.calculate_index(x, y, side)]


world = SolverWorld(4, 3)
print(world.calculate_index(2, 2, SIDE_LEFT))
print(world.calculate_index(2, 2, SIDE_TOP))
print(world.calculate_index(2, 2, SIDE_RIGHT))
print(world.calculate_index(2, 2, SIDE_BOTTOM))
