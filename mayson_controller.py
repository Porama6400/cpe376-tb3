from vector import Vector


class Controller(object):
    def __init__(self):
        self.origin: Vector = Vector(0, 0)
        self.angle_offset: float = 0