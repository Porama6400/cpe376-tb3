class Position(object):
    def __init__(self):
        self.x = 0
        self.y = 0


class Waypoint(object):
    position: Position = None
    should_seek_angle: bool
    should_seek_position: bool


class WaypointController(object):
    current: Waypoint = None
    virtual_position: Position

    def tick(self, current: Position):
        current.
