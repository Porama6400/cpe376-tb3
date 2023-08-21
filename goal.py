class Position(object):
    def __init__(self):
        self.x = 0
        self.y = 0


class Goal(object):
    def __init__(self):
        pass

    def tick(self, ctx: any):
        pass

    def is_completed(self, ctx: any) -> bool:
        pass

