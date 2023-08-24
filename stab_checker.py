class StabChecker(object):
    def __init__(self, size: int, threshold: float):
        self.size = size
        self.threshold = threshold
        self.values: list[float] = []

    def tick(self, value: float) -> bool:
        self.values.append(value)
        if len(self.values) > self.size:
            self.values.pop(0)

            for i in range(0, self.size):
                if self.values[i] > self.threshold:
                    return False

            return True
        else:
            return False
