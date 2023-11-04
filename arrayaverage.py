# flak export
class ArrayAverage(object):
    def __init__(self):
        self.len: int = 0
        self.accumulator_list: list = []
        self.count_list: list = []

    def tick(self, data: list):
        if self.len == 0:
            self.len = len(data)
            self.accumulator_list = [0] * self.len
            self.count_list = [0] * self.len
        elif self.len != len(data):
            raise Exception("ArrayAverage length mismatch")

        for i in range(0, self.len):
            d = data[i]
            if d != 0:
                self.accumulator_list[i] += d
                self.count_list[i] += 1

    def average(self) -> list:
        result = [0] * self.len
        for i, accumulated in enumerate(self.accumulator_list):
            if self.count_list[i] == 0:
                result[i] = 1000000
            else:
                result[i] = accumulated / self.count_list[i]
        return result

    def min_index(self) -> int:
        avg = self.average()
        min_value = 1000000
        min_index = -1
        for i, e in enumerate(avg):
            if self.count_list[i] == 0:
                continue
            if e < min_value:
                min_value = e
                min_index = i
        return min_index

    def max_index(self) -> int:
        avg = self.average()
        max_value = 0
        max_index = -1
        for i, e in enumerate(avg):
            if self.count_list[i] == 0:
                continue
            if e > max_value:
                max_value = e
                max_index = i
        return max_index
