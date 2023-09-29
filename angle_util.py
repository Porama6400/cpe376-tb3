import math
import unittest

PI2 = math.pi * 2


def angle_normalize(rad: float) -> float:
    while rad < 0:
        rad += PI2
    while rad > PI2:
        rad -= PI2
    return rad


def angle_calculate_delta(rad_a: float, rad_b: float):
    delta = rad_b - rad_a
    if abs(delta) <= math.pi:
        return delta
    elif rad_a < rad_b:
        return rad_b - (rad_a + PI2)
    else:
        return (rad_b + PI2) - rad_a


def angle_nearest_range(data: list, deg_a: int, deg_b: int) -> int:
    min_value = 1000000
    min_index = -1
    for i in range(deg_a, deg_b):
        val = data[i]
        if 0 < val < min_value:
            min_value = val
            min_index = i

    return min_index


def angle_distance(data: list, a: int, b: int) -> float:
    last = 1000000
    r = range(a, b)
    for index in r:
        data_len = len(data)
        if index < 0:
            index += data_len
        if last > data[index] > 0.01:
            last = data[index]
    return last


class TestAngleUtil(unittest.TestCase):

    def test_angle_normalize(self):
        self.assertAlmostEqual(angle_normalize(3 * math.pi), math.pi)
        self.assertAlmostEqual(angle_normalize(-5 * math.pi), math.pi)
        self.assertAlmostEqual(angle_normalize(0), 0)

    def test_angle_calculate_delta(self):
        self.assertAlmostEqual(angle_calculate_delta(0, math.pi * 0.5), math.pi * 0.5)
        self.assertAlmostEqual(angle_calculate_delta(math.pi * 0.5, 0), math.pi * -0.5)
        self.assertAlmostEqual(angle_calculate_delta(math.pi * 1.75, math.pi * 0.25), math.pi * 0.5)

    def test_angle_distance(self):
        data = [0.5, 0, 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.1]
        self.assertAlmostEqual(angle_distance(data, -2, 2), 0.1)
        self.assertAlmostEqual(angle_distance(data, 0, 3), 0.1)
        self.assertAlmostEqual(angle_distance(data, 3, 6), 0.2)


if __name__ == '__main__':
    unittest.main()
