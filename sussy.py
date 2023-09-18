def sussy_activation_linear(val: float, low: float, high: float) -> float:
    if low > high:
        return 1 - sussy_activation_linear(val, high, low)
    if val < low:
        return 0
    if val > high:
        return 1
    return (val - low) / (high - low)


def sussy_add(to: list[float], add: list[float], multiplier: float = 1):
    for i in range(0, len(to)):
        to[i] += add[i] * multiplier


def sussy_multiply(to: list[float], multiplier: list[float]):
    for i in range(0, len(to)):
        to[i] *= multiplier[i]


def sussy_cap_abs(to: list[float], cap: list[float]):
    for i in range(0, len(to)):
        if to[i] > cap[i]:
            to[i] = cap[i]
        elif to[i] < -cap[i]:
            to[i] = -cap[i]


def sussy_calculate(dist_left: float, dist_front: float, dist_right: float) -> list[float]:
    far_left = sussy_activation_linear(dist_left, 5, 15)
    near_left = 1 - far_left
    far_front = sussy_activation_linear(dist_front, 5, 15)
    near_front = 1 - far_front
    far_right = sussy_activation_linear(dist_right, 5, 15)
    near_right = 1 - far_right

    result = [0, 0]
    maximumSpeed = [0.2, 2]

    sussy_add(result, [0, 1], near_left * near_front * far_right)
    sussy_add(result, [0, -1], far_left * near_front * near_right)
    sussy_add(result, [0.5, 0], near_left * far_front * near_right)
    sussy_add(result, [1, 0], far_left * far_front * far_right)
    sussy_multiply(result, maximumSpeed)
    sussy_cap_abs(result, maximumSpeed)
    return result


print(sussy_calculate(0, 30, 0))
