class PidController(object):
    def __init__(self):
        self.proportional_gain: float = 0.5
        self.integral_gain: float = 0.3
        self.integral_time: int = 10
        self.derivative_gain: float = 0
        self.derivative_time: int = 15

        self.output_cap: float = 0.1
        self.output_multi: float = 1
        self.time_delta: float = 0.1

        self.history_array: list = []

        self.last_input: float = 0
        self.last_output: float = 0

    def reset(self):
        self.history_array = [0] * max(self.derivative_time, self.integral_time)

    def calculate_integral(self) -> float:
        if self.integral_gain == 0:
            return 0

        if len(self.history_array) < self.integral_time:
            return 0

        accu = 0.0
        for e in range(self.integral_time):
            accu += self.history_array[e] * self.time_delta

        return accu * self.integral_gain

    def calculate_derivative(self) -> float:
        if self.derivative_gain == 0:
            return 0

        if len(self.history_array) < self.derivative_time:
            return 0

        derivative = (self.history_array[0] - self.history_array[self.derivative_time - 1]) / self.derivative_time
        return derivative * self.derivative_gain

    def tick(self, value: float):
        # append history
        self.history_array.append(value)
        if len(self.history_array) > max(self.integral_time, self.derivative_time):
            self.history_array.pop(0)

        self.last_input = value
        self.last_output = self.output_multi * (
                value * self.proportional_gain +
                self.calculate_integral() +
                self.calculate_derivative()
        )

        # capping output amplitude
        if self.last_output > self.output_cap:
            return self.output_cap
        if self.last_output < -self.output_cap:
            return -self.output_cap
        return self.last_output
