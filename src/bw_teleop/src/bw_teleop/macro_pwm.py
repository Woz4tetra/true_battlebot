import math
import time


class MacroPwm:
    def __init__(self, cycle_time: float, min_value: float, max_value: float) -> None:
        self.cycle_time = cycle_time
        self.min_value = min_value
        self.max_value = max_value
        self.prev_time = time.perf_counter()

    def update(self, value: float) -> float:
        abs_value = abs(value)
        if abs_value > self.max_value or abs_value < self.min_value:
            return value
        now = time.perf_counter()
        percent_cycle = abs_value / (self.max_value - self.min_value)
        switch_over_time = self.cycle_time * percent_cycle + self.prev_time
        over_cycle_time = self.cycle_time + self.prev_time
        if now > over_cycle_time:
            self.prev_time = now
        if now > switch_over_time:
            return math.copysign(self.min_value, value)
        return math.copysign(self.max_value, value)
