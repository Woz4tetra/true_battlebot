import math
import time


class MacroPwm:
    def __init__(self, cycle_time: float, neutral_value: float, min_value: float, max_value: float) -> None:
        self.cycle_time = cycle_time
        self.neutral_value = neutral_value
        self.min_value = min_value
        self.max_value = max_value
        self.prev_time = time.perf_counter()

    def update(self, value: float) -> float:
        if not (self.min_value <= value <= self.max_value):
            return value
        now = time.perf_counter()
        max_value = self.max_value if value > self.neutral_value else self.min_value
        percent_cycle = value / (max_value - self.neutral_value)
        switch_over_time = self.cycle_time * percent_cycle + self.prev_time
        over_cycle_time = self.cycle_time + self.prev_time
        if now > over_cycle_time:
            self.prev_time = now
        if now > switch_over_time:
            return math.copysign(self.neutral_value, value)
        return math.copysign(max_value, value)
