import math


class MacroPwmFilter:
    """
    A class to simulate a PWM signal but in a much larger time scale.

    The average output over time will be the a percentage value between the active and inactive
    values at the specified duty cycle. The time window is period of one cycle of the PWM signal.
    """

    def __init__(self, cycle_time: float, active_value: float, inactive_value: float = 0.0) -> None:
        self.cycle_time = cycle_time
        self.inactive_value = inactive_value
        self.active_value = active_value
        self.prev_time = 0.0

    def update(self, now: float, value: float) -> float:
        abs_value = abs(value)
        if abs_value > self.active_value or abs_value < self.inactive_value:
            return value
        percent_cycle = abs_value / (self.active_value - self.inactive_value)
        switch_over_time = self.cycle_time * percent_cycle + self.prev_time
        over_cycle_time = self.cycle_time + self.prev_time
        if now > over_cycle_time:
            self.prev_time = now
        if now > switch_over_time:
            return math.copysign(self.inactive_value, value)
        return math.copysign(self.active_value, value)
