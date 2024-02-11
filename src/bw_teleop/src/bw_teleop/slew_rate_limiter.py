import math
import time
from typing import Optional


class SlewLimiter:
    def __init__(
        self, acceleration_limit: float, deceleration_limit: Optional[float] = None, initial_value: float = 0.0
    ):
        if deceleration_limit is None:
            deceleration_limit = acceleration_limit
        self.accel_limit = abs(acceleration_limit)
        self.decel_limit = -abs(deceleration_limit)
        self.prev_time = 0.0
        self.prev_value = initial_value

    @staticmethod
    def clamp(value, lower, upper):
        return max(lower, min(value, upper))

    def calculate(self, input):
        current_time = self.get_time()
        dt = current_time - self.prev_time
        self.prev_time = current_time
        self.prev_value += self.clamp(input - self.prev_value, self.decel_limit * dt, self.accel_limit * dt)
        return self.prev_value

    @staticmethod
    def get_time():
        return time.time()

    def reset(self, value):
        self.prev_value = value
        self.prev_time = self.get_time()
