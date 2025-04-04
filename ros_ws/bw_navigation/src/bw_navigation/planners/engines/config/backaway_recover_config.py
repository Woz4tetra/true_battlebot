import math
from dataclasses import dataclass


@dataclass
class BackawayRecoverConfig:
    linear_velocity: float = 1.0  # m/s
    rotate_velocity: float = 8.0  # rad/s
    angle_tolerance_deg: float = 30.0  # degrees

    def __post_init__(self):
        self.angle_tolerance = math.radians(self.angle_tolerance_deg)
