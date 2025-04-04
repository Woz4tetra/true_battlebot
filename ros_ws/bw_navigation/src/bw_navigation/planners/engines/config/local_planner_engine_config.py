import math
from dataclasses import dataclass


@dataclass
class LocalPlannerEngineConfig:
    obstacle_buffer: float = 0.05  # meters
    slowdown_distance: float = 0.5  # meters
    slowdown_angle_deg: float = 45.0  # degrees

    def __post_init__(self):
        self.slowdown_angle = math.radians(self.slowdown_angle_deg)
