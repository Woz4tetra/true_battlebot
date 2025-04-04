import math
from dataclasses import dataclass


@dataclass
class RecoverFromDangerConfig:
    danger_fan_angle: float = 90.0  # degrees
    velocity_threshold: float = 0.1
    reaction_time: float = 0.5  # seconds

    linear_magnitude: float = 2.0
    angular_magnitude: float = 10.0

    def __post_init__(self) -> None:
        self.danger_fan_angle_rad = math.radians(self.danger_fan_angle)
