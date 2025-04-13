import math
from dataclasses import dataclass


@dataclass
class MainBotIntentConfig:
    attack_fan_angle: float = 45.0  # degrees
    velocity_threshold: float = 0.35  # m/s

    def __post_init__(self) -> None:
        self.attack_fan_angle_rad = math.radians(self.attack_fan_angle)
