from dataclasses import dataclass


@dataclass
class RecoverFromDangerConfig:
    angle_tolerance: float = 0.2
    linear_tolerance: float = 2.5
    size_multiplier: float = 1.5
    velocity_threshold: float = 0.1

    linear_magnitude: float = 2.0
    angular_magnitude: float = 10.0
