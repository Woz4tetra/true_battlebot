from dataclasses import dataclass


@dataclass
class ThrashRecoveryConfig:
    direction_change_interval: float = 0.5
    linear_magnitude: float = 4.0
    angular_magnitude: float = 25.0
