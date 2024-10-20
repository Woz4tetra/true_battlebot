from dataclasses import dataclass


@dataclass
class GoToGoalConfig:
    xy_tolerance: float = 0.1
