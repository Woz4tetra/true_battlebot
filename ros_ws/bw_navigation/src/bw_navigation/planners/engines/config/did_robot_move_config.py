from dataclasses import dataclass


@dataclass
class DidRobotMoveConfig:
    move_distance_threshold: float = 0.2  # meters
    move_timeout: float = 1.0  # seconds
