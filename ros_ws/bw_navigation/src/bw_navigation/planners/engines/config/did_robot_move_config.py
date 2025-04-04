from dataclasses import dataclass


@dataclass
class DidRobotMoveConfig:
    move_distance_threshold: float = 0.1  # meters
    move_angle_threshold: float = 5.0  # degrees
    move_timeout: float = 1.0  # seconds
