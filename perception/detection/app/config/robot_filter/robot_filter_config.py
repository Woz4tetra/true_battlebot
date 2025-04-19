from dataclasses import dataclass, field
from typing import List

from bw_shared.enums.frame_id import FrameId


@dataclass
class RobotFilterConfig:
    update_rate: float = 50.0
    map_frame: FrameId = FrameId.MAP
    command_timeout: float = 0.1
    initial_variances: List[float] = field(default_factory=lambda: [0.25, 0.25, 10.0, 1.0, 1.0, 10.0])
    robot_position_covariance: float = 0.01
    robot_orientation_covariance: float = 0.01
    cmd_vel_base_covariance_scalar: float = 0.1
    process_noise: float = 1e-4
    stale_timeout: float = 10.0
    robot_min_radius: float = 0.1
    robot_max_radius: float = 0.4
    field_buffer: float = 0.2
