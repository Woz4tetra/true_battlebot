from dataclasses import dataclass, field
from typing import Literal

from bw_navigation.planners.engines.config.backaway_recover_config import BackawayRecoverConfig
from bw_navigation.planners.engines.config.did_robot_move_config import DidRobotMoveConfig
from bw_navigation.planners.engines.config.recover_from_danger_config import (
    RecoverFromDangerConfig,
)
from bw_navigation.planners.engines.config.thrash_recovery_config import ThrashRecoveryConfig
from bw_shared.pid.config import PidConfig


@dataclass
class PidPlannerConfig:
    type: Literal["PidPlanner"] = "PidPlanner"
    in_bounds_buffer: float = 0.05
    match_angle_distance_threshold: float = 0.1  # meters
    friendly_mirror_proximity: float = 0.25  # meters
    friendly_mirror_magnify: float = 1.5

    backaway_recover: BackawayRecoverConfig = field(default_factory=BackawayRecoverConfig)
    thrash_recovery: ThrashRecoveryConfig = field(default_factory=ThrashRecoveryConfig)
    did_move: DidRobotMoveConfig = field(default_factory=DidRobotMoveConfig)
    linear_pid: PidConfig = field(default_factory=lambda: PidConfig(kp=3.0, ki=0.0, kd=0.0, kf=1.0))
    angular_pid: PidConfig = field(default_factory=lambda: PidConfig(kp=5.0, ki=0.0, kd=0.0, kf=1.0))
    recover_from_danger: RecoverFromDangerConfig = field(default_factory=RecoverFromDangerConfig)
