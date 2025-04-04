from dataclasses import dataclass, field
from typing import Literal

from bw_navigation.planners.engines.config.backaway_recover_config import BackawayRecoverConfig
from bw_navigation.planners.engines.config.did_robot_move_config import DidRobotMoveConfig
from bw_navigation.planners.engines.config.local_planner_engine_config import LocalPlannerEngineConfig
from bw_navigation.planners.engines.config.near_goal_config import NearGoalConfig
from bw_navigation.planners.engines.config.overlay_velocity_config import OverlayVelocityConfig
from bw_navigation.planners.engines.config.ramsete_config import RamseteConfig
from bw_navigation.planners.engines.config.recover_from_danger_config import (
    RecoverFromDangerConfig,
)
from bw_navigation.planners.engines.config.rotate_to_angle_config import RotateToAngleConfig
from bw_navigation.planners.engines.config.thrash_recovery_config import ThrashRecoveryConfig
from bw_navigation.planners.engines.config.trajectory_global_planner_config import TrajectoryGlobalPlannerConfig


@dataclass
class TrajectoryPlannerConfig:
    type: Literal["TrajectoryPlanner"] = "TrajectoryPlanner"
    in_bounds_buffer: float = 0.05

    backaway_recover: BackawayRecoverConfig = field(default_factory=BackawayRecoverConfig)
    global_planner: TrajectoryGlobalPlannerConfig = field(default_factory=TrajectoryGlobalPlannerConfig)
    local_planner: LocalPlannerEngineConfig = field(default_factory=LocalPlannerEngineConfig)
    ramsete: RamseteConfig = field(default_factory=RamseteConfig)
    thrash_recovery: ThrashRecoveryConfig = field(default_factory=ThrashRecoveryConfig)
    recover_from_danger: RecoverFromDangerConfig = field(default_factory=RecoverFromDangerConfig)
    rotate_to_angle: RotateToAngleConfig = field(default_factory=RotateToAngleConfig)
    near_goal: NearGoalConfig = field(default_factory=NearGoalConfig)
    did_move: DidRobotMoveConfig = field(default_factory=DidRobotMoveConfig)
    overlay_velocity: OverlayVelocityConfig = field(default_factory=OverlayVelocityConfig)
