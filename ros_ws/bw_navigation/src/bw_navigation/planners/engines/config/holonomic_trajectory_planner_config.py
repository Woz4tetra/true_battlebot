from dataclasses import dataclass, field
from typing import Literal

from bw_navigation.planners.engines.config.holonomic_local_planner_engine_config import (
    HolonomicLocalPlannerEngineConfig,
)
from bw_navigation.planners.engines.config.holonomic_trajectory_global_planner_config import (
    HolonomicTrajectoryGlobalPlannerConfig,
)


@dataclass
class HolonomicTrajectoryPlannerConfig:
    type: Literal["HolonomicTrajectoryPlanner"] = "HolonomicTrajectoryPlanner"
    in_bounds_buffer: float = 0.05

    global_planner: HolonomicTrajectoryGlobalPlannerConfig = field(
        default_factory=HolonomicTrajectoryGlobalPlannerConfig
    )
    local_planner: HolonomicLocalPlannerEngineConfig = field(default_factory=HolonomicLocalPlannerEngineConfig)
