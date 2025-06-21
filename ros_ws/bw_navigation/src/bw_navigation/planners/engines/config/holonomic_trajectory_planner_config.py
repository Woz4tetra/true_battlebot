from dataclasses import dataclass
from typing import Literal


@dataclass
class HolonomicTrajectoryPlannerConfig:
    type: Literal["HolonomicTrajectoryPlanner"] = "HolonomicTrajectoryPlanner"
