from dataclasses import dataclass, field

from bw_behaviors.config.corner_mode_config import CornerModeConfig
from bw_behaviors.config.go_to_goal_config import GoToGoalConfig


@dataclass
class ContainerConfig:
    corner_offset: float = 0.25
    corner_mode: CornerModeConfig = field(default_factory=CornerModeConfig)
    go_to_goal: GoToGoalConfig = field(default_factory=GoToGoalConfig)
