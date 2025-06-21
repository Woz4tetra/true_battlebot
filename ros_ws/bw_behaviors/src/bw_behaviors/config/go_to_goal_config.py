from dataclasses import dataclass

from bw_tools.messages.goal_strategy import GoalStrategy


@dataclass
class GoToGoalConfig:
    xy_tolerance: float = 0.01
    strategy: GoalStrategy = GoalStrategy.MIRROR_FRIENDLY
