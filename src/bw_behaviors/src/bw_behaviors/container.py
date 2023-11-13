from bw_tools.typing import get_param

from bw_behaviors.managers.corner_manager import CornerManager
from bw_behaviors.managers.get_path_manager import GetPathManager
from bw_behaviors.managers.mode_manager import ModeManager
from bw_behaviors.managers.recommended_goal_manager import RecommendedGoalManager


class Container:
    def __init__(self) -> None:
        self.corner_offset = get_param("~corner_offset", 0.4)
        self.goal_xy_tolerance = get_param("~goal_xy_tolerance", 0.15)

        self.mode_manager = ModeManager()
        self.corner_manager = CornerManager(self.corner_offset)
        self.get_path_manager = GetPathManager(self.goal_xy_tolerance)
        self.recommended_goal_manager = RecommendedGoalManager()
