from bw_tools.get_param import get_param

from bw_behaviors.managers.corner_manager import CornerManager
from bw_behaviors.managers.go_to_goal_manager import GoToGoalManager
from bw_behaviors.managers.mode_manager import ModeManager


class Container:
    def __init__(self) -> None:
        self.corner_offset = get_param("~corner_offset", 0.4)

        self.mode_manager = ModeManager()
        self.corner_manager = CornerManager(self.corner_offset)
        self.go_to_goal_manager = GoToGoalManager()
