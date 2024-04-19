from bw_tools.get_param import get_param

from bw_behaviors.managers.cmd_vel_manager import CmdVelManager
from bw_behaviors.managers.corner_manager import CornerManager
from bw_behaviors.managers.get_path_manager import GetPathManager
from bw_behaviors.managers.mode_manager import ModeManager
from bw_behaviors.managers.recommended_goal_manager import RecommendedGoalManager
from bw_behaviors.managers.recovery_manager import RecoveryManager


class Container:
    def __init__(self) -> None:
        self.corner_offset = get_param("~corner_offset", 0.4)
        self.goal_xy_tolerance = get_param("~goal_xy_tolerance", 0.15)
        self.replan_interval = get_param("~replan_interval", 3.0)

        self.mode_manager = ModeManager()
        self.corner_manager = CornerManager(self.corner_offset)
        self.get_path_manager = GetPathManager(self.goal_xy_tolerance)
        self.recommended_goal_manager = RecommendedGoalManager()
        self.recovery_manager = RecoveryManager()
        self.cmd_vel_manager = CmdVelManager()
