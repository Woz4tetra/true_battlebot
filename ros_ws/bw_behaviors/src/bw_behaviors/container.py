from bw_behaviors.config.container_config import ContainerConfig
from bw_behaviors.managers.corner_manager import CornerManager
from bw_behaviors.managers.go_to_goal_manager import GoToGoalManager
from bw_behaviors.managers.mode_manager import ModeManager


class Container:
    def __init__(self, config: ContainerConfig) -> None:
        self.config = config
        self.mode_manager = ModeManager()
        self.corner_manager = CornerManager(config.corner_offset)
        self.go_to_goal_manager = GoToGoalManager()
