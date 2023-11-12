from bw_tools.typing import get_param

from bw_behaviors.managers.mode_manager import ModeManager


class Container:
    def __init__(self) -> None:
        # self.charging_current_threshold = get_param("~charging_current_threshold", 0.25)

        self.mode_manager = ModeManager()
