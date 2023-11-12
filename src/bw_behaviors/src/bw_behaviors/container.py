from bw_tools.typing import get_param

from bw_behaviors.managers.corner_manager import CornerManager
from bw_behaviors.managers.mode_manager import ModeManager


class Container:
    def __init__(self) -> None:
        self.corner_offset = get_param("~corner_offset", 0.25)

        self.mode_manager = ModeManager()
        self.corner_manager = CornerManager(self.corner_offset)
