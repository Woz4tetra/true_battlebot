from py_trees.behaviour import Behaviour
from py_trees.common import Status

from bw_behaviors.container import Container
from bw_behaviors.modes import Mode


class IsMode(Behaviour):
    def __init__(self, match_mode: Mode, container: Container):
        super().__init__(self.__class__.__name__)
        self.match_mode = match_mode
        self.mode_manager = container.mode_manager

    def update(self) -> Status:
        if self.match_mode == self.mode_manager.mode:
            return Status.SUCCESS
        else:
            return Status.FAILURE
