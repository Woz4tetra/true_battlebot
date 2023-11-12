from py_trees.behaviour import Behaviour
from py_trees.common import Status

from bw_behaviors.container import Container
from bw_behaviors.modes import Mode


class SetMode(Behaviour):
    def __init__(self, mode: Mode, container: Container):
        super().__init__(self.__class__.__name__)
        self.mode = mode
        self.mode_manager = container.mode_manager

    def update(self) -> Status:
        self.mode_manager.set_mode(self.mode)
        return Status.SUCCESS
