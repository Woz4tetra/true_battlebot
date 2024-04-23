from py_trees.behaviour import Behaviour
from py_trees.common import Status

from bw_behaviors.container import Container


class StopDriving(Behaviour):
    def __init__(self, container: Container):
        super().__init__(self.__class__.__name__)
        self.cmd_vel_manager = container.cmd_vel_manager

    def update(self) -> Status:
        self.cmd_vel_manager.stop()
        return Status.SUCCESS
