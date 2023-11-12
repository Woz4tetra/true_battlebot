from py_trees.behaviour import Behaviour
from py_trees.common import Status

from bw_behaviors.container import Container


class SetGoalToCorner(Behaviour):
    def __init__(self, container: Container, concurrency_slot: int = 0) -> None:
        super().__init__(self.__class__.__name__)
        self.concurrency_slot = concurrency_slot
        self.get_path_manager = container.get_path_manager
        self.corner_manager = container.corner_manager

    def update(self) -> Status:
        if corner_goal := self.corner_manager.get_goal():
            self.get_path_manager.set_goal(corner_goal, self.concurrency_slot)
            return Status.SUCCESS
        else:
            return Status.FAILURE
