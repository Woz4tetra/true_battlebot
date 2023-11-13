from py_trees.behaviour import Behaviour
from py_trees.common import Status

from bw_behaviors.container import Container


class SetGoalToOptimalAttack(Behaviour):
    def __init__(self, container: Container, concurrency_slot: int = 0) -> None:
        super().__init__(self.__class__.__name__)
        self.concurrency_slot = concurrency_slot
        self.get_path_manager = container.get_path_manager
        self.recommended_goal_manager = container.recommended_goal_manager

    def update(self) -> Status:
        if goal := self.recommended_goal_manager.get_goal():
            self.get_path_manager.set_goal(goal, self.concurrency_slot)
            return Status.SUCCESS
        else:
            return Status.RUNNING
