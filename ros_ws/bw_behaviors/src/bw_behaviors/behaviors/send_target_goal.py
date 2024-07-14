from py_trees.behaviour import Behaviour
from py_trees.common import Status

from bw_behaviors.container import Container


class SendTargetGoal(Behaviour):
    def __init__(self, container: Container, target_name: str) -> None:
        super().__init__(self.__class__.__name__)
        self.go_to_goal_manager = container.go_to_goal_manager
        self.target_name = target_name

    def initialise(self) -> None:
        self.go_to_goal_manager.send_target_goal(self.target_name)

    def update(self) -> Status:
        return self.go_to_goal_manager.get_status()
