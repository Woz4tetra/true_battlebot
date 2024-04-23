import rospy
from py_trees.behaviour import Behaviour
from py_trees.common import Status

from bw_behaviors.container import Container


class GetPath(Behaviour):
    def __init__(self, container: Container, clear_goal_after_send: bool = True) -> None:
        super().__init__(self.__class__.__name__)
        self.clear_goal_after_send = clear_goal_after_send
        self.get_path_manager = container.get_path_manager
        self.goal_sent = False

    def initialise(self) -> None:
        self.goal_sent = self.get_path_manager.send_goal(self.clear_goal_after_send)

    def update(self) -> Status:
        if not self.goal_sent:
            return Status.FAILURE
        return self.get_path_manager.get_status()

    def terminate(self, new_status: Status) -> None:
        if self.status == Status.RUNNING:
            rospy.loginfo("Canceling MBF get path action")
            self.get_path_manager.cancel()
