import rospy
from bw_tools.messages.target_type import TargetType
from py_trees.behaviour import Behaviour
from py_trees.common import Status

from bw_behaviors.container import Container


class SendTargetGoal(Behaviour):
    def __init__(self, container: Container, target_type: TargetType) -> None:
        super().__init__(self.__class__.__name__)
        self.go_to_goal_manager = container.go_to_goal_manager
        self.target_type = target_type
        rospy.logdebug("SendTargetGoal is ready")

    def initialise(self) -> None:
        self.go_to_goal_manager.send_target_goal(self.target_type)

    def update(self) -> Status:
        return self.go_to_goal_manager.get_status()

    def terminate(self, new_status: Status) -> None:
        self.goal_sent = False
        self.go_to_goal_manager.cancel()
        rospy.loginfo("Cancelled goal")