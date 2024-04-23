import rospy
from py_trees.behaviour import Behaviour
from py_trees.common import Status

from bw_behaviors.container import Container


class WaitForTargetToMove(Behaviour):
    def __init__(self, container: Container, min_next_time_to_move: rospy.Duration, moved_threshold: float) -> None:
        super().__init__(self.__class__.__name__)
        self.recommended_goal_manager = container.recommended_goal_manager
        self.moved_threshold = moved_threshold
        self.min_next_time_to_move = min_next_time_to_move
        self.behavior_start = rospy.Time.now()

    def initialise(self) -> None:
        self.behavior_start = rospy.Time.now()

    def update(self) -> Status:
        if rospy.Time.now() - self.behavior_start < self.min_next_time_to_move:
            return Status.RUNNING
        if self.recommended_goal_manager.distance_to_new_goal() > self.moved_threshold:
            return Status.SUCCESS
        else:
            return Status.RUNNING
