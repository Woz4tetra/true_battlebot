from typing import Optional

import rospy
from bw_interfaces.msg import GoalEngineConfig
from geometry_msgs.msg import PoseStamped
from py_trees.behaviour import Behaviour
from py_trees.common import Status

from bw_behaviors.container import Container


class SendCornerGoal(Behaviour):
    def __init__(
        self,
        container: Container,
        engine_config: Optional[GoalEngineConfig] = None,
        xy_tolerance: Optional[float] = None,
    ) -> None:
        super().__init__(self.__class__.__name__)
        self.go_to_goal_manager = container.go_to_goal_manager
        self.engine_config = engine_config
        self.xy_tolerance = xy_tolerance
        self.corner_manager = container.corner_manager
        self.start_time = rospy.Time.now()
        self.goal = PoseStamped()
        self.goal_sent = False
        rospy.logdebug("SendCornerGoal is ready")

    def initialise(self) -> None:
        self.start_time = rospy.Time.now()
        self.goal_sent = False
        self.goal = PoseStamped()

    def update(self) -> Status:
        if goal := self.corner_manager.get_goal():
            self.goal = goal

        if not self.goal_sent:
            if self.goal.header.stamp > self.start_time:
                rospy.loginfo("Received new goal")
                self.goal_sent = True
                self.go_to_goal_manager.send_pose_goal(self.goal, self.engine_config, xy_tolerance=self.xy_tolerance)
            else:
                return Status.RUNNING

        return self.go_to_goal_manager.get_status()

    def terminate(self, new_status: Status) -> None:
        self.goal_sent = False
        self.go_to_goal_manager.cancel()
        rospy.loginfo("Cancelled goal")
