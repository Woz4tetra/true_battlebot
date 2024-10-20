from typing import Optional

import rospy
from bw_interfaces.msg import GoalEngineConfig
from geometry_msgs.msg import PoseStamped
from py_trees.behaviour import Behaviour
from py_trees.common import Status

from bw_behaviors.container import Container


class SendClickedGoal(Behaviour):
    def __init__(self, container: Container, engine_config: Optional[GoalEngineConfig] = None) -> None:
        super().__init__(self.__class__.__name__)
        self.go_to_goal_manager = container.go_to_goal_manager
        self.engine_config = engine_config
        self.start_time = rospy.Time.now()
        self.goal = PoseStamped()
        self.is_active = False
        self.goal_sent = False
        self.goal_sub = rospy.Subscriber("clicked_goal", PoseStamped, self.goal_callback)
        rospy.logdebug(f"Subscribed to {self.goal_sub.resolved_name}")

    def initialise(self) -> None:
        self.start_time = rospy.Time.now()
        self.is_active = True
        self.goal_sent = False
        self.goal = PoseStamped()
        rospy.loginfo(f"Waiting for goal on {self.goal_sub.resolved_name}")

    def update(self) -> Status:
        if not self.goal_sent:
            return Status.RUNNING
        return self.go_to_goal_manager.get_status()

    def terminate(self, new_status: Status) -> None:
        self.is_active = False
        self.go_to_goal_manager.cancel()
        rospy.loginfo("Cancelled goal")

    def goal_callback(self, msg: PoseStamped) -> None:
        if not self.is_active:
            rospy.logdebug("Received goal while inactive. Ignoring.")
            return
        rospy.loginfo("Received new goal")
        self.go_to_goal_manager.cancel()
        self.go_to_goal_manager.send_pose_goal(msg, self.engine_config)
        self.goal_sent = True
