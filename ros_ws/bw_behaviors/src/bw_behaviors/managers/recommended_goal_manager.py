from typing import Optional

import rospy
from bw_shared.geometry.pose2d import Pose2D
from bw_shared.geometry.pose2d_stamped import Pose2DStamped
from bw_shared.messages.header import Header
from geometry_msgs.msg import PoseStamped


class RecommendedGoalManager:
    def __init__(self) -> None:
        self.goal_pose_sub = rospy.Subscriber("recommended_goal", PoseStamped, self.goal_pose_callback, queue_size=1)
        self.received_goal = Pose2DStamped(Header.auto(""), Pose2D(0.0, 0.0, 0.0))
        self.prev_goal = Pose2DStamped(Header.auto("map"), Pose2D(0.0, 0.0, 0.0))

    def goal_pose_callback(self, msg: PoseStamped) -> None:
        self.received_goal = Pose2DStamped.from_msg(msg)

    def distance_to_new_goal(self) -> float:
        if len(self.received_goal.header.frame_id) == 0:
            return 0.0
        else:
            return self.received_goal.pose.magnitude(self.prev_goal.pose)

    def get_goal(self) -> Optional[Pose2DStamped]:
        if len(self.received_goal.header.frame_id) == 0:
            return None
        else:
            self.prev_goal = self.received_goal
            self.received_goal = Pose2DStamped(Header.auto(""), Pose2D(0.0, 0.0, 0.0))
            return self.prev_goal
