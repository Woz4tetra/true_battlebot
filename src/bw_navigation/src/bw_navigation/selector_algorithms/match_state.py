from dataclasses import dataclass
from functools import cached_property

from bw_interfaces.msg import EstimatedObject
from bw_tools.structs.pose2d import Pose2D
from geometry_msgs.msg import PoseStamped


@dataclass
class MatchState:
    frame_id: str
    controlled_bot: EstimatedObject
    guidance_bot: EstimatedObject
    opponent_bot: EstimatedObject
    field: EstimatedObject

    @cached_property
    def guidance_pose(self) -> Pose2D:
        return Pose2D.from_msg(self.guidance_bot.state.pose.pose)

    @cached_property
    def opponent_pose(self) -> Pose2D:
        return Pose2D.from_msg(self.opponent_bot.state.pose.pose)

    @cached_property
    def controlled_pose(self) -> Pose2D:
        return Pose2D.from_msg(self.controlled_bot.state.pose.pose)

    @cached_property
    def guidance_to_opponent_heading(self) -> float:
        return self.guidance_pose.heading(self.opponent_pose)

    @cached_property
    def guidance_to_opponent_magnitude(self) -> float:
        return self.guidance_pose.magnitude(self.opponent_pose)


@dataclass
class SelectionResult:
    goal: PoseStamped
    ignore_opponent_obstacles: bool
