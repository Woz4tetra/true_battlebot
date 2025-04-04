from dataclasses import dataclass
from functools import cached_property

from bw_interfaces.msg import EstimatedObject
from bw_shared.geometry.field_bounds import FieldBounds2D
from bw_shared.geometry.pose2d import Pose2D
from bw_shared.geometry.pose2d_stamped import Pose2DStamped
from bw_shared.geometry.xy import XY
from bw_shared.messages.header import Header


@dataclass
class MatchState:
    goal_target: EstimatedObject
    robot_states: dict[str, EstimatedObject]
    field_bounds: FieldBounds2D
    controlled_robot_name: str
    friendly_robot_name: str
    avoid_robot_names: list[str]

    @cached_property
    def controlled_robot(self) -> EstimatedObject:
        return self.robot_states[self.controlled_robot_name]

    @cached_property
    def controlled_robot_pose(self) -> Pose2D:
        return Pose2D.from_msg(self.controlled_robot.pose.pose)

    @cached_property
    def controlled_robot_pose_stamped(self) -> Pose2DStamped:
        return Pose2DStamped(Header.from_msg(self.controlled_robot.header), self.controlled_robot_pose)

    @cached_property
    def controlled_robot_point(self) -> XY:
        return XY(self.controlled_robot_pose.x, self.controlled_robot_pose.y)

    @cached_property
    def controlled_robot_width(self) -> float:
        return max(self.controlled_robot.size.x, self.controlled_robot.size.y)

    @cached_property
    def goal_pose(self) -> Pose2D:
        return Pose2D.from_msg(self.goal_target.pose.pose)

    @cached_property
    def goal_point(self) -> XY:
        return XY(self.goal_pose.x, self.goal_pose.y)

    @cached_property
    def distance_to_goal(self) -> float:
        return self.relative_goal.magnitude()

    @cached_property
    def avoid_robot_states(self) -> list[EstimatedObject]:
        return [state for name, state in self.robot_states.items() if (name in self.avoid_robot_names)]

    @cached_property
    def opponent_robot_states(self) -> list[EstimatedObject]:
        return [
            state
            for name, state in self.robot_states.items()
            if (name not in self.avoid_robot_names and name != self.controlled_robot_name)
        ]

    @cached_property
    def friendly_robot(self) -> EstimatedObject:
        return self.robot_states[self.friendly_robot_name]

    @cached_property
    def friendly_robot_pose(self) -> Pose2D:
        return Pose2D.from_msg(self.friendly_robot.pose.pose)

    @cached_property
    def friendly_robot_point(self) -> XY:
        return XY(self.friendly_robot_pose.x, self.friendly_robot_pose.y)

    @cached_property
    def relative_goal(self) -> Pose2D:
        return self.goal_pose.relative_to(self.controlled_robot_pose)
