import numpy as np
from bw_shared.geometry.pose2d_stamped import Pose2DStamped

from bw_navigation.planners.engines.config.did_robot_move_config import DidRobotMoveConfig


class DidRobotMoveEngine:
    def __init__(self, config: DidRobotMoveConfig) -> None:
        self.move_timeout = config.move_timeout
        self.move_distance_threshold = config.move_distance_threshold

    def reset(self) -> None:
        self.prev_move_time = 0.0
        self.prev_poses: list[Pose2DStamped] = []

    def did_move(self, controlled_robot_pose: Pose2DStamped) -> bool:
        self.prev_poses.append(controlled_robot_pose)
        now = controlled_robot_pose.header.stamp
        while self.prev_poses[-1].header.stamp - self.prev_poses[0].header.stamp > self.move_timeout:
            self.prev_poses.pop(0)
        earliest_pose = self.prev_poses[0].pose
        relative_poses = np.array(
            [(pose.pose.x - earliest_pose.x, pose.pose.y - earliest_pose.y) for pose in self.prev_poses]
        )
        position_spread = np.abs(np.ptp(relative_poses, axis=0))
        if position_spread[0] > self.move_distance_threshold or position_spread[1] > self.move_distance_threshold:
            return True
        return now - self.prev_move_time < self.move_timeout
