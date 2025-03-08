from dataclasses import dataclass
from typing import List

import numpy as np
from bw_interfaces.msg import EstimatedObject
from bw_shared.geometry.pose2d import Pose2D
from bw_shared.geometry.twist2d import Twist2D
from bw_shared.messages.header import Header
from geometry_msgs.msg import PoseWithCovariance
from scipy.interpolate import CubicSpline

from bw_object_filter.extrapolation.extrapolator_interface import ExtrapolatorInterface


@dataclass
class SplineKnot:
    timestamp: float
    pose: Pose2D
    twist: Twist2D


class SplineExtrapolator(ExtrapolatorInterface):
    def __init__(self, lookahead_time: float, lookback_window: float) -> None:
        self.lookahead_time = lookahead_time
        self.lookback_window = lookback_window
        self.knots: List[SplineKnot] = []

    def extrapolate(self, state: EstimatedObject) -> EstimatedObject:
        while self.knots and state.header.stamp.to_sec() - self.knots[0].timestamp > self.lookback_window:
            self.knots.pop(0)
        prev_pose_2d = Pose2D.from_msg(state.pose.pose)
        prev_twist_2d = Twist2D.from_msg(state.twist.twist)
        self.knots.append(SplineKnot(state.header.stamp.to_sec(), prev_pose_2d, prev_twist_2d))
        first_timestamp = self.knots[0].timestamp
        duration = self.knots[-1].timestamp - first_timestamp
        timestamps = np.array([knot.timestamp for knot in self.knots]) - first_timestamp
        poses = np.array([knot.pose.to_array() for knot in self.knots])
        velocities = np.array([knot.twist.to_array() for knot in self.knots])

        # Fit cubic splines to the position data using velocity as the first derivative
        spline_pos = CubicSpline(timestamps, poses, bc_type=((1, velocities[0, 0]), (1, velocities[-1, 0])))  # type: ignore

        extrapolation_time = duration + self.lookahead_time
        new_pose_array = spline_pos(extrapolation_time)

        new_pose = PoseWithCovariance(pose=Pose2D(*new_pose_array).to_msg(), covariance=state.pose.covariance)
        header = Header(state.header.stamp.to_sec() + self.lookahead_time, state.header.frame_id, state.header.seq)

        return EstimatedObject(
            header=header.to_msg(),
            child_frame_id=state.child_frame_id,
            pose=new_pose,
            twist=state.twist,
            size=state.size,
            label=state.label,
            keypoints=state.keypoints,
            keypoint_names=state.keypoint_names,
            score=state.score,
        )
