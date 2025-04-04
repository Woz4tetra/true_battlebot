from dataclasses import dataclass
from typing import List

import numpy as np
from bw_interfaces.msg import EstimatedObject
from bw_shared.geometry.pose2d import Pose2D
from bw_shared.geometry.twist2d import Twist2D
from bw_shared.messages.header import Header
from geometry_msgs.msg import PoseWithCovariance, TwistStamped
from numba import njit
from scipy.interpolate import CubicSpline

from bw_object_filter.extrapolation.extrapolator_interface import ExtrapolatorInterface


@dataclass
class SplineKnot:
    timestamp: float
    pose: Pose2D
    twist: Twist2D


@njit
def rotate_velocities(velocities: np.ndarray, angles: np.ndarray) -> np.ndarray:
    rotated_velocities = np.zeros_like(velocities)
    for index in range(len(velocities)):
        angle = angles[index]
        cos_theta = np.cos(angle)
        sin_theta = np.sin(angle)
        rotation_matrix = np.array(
            [
                [cos_theta, -sin_theta],
                [sin_theta, cos_theta],
            ]
        )
        xy = np.array([velocities[index, 0], velocities[index, 1]])
        rotated_xy = np.dot(rotation_matrix, xy)
        rotated_angle = angle + velocities[index, 2]
        rotated_velocities[index] = np.array([rotated_xy[0], rotated_xy[1], rotated_angle])
    return rotated_velocities


def select_knots(knots: List[SplineKnot], num_knots: int) -> List[SplineKnot]:
    """Selects N evenly spaced knots from the list of knots."""
    if len(knots) <= num_knots:
        return knots
    step = len(knots) / num_knots
    selected_knots = [knots[int(i * step)] for i in range(num_knots)]
    return selected_knots


class SplineExtrapolator(ExtrapolatorInterface):
    def __init__(self, lookahead_time: float, lookback_window: float) -> None:
        self.lookahead_time = lookahead_time
        self.lookback_window = lookback_window
        self.knots: List[SplineKnot] = []
        self.num_knots = 3

    def extrapolate(self, state: EstimatedObject, velocities: list[TwistStamped]) -> EstimatedObject:
        prev_pose_2d = Pose2D.from_msg(state.pose.pose)
        prev_twist_2d = Twist2D.from_msg(state.twist.twist)
        self.knots.append(SplineKnot(state.header.stamp.to_sec(), prev_pose_2d, prev_twist_2d))
        while self.knots and state.header.stamp.to_sec() - self.knots[0].timestamp > self.lookback_window:
            self.knots.pop(0)
        if len(self.knots) < 2:
            return state

        selected_knots = select_knots(self.knots, self.num_knots)
        first_timestamp = selected_knots[0].timestamp
        duration = selected_knots[-1].timestamp - first_timestamp
        timestamps = np.array([knot.timestamp for knot in selected_knots]) - first_timestamp
        poses = np.array([knot.pose.to_array() for knot in selected_knots])
        relative_velocities = np.array([knot.twist.to_array() for knot in selected_knots])
        predicted_velocities = rotate_velocities(relative_velocities, poses[:, 2])

        # Fit cubic splines to the position data using velocity as the first derivative
        spline_pos = CubicSpline(
            timestamps,
            poses,
            bc_type=((1, predicted_velocities[0]), (1, predicted_velocities[-1])),  # type: ignore
        )

        extrapolation_time = duration + self.lookahead_time
        new_pose_array = spline_pos(extrapolation_time)

        new_pose_2d = Pose2D(*new_pose_array)
        new_pose_2d.theta = prev_pose_2d.theta
        new_pose = PoseWithCovariance(pose=new_pose_2d.to_msg(), covariance=state.pose.covariance)
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
