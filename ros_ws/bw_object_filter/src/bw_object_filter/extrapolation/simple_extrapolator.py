from bw_interfaces.msg import EstimatedObject
from bw_shared.geometry.pose2d import Pose2D
from bw_shared.messages.header import Header
from geometry_msgs.msg import PoseWithCovariance

from bw_object_filter.extrapolation.extrapolator_interface import ExtrapolatorInterface


class SimpleExtrapolator(ExtrapolatorInterface):
    def __init__(self, lookahead_time: float) -> None:
        self.lookahead_time = lookahead_time

    def extrapolate(self, state: EstimatedObject) -> EstimatedObject:
        prev_pose_2d = Pose2D.from_msg(state.pose.pose)
        vx = state.twist.twist.linear.x
        distance = vx * self.lookahead_time
        transform = Pose2D(distance, 0, 0).transform_by(Pose2D(0, 0, prev_pose_2d.theta))
        new_pose_2d = prev_pose_2d.forward_by(transform)
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
