import genpy
from bw_interfaces.msg import EstimatedObject
from bw_shared.geometry.pose2d import Pose2D
from bw_shared.geometry.twist2d import Twist2D
from bw_shared.messages.header import Header
from geometry_msgs.msg import Pose, PoseWithCovariance, TwistStamped

from bw_object_filter.extrapolation.extrapolator_interface import ExtrapolatorInterface


class SimpleExtrapolator(ExtrapolatorInterface):
    def __init__(self, lookahead_time: genpy.Duration) -> None:
        self.lookahead_time = lookahead_time.to_sec()

    def integrate_velocities(self, start_pose: Pose, velocities: list[TwistStamped]) -> Pose:
        if len(velocities) <= 1:
            return start_pose
        pose_2d = Pose2D.from_msg(start_pose)
        for index in range(len(velocities) - 1):
            next_velocity = Twist2D.from_msg(velocities[index + 1].twist)
            current_time = velocities[index].header.stamp.to_sec()
            next_time = velocities[index + 1].header.stamp.to_sec()
            delta_time = next_time - current_time
            if delta_time <= 0:
                continue

            relative_pose = Pose2D(
                next_velocity.x * delta_time,
                next_velocity.y * delta_time,
                next_velocity.theta * delta_time,
            )
            pose_2d = pose_2d.forward_by(relative_pose)
        return pose_2d.to_msg()

    def extrapolate(self, state: EstimatedObject, velocities: list[TwistStamped]) -> EstimatedObject:
        extrapolated_pose = self.integrate_velocities(state.pose.pose, velocities)
        new_pose = PoseWithCovariance(pose=extrapolated_pose, covariance=state.pose.covariance)
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
