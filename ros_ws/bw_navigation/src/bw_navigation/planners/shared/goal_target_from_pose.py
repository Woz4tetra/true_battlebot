from bw_interfaces.msg import EstimatedObject
from bw_shared.geometry.pose2d import Pose2D
from geometry_msgs.msg import PoseWithCovariance

from bw_navigation.planners.shared.match_state import MatchState


def goal_target_from_pose(new_goal_pose: Pose2D, match_state: MatchState) -> EstimatedObject:
    goal_target = match_state.goal_target
    return EstimatedObject(
        header=goal_target.header,
        child_frame_id=goal_target.child_frame_id,
        pose=PoseWithCovariance(pose=new_goal_pose.to_msg(), covariance=goal_target.pose.covariance),
        twist=goal_target.twist,
        size=goal_target.size,
        label=goal_target.label,
        keypoints=goal_target.keypoints,
        keypoint_names=goal_target.keypoint_names,
        score=goal_target.score,
    )
