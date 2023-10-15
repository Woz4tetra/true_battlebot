from typing import Optional

import PyKDL
import rospy
import tf2_geometry_msgs
import tf2_ros
from geometry_msgs.msg import Pose, PoseStamped, TransformStamped


def lookup_transform(
    tf_buffer, parent_link, child_link, time_window=None, timeout=None, silent=False
) -> Optional[TransformStamped]:
    """
    Call tf_buffer.lookup_transform. Return None if the look up fails
    """
    if time_window is None:
        time_window = rospy.Time(0)
    else:
        time_window = rospy.Time.now() - time_window

    if timeout is None:
        timeout = rospy.Duration(1.0)  # type: ignore

    try:
        return tf_buffer.lookup_transform(parent_link, child_link, time_window, timeout)
    except (
        tf2_ros.LookupException,  # type: ignore
        tf2_ros.ConnectivityException,  # type: ignore
        tf2_ros.ExtrapolationException,  # type: ignore
        tf2_ros.InvalidArgumentException,  # type: ignore
    ) as e:
        if not silent:
            rospy.logwarn("Failed to look up %s to %s. %s" % (parent_link, child_link, e))
        return None


def lookup_pose_in_frame(
    tf_buffer,
    pose_stamped: PoseStamped,
    destination_frame: str,
    time_window=None,
    timeout=None,
    silent=False,
) -> Optional[PoseStamped]:
    """
    Put pose_stamped into the destination frame. Return None if the look up fails.
    """
    transform = lookup_transform(
        tf_buffer,
        destination_frame,
        pose_stamped.header.frame_id,
        time_window,
        timeout,
        silent,
    )
    if transform is None:
        return None
    return tf2_geometry_msgs.do_transform_pose(pose_stamped, transform)


def lookup_pose(
    tf_buffer,
    child_frame: str,
    parent_frame: str,
    time_window=None,
    timeout=None,
    silent=False,
) -> Optional[PoseStamped]:
    pose_stamped = PoseStamped()
    pose_stamped.header.frame_id = child_frame
    pose_stamped.pose.orientation.w = 1.0
    return lookup_pose_in_frame(tf_buffer, pose_stamped, parent_frame, time_window, timeout, silent)


def transform_pose(relative_pose: Pose, root_pose: PoseStamped) -> PoseStamped:
    """
    Transform pose by pose_transform (puts pose relative to pose_transform)
    """
    root_frame = PyKDL.Frame(
        PyKDL.Rotation.Quaternion(
            root_pose.pose.orientation.x,
            root_pose.pose.orientation.y,
            root_pose.pose.orientation.z,
            root_pose.pose.orientation.w,
        ),
        PyKDL.Vector(
            root_pose.pose.position.x,
            root_pose.pose.position.y,
            root_pose.pose.position.z,
        ),
    )
    offset_frame = PyKDL.Frame(
        PyKDL.Rotation.Quaternion(
            relative_pose.orientation.x,
            relative_pose.orientation.y,
            relative_pose.orientation.z,
            relative_pose.orientation.w,
        ),
        PyKDL.Vector(
            relative_pose.position.x,
            relative_pose.position.y,
            relative_pose.position.z,
        ),
    )
    result_frame = root_frame * offset_frame
    transformed_pose = Pose()
    transformed_pose.position.x = result_frame[(0, 3)]
    transformed_pose.position.y = result_frame[(1, 3)]
    transformed_pose.position.z = result_frame[(2, 3)]
    (
        transformed_pose.orientation.x,
        transformed_pose.orientation.y,
        transformed_pose.orientation.z,
        transformed_pose.orientation.w,
    ) = result_frame.M.GetQuaternion()
    result = PoseStamped()
    result.header = root_pose.header
    result.pose = transformed_pose
    return result
