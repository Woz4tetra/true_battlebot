from typing import Iterable

from apriltag_ros.msg import AprilTagDetection, AprilTagDetectionArray
from bw_interfaces.msg import EstimatedObject, EstimatedObjectArray
from bw_shared.camera_calibration.bundles.bundle_detector import BundleResult
from std_msgs.msg import Header


def bundle_result_to_object(header: Header, results: dict[str, BundleResult]) -> EstimatedObjectArray:
    msg = EstimatedObjectArray()
    for name, result in results.items():
        if result.bundle_pose is None:
            continue
        obj = EstimatedObject()
        obj.header = header
        obj.label = name
        obj.pose.pose = result.bundle_pose.to_pose_msg()
        msg.robots.append(obj)
    return msg


def bundle_result_to_apriltag_ros(header: Header, results: Iterable[BundleResult]) -> AprilTagDetectionArray:
    msg = AprilTagDetectionArray()
    msg.header = header
    for result in results:
        if result.bundle_pose is None:
            continue
        tag = AprilTagDetection()
        tag.id = sorted([tag_id for tag_id in result.tag_poses.keys()])
        tag.pose.pose.pose = result.bundle_pose.to_pose_msg()
        tag.pose.header = header
        tag.size = [config.tag_size for config in result.config]
        msg.detections.append(tag)

    return msg
