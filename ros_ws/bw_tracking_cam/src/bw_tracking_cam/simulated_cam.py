#!/usr/bin/env python
import rospy
from apriltag_ros.msg import AprilTagDetectionArray
from bw_interfaces.msg import EstimatedObjectArray
from bw_shared.camera_calibration.bundles.bundle_detector import BundleResult
from bw_shared.configs.robot_fleet_config import RobotConfig
from bw_shared.geometry.transform3d import Transform3D
from bw_tools.configs.rosparam_client import get_shared_config
from bw_tools.get_param import get_param

from bw_tracking_cam.bundle_result_helpers import bundle_result_to_object


class SimulatedCam:
    def __init__(self) -> None:
        shared_config = get_shared_config()
        self.ids_to_configs: dict[frozenset[int], RobotConfig] = {}
        for robot_config in shared_config.robots.robots:
            tags = robot_config.tags
            tag_ids = frozenset(tag.tag_id for tag in tags)
            self.ids_to_configs[tag_ids] = robot_config

        self.camera_name = get_param("~camera_name", "camera")

        self.tag_pub = rospy.Publisher(f"{self.camera_name}/robot_tags", EstimatedObjectArray, queue_size=1)
        self.simulated_tag_sub = rospy.Subscriber(
            f"{self.camera_name}/tag_detections", AprilTagDetectionArray, self.simulated_tag_callback
        )

    def apriltag_ros_to_bundle_result(self, msg: AprilTagDetectionArray) -> dict[str, BundleResult]:
        individual_tags: dict[int, Transform3D] = {}
        bundle_poses: dict[frozenset[int], Transform3D] = {}
        for detection in msg.detections:
            tag_ids = frozenset(detection.id)
            if len(tag_ids) == 1:
                individual_tags[detection.id[0]] = Transform3D.from_position_and_quaternion(
                    detection.pose.pose.pose.position,
                    detection.pose.pose.pose.orientation,
                )
            if tag_ids in self.ids_to_configs:
                bundle_poses[tag_ids] = Transform3D.from_position_and_quaternion(
                    detection.pose.pose.pose.position,
                    detection.pose.pose.pose.orientation,
                )
        results = {}
        for bundle_id, bundle_pose in bundle_poses.items():
            if bundle_id not in self.ids_to_configs:
                rospy.logwarn(f"Bundle ID {bundle_id} not found in configs")
                continue
            config = self.ids_to_configs[bundle_id]
            filtered_individual_tags = {
                tag_id: tag_pose for tag_id, tag_pose in individual_tags.items() if tag_id in bundle_id
            }
            result = BundleResult(config.tags, bundle_pose=bundle_pose, tag_poses=filtered_individual_tags)
            results[config.name] = result
        return results

    def simulated_tag_callback(self, msg: AprilTagDetectionArray):
        bundle_results = self.apriltag_ros_to_bundle_result(msg)
        self.tag_pub.publish(bundle_result_to_object(msg.header, bundle_results))

    def run(self):
        rospy.spin()


if __name__ == "__main__":
    rospy.init_node("simulated_cam", log_level=rospy.DEBUG)
    SimulatedCam().run()
