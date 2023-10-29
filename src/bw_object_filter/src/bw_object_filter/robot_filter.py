from typing import Optional

import rospy
import tf2_ros
from apriltag_ros.msg import AprilTagDetection, AprilTagDetectionArray
from bw_interfaces.msg import EstimatedRobotArray
from bw_tools.dataclass_deserialize import dataclass_deserialize
from bw_tools.structs.transform3d import Transform3D
from bw_tools.transforms import lookup_pose_in_frame
from bw_tools.typing import get_param
from geometry_msgs.msg import (
    Pose,
    PoseStamped,
    PoseWithCovariance,
    Quaternion,
    TransformStamped,
    Twist,
    TwistWithCovariance,
    Vector3,
)
from nav_msgs.msg import Odometry
from std_msgs.msg import Header

from bw_object_filter.filter_models import DriveKalmanModel
from bw_object_filter.robot_measurement_sorter import RobotMeasurementSorter
from bw_object_filter.src.bw_object_filter.covariances import ApriltagHeuristics, CmdVelHeuristics, RobotHeuristics
from bw_object_filter.src.bw_object_filter.robot_config import OUR_TEAM, RobotConfig, RobotFleetConfig


class RobotFilter:
    def __init__(self) -> None:
        robot_config = get_param("robots", None)
        if robot_config is None:
            raise ValueError("Must specify robot_config in the parameter server")

        self.update_rate = get_param("update_rate", 50.0)
        self.update_delay = 1.0 / self.update_rate

        self.map_frame = get_param("map_frame", "map")
        self.robot_frame_prefix = get_param("robot_frame_prefix", "base_link")

        self.apriltag_base_covariance_scalar = get_param("apriltag_base_covariance_scalar", 0.001)
        self.robot_estimate_base_covariance_scalar = get_param("robot_estimate_base_covariance_scalar", 0.01)
        self.cmd_vel_base_covariance_scalar = get_param("cmd_vel_base_covariance_scalar", 0.01)

        self.robots = dataclass_deserialize(RobotFleetConfig, robot_config)
        self.check_unique(self.robots)

        self.robot_names = {bot.id: bot.name for bot in self.robots.robots}

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster()

        self.robots_sub = rospy.Subscriber("estimation/robots", EstimatedRobotArray, self.robot_estimation_callback)
        self.tags_sub = rospy.Subscriber("tag_detections", AprilTagDetectionArray, self.tags_callback)
        self.cmd_vel_subs = [
            rospy.Subscriber(f"{bot.name}/cmd_vel", Twist, self.cmd_vel_callback, callback_args=bot)
            for bot in self.robots.robots
            if bot.team == OUR_TEAM
        ]
        self.filter_state_pubs = {
            bot.id: rospy.Publisher(f"{bot.name}/filter_state", Odometry, queue_size=10) for bot in self.robots.robots
        }

        rotate_quat = (0.5, -0.5, -0.5, -0.5)
        self.apriltag_rotate_tf = Transform3D.from_position_and_quaternion(Vector3(), Quaternion(*rotate_quat))

        self.robot_filters = {bot.id: DriveKalmanModel(self.update_delay) for bot in self.robots.robots}

        self.tag_heurstics = ApriltagHeuristics(self.apriltag_base_covariance_scalar)
        self.robot_heuristics = RobotHeuristics(self.robot_estimate_base_covariance_scalar)
        self.cmd_vel_heuristics = CmdVelHeuristics(self.cmd_vel_base_covariance_scalar)

        self.measurement_sorter = RobotMeasurementSorter(self.robot_filters)

    def check_unique(self, config: RobotFleetConfig) -> None:
        """
        Check that all robot ids are unique.
        """
        ids = [bot.id for bot in config.robots]
        if len(ids) != len(set(ids)):
            raise ValueError("Robot ids must be unique")
        names = [bot.name for bot in config.robots]
        if len(names) != len(set(names)):
            raise ValueError("Robot names must be unique")

    def robot_estimation_callback(self, msg: EstimatedRobotArray) -> None:
        assigned = self.measurement_sorter.get_ids(msg)
        for robot_id, measurement in assigned.items():
            covariance = self.robot_heuristics.compute_covariance(measurement)
            map_pose = self.transform_to_map(measurement.header, measurement.pose)
            if map_pose is None:
                rospy.logwarn(f"Could not transform pose for robot {robot_id}")
                continue
            pose = PoseWithCovariance()
            pose.pose = map_pose
            pose.covariance = covariance
            self.robot_filters[robot_id].update_landmark(pose)

    def tags_callback(self, msg: AprilTagDetectionArray) -> None:
        assert msg.detections is not None
        for detection in msg.detections:
            detection: AprilTagDetection
            pose = detection.pose.pose.pose
            pose.orientation = self.rotate_tag_orientation(pose.orientation, self.apriltag_rotate_tf)

        for detection in msg.detections:
            detection: AprilTagDetection
            covariance = self.tag_heurstics.compute_covariance(detection)
            detection.pose.pose.covariance = covariance
            map_pose = self.transform_to_map(detection.pose.header, detection.pose.pose.pose)
            if map_pose is None:
                rospy.logwarn(f"Could not transform pose for tag {detection}")
                continue
            detection.pose.pose.pose = map_pose

        for detection in msg.detections:
            detection: AprilTagDetection
            if len(detection.id) != 1:
                rospy.logwarn("Bundle detection not supported")
                continue
            tag_id = detection.id[0]
            if tag_id not in self.robot_filters:
                rospy.logwarn(f"Tag id {tag_id} is not a robot")
                continue
            bot_filter = self.robot_filters[tag_id]
            bot_filter.update_landmark(detection.pose.pose)

    def rotate_tag_orientation(
        self,
        tag_orientation: Quaternion,
        rotate_tf: Transform3D,
    ) -> Quaternion:
        tag_tf = Transform3D.from_position_and_quaternion(Vector3(), tag_orientation)
        rotated_tag = tag_tf.transform_by(rotate_tf)
        return rotated_tag.quaternion

    def cmd_vel_callback(self, msg: Twist, robot_config: RobotConfig) -> None:
        measurement = TwistWithCovariance(twist=msg)
        measurement.covariance = self.cmd_vel_heuristics.compute_covariance(measurement)
        self.robot_filters[robot_config.id].update_cmd_vel(measurement)

    def transform_to_map(self, header: Header, pose: Pose) -> Optional[Pose]:
        pose_stamped = PoseStamped(header=header, pose=pose)
        result = lookup_pose_in_frame(self.tf_buffer, pose_stamped, self.map_frame)
        if result is None:
            return None
        else:
            return result.pose

    def predict_all_filters(self) -> None:
        for robot_filter in self.robot_filters.values():
            robot_filter.predict()

    def get_robot_frame_id(self, robot_id: int) -> str:
        return self.robot_frame_prefix + "_" + self.robot_names[robot_id]

    def odom_to_transform(self, odom: Odometry) -> TransformStamped:
        transform = TransformStamped()
        transform.header = odom.header
        transform.child_frame_id = odom.child_frame_id
        transform.transform.translation = Vector3(
            odom.pose.pose.position.x, odom.pose.pose.position.y, odom.pose.pose.position.z
        )
        transform.transform.rotation = odom.pose.pose.orientation
        return transform

    def publish_all_filters(self) -> None:
        transforms = []
        for robot_id, bot_filter in self.robot_filters.items():
            robot_frame_id = self.get_robot_frame_id(robot_id)
            pose, twist = bot_filter.get_state()
            state_msg = Odometry()
            state_msg.header.frame_id = self.map_frame
            state_msg.header.stamp = rospy.Time.now()
            state_msg.child_frame_id = robot_frame_id
            state_msg.pose = pose
            state_msg.twist = twist

            transforms.append(self.odom_to_transform(state_msg))
            self.filter_state_pubs[robot_id].publish(state_msg)

        self.tf_broadcaster.sendTransform(transforms)

    def run(self) -> None:
        rate = rospy.Rate(self.update_rate)
        while not rospy.is_shutdown():
            self.predict_all_filters()
            self.publish_all_filters()
            rate.sleep()
