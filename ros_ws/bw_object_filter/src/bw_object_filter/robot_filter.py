#!/usr/bin/env python
from typing import List, Optional

import numpy as np
import rospy
import tf2_ros
from apriltag_ros.msg import AprilTagDetectionArray
from bw_interfaces.msg import EstimatedObject, EstimatedObjectArray
from bw_shared.configs.robot_fleet_config import RobotConfig, RobotFleetConfig
from bw_shared.enums.label import Label
from bw_shared.enums.robot_team import RobotTeam
from bw_tools.configs.rosparam_client import get_shared_config
from bw_tools.get_param import get_param
from bw_tools.structs.rpy import RPY
from bw_tools.structs.transform3d import Transform3D
from bw_tools.structs.xyz import XYZ
from bw_tools.transforms import lookup_pose_in_frame
from geometry_msgs.msg import (
    Point,
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
from std_msgs.msg import Header as RosHeader

from bw_object_filter.covariances import ApriltagHeuristics, CmdVelHeuristics, RobotHeuristics
from bw_object_filter.filter_models import DriveKalmanModel
from bw_object_filter.robot_measurement_sorter import RobotMeasurementSorter


class RobotFilter:
    def __init__(self) -> None:
        shared_config = get_shared_config()

        self.update_rate = get_param("~update_rate", 50.0)
        self.update_delay = 1.0 / self.update_rate

        self.map_frame = get_param("~map_frame", "map")
        self.robot_frame_prefix = get_param("~robot_frame_prefix", "base_link")
        self.controlled_robot_name = get_param("/robot", "mini_bot")

        self.command_timeout = rospy.Duration.from_sec(get_param("~command_timeout", 0.5))

        self.apriltag_base_covariance_scalar = get_param("~apriltag_base_covariance_scalar", 0.00001)
        self.our_base_covariance = get_param("~our_robot_estimate_base_covariance_scalar", 0.001)
        self.their_base_covariance = get_param("~their_robot_estimate_base_covariance_scalar", 0.001)
        self.cmd_vel_base_covariance_scalar = get_param("~cmd_vel_base_covariance_scalar", 0.01)
        self.friction_factor = get_param("~friction_factor", 0.05)
        self.process_noise = get_param("~process_noise", 1e-4)
        self.motion_speed_threshold = get_param("~motion_speed_threshold", 0.25)
        self.robot_min_radius = get_param("~robot_min_radius", 0.1)
        self.ignore_measurement_near_tag_threshold = get_param("~ignore_measurement_near_tag_threshold", 0.05)
        field_buffer = get_param("~field_buffer", 0.1)
        self.field_buffer = XYZ(field_buffer, field_buffer, field_buffer)

        self.robots = shared_config.robots
        self.check_unique(self.robots)

        self.prev_cmd_time = rospy.Time.now()
        self.field = EstimatedObject()
        self.field_bounds = (XYZ(0.0, 0.0, 0.0), XYZ(0.0, 0.0, 0.0))
        self.robot_names = {}
        for config in self.robots.robots:
            self.robot_names[config.up_id] = config.name
            self.robot_names[config.down_id] = config.name
        self.robot_configs = {config.name: config for config in self.robots.robots}
        self.apriltag_rotate_tf = Transform3D.from_position_and_rpy(Vector3(), RPY((0.0, 0.0, np.pi / 2)))
        self.robot_filters = {
            robot.name: DriveKalmanModel(self.update_delay, self.process_noise, self.friction_factor)
            for robot in self.robots.robots
        }
        self.robot_sizes = {robot.name: robot.radius for robot in self.robots.robots}
        self.robot_max_sizes = {robot.name: robot.radius for robot in self.robots.robots}

        self.tag_heurstics = ApriltagHeuristics(self.apriltag_base_covariance_scalar)
        self.our_robot_heuristics = RobotHeuristics(self.our_base_covariance)
        self.their_robot_heuristics = RobotHeuristics(self.their_base_covariance)
        self.our_robot_cmd_vel_heuristics = CmdVelHeuristics(self.cmd_vel_base_covariance_scalar)

        self.measurement_sorters = {
            Label.ROBOT: RobotMeasurementSorter(
                {
                    name: filter
                    for name, filter in self.robot_filters.items()
                    if self.robot_configs[name].team != RobotTeam.REFEREE
                }
            ),
            Label.REFEREE: RobotMeasurementSorter(
                {
                    name: filter
                    for name, filter in self.robot_filters.items()
                    if self.robot_configs[name].team == RobotTeam.REFEREE
                }
            ),
            Label.FRIENDLY_ROBOT: RobotMeasurementSorter(
                {
                    name: filter
                    for name, filter in self.robot_filters.items()
                    if self.robot_configs[name].team == RobotTeam.OUR_TEAM
                }
            ),
            Label.CONTROLLED_ROBOT: RobotMeasurementSorter(
                {
                    name: filter
                    for name, filter in self.robot_filters.items()
                    if self.robot_configs[name].name == self.controlled_robot_name
                }
            ),
        }

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster()

        self.filter_state_pubs = {
            robot.name: rospy.Publisher(f"{robot.name}/odom", Odometry, queue_size=10) for robot in self.robots.robots
        }
        self.filter_state_array_pub = rospy.Publisher("filtered_states", EstimatedObjectArray, queue_size=50)

        self.robots_sub = rospy.Subscriber(
            "estimation/robots", EstimatedObjectArray, self.robot_estimation_callback, queue_size=50
        )
        self.field_sub = rospy.Subscriber("filter/field", EstimatedObject, self.field_callback, queue_size=1)
        self.tags_sub = rospy.Subscriber("tag_detections", AprilTagDetectionArray, self.tags_callback, queue_size=25)
        self.cmd_vel_subs = [
            rospy.Subscriber(f"{robot.name}/cmd_vel", Twist, self.cmd_vel_callback, callback_args=robot, queue_size=10)
            for robot in self.robots.robots
            if robot.team == RobotTeam.OUR_TEAM
        ]

    def check_unique(self, config: RobotFleetConfig) -> None:
        """
        Check that all robot ids are unique.
        """
        ids = []
        for robot in config.robots:
            if robot.team == RobotTeam.OUR_TEAM:
                ids.append(robot.up_id)
                ids.append(robot.down_id)
        if len(ids) != len(set(ids)):
            raise ValueError("Robot ids must be unique")
        names = [bot.name for bot in config.robots]
        if len(names) != len(set(names)):
            raise ValueError("Robot names must be unique")

    def robot_estimation_callback(self, msg: EstimatedObjectArray) -> None:
        if not self.field_received():
            rospy.logdebug("Field not received. Skipping robot estimation callback.")
            return
        measurements: dict[Label, list[EstimatedObject]] = {label: [] for label in self.measurement_sorters.keys()}
        for robot in msg.robots:
            try:
                label = Label(robot.label)
            except ValueError:
                rospy.logwarn(f"Unknown label {robot.label}")
                continue
            measurements[label].append(robot)
        for label, sorter in self.measurement_sorters.items():
            self.apply_update_with_sorter(sorter, measurements[label])

    def apply_update_with_sorter(
        self, measurement_sorter: RobotMeasurementSorter, robots: List[EstimatedObject]
    ) -> None:
        map_measurements: List[Pose] = []
        for measurement in robots:
            map_pose = self.transform_to_map(measurement.header, measurement.pose.pose)
            if map_pose is None:
                rospy.logwarn(f"Could not transform pose for measurement {measurement}")
                continue
            if not self.is_in_field_bounds(map_pose.position):
                rospy.logdebug(f"{measurement.label} is out of bounds. Skipping.")
                continue
            map_measurements.append(map_pose)

        assigned = measurement_sorter.get_ids(map_measurements)
        for robot_name, measurement_index in assigned.items():
            camera_measurement = robots[measurement_index]
            map_measurement = map_measurements[measurement_index]
            self.apply_sorted_measurement(robot_name, map_measurement, camera_measurement)

    def apply_sorted_measurement(
        self, robot_name: str, map_measurement: Pose, camera_measurement: EstimatedObject
    ) -> None:
        robot_filter = self.robot_filters[robot_name]
        robot_config = self.robot_configs[robot_name]
        if robot_config.team == RobotTeam.OUR_TEAM:
            covariance = self.our_robot_heuristics.compute_covariance(camera_measurement)
        else:
            covariance = self.their_robot_heuristics.compute_covariance(camera_measurement)
            self.robot_sizes[robot_name] = self.get_object_radius(robot_name, camera_measurement)

        pose = PoseWithCovariance()
        pose.pose = map_measurement
        pose.covariance = covariance  # type: ignore
        try:
            robot_filter.update_position(pose)
        except np.linalg.LinAlgError as e:
            rospy.logwarn(f"Failed to update from robot measurement. Resetting filter. {e}")
            robot_filter.reset()

    def get_object_radius(self, robot_name: str, object_estimate: EstimatedObject) -> float:
        measured_radius = (
            max(
                object_estimate.size.x,
                object_estimate.size.y,
                object_estimate.size.z,
            )
            / 2.0
        )
        return min(
            max(
                measured_radius,
                self.robot_min_radius,
            ),
            self.robot_max_sizes[robot_name],
        )

    def tags_callback(self, msg: AprilTagDetectionArray) -> None:
        if not self.field_received():
            rospy.logdebug("Field not received. Skipping tag callback.")
            return
        self.transform_tags_to_map(msg)
        self.apply_tag_measurement(msg)

    def transform_tags_to_map(self, msg: AprilTagDetectionArray) -> None:
        for detection in msg.detections:
            pose = detection.pose.pose.pose
            pose.orientation = self.rotate_tag_orientation(pose.orientation, self.apriltag_rotate_tf)
            covariance = self.tag_heurstics.compute_covariance(detection)
            detection.pose.pose.covariance = covariance  # type: ignore
            map_pose = self.transform_to_map(detection.pose.header, detection.pose.pose.pose)
            if map_pose is None:
                rospy.logwarn(f"Could not transform pose for tag {detection}")
                continue
            detection.pose.pose.pose = map_pose

    def apply_tag_measurement(self, msg: AprilTagDetectionArray) -> None:
        for detection in msg.detections:
            pose = detection.pose.pose.pose
            if not self.is_in_field_bounds(pose.position):
                rospy.logdebug(f"Tag {detection.id} is out of bounds. Skipping.")
                continue
            if len(detection.id) != 1:
                rospy.logwarn("Bundle detection not supported")
                continue
            tag_id = detection.id[0]
            robot_name = self.robot_names.get(tag_id, "")
            if robot_name not in self.robot_filters:
                rospy.logwarn(f"Tag id {tag_id} is not a robot")
                continue
            robot_filter = self.robot_filters[robot_name]
            tag_pose = detection.pose.pose
            try:
                robot_filter.update_pose(tag_pose)
            except np.linalg.LinAlgError as e:
                rospy.logwarn(f"Failed to update from tag. Resetting filter. {e}")
                robot_filter.reset()
                continue

    def rotate_tag_orientation(
        self,
        tag_orientation: Quaternion,
        rotate_tf: Transform3D,
    ) -> Quaternion:
        tag_tf = Transform3D.from_position_and_quaternion(Vector3(), tag_orientation)
        rotated_tag = tag_tf.transform_by(rotate_tf)
        return rotated_tag.quaternion

    def cmd_vel_callback(self, msg: Twist, robot_config: RobotConfig) -> None:
        self.update_cmd_vel(msg, robot_config)
        self.prev_cmd_time = rospy.Time.now()

    def update_cmd_vel(self, msg: Twist, robot_config: RobotConfig):
        if not self.field_received():
            return
        measurement = TwistWithCovariance(twist=msg)
        if robot_config.team == RobotTeam.OUR_TEAM:
            measurement.covariance = self.our_robot_cmd_vel_heuristics.compute_covariance(measurement)  # type: ignore
        else:
            measurement.covariance = self.our_robot_cmd_vel_heuristics.compute_covariance(measurement)  # type: ignore

        robot_name = robot_config.name
        robot_filter = self.robot_filters[robot_name]
        try:
            robot_filter.update_cmd_vel(measurement)
        except np.linalg.LinAlgError as e:
            rospy.logwarn(f"Failed to update from velocity. Resetting filter. {e}")
            robot_filter.reset()

    def field_callback(self, msg: EstimatedObject) -> None:
        self.field = msg
        half_x = msg.size.x / 2
        half_y = msg.size.y / 2
        self.field_bounds = (
            XYZ(-half_x, -half_y, 0.0) - self.field_buffer,
            XYZ(half_x, half_y, msg.size.z) + self.field_buffer,
        )
        rospy.loginfo("Resetting filters.")
        rospy.sleep(0.2)  # wait for changes in TF tree to propagate
        for robot_name, robot_filter in self.robot_filters.items():
            robot_filter.reset()

    def field_received(self) -> bool:
        return self.field.header.stamp != rospy.Time(0)

    def is_in_field_bounds(self, position: Point) -> bool:
        if not self.field_received():
            rospy.logwarn("Field not received. Skipping field bounds check.")
            return False
        xyz = XYZ.from_msg(Vector3(position.x, position.y, position.z))
        return self.field_bounds[0] < xyz < self.field_bounds[1]

    def transform_to_map(self, header: RosHeader, pose: Pose) -> Optional[Pose]:
        pose_stamped = PoseStamped(header=header, pose=pose)
        result = lookup_pose_in_frame(self.tf_buffer, pose_stamped, self.map_frame)
        if result is None:
            return None
        else:
            return result.pose

    def check_cmd_timeout(self) -> None:
        if rospy.Time.now() - self.prev_cmd_time <= self.command_timeout:
            return
        for config in self.robots.robots:
            if config.team == RobotTeam.OUR_TEAM:
                self.update_cmd_vel(Twist(), config)

    def predict_all_filters(self) -> None:
        if not self.field_received():
            return
        for robot_filter in self.robot_filters.values():
            try:
                robot_filter.predict()
            except np.linalg.LinAlgError as e:
                rospy.logwarn(f"Failed predict. Resetting filter. {e}")
                robot_filter.reset()

    def get_robot_frame_id(self, robot_name: str) -> str:
        return self.robot_frame_prefix + "_" + robot_name

    def state_to_transform(self, state_msg: EstimatedObject) -> TransformStamped:
        transform = TransformStamped()
        transform.header = state_msg.header
        transform.child_frame_id = state_msg.child_frame_id
        transform.transform.translation = Vector3(
            state_msg.pose.pose.position.x, state_msg.pose.pose.position.y, state_msg.pose.pose.position.z
        )
        transform.transform.rotation = state_msg.pose.pose.orientation
        return transform

    def state_to_odom(self, state_msg: EstimatedObject) -> Odometry:
        odom = Odometry()
        odom.header = state_msg.header
        odom.child_frame_id = state_msg.child_frame_id
        odom.pose = state_msg.pose
        odom.twist = state_msg.twist
        return odom

    def publish_all_filters(self) -> None:
        transforms = []
        filtered_states = EstimatedObjectArray()
        for robot_name, bot_filter in self.robot_filters.items():
            robot_frame_id = self.get_robot_frame_id(robot_name)
            pose, twist = bot_filter.get_state()
            diameter = self.robot_sizes[robot_name] * 2

            state_msg = EstimatedObject(
                header=RosHeader(frame_id=self.map_frame, stamp=rospy.Time.now()),
                child_frame_id=robot_frame_id,
                pose=pose,
                twist=twist,
                size=Vector3(diameter, diameter, diameter),
                label=robot_name,
            )

            transforms.append(self.state_to_transform(state_msg))
            self.filter_state_pubs[robot_name].publish(self.state_to_odom(state_msg))

            filtered_states.robots.append(state_msg)
        self.filter_state_array_pub.publish(filtered_states)

        self.tf_broadcaster.sendTransform(transforms)

    def run(self) -> None:
        rate = rospy.Rate(self.update_rate)
        while not rospy.is_shutdown():
            self.check_cmd_timeout()
            self.predict_all_filters()
            self.publish_all_filters()
            rate.sleep()


if __name__ == "__main__":
    rospy.init_node("robot_filter", log_level=rospy.DEBUG)
    RobotFilter().run()
