#!/usr/bin/env python
from typing import List, Optional

import rospy
import tf2_ros
from apriltag_ros.msg import AprilTagDetection, AprilTagDetectionArray
from bw_interfaces.msg import EstimatedRobot, EstimatedRobotArray
from bw_tools.structs.transform3d import Transform3D
from bw_tools.transforms import lookup_pose_in_frame
from bw_tools.typing import get_param, seconds_to_duration
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

from bw_object_filter.covariances import ApriltagHeuristics, CmdVelHeuristics, RobotHeuristics
from bw_object_filter.filter_models import DriveKalmanModel
from bw_object_filter.robot_config import OUR_TEAM, RobotConfig, RobotFleetConfig
from bw_object_filter.robot_measurement_sorter import RobotMeasurementSorter


class RobotFilter:
    def __init__(self) -> None:
        robot_config = get_param("~robots", None)
        if robot_config is None:
            raise ValueError("Must specify robot_config in the parameter server")

        self.update_rate = get_param("~update_rate", 50.0)
        self.update_delay = 1.0 / self.update_rate

        self.map_frame = get_param("~map_frame", "map")
        self.robot_frame_prefix = get_param("~robot_frame_prefix", "base_link")

        self.command_timeout = seconds_to_duration(get_param("~command_timeout", 0.5))

        self.apriltag_base_covariance_scalar = get_param("~apriltag_base_covariance_scalar", 0.001)
        self.our_base_covariance = get_param("~our_robot_estimate_base_covariance_scalar", 0.1)
        self.their_base_covariance = get_param("~their_robot_estimate_base_covariance_scalar", 0.01)
        self.cmd_vel_base_covariance_scalar = get_param("~cmd_vel_base_covariance_scalar", 0.01)
        self.friction_factor = get_param("~friction_factor", 0.4)
        self.process_noise = get_param("~process_noise", 1e-4)

        self.robots = RobotFleetConfig.from_config(robot_config)

        self.prev_cmd_time = rospy.Time.now()
        self.robot_names = {config.id: config.name for config in self.robots.robots}
        self.robot_teams = {config.id: config.team for config in self.robots.robots}
        rotate_quat = (0.0, 0.0, -0.707, 0.707)
        self.apriltag_rotate_tf = Transform3D.from_position_and_quaternion(Vector3(), Quaternion(*rotate_quat))
        self.robot_filters = {
            config.id: DriveKalmanModel(self.update_delay, self.process_noise, self.friction_factor)
            for config in self.robots.robots
        }

        self.tag_heurstics = ApriltagHeuristics(self.apriltag_base_covariance_scalar)
        self.our_robot_heuristics = RobotHeuristics(self.our_base_covariance)
        self.their_robot_heuristics = RobotHeuristics(self.their_base_covariance)
        self.cmd_vel_heuristics = CmdVelHeuristics(self.cmd_vel_base_covariance_scalar)
        self.measurement_sorter = RobotMeasurementSorter(self.robot_filters)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster()

        self.filter_state_pubs = {
            robot.id: rospy.Publisher(f"{robot.name}/filter_state", Odometry, queue_size=10)
            for robot in self.robots.robots
        }

        self.robots_sub = rospy.Subscriber("estimation/robots", EstimatedRobotArray, self.robot_estimation_callback)
        self.tags_sub = rospy.Subscriber("tag_detections", AprilTagDetectionArray, self.tags_callback)
        self.cmd_vel_subs = [
            rospy.Subscriber(f"{robot.name}/cmd_vel", Twist, self.cmd_vel_callback, callback_args=robot)
            for robot in self.robots.robots
            if robot.team == OUR_TEAM
        ]

    def robot_estimation_callback(self, msg: EstimatedRobotArray) -> None:
        map_measurements: List[Pose] = []
        for measurement in msg.robots:  # type: ignore
            measurement: EstimatedRobot
            map_pose = self.transform_to_map(measurement.header, measurement.pose)
            if map_pose is None:
                rospy.logwarn(f"Could not transform pose for measurement {measurement}")
                continue
            map_measurements.append(map_pose)

        assigned = self.measurement_sorter.get_ids(map_measurements)
        for robot_id, measurement_index in assigned.items():
            camera_measurement: EstimatedRobot = msg.robots[measurement_index]  # type: ignore
            team = self.robot_teams[robot_id]
            if team == OUR_TEAM:
                covariance = self.our_robot_heuristics.compute_covariance(camera_measurement)
            else:
                covariance = self.their_robot_heuristics.compute_covariance(camera_measurement)
            map_pose = map_measurements[measurement_index]

            pose = PoseWithCovariance()
            pose.pose = map_pose
            pose.covariance = covariance
            self.robot_filters[robot_id].update_position(pose)

    def tags_callback(self, msg: AprilTagDetectionArray) -> None:
        assert msg.detections is not None

        for detection in msg.detections:
            detection: AprilTagDetection
            pose = detection.pose.pose.pose
            pose.orientation = self.rotate_tag_orientation(pose.orientation, self.apriltag_rotate_tf)
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
            robot_filter = self.robot_filters[tag_id]
            robot_filter.update_pose(detection.pose.pose)

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

    def check_cmd_timeout(self) -> None:
        if rospy.Time.now() - self.prev_cmd_time <= self.command_timeout:
            return
        for config in self.robots.robots:
            if config.team == OUR_TEAM:
                self.update_cmd_vel(Twist(), config)

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
            self.check_cmd_timeout()
            self.predict_all_filters()
            self.publish_all_filters()
            rate.sleep()


if __name__ == "__main__":
    rospy.init_node("robot_filter")
    robot_filter = RobotFilter()
    robot_filter.run()
