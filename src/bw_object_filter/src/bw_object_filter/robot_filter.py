#!/usr/bin/env python
from typing import List, Optional, Tuple

import rospy
import tf2_ros
from apriltag_ros.msg import AprilTagDetection, AprilTagDetectionArray
from bw_interfaces.msg import EstimatedObject, EstimatedObjectArray
from bw_tools.configs.robot_config import RobotConfig, RobotFleetConfig, RobotTeam
from bw_tools.structs.header import Header
from bw_tools.structs.labels import Label
from bw_tools.structs.pose2d import Pose2D
from bw_tools.structs.pose2d_stamped import Pose2DStamped
from bw_tools.structs.transform3d import Transform3D
from bw_tools.structs.twist2d import Twist2D
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
from std_msgs.msg import Empty
from std_msgs.msg import Header as RosHeader

from bw_object_filter.covariances import ApriltagHeuristics, CmdVelHeuristics, RobotHeuristics
from bw_object_filter.filter_models import DriveKalmanModel
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
        self.our_base_covariance = get_param("~our_robot_estimate_base_covariance_scalar", 0.001)
        self.their_base_covariance = get_param("~their_robot_estimate_base_covariance_scalar", 0.001)
        self.cmd_vel_base_covariance_scalar = get_param("~cmd_vel_base_covariance_scalar", 0.01)
        self.friction_factor = get_param("~friction_factor", 0.05)
        self.process_noise = get_param("~process_noise", 1e-4)
        self.motion_speed_threshold = get_param("~motion_speed_threshold", 0.25)
        self.robot_min_radius = get_param("~robot_min_radius", 0.1)
        self.robot_max_radius = get_param("~robot_max_radius", 0.4)
        self.field_received = not get_param("~activate_on_reset", True)

        self.robots = RobotFleetConfig.from_dict(robot_config)
        self.check_unique(self.robots)

        self.prev_cmd_time = rospy.Time.now()
        self.robot_names = {}
        for config in self.robots.robots:
            self.robot_names[config.up_id] = config.name
            self.robot_names[config.down_id] = config.name
        self.robot_configs = {config.name: config for config in self.robots.robots}
        rotate_quat = (0.0, 0.0, -0.707, 0.707)
        self.apriltag_rotate_tf = Transform3D.from_position_and_quaternion(Vector3(), Quaternion(*rotate_quat))
        self.robot_filters = {
            robot.name: DriveKalmanModel(self.update_delay, self.process_noise, self.friction_factor, robot.radius)
            for robot in self.robots.robots
        }
        self.prev_opponent_pose = {
            robot.name: Pose2DStamped.empty() for robot in self.robots.robots if robot.team != RobotTeam.OUR_TEAM
        }
        self.prev_motion_opponent_pose = {
            robot.name: Pose2D(0.0, 0.0, 0.0) for robot in self.robots.robots if robot.team != RobotTeam.OUR_TEAM
        }

        self.tag_heurstics = ApriltagHeuristics(self.apriltag_base_covariance_scalar)
        self.our_robot_heuristics = RobotHeuristics(self.our_base_covariance)
        self.their_robot_heuristics = RobotHeuristics(self.their_base_covariance)
        self.our_robot_cmd_vel_heuristics = CmdVelHeuristics(self.cmd_vel_base_covariance_scalar)
        self.their_robot_cmd_vel_heuristics = CmdVelHeuristics(self.cmd_vel_base_covariance_scalar)
        self.their_robot_cmd_vel_heuristics.base_covariance[5, 5] *= 1e6

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
        }

        self.tf_buffer = tf2_ros.Buffer()  # type: ignore
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)  # type: ignore
        self.tf_broadcaster = tf2_ros.TransformBroadcaster()  # type: ignore

        self.filter_state_pubs = {
            robot.name: rospy.Publisher(f"{robot.name}/odom", Odometry, queue_size=10) for robot in self.robots.robots
        }
        self.filter_state_array_pub = rospy.Publisher("filtered_states", EstimatedObjectArray, queue_size=50)

        self.robots_sub = rospy.Subscriber(
            "estimation/robots", EstimatedObjectArray, self.robot_estimation_callback, queue_size=50
        )
        self.reset_sub = rospy.Subscriber("reset_filters", Empty, self.reset_callback, queue_size=1)
        self.tags_sub = rospy.Subscriber("tag_detections", AprilTagDetectionArray, self.tags_callback, queue_size=25)
        self.cmd_vel_subs = [
            rospy.Subscriber(
                f"{robot.name}/cmd_vel/absolute", Twist, self.cmd_vel_callback, callback_args=robot, queue_size=10
            )
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

    def is_id_right_side_up(self, id: int) -> Optional[bool]:
        for config in self.robots.robots:
            if id == config.down_id:
                return False
            elif id == config.up_id:
                return True
        return None

    def robot_estimation_callback(self, msg: EstimatedObjectArray) -> None:
        if not self.field_received:
            rospy.logdebug("Field not received. Skipping robot estimation callback.")
            return
        measurements = {label: [] for label in self.measurement_sorters.keys()}
        for robot in msg.robots:  # type: ignore
            robot: EstimatedObject
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
            map_pose = self.transform_to_map(measurement.header, measurement.state.pose.pose)
            if map_pose is None:
                rospy.logwarn(f"Could not transform pose for measurement {measurement}")
                continue
            map_measurements.append(map_pose)

        assigned = measurement_sorter.get_ids(map_measurements)
        for robot_name, measurement_index in assigned.items():
            robot_filter = self.robot_filters[robot_name]
            camera_measurement = robots[measurement_index]
            robot_config = self.robot_configs[robot_name]
            map_pose = map_measurements[measurement_index]
            if robot_config.team == RobotTeam.OUR_TEAM:
                covariance = self.our_robot_heuristics.compute_covariance(camera_measurement)
            else:
                covariance = self.their_robot_heuristics.compute_covariance(camera_measurement)
                # map_pose, velocity = self.get_delta_measurements(
                #     camera_measurement.header.stamp.to_sec(), robot_name, map_pose
                # )
                # self.update_cmd_vel(velocity, robot_config)
                self.update_cmd_vel(Twist(), robot_config)
                robot_filter.object_radius = self.get_object_radius(camera_measurement)

            pose = PoseWithCovariance()
            pose.pose = map_pose
            pose.covariance = covariance
            # robot_filter.update_pose(pose)
            robot_filter.update_position(pose)

    def get_object_radius(self, object_estimate: EstimatedObject) -> float:
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
            self.robot_max_radius,
        )

    def get_delta_measurements(self, stamp: float, robot_name: str, pose: Pose) -> Tuple[Pose, Twist]:
        dt = stamp - self.prev_opponent_pose[robot_name].header.stamp
        current_pose2d = Pose2D.from_msg(pose)
        prev_pose2d = self.prev_opponent_pose[robot_name]
        self.prev_opponent_pose[robot_name] = Pose2DStamped(Header.auto(stamp=stamp), current_pose2d)

        delta_pose = current_pose2d.relative_to(prev_pose2d.pose)
        velocity = Twist2D(delta_pose.x / dt, delta_pose.y / dt, delta_pose.theta / dt)
        prev_motion_pose2d = self.prev_motion_opponent_pose[robot_name]
        heading_pose = Pose2D(current_pose2d.x, current_pose2d.y, current_pose2d.heading(prev_motion_pose2d))
        if velocity.speed() > self.motion_speed_threshold:
            self.prev_motion_opponent_pose[robot_name] = prev_pose2d.pose

        return heading_pose.to_msg(), velocity.to_msg()

    def tags_callback(self, msg: AprilTagDetectionArray) -> None:
        if not self.field_received:
            rospy.logdebug("Field not received. Skipping robot estimation callback.")
            return
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
            robot_name = self.robot_names.get(tag_id, "")
            if robot_name not in self.robot_filters:
                rospy.logwarn(f"Tag id {tag_id} is not a robot")
                continue
            robot_filter = self.robot_filters[robot_name]
            robot_filter.update_pose(detection.pose.pose)

            is_right_side_up = self.is_id_right_side_up(tag_id)
            if is_right_side_up is not None:
                robot_filter.is_right_side_up = is_right_side_up

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
        if robot_config.team == RobotTeam.OUR_TEAM:
            measurement.covariance = self.our_robot_cmd_vel_heuristics.compute_covariance(measurement)
        else:
            measurement.covariance = self.our_robot_cmd_vel_heuristics.compute_covariance(measurement)

        robot_filter = self.robot_filters[robot_config.name]
        robot_filter.update_cmd_vel(measurement)

    def reset_callback(self, _: Empty) -> None:
        self.field_received = True
        rospy.loginfo("Resetting filters.")
        rospy.sleep(0.25)
        for robot_filter in self.robot_filters.values():
            robot_filter.reset()

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
        for robot_filter in self.robot_filters.values():
            robot_filter.predict()

    def get_robot_frame_id(self, robot_name: str) -> str:
        return self.robot_frame_prefix + "_" + robot_name

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
        filtered_states = EstimatedObjectArray()
        for robot_name, bot_filter in self.robot_filters.items():
            robot_frame_id = self.get_robot_frame_id(robot_name)
            pose, twist = bot_filter.get_state()
            state_msg = Odometry()
            state_msg.header.frame_id = self.map_frame
            state_msg.header.stamp = rospy.Time.now()
            state_msg.child_frame_id = robot_frame_id
            state_msg.pose = pose
            state_msg.twist = twist

            transforms.append(self.odom_to_transform(state_msg))
            self.filter_state_pubs[robot_name].publish(state_msg)

            diameter = bot_filter.object_radius * 2
            filtered_states.robots.append(  # type: ignore
                EstimatedObject(
                    header=state_msg.header,
                    state=state_msg,
                    size=Vector3(diameter, diameter, diameter),
                    label=robot_name,
                )
            )
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
    rospy.init_node("robot_filter")
    robot_filter = RobotFilter()
    robot_filter.run()
