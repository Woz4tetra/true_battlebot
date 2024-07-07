#!/usr/bin/env python
from typing import Optional

import numpy as np
import rospy
import tf2_ros
from bw_interfaces.msg import EstimatedObject, EstimatedObjectArray, RobotFleetConfigMsg
from bw_shared.configs.robot_fleet_config import RobotConfig, RobotFleetConfig
from bw_shared.enums.label import Label
from bw_shared.enums.robot_team import RobotTeam
from bw_shared.geometry.rpy import RPY
from bw_shared.geometry.transform3d import Transform3D
from bw_shared.geometry.xy import XY
from bw_shared.geometry.xyz import XYZ
from bw_tools.configs.rosparam_client import get_shared_config
from bw_tools.get_param import get_param
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

from bw_object_filter.covariances import ApriltagHeuristics, CmdVelHeuristics, RobotStaticHeuristics
from bw_object_filter.estimation_topic_metadata import EstimationTopicMetadata
from bw_object_filter.filter_models import DriveKalmanModel
from bw_object_filter.filter_models.helpers import (
    NUM_STATES,
    measurement_to_pose,
    measurement_to_twist,
)
from bw_object_filter.robot_measurement_sorter import RobotMeasurementSorter


class RobotFilter:
    def __init__(self) -> None:
        shared_config = get_shared_config()
        self.all_robots_config = shared_config.robots

        self.update_rate = get_param("~update_rate", 50.0)
        self.update_delay = 1.0 / self.update_rate

        self.map_frame = get_param("~map_frame", "map")
        self.controlled_robot_name = get_param("~controlled_robot_name", "mini_bot")
        self.estimation_topics = [EstimationTopicMetadata.from_dict(d) for d in get_param("~estimation_topics", [])]
        self.tag_topics = get_param("~tag_topics", [])

        self.command_timeout = rospy.Duration.from_sec(get_param("~command_timeout", 0.5))

        self.apriltag_base_covariance_scalar = get_param("~apriltag_base_covariance_scalar", 0.00001)
        initial_variances = get_param("~initial_variances", [0.25, 0.25, 10.0, 1.0, 1.0, 10.0])
        self.cmd_vel_base_covariance_scalar = get_param("~cmd_vel_base_covariance_scalar", 0.01)
        self.friction_factor = get_param("~friction_factor", 0.8)
        self.process_noise = get_param("~process_noise", 1e-4)
        self.motion_speed_threshold = get_param("~motion_speed_threshold", 0.25)
        self.robot_min_radius = get_param("~robot_min_radius", 0.1)
        self.ignore_measurement_near_tag_threshold = get_param("~ignore_measurement_near_tag_threshold", 0.05)
        field_buffer = get_param("~field_buffer", 0.2)
        self.field_buffer = XYZ(field_buffer, field_buffer, field_buffer)

        initial_state = np.zeros(NUM_STATES)
        initial_covariance = np.diag(initial_variances)
        self.initial_pose = measurement_to_pose(initial_state, initial_covariance)
        self.initial_twist = measurement_to_twist(initial_state, initial_covariance)

        self.check_unique(self.all_robots_config)

        self.non_opponent_robot_configs = [
            robot for robot in self.all_robots_config.robots if robot.team != RobotTeam.THEIR_TEAM
        ]

        self.field = EstimatedObject()
        self.field_bounds = (XYZ(0.0, 0.0, 0.0), XYZ(0.0, 0.0, 0.0))
        self.filter_bounds = (XY(0.0, 0.0), XY(0.0, 0.0))
        self.apriltag_rotate_tf = Transform3D.from_position_and_rpy(Vector3(), RPY((0.0, 0.0, np.pi / 2)))

        self.tag_heurstics = ApriltagHeuristics(self.apriltag_base_covariance_scalar)
        self.robot_heuristics = {
            topic.topic: RobotStaticHeuristics(topic.position_covariance, topic.orientation_covariance)
            for topic in self.estimation_topics
        }
        self.robot_cmd_vel_heuristics = CmdVelHeuristics(self.cmd_vel_base_covariance_scalar)

        # measurements labels are checked and applied in this order
        self.measurement_sorters_priority = [
            Label.REFEREE,
            Label.CONTROLLED_ROBOT,
            Label.FRIENDLY_ROBOT,
            Label.ROBOT,
        ]

        self.robot_filters: list[DriveKalmanModel] = []
        self.label_to_filter: dict[Label, list[DriveKalmanModel]] = {}
        self.team_to_filter: dict[RobotTeam, list[DriveKalmanModel]] = {}
        self.label_to_filter_initialization: dict[Label, list[DriveKalmanModel]] = {}
        self.name_to_filter: dict[str, DriveKalmanModel] = {}
        self.filter_state_pubs: dict[str, rospy.Publisher] = {}

        self.measurement_sorter = RobotMeasurementSorter()
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster()
        self.filter_state_array_pub = rospy.Publisher("filtered_states", EstimatedObjectArray, queue_size=50)

        self.initialize(self.all_robots_config.robots)

        if not self.estimation_topics:
            rospy.logwarn("No estimation topics provided. Will not receive robot estimation data.")
        self.robots_subs: dict[str, rospy.Subscriber] = {}
        for topic_info in self.estimation_topics:
            self.robots_subs[topic_info.topic] = rospy.Subscriber(
                topic_info.topic,
                EstimatedObjectArray,
                lambda msg, data=topic_info: self.robot_estimation_callback(data, msg),
                queue_size=50,
            )

        self.field_sub = rospy.Subscriber("filter/field", EstimatedObject, self.field_callback, queue_size=1)
        self.tags_subs: dict[str, rospy.Subscriber] = {}
        for topic in self.tag_topics:
            self.tags_subs[topic] = rospy.Subscriber(topic, EstimatedObjectArray, self.tags_callback, queue_size=25)
        self.cmd_vel_subs = [
            rospy.Subscriber(
                f"{filter.config.name}/cmd_vel", Twist, self.cmd_vel_callback, callback_args=filter, queue_size=10
            )
            for filter in self.robot_filters
            if filter.config.team == RobotTeam.OUR_TEAM
        ]
        self.opponent_fleet_sub = rospy.Subscriber(
            "opponent_fleet", RobotFleetConfigMsg, self.opponent_fleet_callback, queue_size=1
        )

    def initialize(self, robot_fleet: list[RobotConfig]) -> None:
        self.robot_filters = self._init_filters(robot_fleet)
        self._init_label_to_filter(self.robot_filters)
        self._init_label_to_filter_initialization(self.robot_filters)
        self._init_name_to_filter(self.robot_filters)
        self._init_filter_state_pubs(robot_fleet)

    def _init_filters(self, robot_fleet: list[RobotConfig]) -> list[DriveKalmanModel]:
        return [
            DriveKalmanModel(robot_config, self.update_delay, self.process_noise, self.friction_factor)
            for robot_config in robot_fleet
        ]

    def _init_label_to_filter(self, robot_filters: list[DriveKalmanModel]) -> None:
        self.label_to_filter = {
            Label.ROBOT: [filter for filter in robot_filters if filter.config.team != RobotTeam.REFEREE],
            Label.FRIENDLY_ROBOT: [
                filter
                for filter in robot_filters
                if filter.config.team == RobotTeam.OUR_TEAM and filter.config.name != self.controlled_robot_name
            ],
            Label.CONTROLLED_ROBOT: [
                filter for filter in robot_filters if filter.config.name == self.controlled_robot_name
            ],
            Label.REFEREE: [filter for filter in robot_filters if filter.config.team == RobotTeam.REFEREE],
        }
        self.team_to_filter = {
            RobotTeam.OUR_TEAM: [filter for filter in robot_filters if filter.config.team == RobotTeam.OUR_TEAM],
            RobotTeam.THEIR_TEAM: [filter for filter in robot_filters if filter.config.team == RobotTeam.THEIR_TEAM],
            RobotTeam.REFEREE: [filter for filter in robot_filters if filter.config.team == RobotTeam.REFEREE],
        }

    def _init_label_to_filter_initialization(self, robot_filters: list[DriveKalmanModel]) -> None:
        self.label_to_filter_initialization = {label: [] for label in self.label_to_filter.keys()}
        for filter in robot_filters:
            team = filter.config.team
            if filter.config.name == self.controlled_robot_name:
                self.label_to_filter_initialization[Label.CONTROLLED_ROBOT].append(filter)
            elif team == RobotTeam.OUR_TEAM:
                self.label_to_filter_initialization[Label.FRIENDLY_ROBOT].append(filter)
            elif team == RobotTeam.THEIR_TEAM:
                self.label_to_filter_initialization[Label.ROBOT].append(filter)
            elif team == RobotTeam.REFEREE:
                self.label_to_filter_initialization[Label.REFEREE].append(filter)
            else:
                raise ValueError(f"Filter doesn't have an initialization label: {filter.config.name}")

    def _init_name_to_filter(self, robot_filters: list[DriveKalmanModel]) -> None:
        self.name_to_filter = {}
        for robot_filter in robot_filters:
            self.name_to_filter[robot_filter.config.name] = robot_filter

    def _init_filter_state_pubs(self, robot_fleet: list[RobotConfig]) -> None:
        self.filter_state_pubs = {
            robot.name: rospy.Publisher(f"{robot.name}/odom", Odometry, queue_size=10) for robot in robot_fleet
        }

    def check_unique(self, config: RobotFleetConfig) -> None:
        """
        Check that all robot ids are unique.
        """
        ids = []
        for robot in config.robots:
            if robot.team == RobotTeam.OUR_TEAM:
                ids.extend([tag.tag_id for tag in robot.tags])
        if len(ids) != len(set(ids)):
            raise ValueError("Robot ids must be unique")
        names = [bot.name for bot in config.robots]
        if len(names) != len(set(names)):
            raise ValueError("Robot names must be unique")

    def robot_estimation_callback(self, metadata: EstimationTopicMetadata, msg: EstimatedObjectArray) -> None:
        if not self.field_received():
            rospy.logdebug("Field not received. Skipping robot estimation callback.")
            return
        measurements: dict[Label, list[EstimatedObject]] = {label: [] for label in self.label_to_filter.keys()}
        for robot in msg.robots:
            try:
                label = Label(robot.label)
            except ValueError:
                rospy.logwarn(f"Unknown label {robot.label}")
                continue
            if all([c == 0 for c in robot.pose.covariance]):
                robot.pose.covariance = self.robot_heuristics[metadata.topic].compute_covariance(robot)  # type: ignore
            measurements[label].append(robot)
        for label, robot_filters in self.label_to_filter_initialization.items():
            for robot_filter in robot_filters:
                if not robot_filter.is_initialized and len(measurements[label]) > 0:
                    measurement = measurements[label].pop()
                    measurement.pose.covariance = self.initial_pose.covariance
                    robot_filter.teleport(measurement.pose, self.initial_twist)
                    rospy.loginfo(f"Initialized {robot_filter.config.name} from {label} measurement.")
        for label in self.measurement_sorters_priority:
            if len(measurements[label]) == 0:
                continue
            self.apply_update_with_sorter(
                self.label_to_filter[label],
                measurements[label],
                metadata.use_orientation,
            )

    def apply_update_with_sorter(
        self,
        available_filters: list[DriveKalmanModel],
        robot_measurements: list[EstimatedObject],
        use_orientation: bool,
    ) -> None:
        map_measurements: list[PoseWithCovariance] = []
        for measurement in robot_measurements:
            map_pose = self.transform_to_map(measurement.header, measurement.pose.pose)
            if map_pose is None:
                rospy.logwarn(f"Could not transform pose for measurement {measurement}")
                continue
            if not self.is_in_field_bounds(map_pose.position):
                rospy.logdebug(f"{measurement.label} is out of bounds. Skipping.")
                continue
            map_measurements.append(
                PoseWithCovariance(
                    map_pose,
                    covariance=measurement.pose.covariance,
                )
            )

        assigned = self.measurement_sorter.get_ids(available_filters, map_measurements)
        for filter_index, measurement_index in assigned.items():
            camera_measurement = robot_measurements[measurement_index]
            map_measurement = map_measurements[measurement_index]
            self.apply_sorted_measurement(
                available_filters[filter_index], map_measurement, camera_measurement, use_orientation
            )

    def apply_sorted_measurement(
        self,
        robot_filter: DriveKalmanModel,
        map_measurement: PoseWithCovariance,
        camera_measurement: EstimatedObject,
        use_orientation: bool,
    ) -> None:
        robot_filter.update_radius(camera_measurement.size)

        try:
            if use_orientation:
                robot_filter.update_pose(map_measurement)
            else:
                robot_filter.update_position(map_measurement)
        except np.linalg.LinAlgError as e:
            rospy.logwarn(f"Failed to update from robot measurement. Resetting filter. {e}")
            robot_filter.reset()

    def tags_callback(self, msg: EstimatedObjectArray) -> None:
        if not self.field_received():
            rospy.logdebug("Field not received. Skipping tag callback.")
            return
        self.transform_tags_to_map(msg)
        self.apply_tag_measurement(msg)

    def transform_tags_to_map(self, msg: EstimatedObjectArray) -> None:
        for detection in msg.robots:
            pose = detection.pose.pose
            pose.orientation = self.rotate_tag_orientation(pose.orientation, self.apriltag_rotate_tf)
            covariance = self.tag_heurstics.compute_covariance(detection)
            detection.pose.pose.covariance = covariance  # type: ignore
            map_pose = self.transform_to_map(detection.header, detection.pose.pose)
            if map_pose is None:
                rospy.logwarn(f"Could not transform pose for tag {detection}")
                continue
            detection.pose.pose = map_pose

    def apply_tag_measurement(self, msg: EstimatedObjectArray) -> None:
        for detection in msg.robots:
            pose = detection.pose.pose
            if not self.is_in_field_bounds(pose.position):
                rospy.logdebug(f"Tag for {detection.label} is out of bounds. Skipping.")
                continue
            covariance = self.tag_heurstics.compute_covariance(detection)
            detection.pose.covariance = covariance  # type: ignore
            if robot_filter := self.name_to_filter.get(detection.label, None):
                try:
                    robot_filter.update_pose(detection.pose)
                except np.linalg.LinAlgError as e:
                    rospy.logwarn(f"Failed to update from tag. Resetting filter. {e}")
                    robot_filter.reset()
                    continue
            else:
                rospy.logwarn(f"Tag for {detection.label} not found in filters.")

    def rotate_tag_orientation(
        self,
        tag_orientation: Quaternion,
        rotate_tf: Transform3D,
    ) -> Quaternion:
        tag_tf = Transform3D.from_position_and_quaternion(Vector3(), tag_orientation)
        rotated_tag = tag_tf.transform_by(rotate_tf)
        return rotated_tag.quaternion

    def cmd_vel_callback(self, msg: Twist, robot_filter: DriveKalmanModel) -> None:
        if not self.field_received():
            return
        measurement = TwistWithCovariance(twist=msg)
        measurement.covariance = self.robot_cmd_vel_heuristics.compute_covariance(measurement)  # type: ignore

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
        self.filter_bounds = (
            XY(self.field_bounds[0].x, self.field_bounds[0].y),
            XY(self.field_bounds[1].x, self.field_bounds[1].y),
        )
        rospy.loginfo("Resetting filters.")
        rospy.sleep(0.2)  # wait for changes in TF tree to propagate
        for robot_filter in self.robot_filters:
            robot_filter.reset()

    def field_received(self) -> bool:
        return self.field.header.stamp != rospy.Time(0)

    def opponent_fleet_callback(self, msg: RobotFleetConfigMsg) -> None:
        opponent_fleet = RobotFleetConfig.from_msg(msg)
        rospy.loginfo(
            f"Received opponent fleet. Resetting filters. "
            f"{len(self.non_opponent_robot_configs)} friendly robots. "
            f"{len(opponent_fleet.robots)} opponents."
        )
        robot_fleet = self.non_opponent_robot_configs + opponent_fleet.robots
        self.initialize(robot_fleet)

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

    def predict_all_filters(self) -> None:
        if not self.field_received():
            return
        for robot_filter in self.robot_filters:
            try:
                robot_filter.predict()
            except np.linalg.LinAlgError as e:
                rospy.logwarn(f"Failed predict. Resetting filter. {e}")
                robot_filter.reset()
                continue
            if not robot_filter.is_in_bounds(self.filter_bounds[0], self.filter_bounds[1]) or robot_filter.is_stale():
                robot_filter.reset()

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
        now = rospy.Time.now()
        filtered_states = EstimatedObjectArray()
        for robot_filter in self.robot_filters:
            pose, twist = robot_filter.get_state()
            robot_name = robot_filter.config.name
            diameter = robot_filter.object_radius * 2

            state_msg = EstimatedObject(
                header=RosHeader(frame_id=self.map_frame, stamp=now),
                child_frame_id=robot_name,
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
            self.predict_all_filters()
            self.publish_all_filters()
            rate.sleep()


if __name__ == "__main__":
    rospy.init_node("robot_filter", log_level=rospy.DEBUG)
    RobotFilter().run()
