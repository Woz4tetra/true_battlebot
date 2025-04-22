#!/usr/bin/env python
import copy
import logging
import random
from typing import Optional

import numpy as np
import rospy
from app.config.robot_filter.robot_filter_config import RobotFilterConfig
from app.robot_filter.cmd_vel_tracker import CmdVelTracker
from app.robot_filter.covariances import CmdVelHeuristics, RobotStaticHeuristics
from app.robot_filter.filter_models import DriveKalmanModel, TrackingModel
from app.robot_filter.filter_models.drive_kf_impl import NUM_STATES
from app.robot_filter.filter_models.helpers import measurement_to_pose, measurement_to_twist
from app.robot_filter.filter_models.model_base import ModelBase
from app.robot_filter.robot_measurement_sorter import RobotMeasurementSorter
from bw_interfaces.msg import EstimatedObject, EstimatedObjectArray, RobotFleetConfigMsg
from bw_shared.configs.robot_fleet_config import RobotConfig, RobotFleetConfig
from bw_shared.enums.label import Label
from bw_shared.enums.robot_team import RobotTeam
from bw_shared.geometry.pose2d import Pose2D
from bw_shared.geometry.transform3d import Transform3D
from bw_shared.geometry.transform3d_stamped import Transform3DStamped
from bw_shared.geometry.xy import XY
from bw_shared.messages.field import Field
from bw_shared.messages.header import Header
from geometry_msgs.msg import (
    Pose,
    PoseWithCovariance,
    Twist,
    TwistWithCovariance,
    Vector3,
)
from nav_msgs.msg import Odometry
from perception_tools.rosbridge.ros_poll_subscriber import RosPollSubscriber
from perception_tools.rosbridge.ros_publisher import RosPublisher
from perception_tools.rosbridge.transform_broadcaster_bridge import TransformBroadcasterBridge
from std_msgs.msg import Empty
from std_msgs.msg import Header as RosHeader


class RobotFilter:
    def __init__(
        self,
        config: RobotFilterConfig,
        robots_config: RobotFleetConfig,
        opponent_fleet_sub: RosPollSubscriber[RobotFleetConfigMsg],
        cmd_vel_subs: dict[str, RosPollSubscriber[Twist]],
        reset_filters_sub: RosPollSubscriber[Empty],
        filter_state_array_pub: RosPublisher[EstimatedObjectArray],
        tf_broadcaster: TransformBroadcasterBridge,
    ) -> None:
        self.logger = logging.getLogger(self.__class__.__name__)
        self.all_robots_config = robots_config

        self.update_rate = config.update_rate
        self.update_delay = 1.0 / self.update_rate
        self.map_frame = config.map_frame.value
        self.command_timeout = rospy.Duration.from_sec(config.command_timeout)
        initial_variances = config.initial_variances
        self.cmd_vel_base_covariance_scalar = config.cmd_vel_base_covariance_scalar
        self.process_noise = config.process_noise
        self.stale_timeout = config.stale_timeout
        self.robot_min_radius = config.robot_min_radius
        self.robot_max_radius = config.robot_max_radius
        field_buffer = config.field_buffer
        self.field_buffer = XY(field_buffer, field_buffer)

        initial_state = np.zeros(NUM_STATES)
        initial_covariance = np.diag(initial_variances)
        self.initial_pose = measurement_to_pose(initial_state, initial_covariance)
        self.initial_twist = measurement_to_twist(initial_state, initial_covariance)

        self.non_opponent_robot_configs = [
            robot for robot in self.all_robots_config.robots if robot.team != RobotTeam.THEIR_TEAM
        ]

        self.field = EstimatedObject()
        self.filter_bounds = (XY(0.0, 0.0), XY(0.0, 0.0))

        self.robot_heuristics = RobotStaticHeuristics(
            config.robot_position_covariance, config.robot_orientation_covariance
        )
        self.robot_cmd_vel_heuristics = CmdVelHeuristics(self.cmd_vel_base_covariance_scalar)

        self.robot_filters: list[ModelBase] = []
        self.label_to_filter: dict[Label, list[ModelBase]] = {}
        self.label_to_filter_initialization: dict[Label, list[ModelBase]] = {}
        self.name_to_filter: dict[str, ModelBase] = {}
        self.filter_state_pubs: dict[str, RosPublisher] = {}

        self.cmd_vel_trackers: dict[str, CmdVelTracker] = {}
        self.measurement_sorter = RobotMeasurementSorter()

        self.tf_broadcaster = tf_broadcaster
        self.filter_state_array_pub = filter_state_array_pub

        self.opponent_fleet_sub = opponent_fleet_sub
        self.cmd_vel_subs = cmd_vel_subs
        self.reset_filters_sub = reset_filters_sub

        self._initialize(self.all_robots_config.robots)

        self.logger.info("Robot filter initialized.")

    def update_robot_estimations(self, msg: EstimatedObjectArray, field: Field) -> None:
        if not self._field_received():
            self.logger.debug("Field not received. Skipping robot estimation callback.")
            return
        measurements = self._make_measurements(msg, field)

        for label, label_measurements in measurements.items():
            self._apply_update_with_sorter(self.label_to_filter[label], label_measurements)

    def update_field(self, field: Field) -> None:
        if self._field_received():
            return
        self.logger.info("Field received.")
        self.field = field
        self.filter_bounds = (field.bounds_2d[0] - self.field_buffer, field.bounds_2d[1] + self.field_buffer)
        self.logger.info("Resetting filters.")
        self._reset_filters()

    def update(self) -> EstimatedObjectArray:
        if self.reset_filters_sub.receive():
            self._reset_filters()
        if opponent_fleet_msg := self.opponent_fleet_sub.receive():
            self._update_opponent_fleet(opponent_fleet_msg)
        self._predict_all_filters()
        filtered_states = self._publish_all_filters()
        self._update_cmd_vels()
        return filtered_states

    def _initialize(self, robot_fleet: list[RobotConfig]) -> None:
        self.field = EstimatedObject()
        self.robot_filters = self._init_filters(robot_fleet)
        self._init_label_to_filter(self.robot_filters)
        self._init_name_to_filter(self.robot_filters)
        self._init_filter_state_pubs(robot_fleet)
        self._init_cmd_vel_trackers()
        self._reset_filters()

    def _update_opponent_fleet(self, msg: RobotFleetConfigMsg) -> None:
        opponent_fleet = RobotFleetConfig.from_msg(msg)
        self.logger.info(
            f"Received opponent fleet. Resetting filters. "
            f"{len(self.non_opponent_robot_configs)} friendly robots. "
            f"{len(opponent_fleet.robots)} opponents."
        )
        robot_fleet = self.non_opponent_robot_configs + opponent_fleet.robots
        self._initialize(robot_fleet)

    def _init_filters(self, robot_fleet: list[RobotConfig]) -> list[ModelBase]:
        filters: list[ModelBase] = []
        for robot_config in robot_fleet:
            if robot_config.team == RobotTeam.OUR_TEAM:
                robot_filter = DriveKalmanModel(
                    robot_config,
                    self.update_delay,
                    self.process_noise,
                    self.stale_timeout,
                    self.robot_min_radius,
                    self.robot_max_radius,
                )
            else:
                robot_filter = TrackingModel(
                    robot_config,
                    self.update_delay,
                    self.process_noise,
                    self.stale_timeout,
                    self.robot_min_radius,
                    self.robot_max_radius,
                )
            filters.append(robot_filter)
        return filters

    def _init_label_to_filter(self, robot_filters: list[ModelBase]) -> None:
        self.label_to_filter = {}
        for filter in robot_filters:
            team = filter.config.team
            if filter.config.is_controlled:
                self.label_to_filter.setdefault(Label.CONTROLLED_ROBOT, []).append(filter)
            elif team == RobotTeam.OUR_TEAM:
                self.label_to_filter.setdefault(Label.FRIENDLY_ROBOT, []).append(filter)
            elif team == RobotTeam.THEIR_TEAM:
                self.label_to_filter.setdefault(Label.ROBOT, []).append(filter)
            elif team == RobotTeam.REFEREE:
                self.label_to_filter.setdefault(Label.REFEREE, []).append(filter)
            else:
                raise ValueError(f"Filter doesn't have an initialization label: {filter.config.name}")

    def _init_name_to_filter(self, robot_filters: list[ModelBase]) -> None:
        self.name_to_filter = {}
        for robot_filter in robot_filters:
            self.name_to_filter[robot_filter.config.name] = robot_filter

    def _init_filter_state_pubs(self, robot_fleet: list[RobotConfig]) -> None:
        existing_robots = set(self.filter_state_pubs.keys())
        new_fleet_names = {robot.name for robot in robot_fleet}
        robots_to_remove = existing_robots - new_fleet_names
        robots_to_add = new_fleet_names - existing_robots
        for robot_name in robots_to_remove:
            self.filter_state_pubs[robot_name].unregister()
            del self.filter_state_pubs[robot_name]
        for robot_name in robots_to_add:
            self.filter_state_pubs[robot_name] = RosPublisher(f"/{robot_name}/odom", Odometry, queue_size=10)

    def _init_cmd_vel_trackers(self) -> None:
        self.cmd_vel_trackers = {
            robot_name: CmdVelTracker(self.robot_cmd_vel_heuristics, command_timeout=self.command_timeout)
            for robot_name in self.cmd_vel_subs.keys()
        }

    def _make_measurements(
        self,
        robot_measurements: EstimatedObjectArray,
        field: Field,
    ) -> dict[Label, list[EstimatedObject]]:
        measurements: dict[Label, list[EstimatedObject]] = {label: [] for label in self.label_to_filter.keys()}
        for robot in robot_measurements.robots:
            try:
                label = Label(robot.label)
            except ValueError:
                self.logger.warning(f"Unknown label {robot.label}")
                continue
            if all([c == 0 for c in robot.pose.covariance]):
                robot.pose.covariance = self.robot_heuristics.compute_covariance(robot)
            map_pose = self._transform_measurement_in_camera_to_map(robot, field)
            if map_pose is None:
                self.logger.warning(f"Could not transform pose for measurement {robot.label}")
                continue
            map_robot = copy.deepcopy(robot)
            map_robot.pose.pose = map_pose
            measurements[label].append(map_robot)
        return measurements

    def _apply_update_with_sorter(
        self, available_filters: list[ModelBase], map_measurements: list[EstimatedObject]
    ) -> None:
        map_measurements_2d = [Pose2D.from_msg(measurement.pose.pose) for measurement in map_measurements]
        assigned = self.measurement_sorter.get_ids(available_filters, map_measurements_2d)
        for filter_index, measurement_index in assigned.items():
            map_measurement = map_measurements[measurement_index]
            self._apply_sorted_measurement(available_filters[filter_index], map_measurement)

    def _apply_sorted_measurement(self, robot_filter: ModelBase, map_measurement: EstimatedObject) -> None:
        robot_filter.update_radius(map_measurement.size)
        try:
            robot_filter.update_pose(map_measurement.pose)
        except np.linalg.LinAlgError as e:
            self.logger.warning(f"Failed to update from robot measurement. Resetting filter. {e}")
            robot_filter.reset()
        robot_filter.update_filter_time(map_measurement.header.stamp)

    def _apply_cmd_vel(self, robot_filter: ModelBase, measurement: TwistWithCovariance) -> None:
        try:
            robot_filter.update_cmd_vel(measurement)
        except np.linalg.LinAlgError as e:
            self.logger.warning(f"Failed to update from velocity. Resetting filter. {e}")
            robot_filter.reset()

    def _update_cmd_vels(self) -> None:
        if not self._field_received():
            return
        for robot_name, tracker in self.cmd_vel_trackers.items():
            if twist_msg := self.cmd_vel_subs[robot_name].receive():
                tracker.set_velocity(twist_msg)
            robot_filter = self.name_to_filter[robot_name]
            cmd_vel = tracker.get_velocity()
            self._apply_cmd_vel(robot_filter, cmd_vel)

    def _reset_filters(self) -> None:
        self.logger.info("Resetting filters.")
        for robot_filter in self.robot_filters:
            robot_filter.reset()
            self._initialize_filter_random(robot_filter)

    def _initialize_filter_random(self, robot_filter: ModelBase) -> None:
        field_size = self.filter_bounds[1] - self.filter_bounds[0]
        initialization_pose = PoseWithCovariance()
        initialization_pose.pose = Pose2D(
            x=2.0 * (random.random() - 0.5) * field_size.x,
            y=2.0 * (random.random() - 0.5) * field_size.y,
            theta=2.0 * np.pi * (random.random() - 0.5),
        ).to_msg()
        initialization_pose.covariance = self.initial_pose.covariance
        robot_filter.teleport(initialization_pose, self.initial_twist)
        self.logger.info(f"Initialized {robot_filter.config.name}.")

    def _field_received(self) -> bool:
        return self.field.header.stamp != rospy.Time(0)

    def _transform_measurement_in_camera_to_map(self, robot: EstimatedObject, field: Field) -> Optional[Pose]:
        if not self._field_received():
            self.logger.warning("Field not received. Skipping transform.")
            return None
        if robot.header.frame_id != field.child_frame_id:
            self.logger.warning(
                f"Header frame id {robot.header.frame_id} does not match measurement frame id {field.child_frame_id}"
            )
            return None
        tf_camera_from_robot = Transform3D.from_pose_msg(robot.pose.pose)
        tf_map_from_robot = field.tf_map_from_camera.forward_by(tf_camera_from_robot)
        return tf_map_from_robot.to_pose_msg()

    def _predict_all_filters(self) -> None:
        if not self._field_received():
            return
        for robot_filter in self.robot_filters:
            try:
                robot_filter.predict()
            except np.linalg.LinAlgError as e:
                self.logger.warning(f"Failed predict. Resetting filter. {e}")
                robot_filter.reset()
                continue
            if not robot_filter.is_in_bounds(self.filter_bounds) or robot_filter.is_stale():
                robot_filter.reset()

    def _state_to_transform(self, state_msg: EstimatedObject) -> Transform3DStamped:
        transform = Transform3DStamped(
            Header.from_msg(state_msg.header),
            state_msg.child_frame_id,
            Transform3D.from_pose_msg(state_msg.pose.pose),
        )
        return transform

    def _state_to_odom(self, state_msg: EstimatedObject) -> Odometry:
        odom = Odometry()
        odom.header = state_msg.header
        odom.child_frame_id = state_msg.child_frame_id
        odom.pose = state_msg.pose
        odom.twist = state_msg.twist
        return odom

    def _publish_all_filters(self) -> EstimatedObjectArray:
        transforms: list[Transform3DStamped] = []
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

            transforms.append(self._state_to_transform(state_msg))
            self.filter_state_pubs[robot_name].publish(self._state_to_odom(state_msg))

            filtered_states.robots.append(state_msg)
        self.filter_state_array_pub.publish(filtered_states)
        self.tf_broadcaster.publish_transforms(*transforms)
        return filtered_states
