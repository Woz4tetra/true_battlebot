import time
from abc import ABC, abstractmethod

import genpy
import numpy as np
import rospy
from bw_shared.configs.robot_fleet_config import RobotConfig
from bw_shared.geometry.xy import XY
from geometry_msgs.msg import PoseWithCovariance, TwistWithCovariance, Vector3

from .drive_kf_impl import NUM_STATES, NUM_STATES_1ST_ORDER
from .helpers import (
    StateArray,
    StateSquareMatrix,
    measurement_to_pose,
    measurement_to_twist,
    pose_to_measurement,
    twist_to_measurement,
)


class ModelBase(ABC):
    def __init__(
        self,
        config: RobotConfig,
        stale_timeout: float = 10.0,
        robot_min_radius: float = 0.05,
        robot_max_radius: float = 0.15,
    ) -> None:
        self.config = config
        self.state: StateArray = np.zeros(NUM_STATES)
        self.covariance: StateSquareMatrix = np.eye(NUM_STATES)
        self.robot_min_radius = robot_min_radius
        self.robot_max_radius = robot_max_radius
        self.stale_timeout = stale_timeout

        self.object_radius = 0.0
        self.stale_timer = self._now()
        self._is_initialized = False
        self.last_position_time = rospy.Time.from_sec(0.0)

        self.reset()

    @abstractmethod
    def predict(self) -> None: ...

    @abstractmethod
    def update_pose(self, msg: PoseWithCovariance) -> None: ...

    @abstractmethod
    def update_position(self, msg: PoseWithCovariance) -> None: ...

    @abstractmethod
    def update_orientation(self, yaw: float, covariance: np.ndarray) -> None: ...

    @abstractmethod
    def update_cmd_vel(self, msg: TwistWithCovariance) -> None: ...

    def _now(self) -> float:
        return time.perf_counter()

    def _clamp_divergent(self) -> None:
        self.state = np.nan_to_num(self.state, copy=False, nan=0.0, posinf=0.0, neginf=0.0)
        self.covariance = np.nan_to_num(self.covariance, copy=False, nan=1e-3, posinf=1e-3, neginf=1e-3)

    def update_radius(self, size: Vector3) -> None:
        measured_radius = max(size.x, size.y, size.z) / 2.0
        self.object_radius = min(
            max(
                measured_radius,
                self.robot_min_radius,
            ),
            self.robot_max_radius,
        )

    def get_state(self) -> tuple[PoseWithCovariance, TwistWithCovariance]:
        pose = measurement_to_pose(self.state, self.covariance)
        return (
            pose,
            measurement_to_twist(self.state, self.covariance),
        )

    def get_covariance(self) -> StateSquareMatrix:
        return self.covariance

    def teleport(self, pose: PoseWithCovariance, twist: TwistWithCovariance = TwistWithCovariance()) -> None:
        if all([c == 0 for c in twist.covariance]):
            twist.covariance = np.eye(6).flatten().tolist()  # type: ignore
        initial_pose, pose_noise = pose_to_measurement(pose)
        initial_twist, twist_noise = twist_to_measurement(twist)
        self.state = np.zeros(NUM_STATES)
        self.state[0:NUM_STATES_1ST_ORDER] = initial_pose
        self.state[NUM_STATES_1ST_ORDER:NUM_STATES] = initial_twist
        self.covariance = np.zeros((NUM_STATES, NUM_STATES))
        self.covariance[0:NUM_STATES_1ST_ORDER, 0:NUM_STATES_1ST_ORDER] = pose_noise
        self.covariance[NUM_STATES_1ST_ORDER:NUM_STATES, NUM_STATES_1ST_ORDER:NUM_STATES] = twist_noise
        self._is_initialized = True
        self.reset_stale_timer()

    def reset(self) -> None:
        self.state = np.zeros(NUM_STATES)
        self.covariance = np.eye(NUM_STATES)
        self._is_initialized = False
        self.stale_timer = 0.0

    def is_in_bounds(self, lower_bound: XY, upper_bound: XY) -> bool:
        return lower_bound.x <= self.state[0] <= upper_bound.x and lower_bound.y <= self.state[1] <= upper_bound.y

    def is_initialized(self) -> bool:
        return self._is_initialized

    def is_stale(self) -> bool:
        return self._now() - self.stale_timer > self.stale_timeout

    def reset_stale_timer(self) -> None:
        self.stale_timer = self._now()

    def update_filter_time(self, measurement_timestamp: genpy.Time) -> None:
        self.last_position_time = measurement_timestamp

    def get_filter_time(self) -> genpy.Time:
        return self.last_position_time
