import time
from threading import Lock
from typing import Tuple

import numpy as np
from bw_shared.geometry.pose2d import Pose2D
from bw_shared.geometry.xy import XY
from geometry_msgs.msg import PoseWithCovariance, TwistWithCovariance

from .helpers import (
    NUM_MEASUREMENTS,
    NUM_STATES,
    NUM_STATES_1ST_ORDER,
    STATE_t,
    StateArray,
    StateSquareMatrix,
    jit_predict,
    jit_update,
    measurement_to_pose,
    measurement_to_twist,
    pose_to_measurement,
    twist_to_measurement,
    warmup,
)


class DriveKalmanModel:
    # define properties assigned in "reset"
    state: StateArray
    covariance: StateSquareMatrix
    process_noise_q: StateSquareMatrix
    _is_initialized: bool
    stale_timer: float
    prev_significant_pose: Pose2D

    def __init__(
        self,
        name: str,
        dt: float,
        process_noise: float = 0.001,
        friction_factor: float = 0.2,
        stale_timeout: float = 1.0,
        significant_distance: float = 0.1,
    ) -> None:
        self.name = name
        self.dt = dt
        self.friction_factor = friction_factor
        self.process_noise = process_noise
        self.stale_timeout = stale_timeout
        self.significant_distance = significant_distance
        self.lock = Lock()

        # measurement function for landmarks. Use only pose.
        self.pose_H = np.zeros((NUM_MEASUREMENTS, NUM_STATES))
        self.pose_H[0:NUM_STATES_1ST_ORDER, 0:NUM_STATES_1ST_ORDER] = np.eye(NUM_MEASUREMENTS)

        # measurement function for segmentation estimations. Use only position.
        self.position_H = np.zeros((NUM_MEASUREMENTS, NUM_STATES))
        self.position_H[0:2, 0:2] = np.eye(2)

        # measurement function for cmd_vel. Use only velocity.
        self.cmd_vel_H = np.zeros((NUM_MEASUREMENTS, NUM_STATES))
        self.cmd_vel_H[0:NUM_STATES_1ST_ORDER, NUM_STATES_1ST_ORDER:NUM_STATES] = np.eye(NUM_MEASUREMENTS)

        self.reset()
        warmup()

    def _now(self) -> float:
        return time.perf_counter()

    def _reset_stale_timer(self) -> None:
        self.stale_timer = self._now()

    def predict(self) -> None:
        with self.lock:
            self._clamp_divergent()
            self.state, self.covariance = jit_predict(
                self.state, self.covariance, self.process_noise_q, self.dt, self.friction_factor
            )

    def update_pose(self, msg: PoseWithCovariance) -> None:
        self._update_model_pose(msg, self.pose_H)

    def update_position(self, msg: PoseWithCovariance) -> None:
        self._update_model_pose(msg, self.position_H)

    def _update_model_pose(self, msg: PoseWithCovariance, observation_model: np.ndarray) -> None:
        if self._is_initialized:
            with self.lock:
                measurement, noise = pose_to_measurement(msg)
                measurement = np.nan_to_num(measurement, copy=False)
                self.state, self.covariance = jit_update(
                    self.state, self.covariance, observation_model, measurement, noise, (STATE_t,)
                )
                self._reset_stale_timer()
                pose2d = Pose2D.from_msg(msg.pose)
                if pose2d.magnitude(self.prev_significant_pose) > self.significant_distance:
                    self.prev_significant_pose = pose2d
        else:
            self.teleport(msg)

    def update_cmd_vel(self, msg: TwistWithCovariance) -> None:
        with self.lock:
            measurement, noise = twist_to_measurement(msg)
            self.state, self.covariance = jit_update(
                self.state, self.covariance, self.cmd_vel_H, measurement, noise, tuple()
            )

    def get_state(self) -> Tuple[PoseWithCovariance, TwistWithCovariance]:
        with self.lock:
            pose = measurement_to_pose(self.state, self.covariance)
            return (
                pose,
                measurement_to_twist(self.state, self.covariance),
            )

    def get_covariance(self) -> StateSquareMatrix:
        return self.covariance

    def _clamp_divergent(self) -> None:
        self.state = np.nan_to_num(self.state, copy=False, nan=0.0, posinf=0.0, neginf=0.0)
        self.covariance = np.nan_to_num(self.covariance, copy=False, nan=1e-3, posinf=1e-3, neginf=1e-3)

    def teleport(self, pose: PoseWithCovariance, twist: TwistWithCovariance = TwistWithCovariance()) -> None:
        with self.lock:
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
            self._reset_stale_timer()
            self.prev_significant_pose = Pose2D.from_msg(pose.pose)

    def reset(self) -> None:
        with self.lock:
            self.state = np.zeros(NUM_STATES)
            self.covariance = np.eye(NUM_STATES)
            self.process_noise_q = np.eye(NUM_STATES) * self.process_noise
            self._is_initialized = False
            self.stale_timer = 0.0
            self.prev_significant_pose = Pose2D.zeros()

    def is_in_bounds(self, lower_bound: XY, upper_bound: XY) -> bool:
        return lower_bound.x <= self.state[0] <= upper_bound.x and lower_bound.y <= self.state[1] <= upper_bound.y

    @property
    def is_initialized(self) -> bool:
        return self._is_initialized

    def is_stale(self) -> bool:
        return self._now() - self.stale_timer > self.stale_timeout
