from threading import Lock
from typing import Tuple

import numpy as np
from geometry_msgs.msg import PoseWithCovariance, TwistWithCovariance

from .filter_model import FilterModel
from .helpers import (
    NUM_MEASUREMENTS,
    NUM_STATES,
    NUM_STATES_1ST_ORDER,
    STATE_t,
    StateArray,
    StateSquareMatrix,
    flip_quat_upside_down,
    jit_predict,
    jit_update,
    measurement_to_pose,
    measurement_to_twist,
    pose_to_measurement,
    twist_to_measurement,
    warmup,
)


class DriveKalmanModel(FilterModel):
    state: StateArray
    covariance: StateSquareMatrix
    process_noise_q: StateSquareMatrix
    is_initialized: bool
    is_right_side_up: bool

    def __init__(self, dt: float, process_noise: float = 0.001, friction_factor: float = 0.2) -> None:
        self.dt = dt
        self.friction_factor = friction_factor
        self.process_noise = process_noise
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

    def predict(self) -> None:
        with self.lock:
            self._clamp_divergent()
            self.state, self.covariance = jit_predict(
                self.state, self.covariance, self.process_noise_q, self.dt, self.friction_factor
            )

    def update_pose(self, msg: PoseWithCovariance) -> None:
        if self.is_initialized:
            measurement, noise = pose_to_measurement(msg)
            measurement = np.nan_to_num(measurement, copy=False)
            self.state, self.covariance = jit_update(
                self.state, self.covariance, self.pose_H, measurement, noise, (STATE_t,)
            )
        else:
            self.teleport(msg)

    def update_position(self, msg: PoseWithCovariance) -> None:
        if self.is_initialized:
            with self.lock:
                measurement, noise = pose_to_measurement(msg)
                measurement = np.nan_to_num(measurement, copy=False)
                self.state, self.covariance = jit_update(
                    self.state, self.covariance, self.position_H, measurement, noise, (STATE_t,)
                )
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
            if not self.is_right_side_up:
                pose.pose.orientation = flip_quat_upside_down(pose.pose.orientation)
            return (
                pose,
                measurement_to_twist(self.state, self.covariance),
            )

    def get_covariance(self) -> StateSquareMatrix:
        return self.covariance

    def _clamp_divergent(self) -> None:
        self.state = np.nan_to_num(self.state, copy=False, nan=0.0, posinf=0.0, neginf=0.0)
        self.covariance = np.nan_to_num(self.covariance, copy=False, nan=1e-3, posinf=1e-3, neginf=1e-3)

    def teleport(self, msg: PoseWithCovariance) -> None:
        with self.lock:
            measurement, measurement_noise = pose_to_measurement(msg)
            self.state = np.zeros(NUM_STATES)
            self.state[0:NUM_STATES_1ST_ORDER] = measurement
            self.covariance = np.eye(NUM_STATES)
            self.covariance[0:NUM_STATES_1ST_ORDER, 0:NUM_STATES_1ST_ORDER] = measurement_noise
            self.is_initialized = True

    def reset(self) -> None:
        with self.lock:
            self.state = np.zeros(NUM_STATES)
            self.covariance = np.eye(NUM_STATES)
            self.process_noise_q = np.eye(NUM_STATES) * self.process_noise

            self.is_initialized = False
            self.is_right_side_up = True
