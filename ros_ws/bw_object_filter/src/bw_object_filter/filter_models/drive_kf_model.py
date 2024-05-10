from threading import Lock

import numpy as np
from geometry_msgs.msg import PoseWithCovariance, Twist

from .filter_model import FilterModel
from .helpers import (
    NUM_MEASUREMENTS,
    NUM_STATES,
    StateArray,
    StateIndices,
    StateSquareMatrix,
    jit_predict,
    jit_update,
    measurement_to_pose,
    pose_to_measurement,
    twist_to_input,
    warmup,
)


class DriveKalmanModel(FilterModel):
    # define properties assigned in "reset"
    state: StateArray
    covariance: StateSquareMatrix
    process_noise_q: StateSquareMatrix
    is_initialized: bool

    def __init__(
        self,
        dt: float,
        process_noise: float = 0.001,
        history_window_seconds: float = 1.0,
    ) -> None:
        self.dt = dt
        self.history_window_seconds = history_window_seconds
        self.process_noise = process_noise
        self.lock = Lock()

        # measurement function for landmarks. Use only pose.
        self.pose_H = np.zeros((NUM_MEASUREMENTS, NUM_STATES))
        self.pose_H[0:NUM_STATES, 0:NUM_STATES] = np.eye(NUM_MEASUREMENTS)

        # measurement function for segmentation estimations. Use only position.
        self.position_H = np.zeros((NUM_MEASUREMENTS, NUM_STATES))
        self.position_H[0:2, 0:2] = np.eye(2)

        self.reset()
        warmup()

    def predict(self, system_input: Twist = Twist()) -> None:
        with self.lock:
            self._clamp_divergent()
            input_u = twist_to_input(system_input)
            self.state, self.covariance = jit_predict(
                self.state, input_u, self.covariance, self.process_noise_q, self.dt
            )

    def update_pose(self, msg: PoseWithCovariance) -> None:
        if self.is_initialized:
            with self.lock:
                measurement, noise = pose_to_measurement(msg)
                measurement = np.nan_to_num(measurement, copy=False)
                self.state, self.covariance = jit_update(
                    self.state, self.covariance, self.pose_H, measurement, noise, (StateIndices.THETA.value,)
                )
        else:
            self.teleport(msg)

    def update_position(self, msg: PoseWithCovariance) -> None:
        if self.is_initialized:
            with self.lock:
                measurement, noise = pose_to_measurement(msg)
                measurement = np.nan_to_num(measurement, copy=False)
                self.state, self.covariance = jit_update(
                    self.state, self.covariance, self.position_H, measurement, noise, (StateIndices.THETA.value,)
                )
        else:
            self.teleport(msg)

    def get_state(self) -> PoseWithCovariance:
        with self.lock:
            return measurement_to_pose(self.state, self.covariance)

    def get_covariance(self) -> StateSquareMatrix:
        return self.covariance

    def _clamp_divergent(self) -> None:
        self.state = np.nan_to_num(self.state, copy=False, nan=0.0, posinf=0.0, neginf=0.0)
        self.covariance = np.nan_to_num(self.covariance, copy=False, nan=1e-3, posinf=1e-3, neginf=1e-3)

    def teleport(self, msg: PoseWithCovariance) -> None:
        with self.lock:
            measurement, measurement_noise = pose_to_measurement(msg)
            self.state = np.zeros(NUM_STATES)
            self.state[0:NUM_STATES] = measurement
            self.covariance = np.eye(NUM_STATES)
            self.covariance[0:NUM_STATES, 0:NUM_STATES] = measurement_noise
            self.is_initialized = True

    def reset(self) -> None:
        with self.lock:
            self.state = np.zeros(NUM_STATES)
            self.covariance = np.eye(NUM_STATES)
            self.process_noise_q = np.eye(NUM_STATES) * self.process_noise
            self.is_initialized = False
