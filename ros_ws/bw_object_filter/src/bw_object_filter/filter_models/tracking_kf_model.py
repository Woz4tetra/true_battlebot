import numpy as np
from bw_shared.configs.robot_fleet_config import RobotConfig
from geometry_msgs.msg import PoseWithCovariance, TwistWithCovariance

from bw_object_filter.filter_models.helpers import (
    NUM_MEASUREMENTS,
    NUM_STATES,
    NUM_STATES_1ST_ORDER,
    STATE_t,
    kf_update,
    orientation_to_measurement,
    pose_to_measurement,
    twist_to_measurement,
)
from bw_object_filter.filter_models.model_base import ModelBase
from bw_object_filter.filter_models.tracking_kf_impl import kf_predict


class TrackingKalmanModel(ModelBase):
    def __init__(
        self,
        config: RobotConfig,
        dt: float,
        process_noise: float = 0.001,
        friction_factor: float = 0.2,
        stale_timeout: float = 10.0,
        robot_min_radius: float = 0.05,
        robot_max_radius: float = 0.15,
    ) -> None:
        super().__init__(config, stale_timeout, robot_min_radius, robot_max_radius)
        self.state_prior = np.zeros(NUM_STATES)
        self.dt = dt
        self.friction_factor = friction_factor
        self.process_noise = process_noise
        self.process_noise_q = np.eye(NUM_STATES) * self.process_noise

        # measurement function for landmarks. Use only pose.
        self.pose_H = np.zeros((NUM_MEASUREMENTS, NUM_STATES))
        self.pose_H[0:NUM_STATES_1ST_ORDER, 0:NUM_STATES_1ST_ORDER] = np.eye(NUM_MEASUREMENTS)

        # measurement function for segmentation estimations. Use only position.
        self.position_H = np.zeros((NUM_MEASUREMENTS, NUM_STATES))
        self.position_H[0:2, 0:2] = np.eye(2)

        # measurement function for imu yaw estimation. Use only yaw.
        self.orientation_H = np.zeros((NUM_MEASUREMENTS, NUM_STATES))
        self.orientation_H[STATE_t, STATE_t] = 1.0

        # measurement function for cmd_vel. Use only velocity.
        self.cmd_vel_H = np.zeros((NUM_MEASUREMENTS, NUM_STATES))
        self.cmd_vel_H[0:NUM_STATES_1ST_ORDER, NUM_STATES_1ST_ORDER:NUM_STATES] = np.eye(NUM_MEASUREMENTS)

    def predict(self) -> None:
        with self.lock:
            self._clamp_divergent()
            next_state, self.covariance = kf_predict(
                self.state, self.state_prior, self.covariance, self.process_noise_q, self.dt, self.friction_factor
            )
            self.state_prior = self.state
            self.state = next_state

    def update_pose(self, msg: PoseWithCovariance) -> None:
        self._update_model_pose(msg, self.pose_H)

    def update_position(self, msg: PoseWithCovariance) -> None:
        self._update_model_pose(msg, self.position_H)

    def update_orientation(self, yaw: float, covariance: np.ndarray) -> None:
        with self.lock:
            measurement, noise = orientation_to_measurement(yaw, covariance[STATE_t, STATE_t])
            self.state, self.covariance = kf_update(
                self.state, self.covariance, self.orientation_H, measurement, noise, (STATE_t,)
            )

    def _update_model_pose(self, msg: PoseWithCovariance, observation_model: np.ndarray) -> None:
        if self._is_initialized:
            with self.lock:
                measurement, noise = pose_to_measurement(msg)
                measurement = np.nan_to_num(measurement, copy=False)
                self.state, self.covariance = kf_update(
                    self.state, self.covariance, observation_model, measurement, noise, (STATE_t,)
                )
                self.reset_stale_timer()
        else:
            self.teleport(msg)

    def update_cmd_vel(self, msg: TwistWithCovariance) -> None:
        with self.lock:
            measurement, noise = twist_to_measurement(msg)
            self.state, self.covariance = kf_update(
                self.state, self.covariance, self.cmd_vel_H, measurement, noise, tuple()
            )
