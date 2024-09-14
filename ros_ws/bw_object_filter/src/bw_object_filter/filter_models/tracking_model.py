import numpy as np
from bw_shared.configs.robot_fleet_config import RobotConfig
from geometry_msgs.msg import PoseWithCovariance, TwistWithCovariance

from .helpers import STATE_t, kf_update, pose_to_measurement
from .model_base import ModelBase
from .tracking_kf_impl import NUM_MEASUREMENTS, NUM_STATES, kf_predict


class TrackingModel(ModelBase):
    def __init__(
        self,
        config: RobotConfig,
        dt: float,
        process_noise: float = 0.001,
        stale_timeout: float = 10.0,
        robot_min_radius: float = 0.05,
        robot_max_radius: float = 0.15,
    ) -> None:
        super().__init__(config, stale_timeout, robot_min_radius, robot_max_radius)
        self.dt = dt
        self.process_noise = process_noise
        self.process_noise_q = np.eye(NUM_STATES) * self.process_noise

        # measurement function for landmarks. Use only pose.
        self.pose_H = np.eye(NUM_MEASUREMENTS)

        # measurement function for segmentation estimations. Use only position.
        self.position_H = np.zeros((NUM_MEASUREMENTS, NUM_STATES))
        self.position_H[0:2, 0:2] = np.eye(2)

    def predict(self) -> None:
        with self.lock:
            self._clamp_divergent()
            next_state, next_covariance = kf_predict(
                self.state[0:NUM_STATES], self.covariance[0:NUM_STATES, 0:NUM_STATES], self.process_noise_q
            )
            self.state[0:NUM_STATES] = next_state
            self.covariance[0:NUM_STATES, 0:NUM_STATES] = next_covariance

    def _update_model_pose(self, msg: PoseWithCovariance, observation_model: np.ndarray) -> None:
        with self.lock:
            measurement, noise = pose_to_measurement(msg)
            measurement = np.nan_to_num(measurement, copy=False)
            next_state, next_covariance = kf_update(
                self.state[0:NUM_STATES],
                self.covariance[0:NUM_STATES, 0:NUM_STATES],
                observation_model,
                measurement,
                noise,
                (STATE_t,),
            )
            self.state[0:NUM_STATES] = next_state
            self.covariance[0:NUM_STATES, 0:NUM_STATES] = next_covariance

            self.reset_stale_timer()

    def update_pose(self, msg: PoseWithCovariance) -> None:
        if not self._is_initialized:
            self.teleport(msg)
        else:
            self._update_model_pose(msg, self.pose_H)

    def update_position(self, msg: PoseWithCovariance) -> None:
        if self._is_initialized:
            self._update_model_pose(msg, self.position_H)

    def update_orientation(self, yaw: float, covariance: np.ndarray) -> None:
        pass  # Not needed for tracking model

    def update_cmd_vel(self, msg: TwistWithCovariance) -> None:
        pass  # Not needed for tracking model
