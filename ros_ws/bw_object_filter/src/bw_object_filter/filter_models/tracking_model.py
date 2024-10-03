import math

import numpy as np
from bw_shared.configs.robot_fleet_config import RobotConfig
from bw_shared.filters.simple_filter import SimpleFilter
from bw_shared.geometry.input_modulus import normalize_angle
from geometry_msgs.msg import PoseWithCovariance, TwistWithCovariance

from .helpers import (
    STATE_t,
    STATE_vt,
    STATE_x,
    STATE_y,
    kf_update,
    pose_to_measurement,
    position_to_measurement,
)
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
        velocity_damping_constant: float = 0.15,
    ) -> None:
        super().__init__(config, stale_timeout, robot_min_radius, robot_max_radius)
        self.dt = dt
        self.process_noise = process_noise
        self.process_noise_q = np.eye(NUM_STATES) * self.process_noise

        self.velocity_filters = [
            SimpleFilter(velocity_damping_constant),
            SimpleFilter(velocity_damping_constant),
            SimpleFilter(velocity_damping_constant),
        ]

        # measurement function for landmarks. Use only pose.
        self.pose_H = np.zeros((NUM_MEASUREMENTS, NUM_STATES))
        self.pose_H[0:NUM_MEASUREMENTS, 0:NUM_MEASUREMENTS] = np.eye(NUM_MEASUREMENTS)

        # measurement function for segmentation estimations. Use only position.
        self.position_H = np.zeros((2, NUM_STATES))
        self.position_H[0:2, 0:2] = np.eye(2)

    def predict(self) -> None:
        with self.lock:
            self._clamp_divergent()
            next_state, next_covariance = kf_predict(
                self.state[0:NUM_STATES], self.covariance[0:NUM_STATES, 0:NUM_STATES], self.process_noise_q
            )
            self.state[0:NUM_STATES] = next_state
            self.covariance[0:NUM_STATES, 0:NUM_STATES] = next_covariance

    def update_pose(self, msg: PoseWithCovariance) -> None:
        if not self._is_initialized:
            self.teleport(msg)
            return
        with self.lock:
            measurement, noise = pose_to_measurement(msg)
            measurement = np.nan_to_num(measurement, copy=False)
            state = self.state[0:NUM_STATES]
            covariance = self.covariance[0:NUM_STATES, 0:NUM_STATES]
            next_state, next_covariance = kf_update(state, covariance, self.pose_H, measurement, noise, (STATE_t,))
            next_twist = self._compute_twist_all(state, next_state)
            self.state[0:NUM_STATES] = next_state
            self.state[NUM_STATES:] = next_twist
            self.covariance[0:NUM_STATES, 0:NUM_STATES] = next_covariance

            self.reset_stale_timer()

    def update_position(self, msg: PoseWithCovariance) -> None:
        if not self._is_initialized:
            return
        measurement, noise = position_to_measurement(msg)
        measurement = np.nan_to_num(measurement, copy=False)
        state = self.state[0:NUM_STATES]
        covariance = self.covariance[0:NUM_STATES, 0:NUM_STATES]
        next_state, next_covariance = kf_update(state, covariance, self.position_H, measurement, noise, tuple())
        next_twist = self._compute_twist_xy(state, next_state, self.state[STATE_vt])
        self.state[0:NUM_STATES] = next_state
        self.state[NUM_STATES:] = next_twist
        self.covariance[0:NUM_STATES, 0:NUM_STATES] = next_covariance

        self.reset_stale_timer()

    def update_orientation(self, yaw: float, covariance: np.ndarray) -> None:
        pass  # Not needed for tracking model

    def update_cmd_vel(self, msg: TwistWithCovariance) -> None:
        pass  # Not needed for tracking model

    def _rotate_velocity_to_base(self, velocities: np.ndarray, state_theta: float) -> np.ndarray:
        state_theta *= -1
        vx = velocities[0]
        vy = velocities[1]
        vx = vx * math.cos(state_theta) - vy * math.sin(state_theta)
        vy = vx * math.sin(state_theta) + vy * math.cos(state_theta)
        return np.array([vx, vy, velocities[2]])

    def _compute_twist_all(self, state: np.ndarray, next_state: np.ndarray) -> np.ndarray:
        state_theta = state[STATE_t]
        delta_state = next_state - state
        dx = delta_state[STATE_x]
        dy = delta_state[STATE_y]
        dyaw = delta_state[STATE_t]
        dyaw = normalize_angle(dyaw)
        velocities = []
        for delta_state, filter in zip([dx, dy, dyaw], self.velocity_filters):
            velocities.append(filter.update(delta_state / self.dt))
        return self._rotate_velocity_to_base(np.array(velocities), state_theta)

    def _compute_twist_xy(self, state: np.ndarray, next_state: np.ndarray, prev_angular_velocity: float) -> np.ndarray:
        state_theta = state[STATE_t]
        delta_state = next_state - state
        dx = delta_state[STATE_x]
        dy = delta_state[STATE_y]
        velocities = []
        for delta_state, filter in zip([dx, dy], self.velocity_filters[0:2]):
            velocities.append(filter.update(delta_state / self.dt))
        velocities.append(prev_angular_velocity)
        return self._rotate_velocity_to_base(np.array(velocities), state_theta)
