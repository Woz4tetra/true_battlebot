from typing import Tuple

import numpy as np
from bw_tools.structs.pose2d import Pose2D
from geometry_msgs.msg import PoseWithCovariance, TwistWithCovariance

from .filter_model import FilterModel
from .helpers import (
    NUM_MEASUREMENTS,
    NUM_STATES,
    NUM_STATES_1ST_ORDER,
    jit_predict,
    jit_update,
    landmark_to_measurement,
    twist_to_measurement,
)


class DriveKalmanModel(FilterModel):
    def __init__(self, dt: float, process_noise: float = 0.0001) -> None:
        self.dt = dt
        self.state = np.zeros(NUM_STATES)
        self.covariance = np.eye(NUM_STATES)
        self.process_noise_Q = np.eye(NUM_STATES) * process_noise

        # measurement function for landmarks. Use only position.
        self.landmark_H = np.zeros((NUM_MEASUREMENTS, NUM_STATES))
        self.landmark_H[0:NUM_STATES_1ST_ORDER, 0:NUM_STATES_1ST_ORDER] = np.eye(NUM_MEASUREMENTS)

        # measurement function for cmd_vel. Use only velocity.
        self.cmd_vel_H = np.zeros((NUM_MEASUREMENTS, NUM_STATES))
        self.cmd_vel_H[0:NUM_STATES_1ST_ORDER, NUM_STATES_1ST_ORDER:NUM_STATES] = np.eye(NUM_MEASUREMENTS)

    def predict(self) -> None:
        self.state, self.covariance = jit_predict(self.state, self.covariance, self.process_noise_Q, self.dt)

    def update_landmark(self, msg: PoseWithCovariance) -> None:
        measurement, noise = landmark_to_measurement(msg)
        self.state, self.covariance = jit_update(self.state, self.covariance, self.landmark_H, measurement, noise)

    def update_cmd_vel(self, msg: TwistWithCovariance) -> None:
        measurement, noise = twist_to_measurement(msg)
        self.state, self.covariance = jit_update(self.state, self.covariance, self.cmd_vel_H, measurement, noise)

    def get_state(self) -> Tuple[PoseWithCovariance, TwistWithCovariance]:
        twist = TwistWithCovariance()
        twist.twist.linear.x = self.state[3]
        twist.twist.linear.y = self.state[4]
        twist.twist.angular.z = self.state[5]
        twist.covariance = (
            self.covariance[NUM_STATES_1ST_ORDER:NUM_STATES, NUM_STATES_1ST_ORDER:NUM_STATES].flatten().tolist()
        )

        pose = PoseWithCovariance()
        pose.pose = Pose2D(self.state[0], self.state[1], self.state[2]).to_msg()
        pose.covariance = self.covariance[0:NUM_STATES_1ST_ORDER, 0:NUM_STATES_1ST_ORDER].flatten().tolist()

        return pose, twist

    def reset(self, msg: PoseWithCovariance) -> None:
        measurement, measurement_noise = landmark_to_measurement(msg)
        self.state = np.zeros(NUM_STATES)
        self.state[0:NUM_STATES_1ST_ORDER] = measurement
        self.covariance = np.eye(NUM_STATES)
        self.covariance[0:NUM_STATES_1ST_ORDER, 0:NUM_STATES_1ST_ORDER] = measurement_noise
