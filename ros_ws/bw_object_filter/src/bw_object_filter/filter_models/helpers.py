import math
from typing import Dict, Tuple

import numpy as np
from bw_shared.geometry.pose2d import Pose2D
from geometry_msgs.msg import PoseWithCovariance, TwistWithCovariance
from numba import njit

NUM_MEASUREMENTS = 3
NUM_POSE_STATES_ROS = 6
NUM_STATES = 6
NUM_STATES_1ST_ORDER = 3


# "enum" for state indices. numba doesn't support python enums
STATE_x = 0
STATE_y = 1
STATE_t = 2
STATE_vx = 3
STATE_vy = 4
STATE_vt = 5

StateArray = np.ndarray
StateSquareMatrix = np.ndarray


def get_index(row: int, col: int, row_length: int) -> int:
    return row * row_length + col


POSE_COVARIANCE_INDICES: Dict[Tuple[int, int], int] = {
    (0, 0): get_index(0, 0, NUM_POSE_STATES_ROS),
    (1, 1): get_index(1, 1, NUM_POSE_STATES_ROS),
    (2, 2): get_index(5, 5, NUM_POSE_STATES_ROS),
}  # fmt: off
TWIST_COVARIANCE_INDICES: Dict[Tuple[int, int], int] = {
    (0, 0): get_index(0, 0, NUM_POSE_STATES_ROS),
    (1, 1): get_index(1, 1, NUM_POSE_STATES_ROS),
    (2, 2): get_index(5, 5, NUM_POSE_STATES_ROS),
}  # fmt: off


def pose_to_measurement(msg: PoseWithCovariance) -> Tuple[np.ndarray, np.ndarray]:
    pose = Pose2D.from_msg(msg.pose)
    measurement = pose.to_array()

    measurement_noise = np.eye(NUM_MEASUREMENTS)
    for mat_index, msg_index in POSE_COVARIANCE_INDICES.items():
        measurement_noise[mat_index] = msg.covariance[msg_index]

    return measurement, measurement_noise


def measurement_to_pose(state: np.ndarray, covariance: np.ndarray) -> PoseWithCovariance:
    pose = PoseWithCovariance()
    pose.pose = Pose2D(state[STATE_x], state[STATE_y], state[STATE_t]).to_msg()
    ros_covariance = [0 for _ in range(NUM_POSE_STATES_ROS * NUM_POSE_STATES_ROS)]
    for mat_index, msg_index in POSE_COVARIANCE_INDICES.items():
        ros_covariance[msg_index] = covariance[mat_index]
    pose.covariance = ros_covariance  # type: ignore
    return pose


def twist_to_measurement(msg: TwistWithCovariance) -> Tuple[np.ndarray, np.ndarray]:
    measurement = np.array([msg.twist.linear.x, msg.twist.linear.y, msg.twist.angular.z])

    measurement_noise = np.eye(NUM_MEASUREMENTS)
    for mat_index, msg_index in TWIST_COVARIANCE_INDICES.items():
        measurement_noise[mat_index] = msg.covariance[msg_index]

    return measurement, measurement_noise


def measurement_to_twist(state: np.ndarray, covariance: np.ndarray) -> TwistWithCovariance:
    twist = TwistWithCovariance()
    twist.twist.linear.x = state[STATE_vx]
    twist.twist.linear.y = state[STATE_vy]
    twist.twist.angular.z = state[STATE_vt]
    ros_covariance = [0 for _ in range(NUM_POSE_STATES_ROS * NUM_POSE_STATES_ROS)]
    for mat_index, msg_index in TWIST_COVARIANCE_INDICES.items():
        ros_covariance[msg_index] = covariance[mat_index]
    twist.covariance = ros_covariance  # type: ignore
    return twist


def orientation_to_measurement(yaw: float, yaw_covariance: float) -> Tuple[np.ndarray, np.ndarray]:
    measurement = np.array([0.0, 0.0, yaw])

    measurement_noise = np.zeros((NUM_MEASUREMENTS, NUM_MEASUREMENTS))
    measurement_noise[2, 2] = yaw_covariance

    return measurement, measurement_noise


@njit
def input_modulus(value: float, min_value: float, max_value: float) -> float:
    """
    Bound the number between min_value and max_value, wrapping around if it goes over.

    Examples:
        input_modulus(1, -1, 3) == 1
        input_modulus(6, -1, 3) == 2
        input_modulus(0, -1, 3) == 0
        input_modulus(5, -1, 3) == 1
    """
    modulus = max_value - min_value

    value -= min_value
    value %= modulus
    value += min_value

    return value


@njit
def normalize_theta(theta):
    # normalize theta to -pi..pi
    return input_modulus(theta, -math.pi, math.pi)


@njit
def state_transition(state, dt, friction_factor):
    x = state[STATE_x]
    y = state[STATE_y]
    theta = normalize_theta(state[STATE_t])

    vx_prev = state[STATE_vx]
    vy_prev = state[STATE_vy]

    vx = vx_prev * math.cos(theta) - vy_prev * math.sin(theta)
    vy = vx_prev * math.sin(theta) + vy_prev * math.cos(theta)
    vt = state[STATE_vt]

    next_state = np.zeros_like(state)
    next_state[STATE_x] = x + dt * vx
    next_state[STATE_y] = y + dt * vy
    next_state[STATE_t] = theta + dt * vt
    next_state[STATE_vx] = vx_prev * friction_factor
    next_state[STATE_vy] = vy_prev * friction_factor
    next_state[STATE_vt] = vt * friction_factor

    return next_state


@njit
def is_invertible(matrix: np.ndarray) -> bool:
    return matrix.shape[0] == matrix.shape[1] and np.linalg.matrix_rank(matrix) == matrix.shape[0]


@njit
def jit_update(
    state_x: np.ndarray,
    covariance_p: np.ndarray,
    observation_model_h: np.ndarray,
    measurement_z: np.ndarray,
    noise_r: np.ndarray,
    angle_wrap_states: Tuple[int, ...],
) -> Tuple[np.ndarray, np.ndarray]:
    y = measurement_z - observation_model_h @ state_x  # error (residual)
    if len(angle_wrap_states) != 0:
        # numba doesn't like it when you supply an empty tuple since it can't
        # resolve the type of its element
        for state_index in angle_wrap_states:
            error = y[state_index]
            error = normalize_theta(error)
            y[state_index] = error
    pht = covariance_p @ observation_model_h.T
    uncertainty = observation_model_h @ pht + noise_r  # project system uncertainty into measurement space
    if is_invertible(uncertainty):
        uncertainty_inv = np.linalg.inv(uncertainty)
    else:
        uncertainty_inv = np.linalg.pinv(uncertainty)
    kalman_gain = pht @ uncertainty_inv  # map system uncertainty into kalman gain
    state_x = state_x + kalman_gain @ y  # predict new x with residual scaled by the kalman gain
    covariance_p = (
        np.eye(len(state_x)) - kalman_gain @ observation_model_h
    ) @ covariance_p  # updated state covariance matrix
    return state_x, covariance_p


@njit
def jit_predict(
    state_x: np.ndarray,
    covariance_p: np.ndarray,
    process_noise_q: np.ndarray,
    dt: float,
    friction_factor: float,
) -> Tuple[np.ndarray, np.ndarray]:
    n = len(state_x)
    n_half = n // 2
    state_transition_model_f = np.eye(n)
    state_transition_model_f[0:n_half, n_half:n] = np.eye(n_half) * dt
    state_x = state_transition(state_x, dt, friction_factor)
    covariance_p = state_transition_model_f @ covariance_p @ state_transition_model_f.T + process_noise_q
    return state_x, covariance_p


def warmup():
    input_modulus(0, -1, 3)
    normalize_theta(0)
    state_transition(np.zeros(NUM_STATES), 0, 0)
    is_invertible(np.eye(NUM_STATES))
    jit_update(
        np.zeros(NUM_STATES), np.eye(NUM_STATES), np.eye(NUM_STATES), np.zeros(NUM_STATES), np.eye(NUM_STATES), ()
    )
    jit_predict(np.zeros(NUM_STATES), np.eye(NUM_STATES), np.eye(NUM_STATES), 0, 0)
