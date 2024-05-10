import math
from enum import IntEnum
from typing import Dict, Tuple

import numpy as np
from bw_tools.structs.pose2d import Pose2D
from geometry_msgs.msg import PoseWithCovariance, Twist
from numba import njit

NUM_MEASUREMENTS = 3
NUM_POSE_STATES_ROS = 6
NUM_STATES = 3
NUM_INPUTS = 3


class StateIndices(IntEnum):
    X = 0
    Y = 1
    THETA = 2


class InputIndices(IntEnum):
    X = 0
    Y = 1
    THETA = 2


StateArray = np.ndarray  # 3x1 array
StateSquareMatrix = np.ndarray  # 3x3 array
InputArray = np.ndarray  # 3x1 array


def get_index(row: int, col: int, row_length: int) -> int:
    return row * row_length + col


POSE_COVARIANCE_INDICES: Dict[Tuple[int, int], int] = {
    (0, 0): get_index(0, 0, NUM_POSE_STATES_ROS),
    (1, 1): get_index(1, 1, NUM_POSE_STATES_ROS),
    (2, 2): get_index(5, 5, NUM_POSE_STATES_ROS),
}


def pose_to_measurement(msg: PoseWithCovariance) -> Tuple[np.ndarray, np.ndarray]:
    pose = Pose2D.from_msg(msg.pose)
    measurement = pose.to_array()

    measurement_noise = np.eye(NUM_MEASUREMENTS)
    for mat_index, msg_index in POSE_COVARIANCE_INDICES.items():
        measurement_noise[mat_index] = msg.covariance[msg_index]

    return measurement, measurement_noise


def measurement_to_pose(state: np.ndarray, covariance: np.ndarray) -> PoseWithCovariance:
    pose = PoseWithCovariance()
    pose.pose = Pose2D(state[StateIndices.X], state[StateIndices.Y], state[StateIndices.THETA]).to_msg()
    ros_covariance = [0 for _ in range(NUM_POSE_STATES_ROS * NUM_POSE_STATES_ROS)]
    for mat_index, msg_index in POSE_COVARIANCE_INDICES.items():
        ros_covariance[msg_index] = covariance[mat_index]
    pose.covariance = ros_covariance  # type: ignore
    return pose


def twist_to_input(twist: Twist) -> np.ndarray:
    return np.array([twist.linear.x, twist.linear.y, twist.angular.z])


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
    state_x: StateArray,
    input_u: InputArray,
    covariance_p: StateSquareMatrix,
    process_noise_q: StateSquareMatrix,
    dt: float,
) -> Tuple[np.ndarray, np.ndarray]:
    # this model is very simple because the input matches the output
    state_x += input_u * dt
    covariance_p += process_noise_q
    return state_x, covariance_p


def warmup():
    input_modulus(0, -1, 3)
    normalize_theta(0)
    is_invertible(np.eye(NUM_STATES))
    jit_update(
        np.zeros(NUM_STATES), np.eye(NUM_STATES), np.eye(NUM_STATES), np.zeros(NUM_STATES), np.eye(NUM_STATES), ()
    )
    jit_predict(np.zeros(NUM_STATES), np.zeros(NUM_INPUTS), np.eye(NUM_STATES), np.eye(NUM_STATES), 1.0)
