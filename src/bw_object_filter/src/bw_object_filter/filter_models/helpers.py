import math
from typing import Dict, Tuple

import numpy as np
from bw_tools.structs.pose2d import Pose2D
from bw_tools.structs.transform3d import Transform3D
from geometry_msgs.msg import PoseWithCovariance, Quaternion, TwistWithCovariance, Vector3
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


def pose_to_measurement(
    msg: PoseWithCovariance,
) -> Tuple[np.ndarray, np.ndarray]:
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
    pose.covariance = ros_covariance
    return pose


ROTATE_ABOUT_X = Quaternion(*(1.0, 0.0, 0.0, 0.0))


def flip_quat_upside_down(quat: Quaternion) -> Quaternion:
    rotate_tf = Transform3D.from_position_and_quaternion(Vector3(), ROTATE_ABOUT_X)
    main_tf = Transform3D.from_position_and_quaternion(Vector3(), quat)
    flipped_tf = main_tf.transform_by(rotate_tf)
    return flipped_tf.quaternion


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
    twist.covariance = ros_covariance
    return twist


@njit
def normalize_theta(theta):
    # normalize theta to -pi..pi
    theta = theta % (2 * math.pi)
    if abs(theta) > math.pi:
        if theta > 0:
            return theta - 2 * math.pi
        else:
            return theta + 2 * math.pi
    return theta


@njit
def state_transition_fn(state, dt, friction_factor):
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
    next_state[STATE_vx] = vx_prev - np.sign(vx_prev) * friction_factor * dt
    next_state[STATE_vy] = vy_prev - np.sign(vy_prev) * friction_factor * dt
    next_state[STATE_vt] = vt - np.sign(vt) * friction_factor * dt

    return next_state


@njit
def input_modulus(value, min_value, max_value):
    modulus = max_value - min_value

    # Wrap input if it's above the maximum input
    num_max = int((value - min_value) / modulus)
    value -= num_max * modulus

    # Wrap input if it's below the minimum input
    num_min = int((value - max_value) / modulus)
    value -= num_min * modulus

    return value


@njit
# flake8: noqa: N803 N806
def jit_update(x, P, H, z, R, angle_wrapped=False):
    y = z - H @ x  # error (residual)
    if angle_wrapped:
        angle_error = y[STATE_t]
        angle_error = input_modulus(angle_error, -math.pi, math.pi)
        y[STATE_t] = angle_error
    PHT = P @ H.T
    S = H @ PHT + R  # project system uncertainty into measurement space
    SI = np.linalg.inv(S)
    K = PHT @ SI  # map system uncertainty into kalman gain
    x = x + K @ y  # predict new x with residual scaled by the kalman gain
    P = (np.eye(len(x)) - K @ H) @ P  # updated state covariance matrix
    return x, P


@njit
# flake8: noqa: N803 N806
def jit_predict(x, P, Q, dt, friction_factor):
    n = len(x)
    n_half = n // 2
    F = np.eye(n)
    F[0:n_half, n_half:n] = np.eye(n_half) * dt
    x = state_transition_fn(x, dt, friction_factor)
    P = F @ P @ F.T + Q
    return x, P
