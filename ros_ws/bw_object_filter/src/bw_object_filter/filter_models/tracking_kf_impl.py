import math

import numpy as np
from numba import njit

from bw_object_filter.filter_models.helpers import (
    STATE_t,
    STATE_vt,
    STATE_vx,
    STATE_vy,
    STATE_x,
    STATE_y,
)

from .helpers import normalize_theta


@njit
def kf_state_transition(state, state_prior, dt, friction_factor):
    x = state[STATE_x]
    y = state[STATE_y]
    theta = normalize_theta(state[STATE_t])

    prev_x = state_prior[STATE_x]
    prev_y = state_prior[STATE_y]

    vx_prev = (x - prev_x) / dt
    vy_prev = (y - prev_y) / dt

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
def kf_predict(
    state_x: np.ndarray,
    state_prior_x: np.ndarray,
    covariance_p: np.ndarray,
    process_noise_q: np.ndarray,
    dt: float,
    friction_factor: float,
) -> tuple[np.ndarray, np.ndarray]:
    n = len(state_x)
    n_half = n // 2
    state_transition_model_f = np.eye(n)
    state_transition_model_f[0:n_half, n_half:n] = np.eye(n_half) * dt
    state_x = kf_state_transition(state_x, state_prior_x, dt, friction_factor)
    covariance_p = state_transition_model_f @ covariance_p @ state_transition_model_f.T + process_noise_q
    return state_x, covariance_p
