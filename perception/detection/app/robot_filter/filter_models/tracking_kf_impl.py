import numpy as np
from app.robot_filter.filter_models.helpers import (
    STATE_t,
    STATE_x,
    STATE_y,
)
from numba import njit

from .helpers import normalize_theta

NUM_STATES = 3
NUM_MEASUREMENTS = 3


@njit(cache=True)
def kf_state_transition(state: np.ndarray) -> np.ndarray:
    x = state[STATE_x]
    y = state[STATE_y]
    theta = normalize_theta(state[STATE_t])

    next_state = np.zeros_like(state)
    next_state[STATE_x] = x
    next_state[STATE_y] = y
    next_state[STATE_t] = theta

    return next_state


@njit(cache=True)
def kf_predict(
    state_x: np.ndarray, covariance_p: np.ndarray, process_noise_q: np.ndarray
) -> tuple[np.ndarray, np.ndarray]:
    n = len(state_x)
    state_transition_model_f = np.eye(n)
    state_x = kf_state_transition(state_x)
    covariance_p = state_transition_model_f @ covariance_p @ state_transition_model_f.T + process_noise_q
    return state_x, covariance_p
