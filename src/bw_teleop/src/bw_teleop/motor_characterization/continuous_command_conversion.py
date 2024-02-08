import math
from dataclasses import dataclass
from typing import Callable


@dataclass
class ConversionConstants:
    upper_coeffs: tuple[float, ...]
    lower_coeffs: tuple[float, ...]
    upper_freq: float
    lower_freq: float
    upper_vel: float
    lower_vel: float
    ground_vel_to_frequency: float


def fit_function(x, a, b, c, d):
    return a * math.exp(b * x + c) + d


def linear_function(x, a, b):
    return (a / b) * x


def frequency_to_velocity(
    frequency: float,
    lower_nonlinear_fit: Callable[[float], float],
    upper_nonlinear_fit: Callable[[float], float],
    lower_linear_fit: Callable[[float], float],
    upper_linear_fit: Callable[[float], float],
    lower_freq: float,
    upper_freq: float,
) -> float:
    if frequency < lower_freq:
        return -lower_nonlinear_fit(-frequency)
    elif frequency > upper_freq:
        return upper_nonlinear_fit(frequency)
    elif frequency < 0:
        return lower_linear_fit(frequency)
    else:
        return upper_linear_fit(frequency)


def get_conversion_function(constants: ConversionConstants) -> Callable[[float], float]:
    upper_coeffs = constants.upper_coeffs
    lower_coeffs = constants.lower_coeffs
    upper_freq = constants.upper_freq
    lower_freq = constants.lower_freq
    upper_vel = constants.upper_vel
    lower_vel = constants.lower_vel
    ground_vel_to_frequency = constants.ground_vel_to_frequency

    def upper_nonlinear_fit(freq):
        return fit_function(freq, *upper_coeffs)

    def lower_nonlinear_fit(freq):
        return fit_function(freq, *lower_coeffs)

    def upper_linear_fit(freq):
        return linear_function(freq, upper_vel, upper_freq)

    def lower_linear_fit(freq):
        return linear_function(freq, lower_vel, lower_freq)

    def conversion_function(ground_velocity: float) -> float:
        return frequency_to_velocity(
            ground_velocity * ground_vel_to_frequency,
            lower_nonlinear_fit,
            upper_nonlinear_fit,
            lower_linear_fit,
            upper_linear_fit,
            lower_freq,
            upper_freq,
        )

    return conversion_function
