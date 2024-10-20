from typing import TypeVar

import numpy as np

T = TypeVar("T", float, np.ndarray)


def input_modulus(value: T, min_value: float, max_value: float) -> T:
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


def normalize_angle(angle: T) -> T:
    # normalize angle to -pi..pi
    return input_modulus(angle, -np.pi, np.pi)
