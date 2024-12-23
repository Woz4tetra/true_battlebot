import math


def nearest_square(value: float, upper: bool = True) -> int:
    """Find the nearest square number to a given value.

    Args:
        value: The value to find the nearest square number to.

    Returns:
        The nearest square number to the given value.
    """
    sqrt = math.sqrt(value)
    if upper:
        return math.ceil(sqrt) ** 2
    return math.floor(sqrt) ** 2
