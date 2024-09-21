from typing import Optional


class SimpleFilter:
    """
    A simple iterative damping filter
    """

    def __init__(self, damping_constant: Optional[float]) -> None:
        self.damping_constant = damping_constant
        self.value: float = 0.0

    def reset(self, value: float) -> None:
        self.value = value

    def update(self, value: float) -> float:
        if self.damping_constant is None or self.damping_constant == 0.0:
            self.value = value
        else:
            self.value += self.damping_constant * (value - self.value)
        return self.value
