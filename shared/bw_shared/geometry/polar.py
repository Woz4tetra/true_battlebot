from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Any, Sequence, Tuple, Union, overload

from bw_shared.geometry.xy import XY, XYInterface


@dataclass(eq=True)
class Polar(Sequence):
    def __init__(self, radius: float, theta: float) -> None:
        self.radius = radius
        self.theta = theta

    def to_tuple(self) -> Tuple[float, float]:
        return self.radius, self.theta

    @overload
    def __getitem__(self, idx: int) -> float: ...

    @overload
    def __getitem__(self, idx: slice) -> Tuple[float, ...]: ...

    def __getitem__(self, idx: Union[int, slice]) -> Union[float, Tuple[float, ...]]:
        return self.to_tuple()[idx]

    def __len__(self) -> int:
        return len(self.to_tuple())

    @classmethod
    def from_xy(cls, xy: XYInterface) -> Polar:
        return Polar(xy.magnitude(), xy.heading())

    def to_xy(self) -> XY:
        return XY(self.radius * math.cos(self.theta), self.radius * math.sin(self.theta))

    def __eq__(self, other: Any) -> bool:
        if not isinstance(other, Polar):
            return False
        return self.radius == other.radius and self.theta == other.theta

    def __lt__(self, other: Any) -> bool:
        return self.radius < other.r and self.theta < other.theta

    def __le__(self, other: Any) -> bool:
        return self.radius <= other.r and self.theta <= other.theta

    def __gt__(self, other: Any) -> bool:
        return self.radius > other.r and self.theta > other.theta

    def __ge__(self, other: Any) -> bool:
        return self.radius >= other.r and self.theta >= other.theta
