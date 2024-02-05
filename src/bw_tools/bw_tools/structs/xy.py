from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Optional, Sequence, Tuple, Union, overload

from geometry_msgs.msg import Vector3


@dataclass
class XY(Sequence):
    x: float
    y: float

    def to_tuple(self) -> Tuple[float, float]:
        return self.x, self.y

    @overload
    def __getitem__(self, idx: int) -> float: ...

    @overload
    def __getitem__(self, idx: slice) -> Tuple[float, ...]: ...

    def __getitem__(self, idx: Union[int, slice]) -> Union[float, Tuple[float, ...]]:
        return self.to_tuple()[idx]

    def __len__(self) -> int:
        return len(self.to_tuple())

    def magnitude(self, other: Optional[XY] = None) -> float:
        if other:
            dx = self.x - other.x
            dy = self.y - other.y
        else:
            dx = self.x
            dy = self.y
        return math.sqrt(dx * dx + dy * dy)

    def __sub__(self, other: XY) -> XY:
        return XY(self.x - other.x, self.y - other.y)

    def __add__(self, other: XY) -> XY:
        return XY(self.x + other.x, self.y + other.y)

    def __radd__(self, other: XY) -> XY:
        return self + other

    def __eq__(self, __value: object) -> bool:
        if not isinstance(__value, XY):
            return False
        return self.x == __value.x and self.y == __value.y

    def __lt__(self, other: XY) -> bool:
        return self.x < other.x and self.y < other.y

    @classmethod
    def from_msg(cls, msg: Vector3) -> XY:
        return XY(msg.x, msg.y)

    def to_msg(self) -> Vector3:
        return Vector3(x=self.x, y=self.y, z=0.0)
