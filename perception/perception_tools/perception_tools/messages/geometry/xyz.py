from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Any, Sequence, Tuple, Union, overload


@dataclass
class XYZ(Sequence):
    x: float
    y: float
    z: float

    def to_tuple(self) -> Tuple[float, float, float]:
        return self.x, self.y, self.z

    @overload
    def __getitem__(self, idx: int) -> float: ...

    @overload
    def __getitem__(self, idx: slice) -> Tuple[float, ...]: ...

    def __getitem__(self, idx: Union[int, slice]) -> Union[float, Tuple[float, ...]]:
        return self.to_tuple()[idx]

    def __len__(self) -> int:
        return len(self.to_tuple())

    def magnitude(self, other: Any = None) -> float:
        if other is not None:
            dx = self.x - other.x
            dy = self.y - other.y
            dz = self.z - other.z
        else:
            dx = self.x
            dy = self.y
            dz = self.z
        return math.sqrt(dx * dx + dy * dy + dz * dz)

    def __sub__(self, other: Any) -> XYZ:
        return XYZ(self.x - other.x, self.y - other.y, self.z - other.z)

    def __add__(self, other: Any) -> XYZ:
        return XYZ(self.x + other.x, self.y + other.y, self.z + other.z)

    def __radd__(self, other: Any) -> XYZ:
        return self + other

    def __eq__(self, __value: Any) -> bool:
        if not isinstance(__value, XYZ):
            return False
        return self.x == __value.x and self.y == __value.y and self.z == __value.z

    def __lt__(self, other: Any) -> bool:
        return self.x < other.x and self.y < other.y and self.z < other.z
