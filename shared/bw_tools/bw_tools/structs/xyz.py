from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Optional, Tuple

from geometry_msgs.msg import Vector3

from bw_tools.structs.xy import XY


@dataclass
class XYZ(XY):
    z: float

    def to_tuple(self) -> Tuple[float, float, float]:
        return self.x, self.y, self.z

    def magnitude(self, other: Optional[XYZ] = None) -> float:
        if other:
            dx = self.x - other.x
            dy = self.y - other.y
            dz = self.z - other.z
        else:
            dx = self.x
            dy = self.y
            dz = self.z
        return math.sqrt(dx * dx + dy * dy + dz * dz)

    def __sub__(self, other: XYZ) -> XYZ:
        return XYZ(self.x - other.x, self.y - other.y, self.z - other.z)

    def __add__(self, other: XYZ) -> XYZ:
        return XYZ(self.x + other.x, self.y + other.y, self.z + other.z)

    def __radd__(self, other: XYZ) -> XYZ:
        return self + other

    def __eq__(self, __value: object) -> bool:
        if not isinstance(__value, XYZ):
            return False
        return self.x == __value.x and self.y == __value.y and self.z == __value.z

    def __lt__(self, other: XYZ) -> bool:
        return self.x < other.x and self.y < other.y and self.z < other.z

    @classmethod
    def from_xy(cls, xy: XY, z: float = 0.0) -> XYZ:
        return XYZ(xy.x, xy.y, z)

    def to_xy(self) -> XY:
        return XY(self.x, self.y)

    @classmethod
    def from_msg(cls, msg: Vector3) -> XYZ:
        return XYZ(msg.x, msg.y, msg.z)

    def to_msg(self) -> Vector3:
        return Vector3(x=self.x, y=self.y, z=self.z)
