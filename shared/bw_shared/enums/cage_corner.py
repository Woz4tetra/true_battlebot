from __future__ import annotations

from enum import IntEnum

from bw_interfaces.msg import CageCorner as RosCageCorner


class CageCorner(IntEnum):
    BLUE_SIDE = 0
    RED_SIDE = 1

    def to_msg(self) -> RosCageCorner:
        return RosCageCorner(self.value)

    @classmethod
    def from_msg(cls, msg: RosCageCorner) -> CageCorner:
        return cls(msg.type)

    @classmethod
    def from_str(cls, string: str) -> CageCorner:
        return cls[string]
