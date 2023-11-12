from __future__ import annotations

from enum import IntEnum

from bw_interfaces.msg import CageCorner as RosCageCorner


class CageCorner(IntEnum):
    DOOR_SIDE = 0
    FAR_SIDE = 1

    @classmethod
    def from_msg(cls, msg: RosCageCorner) -> CageCorner:
        return cls(msg.type)

    @classmethod
    def from_str(cls, string: str) -> CageCorner:
        return cls[string]
