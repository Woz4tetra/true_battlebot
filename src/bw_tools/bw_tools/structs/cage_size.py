from __future__ import annotations

from enum import IntEnum

from bw_interfaces.msg import CageSize as RosCageSize


class CageSize(IntEnum):
    CAGE_3LB = 0

    @classmethod
    def from_msg(cls, msg: RosCageSize) -> CageSize:
        return cls(msg.size)

    @classmethod
    def from_str(cls, string: str) -> CageSize:
        return {
            "3lb": cls.CAGE_3LB,
        }[string]
