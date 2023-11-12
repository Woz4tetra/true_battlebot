from __future__ import annotations

from enum import IntEnum

from bw_interfaces.msg import BehaviorMode as RosBehaviorMode


class BehaviorMode(IntEnum):
    IDLE = 0
    CORNER = 1
    FIGHT = 2

    def to_msg(self) -> RosBehaviorMode:
        return RosBehaviorMode(self.value)

    @classmethod
    def from_msg(cls, msg: RosBehaviorMode) -> BehaviorMode:
        return cls(msg.mode)
