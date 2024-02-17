from __future__ import annotations

from ctypes import c_int16

from bw_tools.structs.teleop_bridge.packet import Packet, build_struct
from bw_tools.structs.teleop_bridge.packet_type import PacketType


class MotorCommand(Packet):
    TYPE = PacketType.SUBTYPE
    STRUCT = build_struct(
        [
            ("velocity", c_int16),
        ]
    )

    @classmethod
    def from_values(cls, velocity: float) -> MotorCommand:
        return cls(cls.STRUCT(int(velocity * 1000)))

    @property
    def velocity(self) -> int:
        return self.struct.velocity
