from __future__ import annotations

from ctypes import c_int8, c_uint8

from bw_tools.structs.teleop_bridge.packet import Packet, build_struct
from bw_tools.structs.teleop_bridge.packet_type import PacketType


class MotorCommand(Packet):
    TYPE = PacketType.SUBTYPE
    STRUCT = build_struct(
        [
            ("direction", c_int8),
            ("speed", c_uint8),
        ]
    )

    @classmethod
    def from_values(cls, velocity: int) -> MotorCommand:
        direction = 1 if velocity > 0 else -1
        speed = abs(velocity)
        return cls(cls.STRUCT(direction, speed))

    @property
    def direction(self) -> int:
        return self.struct.direction

    @property
    def speed(self) -> int:
        return self.struct.speed
