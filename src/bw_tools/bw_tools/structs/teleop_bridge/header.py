from __future__ import annotations

from ctypes import c_uint8, c_uint16

from bw_tools.structs.teleop_bridge.packet import Packet, build_struct
from bw_tools.structs.teleop_bridge.packet_type import PacketType

MAX_PACKET_SIZE = 512


class Header(Packet):
    TYPE = PacketType.SUBTYPE
    STRUCT = build_struct(
        [
            ("size", c_uint16),
            ("type", c_uint8),
            ("device_id", c_uint8),
        ]
    )

    @classmethod
    def from_values(cls, size: int, type: PacketType, device_id: int) -> Header:
        return cls(cls.STRUCT(size, type.value, device_id))

    @property
    def device_id(self) -> int:
        return self.struct.device_id

    @property
    def type(self) -> PacketType:
        return PacketType(self.struct.type)

    @property
    def size(self) -> int:
        return self.struct.size
