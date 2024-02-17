from __future__ import annotations

from ctypes import c_uint32

from bw_tools.structs.teleop_bridge.header import Header
from bw_tools.structs.teleop_bridge.packet import Packet, build_struct
from bw_tools.structs.teleop_bridge.packet_type import PacketType


class PingInfo(Packet):
    TYPE = PacketType.PING
    STRUCT = build_struct(
        [
            ("header", Header.STRUCT),
            ("timestamp", c_uint32),
        ]
    )

    @classmethod
    def from_values(cls, device_id: int, timestamp: int) -> PingInfo:
        return cls(cls.STRUCT(Header.from_values(cls.sizeof(), cls.TYPE, device_id).struct, timestamp))

    @property
    def header(self) -> Header:
        return self.struct.header

    @property
    def timestamp(self) -> int:
        return self.struct.timestamp
