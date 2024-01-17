from __future__ import annotations

from ctypes import LittleEndianStructure, c_uint32, sizeof

from bw_tools.structs.teleop_bridge.header import Header
from bw_tools.structs.teleop_bridge.header_type import HeaderType


class PingInfo(LittleEndianStructure):
    _pack_ = 1
    _fields_ = [
        ("_header", Header),
        ("_timestamp", c_uint32),
    ]

    @property
    def timestamp(self) -> int:
        return self._timestamp

    @property
    def header(self) -> Header:
        return self._header

    def to_bytes(self) -> bytes:
        self._header = Header(PingInfo.sizeof(), HeaderType.PING, self.header.device_id)
        return bytes(self)

    @classmethod
    def from_bytes(cls, packet: bytes) -> PingInfo:
        self = cls.from_buffer_copy(packet)
        if self.header.size != cls.sizeof():
            raise ValueError(f"Invalid packet size: {self.header.size}")
        if self.header.type != HeaderType.PING:
            raise ValueError(f"Invalid packet type: {self.header.type}")
        return self

    @classmethod
    def sizeof(cls) -> int:
        return sizeof(cls)
