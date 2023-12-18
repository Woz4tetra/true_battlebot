from __future__ import annotations

import struct

from bw_tools.structs.teleop_bridge.header import Header
from bw_tools.structs.teleop_bridge.header_type import HeaderType


class PingInfo:
    byte_code = "<I"
    length = struct.calcsize(byte_code)

    def __init__(self, device_id: int, timestamp: int) -> None:
        self.device_id = device_id
        self.timestamp = timestamp

    def as_bytes(self) -> bytes:
        data = struct.pack("<I", self.timestamp)
        header = Header(self.device_id, HeaderType.PING, len(data))
        return header.as_bytes() + data

    @classmethod
    def from_bytes(cls, header: Header, packet: bytes) -> PingInfo:
        if header.size != cls.length + Header.length:
            raise ValueError(f"Invalid packet size: {header.size}")
        if header.type != HeaderType.PING:
            raise ValueError(f"Invalid packet type: {header.type}")
        packet = packet[Header.length : header.size]
        (timestamp,) = struct.unpack(cls.byte_code, packet)
        return cls(header.device_id, timestamp)
