from __future__ import annotations

import struct

from bw_tools.structs.teleop_bridge.header_type import HeaderType

MAX_PACKET_SIZE = 512


class Header:
    byte_code = "<HBB"
    length = struct.calcsize(byte_code)

    def __init__(self, device_id: int, type: HeaderType, size: int = 0) -> None:
        self.device_id = device_id
        self.type = type
        self.size = size + self.length

    def as_bytes(self) -> bytes:
        return struct.pack(self.byte_code, self.size, self.type.value, self.device_id)

    @classmethod
    def from_bytes(cls, packet: bytes) -> Header:
        size, type, device_id = struct.unpack(cls.byte_code, packet[: cls.length])
        return cls(device_id, HeaderType(type), size)
