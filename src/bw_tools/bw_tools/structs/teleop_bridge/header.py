from __future__ import annotations

from ctypes import LittleEndianStructure, c_uint8, c_uint16, sizeof

from bw_tools.structs.teleop_bridge.header_type import HeaderType

MAX_PACKET_SIZE = 512


class Header(LittleEndianStructure):
    _fields_ = [
        ("_size", c_uint16),
        ("_type", c_uint8),
        ("_device_id", c_uint8),
    ]

    @classmethod
    def from_id(cls, device_id: int) -> Header:
        return cls(0, 0, device_id)

    @property
    def device_id(self) -> int:
        return self._device_id

    @property
    def type(self) -> HeaderType:
        return HeaderType(self._type)

    @property
    def size(self) -> int:
        return self._size

    @classmethod
    def from_bytes(cls, packet: bytes) -> Header:
        return cls.from_buffer_copy(packet[: sizeof(cls)])

    def to_bytes(self) -> bytes:
        return bytes(self)

    @classmethod
    def sizeof(cls) -> int:
        return sizeof(cls)
