from __future__ import annotations

from ctypes import Structure, c_uint8, c_uint16, sizeof
from enum import IntEnum


class HeaderType(IntEnum):
    MOTOR = 1
    PING = 2
    CONFIG = 3


class Header(Structure):
    _fields_ = [
        ("_device_id", c_uint8),
        ("_type", c_uint8),
        ("_size", c_uint16),
    ]

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


header = Header.from_bytes(b"1\x0134")
print(header.device_id)
print(header.type)
print(header.size)
print(header.to_bytes())


header = Header(49, HeaderType.MOTOR, 0x3433)
print(header.device_id)
print(header.type)
print(header.size)
print(header.to_bytes())
