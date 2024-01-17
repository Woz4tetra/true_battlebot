from ctypes import Structure, c_uint8, c_uint16, sizeof
from enum import IntEnum


class HeaderType(IntEnum):
    MOTOR = 1
    PING = 2
    CONFIG = 3


class Header:
    class Struct(Structure):
        _fields_ = [
            ("_device_id", c_uint8),
            ("_type", c_uint8),
            ("_size", c_uint16),
        ]

    def __init__(self, device_id: int, type: HeaderType, size: int = 0) -> None:
        self.device_id = device_id
        self.type = type
        self.size = size + sizeof(Header.Struct)

    @classmethod
    def from_bytes(cls, packet: bytes) -> "Header":
        struct = Header.Struct.from_buffer_copy(packet[: sizeof(Header.Struct)])
        return cls(struct._device_id, HeaderType(struct._type), struct._size - sizeof(Header.Struct))


header = Header.from_bytes(b"1\x0134")
print(header.device_id)
print(header.type)
print(header.size)
