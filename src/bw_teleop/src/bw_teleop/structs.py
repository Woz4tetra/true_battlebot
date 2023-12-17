from __future__ import annotations

import struct
from enum import IntEnum
from typing import List


class HeaderType(IntEnum):
    MOTOR = 1
    PING = 2


class Header:
    byte_code = "<HBB"
    length = struct.calcsize(byte_code)

    def __init__(self, device_id: int, type: HeaderType, size: int = 0) -> None:
        self.device_id = device_id
        self.type = type
        self.size = size

    def as_bytes(self) -> bytes:
        return struct.pack(self.byte_code, self.size + self.length, self.type.value, self.device_id)

    @classmethod
    def from_bytes(cls, packet: bytes) -> Header:
        size, type, device_id = struct.unpack(cls.byte_code, packet[: cls.length])
        return cls(device_id, HeaderType(type), size)


class MotorCommand:
    byte_code = "<bB"
    length = struct.calcsize(byte_code)

    def __init__(self, direction: int, speed: int) -> None:
        self.direction = direction
        self.speed = speed

    def as_bytes(self) -> bytes:
        return struct.pack(self.byte_code, self.direction, self.speed)


class MotorDescription:
    def __init__(self, device_id: int, commands: List[MotorCommand]) -> None:
        self.device_id = device_id
        self.commands = commands

    @property
    def num_channels(self) -> int:
        return len(self.commands)

    def as_bytes(self) -> bytes:
        data = struct.pack("<B", self.num_channels) + b"".join([c.as_bytes() for c in self.commands])
        header = Header(self.device_id, HeaderType.MOTOR, len(data))
        return header.as_bytes() + data


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
