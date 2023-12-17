from __future__ import annotations

import struct
from dataclasses import asdict, dataclass

from dacite import from_dict

from bw_tools.structs.teleop_bridge.header import Header
from bw_tools.structs.teleop_bridge.header_type import HeaderType
from bw_tools.structs.teleop_bridge.serial_packet import to_serial_packet


def fill_string(string: str, max_length: int) -> bytes:
    return string.encode() + b"\0" * (max_length - len(string))


@dataclass
class Config:
    ssid: str
    port: int
    device_id: int
    password: str = ""

    @classmethod
    def from_dict(cls, data: dict) -> Config:
        return from_dict(data_class=cls, data=data)

    def to_dict(self) -> dict:
        return asdict(self)

    def as_bytes(self) -> bytes:
        ssid = fill_string(self.ssid, 33)
        password = fill_string(self.password, 64)
        port = struct.pack("<H", self.port)
        data = port + ssid + password + b"\0"
        header = Header(self.device_id, HeaderType.CONFIG, len(data))
        return header.as_bytes() + data

    def as_serial_bytes(self) -> bytes:
        return to_serial_packet(self.as_bytes())

    @classmethod
    def from_bytes(cls, packet: bytes) -> Config:
        header = Header.from_bytes(packet)
        if header.type != HeaderType.CONFIG:
            raise ValueError(f"Invalid packet type: {header.type}")
        packet = packet[Header.length : header.size]
        port = struct.unpack("<H", packet[0:2])[0]
        packet = packet[2:]
        ssid = packet[0:33].decode().strip("\0")
        password = packet[33:].decode().strip("\0")
        return cls(ssid, port, header.device_id, password)
