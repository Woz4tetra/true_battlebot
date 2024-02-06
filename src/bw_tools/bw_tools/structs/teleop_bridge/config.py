from __future__ import annotations

from ctypes import c_byte, c_uint16
from dataclasses import asdict, dataclass

from dacite import from_dict

from bw_tools.structs.teleop_bridge.header import Header
from bw_tools.structs.teleop_bridge.packet import Packet, build_struct
from bw_tools.structs.teleop_bridge.packet_type import PacketType
from bw_tools.structs.teleop_bridge.serial_packet import to_serial_packet
from bw_tools.structs.teleop_bridge.string_array import array_to_string, fill_string


class ConfigPacket(Packet):
    TYPE = PacketType.CONFIG

    SsidBuffer = c_byte * 33
    PasswordBuffer = c_byte * 65  # expected size is 64, but packet lengths don't match up if an extra byte isn't added

    STRUCT = build_struct(
        [
            ("header", Header.STRUCT),
            ("port", c_uint16),
            ("ssid", SsidBuffer),
            ("password", PasswordBuffer),
        ]
    )

    @classmethod
    def from_values(cls, device_id: int, port: int, ssid: str, password: str) -> ConfigPacket:
        ssid_bytes = fill_string(ssid, 33)  # max length of ssid is 32 + null terminator
        password_bytes = fill_string(password, 64)  # max length of password is 63 + null terminator
        ssid_buffer = cls.SsidBuffer(*ssid_bytes)
        password_buffer = cls.PasswordBuffer(*password_bytes)
        return cls(
            cls.STRUCT(
                Header.from_values(cls.sizeof(), cls.TYPE, device_id),
                port,
                ssid_buffer,
                password_buffer,
            )
        )

    @property
    def header(self) -> Header:
        return self.struct.header

    @property
    def port(self) -> int:
        return self.struct.port

    @property
    def ssid(self) -> str:
        return array_to_string(self.struct.ssid)

    @property
    def password(self) -> str:
        return array_to_string(self.struct.password)


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

    def to_bytes(self) -> bytes:
        return ConfigPacket.from_values(self.device_id, self.port, self.ssid, self.password).to_bytes()

    def to_serial_bytes(self) -> bytes:
        return to_serial_packet(self.to_bytes())

    @classmethod
    def from_bytes(cls, packet: bytes) -> Config:
        struct = ConfigPacket.from_bytes(packet)
        header = struct.header
        if header.type != PacketType.CONFIG:
            raise ValueError(f"Invalid packet type: {header.type}")
        return cls(struct.ssid, struct.port, header.device_id, struct.password)
