from __future__ import annotations

from ctypes import LittleEndianStructure, c_byte, c_char_p, c_uint16, cast, sizeof
from dataclasses import asdict, dataclass

from dacite import from_dict

from bw_tools.structs.teleop_bridge.header import Header
from bw_tools.structs.teleop_bridge.header_type import HeaderType
from bw_tools.structs.teleop_bridge.serial_packet import to_serial_packet


def fill_string(string: str, max_length: int) -> bytes:
    return string.encode() + b"\0" * (max_length - len(string))


SsidBuffer = c_byte * 33
PasswordBuffer = c_byte * 65  # expected size is 64, but packet lengths don't match up if an extra byte isn't added


def array_to_string(array: c_byte) -> str:
    value = cast(array, c_char_p).value
    if value is None:
        raise ValueError(f"Failed to cast array to string: {array}")
    return value.decode().strip("\0")


class ConfigStruct(LittleEndianStructure):
    _pack_ = 1
    _fields_ = [
        ("_header", Header),
        ("_port", c_uint16),
        ("_ssid", SsidBuffer),
        ("_password", PasswordBuffer),
    ]

    @property
    def header(self) -> Header:
        return self._header

    @property
    def port(self) -> int:
        return self._port

    @property
    def ssid(self) -> str:
        return array_to_string(self._ssid)

    @property
    def password(self) -> str:
        return array_to_string(self._password)

    def to_bytes(self) -> bytes:
        return bytes(self)

    @classmethod
    def from_bytes(cls, packet: bytes) -> ConfigStruct:
        return cls.from_buffer_copy(packet)

    @classmethod
    def sizeof(cls) -> int:
        return sizeof(cls)


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
        ssid = fill_string(self.ssid, 33)  # max length of ssid is 32 + null terminator
        password = fill_string(self.password, 64)  # max length of password is 63 + null terminator
        ssid_buffer = SsidBuffer(*ssid)
        password_buffer = PasswordBuffer(*password)
        struct = ConfigStruct(
            Header(ConfigStruct.sizeof(), HeaderType.CONFIG, self.device_id), self.port, ssid_buffer, password_buffer
        )
        return struct.to_bytes()

    def to_serial_bytes(self) -> bytes:
        return to_serial_packet(self.to_bytes())

    @classmethod
    def from_bytes(cls, packet: bytes) -> Config:
        struct = ConfigStruct.from_bytes(packet)
        header = struct.header
        if header.type != HeaderType.CONFIG:
            raise ValueError(f"Invalid packet type: {header.type}")
        return cls(struct.ssid, struct.port, header.device_id, struct.password)
