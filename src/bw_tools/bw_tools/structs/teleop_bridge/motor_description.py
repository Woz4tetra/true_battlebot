import struct
from typing import List

from bw_tools.structs.teleop_bridge.header import Header
from bw_tools.structs.teleop_bridge.header_type import HeaderType
from bw_tools.structs.teleop_bridge.motor_command import MotorCommand


class MotorDescription:
    def __init__(self, device_id: int, commands: List[MotorCommand]) -> None:
        self.device_id = device_id
        self.commands = commands

    @property
    def num_channels(self) -> int:
        return len(self.commands)

    def to_bytes(self) -> bytes:
        data = struct.pack("<B", self.num_channels) + b"".join([c.to_bytes() for c in self.commands])
        header = Header(self.device_id, HeaderType.MOTOR, len(data) + Header.sizeof())
        return header.to_bytes() + data
