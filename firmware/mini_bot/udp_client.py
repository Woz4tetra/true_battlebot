import socket
import struct
import time
from dataclasses import dataclass
from enum import IntEnum
from typing import List


class HeaderType(IntEnum):
    MOTOR = 1


@dataclass
class Header:
    device_id: int
    type: HeaderType

    def as_bytes(self, size: int) -> bytes:
        return struct.pack("<HBB", size + 4, self.type.value, self.device_id)


@dataclass
class MotorCommand:
    direction: int
    speed: int

    def as_bytes(self) -> bytes:
        return struct.pack("<BB", self.direction, self.speed)


@dataclass
class MotorDescription:
    device_id: int
    commands: List[MotorCommand]

    @property
    def num_channels(self) -> int:
        return len(self.commands)

    def as_bytes(self) -> bytes:
        data = struct.pack("<B", self.num_channels) + b"".join([c.as_bytes() for c in self.commands])
        header = Header(self.device_id, HeaderType.MOTOR)
        return header.as_bytes(len(data)) + data


sock = socket.socket(type=socket.SOCK_DGRAM)
sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
sock.bind(("0.0.0.0", 4176))
command = 0
while True:
    packet = MotorDescription(1, [MotorCommand(1, command), MotorCommand(1, command)]).as_bytes()
    command = (command + 1) % 255
    sock.sendto(packet, ("192.168.8.255", 4176))
    time.sleep(0.02)
