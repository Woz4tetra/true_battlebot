from __future__ import annotations

from ctypes import Structure, c_uint8, sizeof
from typing import Dict, List, Tuple, Type

from bw_tools.structs.teleop_bridge.header import Header
from bw_tools.structs.teleop_bridge.motor_command import MotorCommand
from bw_tools.structs.teleop_bridge.packet import Packet, SupportedTypes, build_struct
from bw_tools.structs.teleop_bridge.packet_type import PacketType

BASE_FIELDS: List[Tuple[str, SupportedTypes]] = [("header", Header.STRUCT), ("num_channels", c_uint8)]
STRUCT_CACHE: Dict[int, Type[Structure]] = {0: build_struct(BASE_FIELDS)}


class MotorDescription(Packet):
    TYPE = PacketType.MOTOR
    STRUCT = STRUCT_CACHE[0]

    def __init__(self, struct: Structure) -> None:
        super().__init__(struct)

    @classmethod
    def from_values(cls, device_id: int, motors: List[MotorCommand]) -> MotorDescription:
        num_channels = len(motors)
        if num_channels not in STRUCT_CACHE:
            STRUCT_CACHE[num_channels] = build_struct(
                BASE_FIELDS + [(f"motor_{i}", MotorCommand.STRUCT) for i in range(num_channels)]
            )
        struct_cls = STRUCT_CACHE[num_channels]
        return cls(
            struct_cls(
                Header.from_values(sizeof(struct_cls), cls.TYPE, device_id).struct,
                num_channels,
                *[motor.struct for motor in motors],
            )
        )

    @property
    def header(self) -> Header:
        return Header(self.struct.header)

    @property
    def num_channels(self) -> int:
        return self.struct.num_channels

    def get_channel(self, channel: int) -> MotorCommand:
        return MotorCommand(self.struct.__getattribute__(f"motor_{channel}"))

    @classmethod
    def from_bytes(cls, data: bytes) -> MotorDescription:
        base = super().from_bytes(data)
        lengthed_struct = STRUCT_CACHE[base.struct.num_channels]
        return cls(lengthed_struct.from_buffer_copy(data))
