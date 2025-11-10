from __future__ import annotations

from dataclasses import dataclass

from bw_shared.messages.dataclass_utils import from_dict, to_dict
from bw_shared.radio.crsf.crsf_frame_type import FrameType
from bw_shared.radio.crsf.crsf_packet_base import CrsfPacketBase


@dataclass
class CrsfBattery(CrsfPacketBase):
    type: FrameType = FrameType.BATTERY
    voltage: float = 0.0
    current: float = 0.0
    consumption: float = 0.0

    @classmethod
    def from_bytes(cls, payload: bytes):
        voltage = int.from_bytes(payload[:2], "big") / 10
        current = int.from_bytes(payload[2:4], "big") / 10
        consumption = int.from_bytes(payload[4:7], "big")
        return cls(voltage=voltage, current=current, consumption=consumption)

    @classmethod
    def from_dict(cls, data: dict) -> CrsfBattery:
        return from_dict(cls, data)

    def to_dict(self) -> dict:
        return to_dict(self)
