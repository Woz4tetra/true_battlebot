from __future__ import annotations

from dataclasses import dataclass

from bw_shared.messages.dataclass_utils import from_dict, to_dict
from bw_shared.radio.crsf.crsf_frame_type import FrameType
from bw_shared.radio.crsf.crsf_packet_base import CrsfPacketBase


@dataclass
class CrsfFlightMode(CrsfPacketBase):
    type: FrameType = FrameType.FLIGHT_MODE
    flight_mode: str = ""

    @classmethod
    def from_bytes(cls, payload: bytes):
        if payload[-1] != 0:
            raise ValueError("Flight mode string is not null-terminated")
        return cls(flight_mode=payload[:-1].decode("utf-8"))

    @classmethod
    def from_dict(cls, data: dict) -> CrsfFlightMode:
        return from_dict(cls, data)

    def to_dict(self) -> dict:
        return to_dict(self)
