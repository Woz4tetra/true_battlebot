from bw_shared.radio.crsf.crsf_frame_type import FrameType
from bw_shared.radio.crsf.crsf_packet_base import CrsfPacketBase


class CrsfFlightMode(CrsfPacketBase):
    type = FrameType.FLIGHT_MODE

    def __init__(self, flight_mode: str = "") -> None:
        self.flight_mode = flight_mode

    @classmethod
    def from_bytes(cls, payload: bytes):
        if payload[-1] != 0:
            raise ValueError("Flight mode string is not null-terminated")
        return cls(flight_mode=payload[:-1].decode("utf-8"))
