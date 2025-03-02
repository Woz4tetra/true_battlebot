from bw_shared.radio.crsf.crsf_frame_type import FrameType
from bw_shared.radio.crsf.crsf_packet_base import CrsfPacketBase


class CrsfBattery(CrsfPacketBase):
    type = FrameType.BATTERY

    def __init__(self, voltage: float = 0.0, current: float = 0.0, consumption: float = 0.0) -> None:
        self.voltage = voltage
        self.current = current
        self.consumption = consumption

    @classmethod
    def from_bytes(cls, payload: bytes):
        voltage = int.from_bytes(payload[:2], "big") / 10
        current = int.from_bytes(payload[2:4], "big") / 10
        consumption = int.from_bytes(payload[4:7], "big")
        return cls(voltage=voltage, current=current, consumption=consumption)
