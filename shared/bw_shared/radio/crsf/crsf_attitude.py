from bw_shared.radio.crsf.crsf_frame_type import FrameType
from bw_shared.radio.crsf.crsf_packet_base import CrsfPacketBase


class CrsfAttitude(CrsfPacketBase):
    type = FrameType.ATTITUDE

    def __init__(self, roll: float = 0.0, pitch: float = 0.0, yaw: float = 0.0) -> None:
        self.roll = roll
        self.pitch = pitch
        self.yaw = yaw

    @classmethod
    def from_bytes(cls, payload: bytes):
        roll = int.from_bytes(payload[:2], "big", signed=True) / 10000
        pitch = int.from_bytes(payload[2:4], "big", signed=True) / 10000
        yaw = int.from_bytes(payload[4:6], "big", signed=True) / 10000
        return cls(roll=roll, pitch=pitch, yaw=yaw)
