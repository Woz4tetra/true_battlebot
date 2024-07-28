from abc import ABC, abstractmethod
from enum import IntEnum
from typing import Optional, Type, TypeVar, Union, get_args


class FrameType(IntEnum):
    BATTERY = 0x08
    LINK_STATISTICS = 0x14
    ATTITUDE = 0x1E
    FLIGHT_MODE = 0x21


T = TypeVar("T", bound="CrsfPacketBase")


class CrsfPacketBase(ABC):
    type: FrameType

    @classmethod
    @abstractmethod
    def from_bytes(cls: Type[T], payload: bytes) -> T: ...


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


class CrsfLinkStatistics(CrsfPacketBase):
    type = FrameType.LINK_STATISTICS

    def __init__(
        self,
        up_rssi_ant1: int = 0,
        up_rssi_ant2: int = 0,
        up_link_quality: int = 0,
        up_snr: int = 0,
        active_antenna: int = 0,
        rf_profile: int = 0,
        up_rf_power: int = 0,
        down_rssi: int = 0,
        down_link_quality: int = 0,
        down_snr: int = 0,
    ) -> None:
        self.up_rssi_ant1 = up_rssi_ant1  # Uplink RSSI Antenna 1 (dBm * -1)
        self.up_rssi_ant2 = up_rssi_ant2  # Uplink RSSI Antenna 2 (dBm * -1)
        self.up_link_quality = up_link_quality  # Uplink Package success rate / Link quality (%)
        self.up_snr = up_snr  # Uplink SNR (dB)
        self.active_antenna = active_antenna  # number of currently best antenna
        self.rf_profile = rf_profile  # enum {4fps = 0 , 50fps, 150fps}
        self.up_rf_power = up_rf_power  # enum {0mW = 0, 10mW, 25mW, 100mW, 500mW, 1000mW, 2000mW, 250mW, 50mW}
        self.down_rssi = down_rssi  # Downlink RSSI (dBm * -1)
        self.down_link_quality = down_link_quality  # Downlink Package success rate / Link quality (%)
        self.down_snr = down_snr  # Downlink SNR (dB)

    @classmethod
    def from_bytes(cls, payload: bytes):
        up_rssi_ant1 = payload[0]
        up_rssi_ant2 = payload[1]
        up_link_quality = payload[2]
        up_snr = int.from_bytes(payload[3:4], "big", signed=True)
        active_antenna = payload[4]
        rf_profile = payload[5]
        up_rf_power = payload[6]
        down_rssi = payload[7]
        down_link_quality = payload[8]
        down_snr = int.from_bytes(payload[9:10], "big", signed=True)
        return cls(
            up_rssi_ant1=up_rssi_ant1,
            up_rssi_ant2=up_rssi_ant2,
            up_link_quality=up_link_quality,
            up_snr=up_snr,
            active_antenna=active_antenna,
            rf_profile=rf_profile,
            up_rf_power=up_rf_power,
            down_rssi=down_rssi,
            down_link_quality=down_link_quality,
            down_snr=down_snr,
        )

    def to_dict(self):
        return {
            "up_rssi_ant1": self.up_rssi_ant1,
            "up_rssi_ant2": self.up_rssi_ant2,
            "up_link_quality": self.up_link_quality,
            "up_snr": self.up_snr,
            "active_antenna": self.active_antenna,
            "rf_profile": self.rf_profile,
            "up_rf_power": self.up_rf_power,
            "down_rssi": self.down_rssi,
            "down_link_quality": self.down_link_quality,
            "down_snr": self.down_snr,
        }


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


class CrsfFlightMode(CrsfPacketBase):
    type = FrameType.FLIGHT_MODE

    def __init__(self, flight_mode: str = "") -> None:
        self.flight_mode = flight_mode

    @classmethod
    def from_bytes(cls, payload: bytes):
        if payload[-1] != 0:
            raise ValueError("Flight mode string is not null-terminated")
        return cls(flight_mode=payload[:-1].decode("utf-8"))


CrsfPacket = Union[CrsfBattery, CrsfLinkStatistics, CrsfAttitude, CrsfFlightMode]

# CRC8 implementation with polynom = x^8+x^7+x^6+x^4+x^2+1 (0xD5)
# fmt: off
crc8tab = [
    0x00, 0xD5, 0x7F, 0xAA, 0xFE, 0x2B, 0x81, 0x54,
    0x29, 0xFC, 0x56, 0x83, 0xD7, 0x02, 0xA8, 0x7D,
    0x52, 0x87, 0x2D, 0xF8, 0xAC, 0x79, 0xD3, 0x06,
    0x7B, 0xAE, 0x04, 0xD1, 0x85, 0x50, 0xFA, 0x2F,
    0xA4, 0x71, 0xDB, 0x0E, 0x5A, 0x8F, 0x25, 0xF0,
    0x8D, 0x58, 0xF2, 0x27, 0x73, 0xA6, 0x0C, 0xD9,
    0xF6, 0x23, 0x89, 0x5C, 0x08, 0xDD, 0x77, 0xA2,
    0xDF, 0x0A, 0xA0, 0x75, 0x21, 0xF4, 0x5E, 0x8B,
    0x9D, 0x48, 0xE2, 0x37, 0x63, 0xB6, 0x1C, 0xC9,
    0xB4, 0x61, 0xCB, 0x1E, 0x4A, 0x9F, 0x35, 0xE0,
    0xCF, 0x1A, 0xB0, 0x65, 0x31, 0xE4, 0x4E, 0x9B,
    0xE6, 0x33, 0x99, 0x4C, 0x18, 0xCD, 0x67, 0xB2,
    0x39, 0xEC, 0x46, 0x93, 0xC7, 0x12, 0xB8, 0x6D,
    0x10, 0xC5, 0x6F, 0xBA, 0xEE, 0x3B, 0x91, 0x44,
    0x6B, 0xBE, 0x14, 0xC1, 0x95, 0x40, 0xEA, 0x3F,
    0x42, 0x97, 0x3D, 0xE8, 0xBC, 0x69, 0xC3, 0x16,
    0xEF, 0x3A, 0x90, 0x45, 0x11, 0xC4, 0x6E, 0xBB,
    0xC6, 0x13, 0xB9, 0x6C, 0x38, 0xED, 0x47, 0x92,
    0xBD, 0x68, 0xC2, 0x17, 0x43, 0x96, 0x3C, 0xE9,
    0x94, 0x41, 0xEB, 0x3E, 0x6A, 0xBF, 0x15, 0xC0,
    0x4B, 0x9E, 0x34, 0xE1, 0xB5, 0x60, 0xCA, 0x1F,
    0x62, 0xB7, 0x1D, 0xC8, 0x9C, 0x49, 0xE3, 0x36,
    0x19, 0xCC, 0x66, 0xB3, 0xE7, 0x32, 0x98, 0x4D,
    0x30, 0xE5, 0x4F, 0x9A, 0xCE, 0x1B, 0xB1, 0x64,
    0x72, 0xA7, 0x0D, 0xD8, 0x8C, 0x59, 0xF3, 0x26,
    0x5B, 0x8E, 0x24, 0xF1, 0xA5, 0x70, 0xDA, 0x0F,
    0x20, 0xF5, 0x5F, 0x8A, 0xDE, 0x0B, 0xA1, 0x74,
    0x09, 0xDC, 0x76, 0xA3, 0xF7, 0x22, 0x88, 0x5D,
    0xD6, 0x03, 0xA9, 0x7C, 0x28, 0xFD, 0x57, 0x82,
    0xFF, 0x2A, 0x80, 0x55, 0x01, 0xD4, 0x7E, 0xAB,
    0x84, 0x51, 0xFB, 0x2E, 0x7A, 0xAF, 0x05, 0xD0,
    0xAD, 0x78, 0xD2, 0x07, 0x53, 0x86, 0x2C, 0xF9
]
# fmt: on


def crc8(buffer):
    crc = 0
    for c in buffer:
        crc = crc8tab[crc ^ c]
    return crc


CRSF_RC_START_BYTES = (
    0x00,
    0xEE,
    0xEA,
)


class CrsfParser:
    def __init__(self) -> None:
        self.buffer = b""

    def _parse_single_packet(self, packet: bytes) -> tuple[Optional[CrsfPacket], str]:
        crc = packet[-1]
        calc_crc = crc8(packet[2:-1])
        if crc != calc_crc:
            return None, f"CRC mismatch. Expected {calc_crc}, got {crc}"
        command = packet[2]
        payload = packet[3:-1]

        for packet_cls in get_args(CrsfPacket):
            if packet_cls.type == command:
                return packet_cls.from_bytes(payload), ""
        return None, f"Unsupported command {command}"

    def parse(self, buffer: bytes) -> list[tuple[CrsfPacket, str]]:
        buffer = self.buffer + buffer
        results: list[tuple[CrsfPacket, str]] = []
        while len(buffer) > 4:
            if buffer[0] not in CRSF_RC_START_BYTES:
                buffer = buffer[1:]
                continue
            frame_length = buffer[1]
            if not (2 <= frame_length <= 64):
                buffer = buffer[1:]
                continue
            frame_length += 2  # Add 2 bytes for CRC
            if len(buffer) < frame_length:
                break
            if result := self._parse_single_packet(buffer[:frame_length]):
                packet, error_msg = result
                if packet:
                    results.append((packet, error_msg))
            buffer = buffer[frame_length:]
        self.buffer = buffer
        return results
