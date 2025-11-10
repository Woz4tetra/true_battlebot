from __future__ import annotations

from dataclasses import dataclass

from bw_shared.messages.dataclass_utils import from_dict, to_dict
from bw_shared.radio.crsf.crsf_frame_type import FrameType
from bw_shared.radio.crsf.crsf_packet_base import CrsfPacketBase


@dataclass
class CrsfLinkStatistics(CrsfPacketBase):
    type: FrameType = FrameType.LINK_STATISTICS
    up_rssi_ant1: int = 0  # Uplink RSSI Antenna 1 (dBm * -1)
    up_rssi_ant2: int = 0  # Uplink RSSI Antenna 2 (dBm * -1)
    up_link_quality: int = 0  # Uplink Package success rate / Link quality (%)
    up_snr: int = 0  # Uplink SNR (dB)
    active_antenna: int = 0  # number of currently best antenna
    rf_profile: int = 0  # enum {4fps = 0 , 50fps, 150fps}
    up_rf_power: int = 0  # enum {0mW = 0, 10mW, 25mW, 100mW, 500mW, 1000mW, 2000mW, 250mW, 50mW}
    down_rssi: int = 0  # Downlink RSSI (dBm * -1)
    down_link_quality: int = 0  # Downlink Package success rate / Link quality (%)
    down_snr: int = 0  # Downlink SNR (dB)

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

    @classmethod
    def from_dict(cls, data: dict) -> CrsfLinkStatistics:
        return from_dict(cls, data)

    def to_dict(self) -> dict:
        return to_dict(self)
