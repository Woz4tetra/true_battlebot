from bw_shared.radio.crsf.crsf_frame_type import FrameType
from bw_shared.radio.crsf.crsf_packet_base import CrsfPacketBase


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
