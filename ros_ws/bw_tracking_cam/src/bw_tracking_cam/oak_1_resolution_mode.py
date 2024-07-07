import depthai as dai
from bw_shared.enums.enum_auto_lower import EnumAutoLowerStr, auto


class Oak1ResolutionMode(EnumAutoLowerStr):
    MODE_1080_P = auto()
    MODE_4_K = auto()
    MODE_12_MP = auto()
    MODE_1352X1012 = auto()
    MODE_2024X1520 = auto()

    def to_dai(self) -> dai.ColorCameraProperties.SensorResolution:
        return {
            Oak1ResolutionMode.MODE_1080_P: dai.ColorCameraProperties.SensorResolution.THE_1080_P,
            Oak1ResolutionMode.MODE_4_K: dai.ColorCameraProperties.SensorResolution.THE_4_K,
            Oak1ResolutionMode.MODE_12_MP: dai.ColorCameraProperties.SensorResolution.THE_12_MP,
            Oak1ResolutionMode.MODE_1352X1012: dai.ColorCameraProperties.SensorResolution.THE_1352X1012,
            Oak1ResolutionMode.MODE_2024X1520: dai.ColorCameraProperties.SensorResolution.THE_2024X1520,
        }[self]
