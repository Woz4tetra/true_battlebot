import pyzed.sl as sl
from bw_shared.enums.enum_auto_lower import EnumAutoLowerStr, auto


class Zed2iResolutions(str, EnumAutoLowerStr):
    MODE_AUTO = auto()
    MODE_VGA = auto()
    MODE_HD720 = auto()
    MODE_HD1080 = auto()
    MODE_HD2K = auto()

    def to_zed(self) -> int:
        return {
            Zed2iResolutions.MODE_AUTO: sl.RESOLUTION.AUTO,
            Zed2iResolutions.MODE_VGA: sl.RESOLUTION.VGA,
            Zed2iResolutions.MODE_HD720: sl.RESOLUTION.HD720,
            Zed2iResolutions.MODE_HD1080: sl.RESOLUTION.HD1080,
            Zed2iResolutions.MODE_HD2K: sl.RESOLUTION.HD2K,
        }[self]
