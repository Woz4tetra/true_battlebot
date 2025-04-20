from bw_shared.enums.enum_auto_lower import EnumAutoLowerStr, auto


class FrameId(str, EnumAutoLowerStr):
    MAP = auto()
    MAP_RELATIVE = auto()
    WORLD_CAMERA_0 = auto()
