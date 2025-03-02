from enum import IntEnum


class FrameType(IntEnum):
    BATTERY = 0x08
    LINK_STATISTICS = 0x14
    ATTITUDE = 0x1E
    FLIGHT_MODE = 0x21
