from enum import IntEnum


class PacketType(IntEnum):
    SUBTYPE = 0
    MOTOR = 1
    PING = 2
    CONFIG = 3
    IMU = 4
