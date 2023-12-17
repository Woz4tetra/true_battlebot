import struct


class MotorCommand:
    byte_code = "<bB"
    length = struct.calcsize(byte_code)

    def __init__(self, direction: int, speed: int) -> None:
        self.direction = direction
        self.speed = speed

    def as_bytes(self) -> bytes:
        return struct.pack(self.byte_code, self.direction, self.speed)
