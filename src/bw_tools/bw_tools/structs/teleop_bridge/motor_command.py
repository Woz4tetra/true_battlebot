from ctypes import Structure, c_int8, c_uint8


class MotorCommand(Structure):
    _fields_ = [
        ("_direction", c_int8),
        ("_speed", c_uint8),
    ]

    @property
    def direction(self) -> int:
        return self._direction

    @property
    def speed(self) -> int:
        return self._speed

    def to_bytes(self) -> bytes:
        return bytes(self)
