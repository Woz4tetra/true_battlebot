from __future__ import annotations

from dataclasses import dataclass

from std_msgs.msg import ColorRGBA


@dataclass
class Color:
    r: float
    g: float
    b: float
    a: float

    @classmethod
    def from_msg(cls, msg: ColorRGBA) -> Color:
        return cls(msg.r, msg.g, msg.b, msg.a)

    def to_msg(self) -> ColorRGBA:
        return ColorRGBA(self.r, self.g, self.b, self.a)

    def to_cv_color(self) -> tuple:
        return (int(self.r * 255), int(self.g * 255), int(self.b * 255))
