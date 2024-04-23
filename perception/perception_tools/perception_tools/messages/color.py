from __future__ import annotations

from dataclasses import dataclass


@dataclass
class Color:
    r: float
    g: float
    b: float
    a: float

    def to_cv_color(self) -> tuple:
        return (int(self.r * 255), int(self.g * 255), int(self.b * 255))
