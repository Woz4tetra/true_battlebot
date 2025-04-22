from dataclasses import dataclass
from typing import Literal


@dataclass
class TrianglePatternFinderConfig:
    type: Literal["TrianglePatternFinder"] = "TrianglePatternFinder"
    threshold: int = 200
    contour_smoothing: float = 0.07
