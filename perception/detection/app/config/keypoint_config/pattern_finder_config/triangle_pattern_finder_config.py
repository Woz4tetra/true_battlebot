from dataclasses import dataclass
from typing import Literal


@dataclass
class TrianglePatternFinderConfig:
    type: Literal["TrianglePatternFinder"] = "TrianglePatternFinder"
