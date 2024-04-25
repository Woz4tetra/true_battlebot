from dataclasses import dataclass
from typing import Literal


@dataclass
class ZedConfig:
    type: Literal["ZedCamera"] = "ZedCamera"
