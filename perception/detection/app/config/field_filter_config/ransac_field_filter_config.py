from dataclasses import dataclass
from typing import Literal


@dataclass
class RansacFieldFilterConfig:
    type: Literal["RansacFieldFilter"] = "RansacFieldFilter"
