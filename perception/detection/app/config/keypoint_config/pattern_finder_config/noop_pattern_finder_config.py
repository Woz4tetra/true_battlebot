from dataclasses import dataclass
from typing import Literal


@dataclass
class NoopPatternFinderConfig:
    type: Literal["NoopPatternFinder"] = "NoopPatternFinder"
