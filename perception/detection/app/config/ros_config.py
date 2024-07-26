from dataclasses import dataclass, field
from typing import List


@dataclass
class RosConfig:
    log: bool = False
    exclude_filters: List[str] = field(default_factory=list)
