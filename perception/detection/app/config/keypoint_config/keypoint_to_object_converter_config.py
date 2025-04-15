from dataclasses import dataclass, field
from typing import Literal

from bw_shared.enums.label import Label


@dataclass
class KeypointToObjectConverterConfig:
    type: Literal["KeypointToObjectConverter"] = "KeypointToObjectConverter"
    included_labels: list[Label] = field(default_factory=list)
