from dataclasses import dataclass
from typing import Literal


@dataclass
class NoopKeypointConfig:
    type: Literal["NoopKeypoint"] = "NoopKeypoint"
