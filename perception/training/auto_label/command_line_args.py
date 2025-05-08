from typing import Protocol


class CommandLineArgs(Protocol):
    config: str
    keypoints_config: str
    fill: bool
