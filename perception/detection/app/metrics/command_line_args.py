from typing import Protocol


class CommandLineArgs(Protocol):
    config: str
    video_file: str
