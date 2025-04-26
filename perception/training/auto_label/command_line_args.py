from typing import Protocol


class CommandLineArgs(Protocol):
    config: str
