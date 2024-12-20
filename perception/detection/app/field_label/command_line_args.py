from typing import Literal, Protocol, Union


class BagCommandLineArgs(Protocol):
    command: Literal["bag"]
    bag_file: str


class TopicCommandLineArgs(Protocol):
    command: Literal["topic"]


CommandLineArgs = Union[BagCommandLineArgs, TopicCommandLineArgs]
