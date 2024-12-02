from typing import Literal, Protocol, Union


class BagCommandLineArgs(Protocol):
    command: Literal["bag"]
    config: str
    bag_file: str


class VideoCommandLineArgs(Protocol):
    command: Literal["video"]
    config: str
    bag_file: str
    video_file: str


class TopicCommandLineArgs(Protocol):
    config: str
    command: Literal["topic"]


CommandLineArgs = Union[BagCommandLineArgs, TopicCommandLineArgs, VideoCommandLineArgs]
