from typing import Literal, Protocol, Union


class BagCommandLineArgs(Protocol):
    command: Literal["bag"]
    config: str
    bag_file: str


class NhrlCamCommandLineArgs(Protocol):
    command: Literal["nhrl"]
    config: str
    svo_file: str
    video_file: str


class TopicCommandLineArgs(Protocol):
    config: str
    command: Literal["topic"]


class SvoCommandLineArgs(Protocol):
    command: Literal["svo"]
    config: str
    svo_file: str


CommandLineArgs = Union[BagCommandLineArgs, TopicCommandLineArgs, NhrlCamCommandLineArgs, SvoCommandLineArgs]
