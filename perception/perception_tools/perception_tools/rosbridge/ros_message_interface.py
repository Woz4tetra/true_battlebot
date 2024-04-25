from typing import Protocol, Type, TypeVar

from perception_tools.rosbridge.types import RawRosMessage

T = TypeVar("T", bound="RosMessageInterface")


class RosMessageInterface(Protocol):
    type: str

    def to_raw(self) -> RawRosMessage:
        pass

    @classmethod
    def from_raw(cls: Type[T], msg: RawRosMessage) -> T:
        pass
