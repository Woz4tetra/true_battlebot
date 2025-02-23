from abc import ABC, abstractmethod
from typing import Type, TypeVar

from bw_shared.radio.crsf.crsf_frame_type import FrameType

T = TypeVar("T", bound="CrsfPacketBase")


class CrsfPacketBase(ABC):
    type: FrameType

    @classmethod
    @abstractmethod
    def from_bytes(cls: Type[T], payload: bytes) -> T: ...
