from __future__ import annotations

import time
from dataclasses import dataclass
from typing import Optional

from bw_shared.context_sequence_counter import ContextSequenceCounter
from perception_tools.rosbridge.types import RawRosMessage


@dataclass(eq=True)
class Header:
    stamp: float
    frame_id: str
    seq: int
    type: str = "std_msgs/Header"

    @classmethod
    def auto(cls, frame_id: str = "", stamp: float = float("nan"), seq: Optional[int] = None) -> "Header":
        if stamp != stamp:
            stamp = time.time()
        if seq is None:
            seq = ContextSequenceCounter.seq()
        return cls(stamp, frame_id, seq)

    def __str__(self) -> str:
        return "%s(stamp=%0.3f, frame_id=%s, seq=%d)" % (self.__class__.__name__, self.stamp, self.frame_id, self.seq)

    def to_raw(self) -> RawRosMessage:
        return {
            "stamp": {"sec": int(self.stamp), "nsec": int((self.stamp % 1) * 1e9)},
            "frame_id": self.frame_id,
            "seq": self.seq,
        }

    @classmethod
    def from_raw(cls, msg: RawRosMessage) -> Header:
        sec = msg["stamp"]["secs"]
        nsec = msg["stamp"]["nsecs"]
        stamp = sec + nsec * 1e-9
        return cls(stamp, msg["frame_id"], msg["seq"])

    __repr__ = __str__
