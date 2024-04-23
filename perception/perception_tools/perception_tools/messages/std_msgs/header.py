import time
from dataclasses import dataclass
from typing import Optional

from bw_shared.context_sequence_counter import ContextSequenceCounter


@dataclass(frozen=True, eq=True)
class Header:
    stamp: float
    frame_id: str
    seq: int

    @classmethod
    def auto(cls, frame_id: str = "", stamp: float = float("nan"), seq: Optional[int] = None) -> "Header":
        if stamp != stamp:
            stamp = time.time()
        if seq is None:
            seq = ContextSequenceCounter.seq()
        return cls(stamp, frame_id, seq)

    def __str__(self) -> str:
        return "%s(stamp=%0.3f, frame_id=%s, seq=%d)" % (self.__class__.__name__, self.stamp, self.frame_id, self.seq)

    __repr__ = __str__
