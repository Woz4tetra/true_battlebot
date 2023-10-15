import time
from dataclasses import dataclass
from typing import Optional

import rospy
from std_msgs.msg import Header as RosHeader

from bw_tools.structs.context_sequence_counter import ContextSequenceCounter
from bw_tools.typing.basic import seconds_to_duration


@dataclass(frozen=True, eq=True)
class Header:
    stamp: float
    frame_id: str
    seq: int

    @classmethod
    def auto(cls, frame_id: str = '', stamp: float = float("nan"), seq: Optional[int] = None) -> "Header":
        if stamp != stamp:
            if rospy.core.is_initialized():
                stamp = rospy.Time.now().to_sec()
            else:
                stamp = time.time()
        if seq is None:
            seq = ContextSequenceCounter.seq()
        return cls(stamp, frame_id, seq)

    @classmethod
    def from_msg(cls, msg: RosHeader) -> "Header":
        return cls(msg.stamp.to_sec(), msg.frame_id, msg.seq)

    def to_msg(self) -> RosHeader:
        return RosHeader(stamp=seconds_to_duration(self.stamp), frame_id=self.frame_id, seq=self.seq)

    def __str__(self):
        return "%s(stamp=%0.3f, frame_id=%s, seq=%d)" % (self.__class__.__name__, self.stamp, self.frame_id, self.seq)

    __repr__ = __str__
