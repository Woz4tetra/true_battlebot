import inspect
import pickle
from types import FrameType
from typing import Dict, Optional


def _jump_up_n_frames(frame: Optional[FrameType], num_parents: int) -> FrameType:
    for _ in range(num_parents):
        if frame is None:
            raise RuntimeError('Cannot get caller id')
        frame = frame.f_back
    if frame is None:
        raise RuntimeError('Cannot get caller id')
    return frame


def _frame_to_caller_id(frame: FrameType) -> bytes:
    """Borrowed from rospy.core"""
    caller_id = (
        inspect.getabsfile(frame),
        frame.f_lineno,
        frame.f_lasti,
    )
    return pickle.dumps(caller_id)


class ContextSequenceCounter:
    _counter_table: Dict[bytes, int] = {}

    @classmethod
    def seq(cls) -> int:
        frame = inspect.currentframe()
        frame = _jump_up_n_frames(frame, 1)
        caller_id = _frame_to_caller_id(frame)
        counter = cls._counter_table.get(caller_id, 0)
        cls._counter_table[caller_id] = counter + 1
        return counter
