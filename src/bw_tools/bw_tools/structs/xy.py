from dataclasses import dataclass
from typing import Sequence, Tuple, Union, overload


@dataclass(frozen=True)
class XY(Sequence):
    x: float
    y: float

    def to_tuple(self) -> Tuple[float, float]:
        return self.x, self.y

    @overload
    def __getitem__(self, idx: int) -> float:
        ...

    @overload
    def __getitem__(self, idx: slice) -> Tuple[float, ...]:
        ...

    def __getitem__(self, idx: Union[int, slice]) -> Union[float, Tuple[float, ...]]:
        return self.to_tuple()[idx]

    def __len__(self) -> int:
        return len(self.to_tuple())
