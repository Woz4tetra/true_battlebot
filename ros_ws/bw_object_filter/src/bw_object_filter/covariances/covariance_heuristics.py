from abc import abstractmethod
from typing import Generic, List, TypeVar

T = TypeVar("T")


class CovarianceHeuristics(Generic[T]):
    @abstractmethod
    def compute_covariance(self, measurement: T) -> List[float]:
        pass
