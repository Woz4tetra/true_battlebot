from abc import ABC, abstractmethod

from bw_interfaces.msg import EstimatedObject


class ExtrapolatorInterface(ABC):
    @abstractmethod
    def extrapolate(self, state: EstimatedObject) -> EstimatedObject:
        pass
