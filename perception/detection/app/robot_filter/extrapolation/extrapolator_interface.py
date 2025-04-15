from abc import ABC, abstractmethod

from bw_interfaces.msg import EstimatedObject
from geometry_msgs.msg import TwistStamped


class ExtrapolatorInterface(ABC):
    @abstractmethod
    def extrapolate(self, state: EstimatedObject, velocities: list[TwistStamped]) -> EstimatedObject:
        pass
