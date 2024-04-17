from abc import ABC, abstractmethod


class UiBase(ABC):
    def __init__(self, window) -> None:
        pass

    @abstractmethod
    def pack(self) -> None:
        pass
