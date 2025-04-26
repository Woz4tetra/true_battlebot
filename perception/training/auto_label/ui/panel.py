import tkinter as tk
from abc import ABC, abstractmethod


class Panel(ABC):
    def __init__(self, window: tk.Tk) -> None:
        self.window = window

    @abstractmethod
    def pack(self) -> None:
        pass
