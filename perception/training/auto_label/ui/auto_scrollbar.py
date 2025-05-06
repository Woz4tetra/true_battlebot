import tkinter as tk
from tkinter import ttk
from typing import Any


class AutoScrollbar(ttk.Scrollbar):
    """A scrollbar that hides itself if it's not needed. Works only for grid geometry manager"""

    def set(self, lo: float | str, hi: float | str) -> None:
        if float(lo) <= 0.0 and float(hi) >= 1.0:
            self.grid_remove()
        else:
            self.grid()
            ttk.Scrollbar.set(self, lo, hi)

    def pack(self, **kw: Any) -> None:  # type: ignore
        raise tk.TclError("Cannot use pack with the widget " + self.__class__.__name__)

    def place(self, **kw: Any) -> None:  # type: ignore
        raise tk.TclError("Cannot use place with the widget " + self.__class__.__name__)
