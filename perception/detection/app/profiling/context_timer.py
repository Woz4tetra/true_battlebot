import logging
import time


class ContextTimer:
    def __init__(self, name):
        self.name = name
        self.start_time = 0.0
        self.end_time = 0.0
        self.logger = logging.getLogger(self.__class__.__name__)

    def __enter__(self):
        self.start_time = time.perf_counter()
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.end_time = time.perf_counter()
        self.logger.debug(f"{self.name} took {self.end_time - self.start_time:0.6f} seconds")
