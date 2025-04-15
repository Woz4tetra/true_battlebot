import logging
import sys

import numpy as np
import rospy  # noqa: F401  loads ROS loggers.

from perception_tools.fix_rosgraph_logging import fix_rosgraph_logging

DEFAULT_FORMATTER = logging.Formatter("[%(levelname)s] [%(name)s] %(asctime)s: %(message)s")


def initialize(log_level: int = logging.DEBUG, formatter: logging.Formatter | None = DEFAULT_FORMATTER) -> None:
    fix_rosgraph_logging()

    class CustomLogger(logging.Logger):
        def __init__(self, name: str):
            super().__init__(name)
            handler = logging.StreamHandler(sys.stdout)
            handler.setFormatter(formatter)
            super().addHandler(handler)

    logging.setLoggerClass(CustomLogger)

    logging.getLogger().setLevel(logging.DEBUG)

    loggers = [logging.getLogger(name) for name in logging.root.manager.loggerDict]
    handle = logging.StreamHandler()
    handle.setLevel(log_level)
    if formatter:
        handle.setFormatter(formatter)

    for logger in loggers:
        logger.addHandler(handle)
        logger.setLevel(log_level)

    # in case any libraries have added any handlers to the root logger
    logging.root.handlers.clear()

    # Do not keep track of processes to increase performance
    logging.logProcesses = False

    # Set numpy print options, which will determine way numpy arrays are displayed
    np.set_printoptions(formatter={"float": "{: 0.6f}".format})
