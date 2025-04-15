import logging

from rosgraph import roslogging


def fix_rosgraph_logging() -> None:
    # Workaround to prevent hangs when a call is made to the broken ros logger
    roslogging.RospyLogger.findCaller = logging.Logger.findCaller  # type: ignore
