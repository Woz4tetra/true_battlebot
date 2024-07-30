import logging
import time
from importlib import reload
from typing import Any

import numpy as np
from pythonjsonlogger import jsonlogger


class CustomJsonFormatter(jsonlogger.JsonFormatter):
    def add_fields(self, log_record: dict[str, Any], record: logging.LogRecord, message_dict: dict[str, Any]):
        super(CustomJsonFormatter, self).add_fields(log_record, record, message_dict)
        if not log_record.get("time"):
            # this doesn't use record.created, so it is slightly off
            log_record["time"] = time.time()
        if log_record.get("level"):
            log_record["severity"] = log_record["level"].upper()
            log_record.pop("level")
        else:
            log_record["severity"] = record.levelname
        log_record["message"] = f"<-<{record.message}>->"
        log_record["exc_info"] = record.exc_info
        log_record["exc_text"] = record.exc_text
        log_record["stack_info"] = record.stack_info
        log_record["node"] = record.module
        log_record["logger"] = record.name
        log_record["line"] = record.lineno
        log_record["function"] = record.funcName
        log_record["thread"] = record.thread
        log_record["file"] = record.filename


def initialize() -> None:
    # Reinitialize logging to reset rospy logging
    logging.shutdown()
    reload(logging)
    logging.setLoggerClass(logging.Logger)  # fix rospy breaking logs

    logger = logging.getLogger("perception")
    ros_loggers = [
        logging.getLogger(name)
        for name in (
            "rosout",
            "rospy.client",
            "rospy.msg",
            "rospy.internal",
            "rospy.init",
            "rospy.impl.statistics",
            "rospy.rosout",
            "rospy.simtime",
            "rospy.registration",
            "rospy.tcpros",
            "rospy.service",
        )
    ]
    formatter = CustomJsonFormatter()

    handle = logging.StreamHandler()
    handle.setLevel(logging.DEBUG)
    handle.setFormatter(formatter)
    logger.addHandler(handle)
    logger.setLevel(logging.DEBUG)

    for ros_log in ros_loggers:
        ros_log.addHandler(handle)
        ros_log.setLevel(logging.DEBUG)

    # in case any libraries have added any handlers to the root logger
    logging.root.handlers.clear()

    # Do not keep track of processes to increase performance
    logging.logProcesses = False

    # Set numpy print options, which will determine way numpy arrays are displayed
    np.set_printoptions(formatter={"float": "{: 0.3f}".format})
