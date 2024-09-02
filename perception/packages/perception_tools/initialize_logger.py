import logging
from importlib import reload

import numpy as np

DEFAULT_FORMATTER = logging.Formatter("[%(levelname)s] [%(name)s] %(asctime)s: %(message)s")


def initialize(formatter: logging.Formatter | None = DEFAULT_FORMATTER) -> None:
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

    handle = logging.StreamHandler()
    handle.setLevel(logging.DEBUG)
    if formatter:
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
