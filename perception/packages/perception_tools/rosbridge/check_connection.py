from __future__ import annotations

import logging
import socket
from typing import Any


class SocketContext:
    def __init__(self, hostname: str, port: int):
        self.hostname = hostname
        self.port = port
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.settimeout(1)

    def __enter__(self) -> SocketContext:
        return self

    def connect(self) -> None:
        self.sock.connect((self.hostname, self.port))

    def __exit__(self, exc_type: Any, exc_val: Any, exc_tb: Any) -> None:
        self.sock.close()


def check_connection(hostname: str, port: int) -> bool:
    """
    check our connection to the host and port without sending anything

    returns: bool True if connection is successful
    """
    logger = logging.getLogger("perception")

    # See if the connection can be made over any interface.
    with SocketContext(hostname, port) as sock:
        try:
            sock.connect()
            return True
        except (socket.gaierror, socket.error) as e:
            logger.warning(f"Got error while attempting to connect to {(hostname, port)}: {e}")

    # Connection could not be made
    return False
