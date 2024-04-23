from py_trees.behaviour import Behaviour
from py_trees.common import Status

from bw_behaviors.container import Container


class RecoveryBehavior(Behaviour):
    def __init__(self, container: Container, recovery_key: str = "") -> None:
        super().__init__(self.__class__.__name__)
        self.recovery_key = recovery_key
        self.recovery_manager = container.recovery_manager

    def initialise(self) -> None:
        self.recovery_manager.send_recovery(self.recovery_key)

    def update(self) -> Status:
        return self.recovery_manager.get_status()

    def terminate(self, new_status: Status) -> None:
        if self.status == Status.RUNNING:
            self.recovery_manager.cancel()
