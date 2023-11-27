from py_trees.behaviour import Behaviour
from py_trees.composites import Selector, Sequence
from py_trees.decorators import FailureIsRunning, SuccessIsFailure

from bw_behaviors.behaviors.exe_path import ExePath
from bw_behaviors.behaviors.get_path import GetPath
from bw_behaviors.behaviors.recovery_behavior import RecoveryBehavior
from bw_behaviors.container import Container


def make_supplied_goal_behavior(name_prefix: str, select_goal_behavior: Behaviour, container: Container) -> Behaviour:
    return Sequence(
        f"{name_prefix}_sequence",
        memory=True,
        children=[
            select_goal_behavior,
            FailureIsRunning(
                f"{name_prefix}_keep_running",
                Sequence(
                    f"{name_prefix}_exe_sequence",
                    memory=True,
                    children=[
                        RecoveryBehavior(container, "reset_costmap"),
                        GetPath(container, clear_goal_after_send=False),
                        Selector(
                            f"{name_prefix}_exe_selector",
                            memory=True,
                            children=[
                                ExePath(container),
                                SuccessIsFailure("recovery_success_is_failure", RecoveryBehavior(container)),
                            ],
                        ),
                    ],
                ),
            ),
        ],
    )
