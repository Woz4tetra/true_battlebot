from py_trees.behaviour import Behaviour
from py_trees.behaviours import Running
from py_trees.common import ParallelPolicy
from py_trees.composites import Parallel, Selector, Sequence
from py_trees.decorators import FailureIsRunning, Repeat, SuccessIsFailure, Timeout

from bw_behaviors.behaviors.exe_path import ExePath
from bw_behaviors.behaviors.get_path import GetPath
from bw_behaviors.behaviors.recovery_behavior import RecoveryBehavior
from bw_behaviors.container import Container


def make_supplied_goal_behavior(name_prefix: str, select_goal_behavior: Behaviour, container: Container) -> Behaviour:
    # Repeatedly get path to goal every replan_interval
    periodic_get_path = Repeat(
        f"{name_prefix}_get_path_repeat",
        Sequence(
            f"{name_prefix}_periodic_get_path",
            memory=True,
            children=[
                GetPath(container, clear_goal_after_send=False),
                Timeout(
                    f"{name_prefix}_get_path_delay",
                    Running(f"{name_prefix}_get_path_running"),
                    container.replan_interval,
                ),
            ],
        ),
        -1,
    )

    exe_selector = Selector(
        f"{name_prefix}_exe_selector",
        memory=True,
        children=[
            ExePath(container, path_ready_on_start=False),
            SuccessIsFailure("recovery_success_is_failure", RecoveryBehavior(container)),
        ],
    )

    # Execute path to goal
    exe_sequence = Sequence(
        f"{name_prefix}_exe_sequence",
        memory=True,
        children=[
            RecoveryBehavior(container, "reset_costmap"),
            GetPath(container, clear_goal_after_send=False),
            Parallel(
                f"{name_prefix}_parallel",
                policy=ParallelPolicy.SuccessOnSelected([exe_selector], synchronise=False),
                children=[
                    periodic_get_path,
                    exe_selector,
                ],
            ),
        ],
    )
    return Sequence(
        f"{name_prefix}_sequence",
        memory=True,
        children=[
            select_goal_behavior,
            FailureIsRunning(f"{name_prefix}_keep_running", exe_sequence),
        ],
    )
