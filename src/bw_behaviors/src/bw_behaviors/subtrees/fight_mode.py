from bw_tools.typing import seconds_to_duration
from py_trees.behaviour import Behaviour
from py_trees.common import ParallelPolicy
from py_trees.composites import Parallel, Sequence

from bw_behaviors.behaviors.exe_path import ExePath
from bw_behaviors.behaviors.get_path import GetPath
from bw_behaviors.behaviors.set_goal_to_optimal_attack import SetGoalToOptimalAttack
from bw_behaviors.behaviors.wait_for_target_to_move import WaitForTargetToMove
from bw_behaviors.container import Container


def make_fight_behavior(container: Container) -> Behaviour:
    set_goal = SetGoalToOptimalAttack(container, concurrency_slot=0)
    get_path = GetPath(container)
    wait_for_target = WaitForTargetToMove(
        container, min_next_time_to_move=seconds_to_duration(1.0), moved_threshold=0.25
    )
    exe_path = ExePath(container, concurrency_slot=1, path_ready_on_start=False)
    return Parallel(
        "fight_parallel",
        policy=ParallelPolicy.SuccessOnOne(),
        children=[
            Sequence(
                "fight_sequence",
                memory=True,
                children=[set_goal, get_path, wait_for_target],
            ),
            exe_path,
        ],
    )
