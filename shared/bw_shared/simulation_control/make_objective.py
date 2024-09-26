import json

from bw_interfaces.msg import SimulationConfig


def make_objective(objective_name: str, objective_data: dict) -> SimulationConfig:
    return SimulationConfig(name=objective_name, json_data=json.dumps(objective_data))
