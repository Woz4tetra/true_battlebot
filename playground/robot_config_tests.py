from bw_tools.configs.robot_config import RobotConfig, RobotFleetConfig
from bw_tools.dataclass_serialization import dataclass_deserialize

config = {
    "robots": [
        {"name": "mini_bot", "up_id": 41, "down_id": 76},
        {"name": "main_bot", "up_id": 42, "down_id": 99},
        {"name": "opponent_1"},
    ]
}

robots = dataclass_deserialize(RobotFleetConfig, config)
print(robots)
