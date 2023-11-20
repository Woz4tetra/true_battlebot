from bw_tools.configs.robot_config import RobotFleetConfig

config = {
    "robots": [
        {"name": "mini_bot", "up_id": 41, "down_id": 76},
        {"name": "main_bot", "up_id": 42, "down_id": 99},
        {"name": "opponent_1"},
    ]
}

robots = RobotFleetConfig.from_dict(config)
print(robots)
