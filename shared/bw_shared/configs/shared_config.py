from __future__ import annotations

import os
from dataclasses import dataclass

import toml

from bw_shared.configs.maps import MapConfig, Maps
from bw_shared.configs.robots import RobotFleetConfig
from bw_shared.enums.field_type import FieldType
from bw_shared.environment import get_project_dir


@dataclass
class SharedConfig:
    maps: Maps
    robots: RobotFleetConfig

    @classmethod
    def from_files(cls, map_path: str = "", robot_config: str = "") -> SharedConfig:
        if not map_path:
            map_path = os.path.join(get_project_dir(), "shared", "configs", "maps.toml")
        if not robot_config:
            robot_config = os.path.join(get_project_dir(), "shared", "configs", "robots.toml")
        return cls(
            maps=Maps.from_dict(cls._read_toml(map_path)),
            robots=RobotFleetConfig.from_dict(cls._read_toml(robot_config)),
        )

    @classmethod
    def from_dict(cls, data: dict) -> SharedConfig:
        return cls(
            maps=Maps.from_dict(data["maps"]),
            robots=RobotFleetConfig.from_dict(data["robots"]),
        )

    def to_dict(self) -> dict:
        return dict(maps=self.maps.to_dict(), robots=self.robots.to_dict())

    def get_map(self, key: FieldType) -> MapConfig:
        return self.maps.get(key)

    @classmethod
    def _read_toml(cls, path: str) -> dict:
        with open(path, "r") as file:
            return toml.load(file)
