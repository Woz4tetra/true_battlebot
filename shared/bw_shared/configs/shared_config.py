from __future__ import annotations

import os
from dataclasses import dataclass

import toml

from bw_shared.configs.label_config import LabelConfig, LabelsConfig
from bw_shared.configs.maps_config import MapConfig, MapsConfig
from bw_shared.configs.robot_fleet_config import RobotFleetConfig
from bw_shared.enums.field_type import FieldType
from bw_shared.enums.label import Label
from bw_shared.environment import get_project_dir


@dataclass
class SharedConfig:
    maps: MapsConfig
    robots: RobotFleetConfig
    labels: LabelsConfig

    @classmethod
    def from_files(cls, map_path: str = "", robot_config: str = "", label_config: str = "") -> SharedConfig:
        if not map_path:
            map_path = os.path.join(get_project_dir(), "shared", "configs", "maps.toml")
        if not robot_config:
            robot_config = os.path.join(get_project_dir(), "shared", "configs", "robots.toml")
        if not label_config:
            label_config = os.path.join(get_project_dir(), "shared", "configs", "labels.toml")
        return cls(
            maps=MapsConfig.from_dict(cls._read_toml(map_path)),
            robots=RobotFleetConfig.from_dict(cls._read_toml(robot_config)),
            labels=LabelsConfig.from_dict(cls._read_toml(label_config)),
        )

    @classmethod
    def from_dict(cls, data: dict) -> SharedConfig:
        return cls(
            maps=MapsConfig.from_dict(data["maps"]),
            robots=RobotFleetConfig.from_dict(data["robots"]),
            labels=LabelsConfig.from_dict(data["labels"]),
        )

    def to_dict(self) -> dict:
        return dict(maps=self.maps.to_dict(), robots=self.robots.to_dict(), labels=self.labels.to_dict())

    def get_map(self, key: FieldType) -> MapConfig:
        return self.maps.get(key)

    def get_label(self, key: Label) -> LabelConfig:
        return self.labels.get(key)

    @classmethod
    def _read_toml(cls, path: str) -> dict:
        with open(path, "r") as file:
            return toml.load(file)
