from dataclasses import dataclass


@dataclass
class NoopSegmentationConfig:
    type: str = "NoopSegmentation"
