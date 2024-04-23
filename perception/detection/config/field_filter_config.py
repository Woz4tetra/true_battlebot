from dataclasses import dataclass


@dataclass
class FieldFilterConfig:
    stale_image_timeout: float = 1.0
