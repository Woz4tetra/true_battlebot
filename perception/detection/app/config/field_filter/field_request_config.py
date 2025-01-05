from dataclasses import dataclass


@dataclass
class FieldRequestConfig:
    stale_image_timeout: float = 1.0
