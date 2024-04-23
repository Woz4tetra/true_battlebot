from dataclasses import dataclass


@dataclass
class RosBridgeConfig:
    host: str = "0.0.0.0"
    port: int = 9090
