from dataclasses import dataclass, field


@dataclass
class RosBridgeConfig:
    host: str | None = None
    port: int = 9090
    log: bool = False
    log_filters: list[str] = field(default_factory=list)
