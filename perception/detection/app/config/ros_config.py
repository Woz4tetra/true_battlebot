from dataclasses import dataclass, field


@dataclass
class RosConfig:
    log: bool = False
    exclude_filters: list[str] = field(default_factory=list)
    connection_timeout: float = 5.0
