from dataclasses import dataclass


@dataclass
class ActorConfig:
    name: str = "mini_bot"
    model: str = "Mini bot"
    objective: str = ""
