from dataclasses import asdict, is_dataclass


def dataclass_serialize(data) -> dict:
    if not is_dataclass(data):
        raise TypeError(f"data must be a dataclass, got {str(data)[0:100]}")
    return asdict(data)
