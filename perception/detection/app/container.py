from typing import Any, TypeVar

T = TypeVar("T")


class Container:
    def __init__(self) -> None:
        self._object_registry: dict[type | str, Any] = {}

    def register(self, obj: object, key: type[T] | str | None = None) -> None:
        register_key = obj.__class__ if key is None else key
        self._object_registry[register_key] = obj

    def resolve(self, cls: type[T]) -> T:
        return self._object_registry[cls]

    def resolve_by_name(self, cls: str) -> Any:
        return self._object_registry[cls]
