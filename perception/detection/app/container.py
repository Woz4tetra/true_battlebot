from typing import Any, TypeVar, cast

T = TypeVar("T")


class Container:
    def __init__(self) -> None:
        self._object_registry: dict[type | str, Any] = {}

    def is_registered(self, cls: type[T]) -> bool:
        return cls in self._object_registry

    def register(self, obj: object, key: type[T] | str | None = None) -> None:
        register_key = obj.__class__ if key is None else key
        self._object_registry[register_key] = obj

    def resolve(self, cls: type[T]) -> T:
        return cast(T, self._object_registry[cls])

    def resolve_by_name(self, name: str) -> Any:
        return self._object_registry[name]
