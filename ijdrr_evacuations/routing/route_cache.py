"""Versioned time-bucket cache for repeated route requests."""

from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True, slots=True)
class RouteCacheKey:
    from_edge_id: str
    to_edge_id: str
    time_bucket: int
    road_version: int


@dataclass(frozen=True, slots=True)
class RouteValue:
    edges: tuple[str, ...]
    travel_time_s: float | None = None


class RouteCache:
    __slots__ = ("_values", "max_entries")

    def __init__(self, max_entries: int = 100_000) -> None:
        if max_entries <= 0:
            raise ValueError("max_entries must be positive")
        self._values: dict[RouteCacheKey, RouteValue] = {}
        self.max_entries = max_entries

    def get(self, key: RouteCacheKey) -> RouteValue | None:
        return self._values.get(key)

    def put(self, key: RouteCacheKey, value: RouteValue) -> None:
        if len(self._values) >= self.max_entries:
            self._values.clear()
        self._values[key] = value

    def clear(self) -> None:
        self._values.clear()

    def __len__(self) -> int:
        return len(self._values)
