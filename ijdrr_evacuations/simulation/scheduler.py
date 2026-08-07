"""Drift-resistant multi-rate task scheduler."""

from __future__ import annotations

from dataclasses import dataclass


@dataclass(slots=True)
class _Schedule:
    interval_s: float
    next_due_s: float


class MultiRateScheduler:
    __slots__ = ("_schedules", "epsilon")

    def __init__(self, *, epsilon: float = 1e-9) -> None:
        self._schedules: dict[str, _Schedule] = {}
        self.epsilon = epsilon

    def register(
        self,
        name: str,
        *,
        interval_s: float,
        first_due_s: float = 0.0,
    ) -> None:
        if interval_s <= 0:
            raise ValueError("interval_s must be positive")
        if name in self._schedules:
            raise ValueError(f"schedule already registered: {name}")
        self._schedules[name] = _Schedule(interval_s, first_due_s)

    def is_due(self, name: str, now_s: float) -> bool:
        schedule = self._schedules[name]
        if now_s + self.epsilon < schedule.next_due_s:
            return False
        while schedule.next_due_s <= now_s + self.epsilon:
            schedule.next_due_s += schedule.interval_s
        return True

    def next_due(self, name: str) -> float:
        return self._schedules[name].next_due_s
