"""Minimal deterministic time-ordered event queue."""

from __future__ import annotations

from heapq import heappop, heappush
from itertools import count

from .types import SimulationEvent


class EventQueue:
    __slots__ = ("_heap", "_sequence")

    def __init__(self) -> None:
        self._heap: list[tuple[float, int, SimulationEvent]] = []
        self._sequence = count()

    def schedule(self, event: SimulationEvent) -> None:
        heappush(self._heap, (event.time_s, next(self._sequence), event))

    def pop_due(self, now_s: float) -> list[SimulationEvent]:
        due: list[SimulationEvent] = []
        while self._heap and self._heap[0][0] <= now_s:
            _, _, event = heappop(self._heap)
            due.append(event)
        return due

    def peek_time(self) -> float | None:
        return self._heap[0][0] if self._heap else None

    def __len__(self) -> int:
        return len(self._heap)
