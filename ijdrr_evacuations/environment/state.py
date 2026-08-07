"""Environment state shared by all vehicle agents."""

from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum


@dataclass(slots=True)
class RoadState:
    edge_id: str
    open: bool = True
    mean_speed_mps: float = 0.0
    vehicle_count: int = 0
    congested_since_s: float | None = None
    version: int = 0
    last_update_s: float = 0.0

    def set_open(self, is_open: bool, now_s: float) -> bool:
        if self.open == is_open:
            return False
        self.open = is_open
        self.version += 1
        self.last_update_s = now_s
        return True


@dataclass(slots=True)
class ShelterState:
    shelter_id: str
    capacity: int
    edge_id: str
    x: float = 0.0
    y: float = 0.0
    arrival_count: int = 0

    def __post_init__(self) -> None:
        if self.capacity <= 0:
            raise ValueError("shelter capacity must be positive")

    @property
    def occupancy_rate(self) -> float:
        return min(1.0, self.arrival_count / self.capacity)

    @property
    def is_full(self) -> bool:
        return self.arrival_count >= self.capacity

    def register_arrival(self) -> bool:
        was_full = self.is_full
        self.arrival_count += 1
        return not was_full and self.is_full


class HazardKind(str, Enum):
    TSUNAMI = "tsunami"
    FLOODING = "flooding"
    ROAD_BLOCKAGE = "road_blockage"


@dataclass(slots=True)
class HazardState:
    hazard_id: str
    kind: HazardKind
    active: bool = False
    severity: float = 0.0
    affected_edge_ids: frozenset[str] = frozenset()
    started_at_s: float | None = None
    ended_at_s: float | None = None


@dataclass(slots=True)
class EnvironmentState:
    roads: dict[str, RoadState] = field(default_factory=dict)
    shelters: dict[str, ShelterState] = field(default_factory=dict)
    hazards: dict[str, HazardState] = field(default_factory=dict)
    road_version: int = 0

    def mark_road_changed(self) -> None:
        self.road_version += 1
