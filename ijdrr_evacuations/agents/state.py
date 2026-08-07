"""Per-vehicle state without SUMO or service-layer dependencies."""

from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum
from math import inf


class EvacuationStatus(str, Enum):
    """Lifecycle of one vehicle evacuee."""

    WAITING = "waiting"
    ACTIVE = "active"
    COMPLETED = "completed"
    ABANDONED = "abandoned"
    FAILED = "failed"


@dataclass(slots=True)
class VehicleState:
    """Latest mobility snapshot copied from SUMO.

    SUMO remains authoritative for the fields in this class.  ``last_update_s``
    makes the age of the cached snapshot explicit.
    """

    vehicle_id: str
    x: float = 0.0
    y: float = 0.0
    edge_id: str = ""
    lane_id: str = ""
    speed_mps: float = 0.0
    route: tuple[str, ...] = ()
    distance_m: float = 0.0
    stopped_since_s: float | None = None
    last_update_s: float = -inf

    @property
    def position(self) -> tuple[float, float]:
        return self.x, self.y

    def update_mobility(
        self,
        *,
        now_s: float,
        position: tuple[float, float],
        edge_id: str,
        lane_id: str,
        speed_mps: float,
        distance_m: float | None = None,
    ) -> None:
        self.x, self.y = position
        self.edge_id = edge_id
        self.lane_id = lane_id
        self.speed_mps = speed_mps
        if distance_m is not None:
            self.distance_m = distance_m
        self.last_update_s = now_s

        if speed_mps <= 0.1:
            if self.stopped_since_s is None:
                self.stopped_since_s = now_s
        else:
            self.stopped_since_s = None


@dataclass(slots=True)
class DriverState:
    """Stable traits and the small mutable decision state of a driver."""

    normalcy_bias: float = 0.0
    conformity: float = 0.0
    risk_perception: float = 0.5
    information_trust: float = 0.5
    reroute_threshold_s: float = 60.0
    wrong_way_threshold: float = 1.0
    abandonment_threshold_s: float = 300.0
    last_decision_s: float = -inf
    last_action: str = "keep_route"

    def __post_init__(self) -> None:
        for name in (
            "normalcy_bias",
            "conformity",
            "risk_perception",
            "information_trust",
        ):
            value = getattr(self, name)
            if not 0.0 <= value <= 1.0:
                raise ValueError(f"{name} must be between 0 and 1: {value}")


@dataclass(slots=True)
class EvacuationState:
    """Evacuation lifecycle and compact online counters."""

    destination_id: str
    status: EvacuationStatus = EvacuationStatus.WAITING
    started_at_s: float | None = None
    completed_at_s: float | None = None
    failed_at_s: float | None = None
    route_change_count: int = 0
    wrong_way_count: int = 0
    abandonment_count: int = 0
    failure_reason: str | None = None

    def start(self, now_s: float) -> None:
        if self.status is EvacuationStatus.WAITING:
            self.status = EvacuationStatus.ACTIVE
            self.started_at_s = now_s

    def complete(self, now_s: float) -> None:
        if self.status not in {
            EvacuationStatus.COMPLETED,
            EvacuationStatus.ABANDONED,
            EvacuationStatus.FAILED,
        }:
            self.status = EvacuationStatus.COMPLETED
            self.completed_at_s = now_s

    def fail(self, now_s: float, reason: str) -> None:
        if self.status is not EvacuationStatus.COMPLETED:
            self.status = EvacuationStatus.FAILED
            self.failed_at_s = now_s
            self.failure_reason = reason

    def abandon(self, now_s: float) -> None:
        if self.status is EvacuationStatus.ACTIVE:
            self.status = EvacuationStatus.ABANDONED
            self.completed_at_s = now_s
            self.abandonment_count += 1


@dataclass(slots=True)
class DTNNodeState:
    """DTN copy state held only by communication-capable vehicles."""

    enabled: bool
    capacity_messages: int
    buffer_hops: dict[int, int] = field(default_factory=dict)
    seen_message_ids: set[int] = field(default_factory=set)
    last_contact_s: float | None = None
    sent_count: int = 0
    received_count: int = 0
    dropped_count: int = 0
    contact_count: int = 0

    def __post_init__(self) -> None:
        if self.capacity_messages < 0:
            raise ValueError("capacity_messages must be non-negative")


@dataclass(frozen=True, slots=True)
class VehicleAgent:
    """Read-only composition view for debugging and model explanation.

    The simulation hot path operates on :class:`AgentStore` directly, so this
    facade can be created only when a composed view is useful.
    """

    index: int
    vehicle: VehicleState
    driver: DriverState
    evacuation: EvacuationState
    dtn: DTNNodeState | None
