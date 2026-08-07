"""Small immutable event records used by metrics and scheduling."""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
from typing import Mapping, TypeAlias


EventValue: TypeAlias = str | int | float | bool | None


class EventType(str, Enum):
    VEHICLE_DEPARTED = "vehicle_departed"
    VEHICLE_ARRIVED = "vehicle_arrived"
    VEHICLE_STOPPED = "vehicle_stopped"
    CONGESTION_DETECTED = "congestion_detected"
    HAZARD_DETECTED = "hazard_detected"
    MESSAGE_CREATED = "message_created"
    CONTACT_STARTED = "contact_started"
    CONTACT_ENDED = "contact_ended"
    MESSAGE_TRANSFERRED = "message_transferred"
    MESSAGE_DROPPED = "message_dropped"
    ROUTE_CHANGED = "route_changed"
    WRONG_WAY_STARTED = "wrong_way_started"
    VEHICLE_ABANDONED = "vehicle_abandoned"
    SHELTER_REACHED = "shelter_reached"
    SAFE_ELEVATION_REACHED = "safe_elevation_reached"
    EVACUATION_FAILED = "evacuation_failed"


@dataclass(frozen=True, slots=True)
class SimulationEvent:
    time_s: float
    event_type: EventType
    vehicle_id: str | None = None
    subject_id: str | None = None
    data: tuple[tuple[str, EventValue], ...] = ()

    @classmethod
    def create(
        cls,
        *,
        time_s: float,
        event_type: EventType,
        vehicle_id: str | None = None,
        subject_id: str | None = None,
        data: Mapping[str, EventValue] | None = None,
    ) -> "SimulationEvent":
        return cls(
            time_s=time_s,
            event_type=event_type,
            vehicle_id=vehicle_id,
            subject_id=subject_id,
            data=tuple(sorted((data or {}).items())),
        )

    def data_dict(self) -> dict[str, EventValue]:
        return dict(self.data)
