"""Stable output schemas for journal experiments."""

from __future__ import annotations

from dataclasses import asdict, dataclass
from typing import Any


@dataclass(frozen=True, slots=True)
class VehicleSummary:
    run_id: str
    vehicle_id: str
    destination_id: str
    status: str
    started_at_s: float | None
    completed_at_s: float | None
    distance_m: float
    route_change_count: int
    wrong_way_count: int
    abandonment_count: int
    sent_message_count: int
    received_message_count: int
    contact_count: int

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)


@dataclass(frozen=True, slots=True)
class RunManifest:
    schema_version: str
    run_id: str
    scenario_id: str
    master_seed: int
    sumo_seed: int
    config_sha256: str
    git_commit: str | None
    git_dirty: bool | None
    sumo_version: str
    python_version: str
    started_at_iso: str
    stream_seeds: tuple[tuple[str, int], ...]

    def to_dict(self) -> dict[str, Any]:
        data = asdict(self)
        data["stream_seeds"] = dict(self.stream_seeds)
        return data
