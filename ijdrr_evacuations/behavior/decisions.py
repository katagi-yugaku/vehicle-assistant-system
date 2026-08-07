"""Immutable decision and action result values."""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum


class BehaviorAction(str, Enum):
    KEEP_ROUTE = "keep_route"
    REROUTE = "reroute"
    AVOID_CONGESTION = "avoid_congestion"
    WRONG_WAY = "wrong_way"
    ABANDON = "abandon"
    FOLLOW = "follow"
    CHANGE_SHELTER = "change_shelter"
    COMPLETE = "complete"
    FAIL = "fail"


@dataclass(frozen=True, slots=True)
class RouteDecision:
    vehicle_id: str
    route_edges: tuple[str, ...]
    destination_id: str
    expected_travel_time_s: float | None
    reason: str


@dataclass(frozen=True, slots=True)
class BehaviorDecision:
    vehicle_id: str
    action: BehaviorAction
    reason: str
    decided_at_s: float
    route_decision: RouteDecision | None = None


@dataclass(frozen=True, slots=True)
class ActionResult:
    vehicle_id: str
    action: BehaviorAction
    success: bool
    executed_at_s: float
    error: str | None = None
