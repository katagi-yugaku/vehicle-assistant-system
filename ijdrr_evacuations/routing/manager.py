"""Shared routing manager with no direct TraCI import."""

from __future__ import annotations

from typing import Protocol

from ijdrr_evacuations.behavior.decisions import (
    ActionResult,
    BehaviorAction,
    RouteDecision,
)

from .route_cache import RouteCache, RouteCacheKey, RouteValue


class RoutingBackend(Protocol):
    def find_route(
        self,
        from_edge_id: str,
        to_edge_id: str,
    ) -> tuple[tuple[str, ...], float | None]: ...

    def set_vehicle_route(
        self,
        vehicle_id: str,
        route_edges: tuple[str, ...],
    ) -> None: ...


class RoutingManager:
    __slots__ = ("backend", "cache", "time_bucket_s")

    def __init__(
        self,
        *,
        backend: RoutingBackend,
        cache: RouteCache | None = None,
        time_bucket_s: float = 30.0,
    ) -> None:
        if time_bucket_s <= 0:
            raise ValueError("time_bucket_s must be positive")
        self.backend = backend
        self.cache = cache or RouteCache()
        self.time_bucket_s = time_bucket_s

    def decide(
        self,
        *,
        vehicle_id: str,
        from_edge_id: str,
        to_edge_id: str,
        destination_id: str,
        now_s: float,
        road_version: int,
        reason: str,
    ) -> RouteDecision:
        key = RouteCacheKey(
            from_edge_id=from_edge_id,
            to_edge_id=to_edge_id,
            time_bucket=int(now_s // self.time_bucket_s),
            road_version=road_version,
        )
        value = self.cache.get(key)
        if value is None:
            edges, travel_time_s = self.backend.find_route(from_edge_id, to_edge_id)
            value = RouteValue(edges=edges, travel_time_s=travel_time_s)
            self.cache.put(key, value)
        return RouteDecision(
            vehicle_id=vehicle_id,
            route_edges=value.edges,
            destination_id=destination_id,
            expected_travel_time_s=value.travel_time_s,
            reason=reason,
        )

    def execute(self, decision: RouteDecision, now_s: float) -> ActionResult:
        if not decision.route_edges:
            return ActionResult(
                decision.vehicle_id,
                BehaviorAction.REROUTE,
                False,
                now_s,
                "empty_route",
            )
        try:
            self.backend.set_vehicle_route(
                decision.vehicle_id,
                decision.route_edges,
            )
        except Exception as exc:
            return ActionResult(
                decision.vehicle_id,
                BehaviorAction.REROUTE,
                False,
                now_s,
                str(exc),
            )
        return ActionResult(
            decision.vehicle_id,
            BehaviorAction.REROUTE,
            True,
            now_s,
        )
