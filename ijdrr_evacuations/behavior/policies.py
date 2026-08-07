"""Behaviour-policy interface and a deterministic baseline policy."""

from __future__ import annotations

from typing import Protocol, Sequence

from ijdrr_evacuations.agents import (
    DriverState,
    DTNNodeState,
    EvacuationState,
    EvacuationStatus,
    VehicleState,
)
from ijdrr_evacuations.communication import Message, MessageKind
from ijdrr_evacuations.environment import EnvironmentState

from .decisions import BehaviorAction, BehaviorDecision


class BehaviorPolicy(Protocol):
    def decide(
        self,
        *,
        now_s: float,
        vehicle: VehicleState,
        driver: DriverState,
        evacuation: EvacuationState,
        dtn: DTNNodeState | None,
        information: Sequence[Message],
        environment: EnvironmentState,
    ) -> BehaviorDecision: ...


class ThresholdBehaviorPolicy:
    """Small baseline that can be replaced by the journal behaviour model."""

    def decide(
        self,
        *,
        now_s: float,
        vehicle: VehicleState,
        driver: DriverState,
        evacuation: EvacuationState,
        dtn: DTNNodeState | None,
        information: Sequence[Message],
        environment: EnvironmentState,
    ) -> BehaviorDecision:
        del dtn, environment
        if evacuation.status is not EvacuationStatus.ACTIVE:
            return BehaviorDecision(
                vehicle.vehicle_id,
                BehaviorAction.KEEP_ROUTE,
                "evacuation_not_active",
                now_s,
            )

        if (
            vehicle.stopped_since_s is not None
            and now_s - vehicle.stopped_since_s
            >= driver.abandonment_threshold_s
        ):
            return BehaviorDecision(
                vehicle.vehicle_id,
                BehaviorAction.ABANDON,
                "stopped_too_long",
                now_s,
            )

        trusted_road_warning = any(
            message.header.kind
            in {MessageKind.ROAD_CLOSED, MessageKind.CONGESTION}
            and message.header.reliability >= driver.information_trust
            for message in information
        )
        if trusted_road_warning:
            return BehaviorDecision(
                vehicle.vehicle_id,
                BehaviorAction.REROUTE,
                "trusted_dtn_road_information",
                now_s,
            )

        return BehaviorDecision(
            vehicle.vehicle_id,
            BehaviorAction.KEEP_ROUTE,
            "no_threshold_reached",
            now_s,
        )
