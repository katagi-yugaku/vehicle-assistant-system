"""Explicitly ordered, multi-rate simulation step orchestration."""

from __future__ import annotations

from dataclasses import dataclass

from ijdrr_evacuations.behavior import BehaviorDecision
from ijdrr_evacuations.communication import CommunicationTickResult
from ijdrr_evacuations.events import EventType, SimulationEvent

from .context import SimulationContext


@dataclass(frozen=True, slots=True)
class StepResult:
    now_s: float
    departed_vehicle_ids: tuple[str, ...]
    arrived_vehicle_ids: tuple[str, ...]
    communication: CommunicationTickResult | None
    decisions: tuple[BehaviorDecision, ...]


class SimulationEngine:
    __slots__ = ("context",)

    def __init__(self, context: SimulationContext) -> None:
        self.context = context

    def step(self) -> StepResult:
        context = self.context
        context.sumo.simulation_step()
        context.now_s = context.sumo.time_s()
        now_s = context.now_s

        departed = context.sumo.departed_vehicle_ids()
        arrived = context.sumo.arrived_vehicle_ids()

        for vehicle_id in departed:
            if context.agent_store.get_index(vehicle_id) is None:
                continue
            context.sumo.subscribe_vehicle(vehicle_id)
            context.evacuation_manager.handle_departure(
                vehicle_id=vehicle_id,
                now_s=now_s,
                agent_store=context.agent_store,
            )
            self._record(
                SimulationEvent.create(
                    time_s=now_s,
                    event_type=EventType.VEHICLE_DEPARTED,
                    vehicle_id=vehicle_id,
                )
            )

        for vehicle_id in context.sumo.vehicle_ids():
            index = context.agent_store.get_index(vehicle_id)
            if index is None:
                continue
            snapshot = context.sumo.vehicle_snapshot(vehicle_id)
            context.agent_store.vehicles[index].update_mobility(
                now_s=now_s,
                position=snapshot.position,
                edge_id=snapshot.edge_id,
                lane_id=snapshot.lane_id,
                speed_mps=snapshot.speed_mps,
                distance_m=snapshot.distance_m,
            )

        for vehicle_id in arrived:
            if context.agent_store.get_index(vehicle_id) is None:
                continue
            context.evacuation_manager.handle_arrival(
                vehicle_id=vehicle_id,
                now_s=now_s,
                agent_store=context.agent_store,
                environment=context.environment,
            )
            self._record(
                SimulationEvent.create(
                    time_s=now_s,
                    event_type=EventType.VEHICLE_ARRIVED,
                    vehicle_id=vehicle_id,
                )
            )

        for scheduled_event in context.event_queue.pop_due(now_s):
            self._record(scheduled_event)

        communication = None
        if context.scheduler.is_due("communication", now_s):
            communication = context.communication_manager.tick(
                now_s=now_s,
                vehicles=context.agent_store.vehicles,
                dtn_nodes=context.agent_store.dtn_nodes,
            )
            self._record_communication(communication)

        decisions: tuple[BehaviorDecision, ...] = ()
        if context.scheduler.is_due("behavior", now_s):
            active_indices = tuple(
                context.agent_store.index_of(vehicle_id)
                for vehicle_id in context.sumo.vehicle_ids()
                if context.agent_store.get_index(vehicle_id) is not None
            )
            decisions = context.behavior_manager.evaluate(
                now_s=now_s,
                agent_store=context.agent_store,
                environment=context.environment,
                candidate_indices=active_indices,
            )

        return StepResult(now_s, departed, arrived, communication, decisions)

    def _record_communication(self, result: CommunicationTickResult) -> None:
        for contact in result.started_contacts:
            self._record(
                SimulationEvent.create(
                    time_s=self.context.now_s,
                    event_type=EventType.CONTACT_STARTED,
                    subject_id=(
                        f"{contact.left_index}:{contact.right_index}"
                    ),
                    data={"distance_m": contact.distance_m},
                )
            )
        for left_index, right_index in result.ended_pairs:
            self._record(
                SimulationEvent.create(
                    time_s=self.context.now_s,
                    event_type=EventType.CONTACT_ENDED,
                    subject_id=f"{left_index}:{right_index}",
                )
            )
        for transfer in result.transfers:
            self._record(
                SimulationEvent.create(
                    time_s=transfer.time_s,
                    event_type=EventType.MESSAGE_TRANSFERRED,
                    subject_id=str(transfer.message_id),
                    data={
                        "sender_index": transfer.sender_index,
                        "receiver_index": transfer.receiver_index,
                        "hop_count": transfer.hop_count,
                    },
                )
            )

    def _record(self, event: SimulationEvent) -> None:
        self.context.metrics.record(event)
