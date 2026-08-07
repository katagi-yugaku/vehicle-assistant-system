"""Small online counters plus event and vehicle-summary construction."""

from __future__ import annotations

from collections import Counter

from ijdrr_evacuations.agents import AgentStore
from ijdrr_evacuations.events import EventType, SimulationEvent

from .schemas import VehicleSummary


class MetricsCollector:
    __slots__ = ("event_counts", "events", "keep_events")

    def __init__(self, *, keep_events: bool = False) -> None:
        self.event_counts: Counter[EventType] = Counter()
        self.events: list[SimulationEvent] = []
        self.keep_events = keep_events

    def record(self, event: SimulationEvent) -> None:
        self.event_counts[event.event_type] += 1
        if self.keep_events:
            self.events.append(event)

    def record_many(self, events: tuple[SimulationEvent, ...]) -> None:
        for event in events:
            self.record(event)

    def vehicle_summaries(
        self,
        *,
        run_id: str,
        agent_store: AgentStore,
    ) -> tuple[VehicleSummary, ...]:
        rows: list[VehicleSummary] = []
        for index in sorted(agent_store.indices()):
            vehicle = agent_store.vehicles[index]
            evacuation = agent_store.evacuations[index]
            node = agent_store.dtn_nodes.get(index)
            rows.append(
                VehicleSummary(
                    run_id=run_id,
                    vehicle_id=vehicle.vehicle_id,
                    destination_id=evacuation.destination_id,
                    status=evacuation.status.value,
                    started_at_s=evacuation.started_at_s,
                    completed_at_s=evacuation.completed_at_s,
                    distance_m=vehicle.distance_m,
                    route_change_count=evacuation.route_change_count,
                    wrong_way_count=evacuation.wrong_way_count,
                    abandonment_count=evacuation.abandonment_count,
                    sent_message_count=node.sent_count if node else 0,
                    received_message_count=node.received_count if node else 0,
                    contact_count=node.contact_count if node else 0,
                )
            )
        return tuple(rows)
