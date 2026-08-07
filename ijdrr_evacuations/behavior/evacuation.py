"""Event-driven evacuation lifecycle service."""

from __future__ import annotations

from ijdrr_evacuations.agents import AgentStore, EvacuationStatus
from ijdrr_evacuations.environment import EnvironmentState


class EvacuationManager:
    __slots__ = ()

    def handle_departure(
        self,
        *,
        vehicle_id: str,
        now_s: float,
        agent_store: AgentStore,
    ) -> None:
        index = agent_store.index_of(vehicle_id)
        agent_store.evacuations[index].start(now_s)

    def handle_arrival(
        self,
        *,
        vehicle_id: str,
        now_s: float,
        agent_store: AgentStore,
        environment: EnvironmentState,
    ) -> bool:
        index = agent_store.index_of(vehicle_id)
        evacuation = agent_store.evacuations[index]
        evacuation.complete(now_s)
        shelter = environment.shelters.get(evacuation.destination_id)
        return shelter.register_arrival() if shelter is not None else False

    def handle_abandonment(
        self,
        *,
        vehicle_id: str,
        now_s: float,
        agent_store: AgentStore,
    ) -> None:
        index = agent_store.index_of(vehicle_id)
        agent_store.evacuations[index].abandon(now_s)

    def mark_timeout_failures(
        self,
        *,
        now_s: float,
        max_evacuation_time_s: float,
        agent_store: AgentStore,
    ) -> tuple[str, ...]:
        failed: list[str] = []
        for index, evacuation in agent_store.evacuations.items():
            if (
                evacuation.status is EvacuationStatus.ACTIVE
                and evacuation.started_at_s is not None
                and now_s - evacuation.started_at_s >= max_evacuation_time_s
            ):
                evacuation.fail(now_s, "evacuation_timeout")
                failed.append(agent_store.vehicle_id_of(index))
        return tuple(failed)
