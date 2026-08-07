"""Composition root for one simulation run."""

from __future__ import annotations

from dataclasses import dataclass

from ijdrr_evacuations.agents import AgentStore
from ijdrr_evacuations.behavior import BehaviorManager, EvacuationManager
from ijdrr_evacuations.communication import CommunicationManager, MessageRegistry
from ijdrr_evacuations.environment import EnvironmentState
from ijdrr_evacuations.events import EventQueue
from ijdrr_evacuations.metrics import MetricsCollector
from ijdrr_evacuations.routing import RoutingManager

from .scheduler import MultiRateScheduler
from .sumo_adapter import SumoAdapter


@dataclass(slots=True)
class SimulationContext:
    sumo: SumoAdapter
    agent_store: AgentStore
    environment: EnvironmentState
    message_registry: MessageRegistry
    communication_manager: CommunicationManager
    behavior_manager: BehaviorManager
    evacuation_manager: EvacuationManager
    metrics: MetricsCollector
    event_queue: EventQueue
    scheduler: MultiRateScheduler
    routing_manager: RoutingManager | None = None
    now_s: float = 0.0
