"""Shared behaviour orchestration over lightweight state."""

from __future__ import annotations

from collections.abc import Iterable

from ijdrr_evacuations.agents import AgentStore
from ijdrr_evacuations.communication import MessageRegistry
from ijdrr_evacuations.environment import EnvironmentState

from .decisions import BehaviorDecision
from .policies import BehaviorPolicy, ThresholdBehaviorPolicy


class BehaviorManager:
    __slots__ = ("policy", "registry")

    def __init__(
        self,
        *,
        registry: MessageRegistry,
        policy: BehaviorPolicy | None = None,
    ) -> None:
        self.registry = registry
        self.policy = policy or ThresholdBehaviorPolicy()

    def evaluate(
        self,
        *,
        now_s: float,
        agent_store: AgentStore,
        environment: EnvironmentState,
        candidate_indices: Iterable[int] | None = None,
    ) -> tuple[BehaviorDecision, ...]:
        indices = (
            sorted(agent_store.indices())
            if candidate_indices is None
            else sorted(set(candidate_indices))
        )
        decisions: list[BehaviorDecision] = []
        for index in indices:
            vehicle = agent_store.vehicles[index]
            driver = agent_store.drivers[index]
            evacuation = agent_store.evacuations[index]
            node = agent_store.dtn_nodes.get(index)
            information = ()
            if node is not None:
                information = tuple(
                    message
                    for message_id in sorted(node.buffer_hops)
                    if (message := self.registry.get_live(message_id, now_s))
                    is not None
                )
            decision = self.policy.decide(
                now_s=now_s,
                vehicle=vehicle,
                driver=driver,
                evacuation=evacuation,
                dtn=node,
                information=information,
                environment=environment,
            )
            driver.last_decision_s = now_s
            driver.last_action = decision.action.value
            decisions.append(decision)
        return tuple(decisions)
