"""Replaceable DTN forwarding policies."""

from __future__ import annotations

from typing import Protocol

from ijdrr_evacuations.agents import DTNNodeState

from .registry import MessageRegistry


class ForwardingPolicy(Protocol):
    def select_message_ids(
        self,
        *,
        sender: DTNNodeState,
        receiver: DTNNodeState,
        registry: MessageRegistry,
        now_s: float,
    ) -> tuple[int, ...]: ...


class EpidemicPolicy:
    """Forward every live message not previously seen by the receiver."""

    def select_message_ids(
        self,
        *,
        sender: DTNNodeState,
        receiver: DTNNodeState,
        registry: MessageRegistry,
        now_s: float,
    ) -> tuple[int, ...]:
        candidates = []
        for message_id, hop_count in sender.buffer_hops.items():
            if message_id in receiver.seen_message_ids:
                continue
            message = registry.get_live(message_id, now_s)
            if message is None or hop_count >= message.header.hop_limit:
                continue
            candidates.append(message)

        candidates.sort(
            key=lambda message: (
                -message.header.priority,
                message.header.expires_at_s,
                message.message_id,
            )
        )
        return tuple(message.message_id for message in candidates)
