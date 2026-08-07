"""Bounded message buffer operations over :class:`DTNNodeState`."""

from __future__ import annotations

from dataclasses import dataclass
from typing import TYPE_CHECKING

from .message import Message
from .registry import MessageRegistry

if TYPE_CHECKING:
    from ijdrr_evacuations.agents import DTNNodeState


@dataclass(frozen=True, slots=True)
class BufferInsertResult:
    accepted: bool
    duplicate: bool = False
    evicted_message_id: int | None = None
    reason: str = "accepted"


class MessageBuffer:
    """Apply exact duplicate checks and deterministic priority eviction."""

    __slots__ = ("registry",)

    def __init__(self, registry: MessageRegistry) -> None:
        self.registry = registry

    def store(
        self,
        *,
        node: "DTNNodeState",
        message: Message,
        hop_count: int,
        now_s: float,
    ) -> BufferInsertResult:
        message_id = message.message_id
        if not node.enabled:
            return BufferInsertResult(False, reason="node_disabled")
        if message_id in node.seen_message_ids:
            return BufferInsertResult(False, duplicate=True, reason="duplicate")
        if not message.is_alive(now_s):
            return BufferInsertResult(False, reason="expired")

        self.purge_stale(node=node, now_s=now_s)
        evicted: int | None = None
        if len(node.buffer_hops) >= node.capacity_messages:
            evicted = self._eviction_candidate(node=node, now_s=now_s)
            if evicted is None:
                node.dropped_count += 1
                return BufferInsertResult(False, reason="capacity_zero")

            evicted_message = self.registry.get_live(evicted, now_s)
            if (
                evicted_message is not None
                and self._retention_key(evicted_message)
                >= self._retention_key(message)
            ):
                node.dropped_count += 1
                return BufferInsertResult(False, reason="lower_priority")

            node.buffer_hops.pop(evicted, None)
            node.dropped_count += 1

        node.buffer_hops[message_id] = hop_count
        node.seen_message_ids.add(message_id)
        node.received_count += 1
        return BufferInsertResult(True, evicted_message_id=evicted)

    def seed(
        self,
        *,
        node: "DTNNodeState",
        message: Message,
        now_s: float,
    ) -> BufferInsertResult:
        """Insert a newly-created source copy with hop count zero."""

        return self.store(node=node, message=message, hop_count=0, now_s=now_s)

    def purge_stale(self, *, node: "DTNNodeState", now_s: float) -> int:
        stale_ids = [
            message_id
            for message_id in node.buffer_hops
            if self.registry.get_live(message_id, now_s) is None
        ]
        for message_id in stale_ids:
            node.buffer_hops.pop(message_id, None)
        return len(stale_ids)

    def _eviction_candidate(
        self,
        *,
        node: "DTNNodeState",
        now_s: float,
    ) -> int | None:
        candidates = (
            (self._retention_key(message), message_id)
            for message_id in node.buffer_hops
            if (message := self.registry.get_live(message_id, now_s)) is not None
        )
        return min(candidates, default=((), None))[1]

    @staticmethod
    def _retention_key(message: Message) -> tuple[int, float, int]:
        header = message.header
        return header.priority, header.expires_at_s, -header.message_id
