"""Global message ownership and expiration by absolute simulation time."""

from __future__ import annotations

from heapq import heappop, heappush
from typing import Mapping

from .message import Message, MessageHeader, MessageKind, MessagePayloadValue


class MessageRegistry:
    __slots__ = ("_messages", "_expiry_heap", "_next_id")

    def __init__(self) -> None:
        self._messages: dict[int, Message] = {}
        self._expiry_heap: list[tuple[float, int]] = []
        self._next_id = 0

    def create(
        self,
        *,
        kind: MessageKind,
        source_vehicle_id: str,
        subject_id: str,
        now_s: float,
        ttl_s: float,
        payload: Mapping[str, MessagePayloadValue] | None = None,
        priority: int = 0,
        reliability: float = 1.0,
        hop_limit: int = 16,
    ) -> Message:
        if ttl_s <= 0:
            raise ValueError("ttl_s must be positive")

        message_id = self._next_id
        self._next_id += 1
        header = MessageHeader(
            message_id=message_id,
            kind=kind,
            source_vehicle_id=source_vehicle_id,
            subject_id=subject_id,
            created_at_s=now_s,
            expires_at_s=now_s + ttl_s,
            priority=priority,
            reliability=reliability,
            hop_limit=hop_limit,
        )
        message = Message.build(header=header, payload=payload)
        self._messages[message_id] = message
        heappush(self._expiry_heap, (header.expires_at_s, message_id))
        return message

    def get(self, message_id: int) -> Message | None:
        return self._messages.get(message_id)

    def get_live(self, message_id: int, now_s: float) -> Message | None:
        message = self._messages.get(message_id)
        if message is None or not message.is_alive(now_s):
            return None
        return message

    def purge_expired(self, now_s: float) -> tuple[int, ...]:
        removed: list[int] = []
        while self._expiry_heap and self._expiry_heap[0][0] <= now_s:
            expires_at_s, message_id = heappop(self._expiry_heap)
            message = self._messages.get(message_id)
            if (
                message is not None
                and message.header.expires_at_s == expires_at_s
            ):
                del self._messages[message_id]
                removed.append(message_id)
        return tuple(removed)

    def __contains__(self, message_id: object) -> bool:
        return message_id in self._messages

    def __len__(self) -> int:
        return len(self._messages)
