"""Immutable DTN message definitions shared by all vehicle copies."""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
from typing import Mapping, TypeAlias


MessagePayloadValue: TypeAlias = str | int | float | bool


class MessageKind(str, Enum):
    TSUNAMI = "tsunami"
    CONGESTION = "congestion"
    ROAD_CLOSED = "road_closed"
    SHELTER_FULL = "shelter_full"


@dataclass(frozen=True, slots=True)
class MessageHeader:
    """Fields used frequently during forwarding and expiration checks."""

    message_id: int
    kind: MessageKind
    source_vehicle_id: str
    subject_id: str
    created_at_s: float
    expires_at_s: float
    priority: int = 0
    reliability: float = 1.0
    hop_limit: int = 16

    def __post_init__(self) -> None:
        if self.expires_at_s <= self.created_at_s:
            raise ValueError("expires_at_s must be after created_at_s")
        if not 0.0 <= self.reliability <= 1.0:
            raise ValueError("reliability must be between 0 and 1")
        if self.hop_limit < 0:
            raise ValueError("hop_limit must be non-negative")


@dataclass(frozen=True, slots=True)
class Message:
    """One registry-owned message body.

    Per-copy hop counts are intentionally stored in ``DTNNodeState`` rather
    than in this shared object.
    """

    header: MessageHeader
    payload: tuple[tuple[str, MessagePayloadValue], ...] = ()

    @property
    def message_id(self) -> int:
        return self.header.message_id

    def is_alive(self, now_s: float) -> bool:
        return now_s < self.header.expires_at_s

    def payload_dict(self) -> dict[str, MessagePayloadValue]:
        return dict(self.payload)

    @classmethod
    def build(
        cls,
        *,
        header: MessageHeader,
        payload: Mapping[str, MessagePayloadValue] | None = None,
    ) -> "Message":
        return cls(header=header, payload=tuple(sorted((payload or {}).items())))
