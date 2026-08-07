"""Contact-driven, store-carry-forward communication orchestration."""

from __future__ import annotations

from dataclasses import dataclass
from random import Random
from typing import Mapping

from ijdrr_evacuations.agents import DTNNodeState, VehicleState

from .buffer import MessageBuffer
from .contacts import Contact, ContactDetector, ContactHistory
from .forwarding import EpidemicPolicy, ForwardingPolicy
from .registry import MessageRegistry


@dataclass(frozen=True, slots=True)
class TransferRecord:
    time_s: float
    message_id: int
    sender_index: int
    receiver_index: int
    hop_count: int


@dataclass(frozen=True, slots=True)
class CommunicationTickResult:
    contacts: tuple[Contact, ...]
    started_contacts: tuple[Contact, ...]
    ended_pairs: tuple[tuple[int, int], ...]
    transfers: tuple[TransferRecord, ...]
    expired_message_ids: tuple[int, ...]


class CommunicationManager:
    __slots__ = (
        "registry",
        "communication_range_m",
        "detector",
        "forwarding_policy",
        "buffer",
        "contact_history",
        "rng",
    )

    def __init__(
        self,
        *,
        registry: MessageRegistry,
        communication_range_m: float,
        detector: ContactDetector | None = None,
        forwarding_policy: ForwardingPolicy | None = None,
        rng: Random | None = None,
    ) -> None:
        if communication_range_m <= 0:
            raise ValueError("communication_range_m must be positive")
        self.registry = registry
        self.communication_range_m = communication_range_m
        self.detector = detector or ContactDetector()
        self.forwarding_policy = forwarding_policy or EpidemicPolicy()
        self.buffer = MessageBuffer(registry)
        self.contact_history = ContactHistory()
        self.rng = rng or Random(0)

    def tick(
        self,
        *,
        now_s: float,
        vehicles: Mapping[int, VehicleState],
        dtn_nodes: Mapping[int, DTNNodeState],
    ) -> CommunicationTickResult:
        contacts = self.detector.detect(
            vehicles=vehicles,
            dtn_nodes=dtn_nodes,
            communication_range_m=self.communication_range_m,
        )
        contact_update = self.contact_history.update(contacts)
        for contact in contact_update.started:
            dtn_nodes[contact.left_index].contact_count += 1
            dtn_nodes[contact.right_index].contact_count += 1

        transfers: list[TransferRecord] = []
        for contact in contacts:
            left = dtn_nodes[contact.left_index]
            right = dtn_nodes[contact.right_index]
            left.last_contact_s = now_s
            right.last_contact_s = now_s
            transfers.extend(
                self._exchange(
                    now_s=now_s,
                    sender_index=contact.left_index,
                    receiver_index=contact.right_index,
                    sender=left,
                    receiver=right,
                )
            )
            transfers.extend(
                self._exchange(
                    now_s=now_s,
                    sender_index=contact.right_index,
                    receiver_index=contact.left_index,
                    sender=right,
                    receiver=left,
                )
            )

        expired_ids = self.registry.purge_expired(now_s)
        return CommunicationTickResult(
            contacts=contacts,
            started_contacts=contact_update.started,
            ended_pairs=contact_update.ended_pairs,
            transfers=tuple(transfers),
            expired_message_ids=expired_ids,
        )

    def _exchange(
        self,
        *,
        now_s: float,
        sender_index: int,
        receiver_index: int,
        sender: DTNNodeState,
        receiver: DTNNodeState,
    ) -> list[TransferRecord]:
        records: list[TransferRecord] = []
        selected = self.forwarding_policy.select_message_ids(
            sender=sender,
            receiver=receiver,
            registry=self.registry,
            now_s=now_s,
        )
        for message_id in selected:
            message = self.registry.get_live(message_id, now_s)
            if message is None:
                continue
            if self.rng.random() > message.header.reliability:
                continue
            hop_count = sender.buffer_hops[message_id] + 1
            result = self.buffer.store(
                node=receiver,
                message=message,
                hop_count=hop_count,
                now_s=now_s,
            )
            if result.accepted:
                sender.sent_count += 1
                records.append(
                    TransferRecord(
                        time_s=now_s,
                        message_id=message_id,
                        sender_index=sender_index,
                        receiver_index=receiver_index,
                        hop_count=hop_count,
                    )
                )
        return records
