from __future__ import annotations

from random import Random

from ijdrr_evacuations.agents import DTNNodeState, VehicleState
from ijdrr_evacuations.communication import (
    CommunicationManager,
    ContactDetector,
    MessageBuffer,
    MessageKind,
    MessageRegistry,
)


def test_registry_uses_absolute_expiry_time() -> None:
    registry = MessageRegistry()
    message = registry.create(
        kind=MessageKind.TSUNAMI,
        source_vehicle_id="a",
        subject_id="coast",
        now_s=10.0,
        ttl_s=5.0,
    )

    assert registry.get_live(message.message_id, 14.999) is message
    assert registry.get_live(message.message_id, 15.0) is None
    assert registry.purge_expired(15.0) == (message.message_id,)


def test_buffer_evicts_lower_priority_message_deterministically() -> None:
    registry = MessageRegistry()
    buffer = MessageBuffer(registry)
    node = DTNNodeState(True, 1)
    low = registry.create(
        kind=MessageKind.CONGESTION,
        source_vehicle_id="a",
        subject_id="edge-1",
        now_s=0.0,
        ttl_s=100.0,
        priority=1,
    )
    high = registry.create(
        kind=MessageKind.ROAD_CLOSED,
        source_vehicle_id="b",
        subject_id="edge-2",
        now_s=0.0,
        ttl_s=100.0,
        priority=10,
    )

    assert buffer.seed(node=node, message=low, now_s=0.0).accepted
    result = buffer.seed(node=node, message=high, now_s=0.0)
    assert result.accepted
    assert result.evicted_message_id == low.message_id
    assert node.buffer_hops == {high.message_id: 0}


def test_epidemic_transfer_is_exactly_once_per_receiver() -> None:
    registry = MessageRegistry()
    manager = CommunicationManager(
        registry=registry,
        communication_range_m=100.0,
        detector=ContactDetector("spatial_hash"),
        rng=Random(7),
    )
    vehicles = {
        0: VehicleState("a", x=0.0, y=0.0),
        1: VehicleState("b", x=10.0, y=0.0),
    }
    nodes = {0: DTNNodeState(True, 8), 1: DTNNodeState(True, 8)}
    message = registry.create(
        kind=MessageKind.ROAD_CLOSED,
        source_vehicle_id="a",
        subject_id="edge-1",
        now_s=0.0,
        ttl_s=30.0,
        reliability=1.0,
    )
    manager.buffer.seed(node=nodes[0], message=message, now_s=0.0)

    first = manager.tick(now_s=1.0, vehicles=vehicles, dtn_nodes=nodes)
    second = manager.tick(now_s=2.0, vehicles=vehicles, dtn_nodes=nodes)

    assert len(first.started_contacts) == 1
    assert len(first.transfers) == 1
    assert first.transfers[0].hop_count == 1
    assert len(second.started_contacts) == 0
    assert len(second.transfers) == 0
    assert nodes[1].buffer_hops[message.message_id] == 1
