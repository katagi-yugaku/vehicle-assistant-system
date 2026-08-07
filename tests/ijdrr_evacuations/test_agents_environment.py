from __future__ import annotations

import pytest

from ijdrr_evacuations.agents import (
    AgentStore,
    DriverState,
    DTNNodeState,
    EvacuationState,
    EvacuationStatus,
    VehicleState,
)
from ijdrr_evacuations.environment import ShelterState


def test_agent_store_composes_states_without_legacy_agents() -> None:
    store = AgentStore()
    index = store.add(
        vehicle=VehicleState("veh-1"),
        driver=DriverState(),
        evacuation=EvacuationState("shelter-1"),
        dtn=DTNNodeState(True, 8),
    )

    assert index == 0
    assert store.index_of("veh-1") == index
    assert store.agent(index).dtn is store.dtn_nodes[index]
    with pytest.raises(ValueError, match="already registered"):
        store.add(
            vehicle=VehicleState("veh-1"),
            driver=DriverState(),
            evacuation=EvacuationState("shelter-1"),
        )


def test_vehicle_and_evacuation_lifecycle() -> None:
    vehicle = VehicleState("veh-1")
    vehicle.update_mobility(
        now_s=1.0,
        position=(10.0, 20.0),
        edge_id="edge-1",
        lane_id="edge-1_0",
        speed_mps=0.0,
        distance_m=15.0,
    )
    assert vehicle.stopped_since_s == 1.0
    vehicle.update_mobility(
        now_s=2.0,
        position=(12.0, 20.0),
        edge_id="edge-1",
        lane_id="edge-1_0",
        speed_mps=2.0,
    )
    assert vehicle.stopped_since_s is None

    evacuation = EvacuationState("shelter-1")
    evacuation.start(1.0)
    evacuation.complete(10.0)
    evacuation.fail(11.0, "must_not_override_completion")
    assert evacuation.status is EvacuationStatus.COMPLETED
    assert evacuation.completed_at_s == 10.0


def test_shelter_reports_transition_to_full_once() -> None:
    shelter = ShelterState("shelter-1", 2, "edge-1")
    assert shelter.register_arrival() is False
    assert shelter.register_arrival() is True
    assert shelter.register_arrival() is False
    assert shelter.occupancy_rate == 1.0
