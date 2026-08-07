from __future__ import annotations

from types import SimpleNamespace

from ijdrr_evacuations.simulation import LibsumoAdapter


class _VehicleApi:
    def getSubscriptionResults(self, vehicle_id):
        assert vehicle_id == "veh-1"
        return {
            1: (10.0, 20.0),
            2: "edge-1",
            3: "edge-1_0",
            4: 5.0,
            5: 42.0,
        }

    def getPosition(self, vehicle_id):
        raise AssertionError("subscribed position should be used")

    def getRoadID(self, vehicle_id):
        raise AssertionError("subscribed edge should be used")

    def getLaneID(self, vehicle_id):
        raise AssertionError("subscribed lane should be used")

    def getSpeed(self, vehicle_id):
        raise AssertionError("subscribed speed should be used")

    def getDistance(self, vehicle_id):
        raise AssertionError("subscribed distance should be used")


def test_libsumo_adapter_uses_subscription_values_without_fallback_getters() -> None:
    api = SimpleNamespace(
        start=lambda command: None,
        close=lambda: None,
        simulationStep=lambda: None,
        simulation=SimpleNamespace(),
        vehicle=_VehicleApi(),
        constants=SimpleNamespace(
            VAR_POSITION=1,
            VAR_ROAD_ID=2,
            VAR_LANE_ID=3,
            VAR_SPEED=4,
            VAR_DISTANCE=5,
        ),
    )
    adapter = LibsumoAdapter(api)

    snapshot = adapter.vehicle_snapshot("veh-1")

    assert snapshot.position == (10.0, 20.0)
    assert snapshot.edge_id == "edge-1"
    assert snapshot.lane_id == "edge-1_0"
    assert snapshot.speed_mps == 5.0
    assert snapshot.distance_m == 42.0
