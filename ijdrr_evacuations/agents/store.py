"""Dense internal identifiers and state ownership for vehicle agents."""

from __future__ import annotations

from collections.abc import Iterator

from .state import (
    DriverState,
    DTNNodeState,
    EvacuationState,
    VehicleAgent,
    VehicleState,
)


class AgentStore:
    """Own all per-vehicle state while exposing stable integer indices."""

    __slots__ = (
        "_next_index",
        "_index_by_vehicle_id",
        "_vehicle_id_by_index",
        "vehicles",
        "drivers",
        "evacuations",
        "dtn_nodes",
    )

    def __init__(self) -> None:
        self._next_index = 0
        self._index_by_vehicle_id: dict[str, int] = {}
        self._vehicle_id_by_index: dict[int, str] = {}
        self.vehicles: dict[int, VehicleState] = {}
        self.drivers: dict[int, DriverState] = {}
        self.evacuations: dict[int, EvacuationState] = {}
        self.dtn_nodes: dict[int, DTNNodeState] = {}

    def add(
        self,
        *,
        vehicle: VehicleState,
        driver: DriverState,
        evacuation: EvacuationState,
        dtn: DTNNodeState | None = None,
    ) -> int:
        if vehicle.vehicle_id in self._index_by_vehicle_id:
            raise ValueError(f"vehicle already registered: {vehicle.vehicle_id}")

        index = self._next_index
        self._next_index += 1
        self._index_by_vehicle_id[vehicle.vehicle_id] = index
        self._vehicle_id_by_index[index] = vehicle.vehicle_id
        self.vehicles[index] = vehicle
        self.drivers[index] = driver
        self.evacuations[index] = evacuation
        if dtn is not None:
            self.dtn_nodes[index] = dtn
        return index

    def index_of(self, vehicle_id: str) -> int:
        return self._index_by_vehicle_id[vehicle_id]

    def get_index(self, vehicle_id: str) -> int | None:
        return self._index_by_vehicle_id.get(vehicle_id)

    def vehicle_id_of(self, index: int) -> str:
        return self._vehicle_id_by_index[index]

    def agent(self, index: int) -> VehicleAgent:
        return VehicleAgent(
            index=index,
            vehicle=self.vehicles[index],
            driver=self.drivers[index],
            evacuation=self.evacuations[index],
            dtn=self.dtn_nodes.get(index),
        )

    def agent_by_vehicle_id(self, vehicle_id: str) -> VehicleAgent:
        return self.agent(self.index_of(vehicle_id))

    def indices(self) -> Iterator[int]:
        return iter(self.vehicles)

    def __len__(self) -> int:
        return len(self.vehicles)
