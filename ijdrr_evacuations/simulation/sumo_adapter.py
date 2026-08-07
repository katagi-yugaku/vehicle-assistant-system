"""SUMO boundary with a strict libsumo implementation and no TraCI fallback."""

from __future__ import annotations

import importlib
import os
import sys
from dataclasses import dataclass
from pathlib import Path
from types import ModuleType
from typing import Protocol, Sequence


class LibsumoUnavailableError(RuntimeError):
    pass


@dataclass(frozen=True, slots=True)
class VehicleSnapshot:
    position: tuple[float, float]
    edge_id: str
    lane_id: str
    speed_mps: float
    distance_m: float


class SumoAdapter(Protocol):
    def start(self, command: Sequence[str]) -> None: ...

    def close(self) -> None: ...

    def simulation_step(self) -> None: ...

    def time_s(self) -> float: ...

    def min_expected_number(self) -> int: ...

    def departed_vehicle_ids(self) -> tuple[str, ...]: ...

    def arrived_vehicle_ids(self) -> tuple[str, ...]: ...

    def vehicle_ids(self) -> tuple[str, ...]: ...

    def subscribe_vehicle(self, vehicle_id: str) -> None: ...

    def vehicle_snapshot(self, vehicle_id: str) -> VehicleSnapshot: ...

    def find_route(
        self,
        from_edge_id: str,
        to_edge_id: str,
    ) -> tuple[tuple[str, ...], float | None]: ...

    def set_vehicle_route(
        self,
        vehicle_id: str,
        route_edges: tuple[str, ...],
    ) -> None: ...


class LibsumoAdapter:
    """Thin adapter around the Python libsumo binding.

    Import is delayed so unit tests and non-SUMO tooling can use the package
    without installing libsumo.  This class never falls back to socket TraCI.
    """

    __slots__ = ("api", "constants", "started")

    def __init__(self, api: ModuleType) -> None:
        required = ("start", "close", "simulationStep", "simulation", "vehicle")
        missing = [name for name in required if not hasattr(api, name)]
        if missing:
            raise LibsumoUnavailableError(
                "libsumo binding is missing attributes: " + ", ".join(missing)
            )
        self.api = api
        self.constants = api.constants
        self.started = False

    @classmethod
    def load(cls, *, sumo_home: Path | None = None) -> "LibsumoAdapter":
        import_errors: list[BaseException] = []
        if sumo_home is not None:
            os.environ["SUMO_HOME"] = str(sumo_home)
        try:
            return cls(importlib.import_module("libsumo"))
        except (ImportError, ModuleNotFoundError) as exc:
            import_errors.append(exc)
            sys.modules.pop("libsumo", None)

        if sumo_home is not None:
            tools_dir = sumo_home / "tools"
            if tools_dir.is_dir() and str(tools_dir) not in sys.path:
                sys.path.insert(0, str(tools_dir))
        try:
            api = importlib.import_module("libsumo")
        except (ImportError, ModuleNotFoundError) as exc:
            import_errors.append(exc)
            sys.modules.pop("libsumo", None)
            location = f" SUMO_HOME={sumo_home}." if sumo_home else ""
            raise LibsumoUnavailableError(
                "Python libsumo binding could not be imported."
                + location
                + " Install it with `python -m pip install libsumo`, or install "
                "SWIG and rebuild SUMO with ENABLE_PYTHON_BINDINGS=ON. "
                "Import errors: "
                + " | ".join(str(error) for error in import_errors)
            ) from exc
        return cls(api)

    def start(self, command: Sequence[str]) -> None:
        if self.started:
            raise RuntimeError("libsumo simulation is already started")
        self.api.start(list(command))
        self.started = True

    def close(self) -> None:
        if self.started:
            self.api.close()
            self.started = False

    def simulation_step(self) -> None:
        self.api.simulationStep()

    def time_s(self) -> float:
        return float(self.api.simulation.getTime())

    def min_expected_number(self) -> int:
        return int(self.api.simulation.getMinExpectedNumber())

    def departed_vehicle_ids(self) -> tuple[str, ...]:
        return tuple(self.api.simulation.getDepartedIDList())

    def arrived_vehicle_ids(self) -> tuple[str, ...]:
        return tuple(self.api.simulation.getArrivedIDList())

    def vehicle_ids(self) -> tuple[str, ...]:
        return tuple(self.api.vehicle.getIDList())

    def add_vehicle(
        self,
        *,
        vehicle_id: str,
        route_id: str,
        depart_s: float,
        type_id: str = "DEFAULT_VEHTYPE",
    ) -> None:
        self.api.vehicle.add(
            vehicle_id,
            route_id,
            type_id,
            str(depart_s),
        )

    def subscribe_vehicle(self, vehicle_id: str) -> None:
        variables = [
            self.constants.VAR_POSITION,
            self.constants.VAR_ROAD_ID,
            self.constants.VAR_LANE_ID,
            self.constants.VAR_SPEED,
            self.constants.VAR_DISTANCE,
        ]
        self.api.vehicle.subscribe(vehicle_id, variables)

    def vehicle_snapshot(self, vehicle_id: str) -> VehicleSnapshot:
        values = self.api.vehicle.getSubscriptionResults(vehicle_id) or {}
        tc = self.constants
        position = values.get(tc.VAR_POSITION)
        if position is None:
            position = self.api.vehicle.getPosition(vehicle_id)
        edge_id = values.get(tc.VAR_ROAD_ID)
        if edge_id is None:
            edge_id = self.api.vehicle.getRoadID(vehicle_id)
        lane_id = values.get(tc.VAR_LANE_ID)
        if lane_id is None:
            lane_id = self.api.vehicle.getLaneID(vehicle_id)
        speed_mps = values.get(tc.VAR_SPEED)
        if speed_mps is None:
            speed_mps = self.api.vehicle.getSpeed(vehicle_id)
        distance_m = values.get(tc.VAR_DISTANCE)
        if distance_m is None:
            distance_m = self.api.vehicle.getDistance(vehicle_id)
        return VehicleSnapshot(
            position=(float(position[0]), float(position[1])),
            edge_id=str(edge_id),
            lane_id=str(lane_id),
            speed_mps=float(speed_mps),
            distance_m=float(distance_m),
        )

    def vehicle_route(self, vehicle_id: str) -> tuple[str, ...]:
        return tuple(self.api.vehicle.getRoute(vehicle_id))

    def find_route(
        self,
        from_edge_id: str,
        to_edge_id: str,
    ) -> tuple[tuple[str, ...], float | None]:
        result = self.api.simulation.findRoute(from_edge_id, to_edge_id)
        travel_time = getattr(result, "travelTime", None)
        return tuple(result.edges), float(travel_time) if travel_time is not None else None

    def set_vehicle_route(
        self,
        vehicle_id: str,
        route_edges: tuple[str, ...],
    ) -> None:
        self.api.vehicle.setRoute(vehicle_id, list(route_edges))

    def version(self) -> str:
        version = self.api.getVersion()
        if isinstance(version, tuple) and len(version) > 1:
            return str(version[1])
        return str(version)
