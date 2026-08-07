"""Read legacy objects without importing or modifying their classes."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any


@dataclass(frozen=True, slots=True)
class LegacyAgentSnapshot:
    vehicle_id: str
    destination_id: str
    arrival_time_s: float | None
    route_changed: bool
    wrong_way: bool
    abandoned: bool


class LegacySnapshotAdapter:
    """Duck-typed, read-only adapter intended for regression comparison."""

    @staticmethod
    def _read(instance: Any, getter: str, attribute: str, default: Any) -> Any:
        method = getattr(instance, getter, None)
        if callable(method):
            return method()
        return getattr(instance, attribute, default)

    def snapshot(self, legacy_agent: Any) -> LegacyAgentSnapshot:
        return LegacyAgentSnapshot(
            vehicle_id=str(
                self._read(legacy_agent, "get_vehID", "vehID", "")
            ),
            destination_id=str(
                self._read(
                    legacy_agent,
                    "get_target_shelter",
                    "target_shelter",
                    "",
                )
            ),
            arrival_time_s=self._read(
                legacy_agent,
                "get_arrival_time",
                "arrival_time",
                None,
            ),
            route_changed=bool(
                self._read(
                    legacy_agent,
                    "get_evacuation_route_changed_flg",
                    "evacuation_route_changed_flg",
                    False,
                )
            ),
            wrong_way=bool(
                self._read(
                    legacy_agent,
                    "get_wrong_way_driving_flg",
                    "wrong_way_driving_flg",
                    False,
                )
            ),
            abandoned=bool(
                self._read(
                    legacy_agent,
                    "get_vehicle_abandoned_flg",
                    "vehicle_abandoned_flg",
                    False,
                )
            ),
        )
