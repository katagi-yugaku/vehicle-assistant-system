"""Deterministic contact detection with cKDTree and spatial-hash backends."""

from __future__ import annotations

from collections import defaultdict
from dataclasses import dataclass
from math import floor
from typing import Mapping, Protocol

from ijdrr_evacuations.agents import DTNNodeState, VehicleState


@dataclass(frozen=True, slots=True, order=True)
class Contact:
    left_index: int
    right_index: int
    distance_m: float


@dataclass(frozen=True, slots=True)
class ContactUpdate:
    active: tuple[Contact, ...]
    started: tuple[Contact, ...]
    ended_pairs: tuple[tuple[int, int], ...]


class ContactBackend(Protocol):
    def detect(
        self,
        *,
        vehicles: Mapping[int, VehicleState],
        dtn_nodes: Mapping[int, DTNNodeState],
        communication_range_m: float,
    ) -> tuple[Contact, ...]: ...


class SpatialHashContactDetector:
    """Dependency-free expected O(M + C) radius search."""

    def detect(
        self,
        *,
        vehicles: Mapping[int, VehicleState],
        dtn_nodes: Mapping[int, DTNNodeState],
        communication_range_m: float,
    ) -> tuple[Contact, ...]:
        if communication_range_m <= 0:
            raise ValueError("communication_range_m must be positive")

        cell_size = communication_range_m
        cells: dict[tuple[int, int], list[int]] = defaultdict(list)
        capable_indices = sorted(
            index
            for index, node in dtn_nodes.items()
            if node.enabled and index in vehicles
        )
        for index in capable_indices:
            state = vehicles[index]
            key = floor(state.x / cell_size), floor(state.y / cell_size)
            cells[key].append(index)

        range_sq = communication_range_m * communication_range_m
        contacts: list[Contact] = []
        for left_index in capable_indices:
            left = vehicles[left_index]
            cell_x = floor(left.x / cell_size)
            cell_y = floor(left.y / cell_size)
            for dx in (-1, 0, 1):
                for dy in (-1, 0, 1):
                    for right_index in cells.get((cell_x + dx, cell_y + dy), ()):
                        if right_index <= left_index:
                            continue
                        right = vehicles[right_index]
                        delta_x = left.x - right.x
                        delta_y = left.y - right.y
                        distance_sq = delta_x * delta_x + delta_y * delta_y
                        if distance_sq <= range_sq:
                            contacts.append(
                                Contact(left_index, right_index, distance_sq**0.5)
                            )
        return tuple(sorted(contacts))


class CKDTreeContactDetector:
    """SciPy-backed radius pair search for thousands of DTN nodes."""

    def detect(
        self,
        *,
        vehicles: Mapping[int, VehicleState],
        dtn_nodes: Mapping[int, DTNNodeState],
        communication_range_m: float,
    ) -> tuple[Contact, ...]:
        if communication_range_m <= 0:
            raise ValueError("communication_range_m must be positive")
        try:
            import numpy as np
            from scipy.spatial import cKDTree
        except ImportError as exc:
            raise RuntimeError("cKDTree backend requires NumPy and SciPy") from exc

        indices = sorted(
            index
            for index, node in dtn_nodes.items()
            if node.enabled and index in vehicles
        )
        if len(indices) < 2:
            return ()

        positions = np.asarray(
            [(vehicles[index].x, vehicles[index].y) for index in indices],
            dtype=np.float64,
        )
        tree = cKDTree(positions)
        local_pairs = tree.query_pairs(communication_range_m, output_type="ndarray")
        contacts = []
        for left_local, right_local in local_pairs:
            left_index = indices[int(left_local)]
            right_index = indices[int(right_local)]
            delta = positions[int(left_local)] - positions[int(right_local)]
            distance_m = float(np.sqrt(np.dot(delta, delta)))
            contacts.append(Contact(left_index, right_index, distance_m))
        return tuple(sorted(contacts))


class ContactDetector:
    """Select cKDTree when available, otherwise use the standard-library grid."""

    __slots__ = ("backend",)

    def __init__(self, backend: str = "auto") -> None:
        if backend not in {"auto", "ckdtree", "spatial_hash"}:
            raise ValueError(f"unknown contact backend: {backend}")
        if backend == "spatial_hash":
            self.backend: ContactBackend = SpatialHashContactDetector()
        elif backend == "ckdtree":
            self.backend = CKDTreeContactDetector()
        else:
            try:
                import scipy.spatial  # noqa: F401
            except ImportError:
                self.backend = SpatialHashContactDetector()
            else:
                self.backend = CKDTreeContactDetector()

    def detect(
        self,
        *,
        vehicles: Mapping[int, VehicleState],
        dtn_nodes: Mapping[int, DTNNodeState],
        communication_range_m: float,
    ) -> tuple[Contact, ...]:
        return self.backend.detect(
            vehicles=vehicles,
            dtn_nodes=dtn_nodes,
            communication_range_m=communication_range_m,
        )


class ContactHistory:
    """Keep active pairs and aggregate counts, not an unbounded full history."""

    __slots__ = ("active_pairs", "total_started")

    def __init__(self) -> None:
        self.active_pairs: set[tuple[int, int]] = set()
        self.total_started = 0

    def update(self, contacts: tuple[Contact, ...]) -> ContactUpdate:
        current = {(contact.left_index, contact.right_index) for contact in contacts}
        started_pairs = current - self.active_pairs
        ended_pairs = self.active_pairs - current
        contact_by_pair = {
            (contact.left_index, contact.right_index): contact
            for contact in contacts
        }
        started = tuple(contact_by_pair[pair] for pair in sorted(started_pairs))
        self.active_pairs = current
        self.total_started += len(started)
        return ContactUpdate(
            active=contacts,
            started=started,
            ended_pairs=tuple(sorted(ended_pairs)),
        )
