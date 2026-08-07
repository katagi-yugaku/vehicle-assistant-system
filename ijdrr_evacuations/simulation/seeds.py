"""Stable, named random streams derived from a master seed."""

from __future__ import annotations

import hashlib
from dataclasses import dataclass, field
from random import Random


DEFAULT_STREAM_NAMES = (
    "vehicle_generation",
    "dtn_assignment",
    "communication_success",
    "driver_traits",
    "behavior_choice",
    "routing_choice",
    "hazard_events",
)


def derive_seed(master_seed: int, stream_name: str) -> int:
    encoded = f"ijdrr-v1:{master_seed}:{stream_name}".encode("utf-8")
    digest = hashlib.blake2b(encoded, digest_size=8).digest()
    return int.from_bytes(digest, "big", signed=False)


@dataclass(slots=True)
class SeedStreams:
    master_seed: int
    names: tuple[str, ...] = DEFAULT_STREAM_NAMES
    seeds: dict[str, int] = field(init=False)
    _generators: dict[str, Random] = field(init=False, repr=False)

    def __post_init__(self) -> None:
        if len(set(self.names)) != len(self.names):
            raise ValueError("random stream names must be unique")
        self.seeds = {
            name: derive_seed(self.master_seed, name) for name in self.names
        }
        self._generators = {
            name: Random(seed) for name, seed in self.seeds.items()
        }

    def generator(self, name: str) -> Random:
        try:
            return self._generators[name]
        except KeyError as exc:
            raise KeyError(f"unknown random stream: {name}") from exc
