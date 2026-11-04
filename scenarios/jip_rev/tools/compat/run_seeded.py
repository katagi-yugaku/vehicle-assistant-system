"""Run a runner module (original or revised) with a fixed seed and dump a
canonical trace for equivalence checks (Step 2).

The original runners call ``random.seed()`` / ``np.random.seed()`` without an
argument (1-1-2) or seed from ``secrets.randbits(31)`` (1-1-4).  This wrapper
replaces those entropy sources with the given seed WITHOUT editing the runner,
so that the original code and the revised code (``--seed``) start from the
same RNG state.

Usage (run from the repository root whose code should be used):
    PYTHONHASHSEED=0 python tools/compat/run_seeded.py SEED OUT.json \
        scenarios.JIP.1-1-2.simulation.runner_simulator --nogui CONFIG 1.0 1.0 [extra args]

The dump contains:
  arrival_time_by_vehID, route_change_time_by_vehID, route_change_events
  (successful and failed), and counters.  Only module globals are read after
  the run; nothing in the simulation is changed.
"""
from __future__ import annotations

import json
import random
import runpy
import secrets
import sys
from dataclasses import asdict, is_dataclass

import numpy as np


def main() -> None:
    seed = int(sys.argv[1])
    out_path = sys.argv[2]
    module = sys.argv[3]
    sys.argv = [module] + sys.argv[4:]

    _orig_random_seed = random.seed
    _orig_np_seed = np.random.seed

    def random_seed(a=None, *args, **kwargs):
        return _orig_random_seed(seed if a is None else a, *args, **kwargs)

    def np_seed(s=None):
        return _orig_np_seed(seed if s is None else s)

    random.seed = random_seed
    np.random.seed = np_seed
    secrets.randbits = lambda k: seed  # 1-1-4: RANDOM_SEED = secrets.randbits(31)

    g = runpy.run_module(module, run_name="__main__")

    def conv(x):
        if is_dataclass(x):
            return {k: conv(v) for k, v in asdict(x).items()}
        if isinstance(x, (list, tuple)):
            return [conv(v) for v in x]
        if isinstance(x, dict):
            return {str(k): conv(v) for k, v in x.items()}
        if hasattr(x, "value") and type(x).__module__ != "builtins":
            return getattr(x, "value")
        return x

    keys = [
        "ROUTE_CHANGED_VEHICLE_COUNT", "ROUTE_CHANGE_EVENT_COUNT",
        "NORMALCY_BIAS_ROUTE_CHANGE_COUNT", "MAJORITY_BIAS_ROUTE_CHANGE_COUNT",
        "NEW_VEHICLE_COUNT",
    ]
    dump = {
        "seed": seed,
        "module": module,
        "argv": sys.argv,
        "arrival_time_by_vehID": conv(g.get("arrival_time_by_vehID_dict", {})),
        "route_change_time_by_vehID": conv(g.get("route_change_time_by_vehID_dict", {})),
        "route_change_events": conv(g.get("route_change_events", [])),
        "counters": {k: g.get(k) for k in keys},
    }
    with open(out_path, "w") as f:
        json.dump(dump, f, indent=1, sort_keys=True)


if __name__ == "__main__":
    main()
