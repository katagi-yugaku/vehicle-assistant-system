# IJDRR evacuation simulation package

This package is independent from `evacsim.agents`.  The four legacy agent
files are not imported or modified by the simulation hot path.

## Package boundaries

- `agents`: lightweight per-vehicle state and `AgentStore`
- `communication`: immutable messages, registry, contacts, buffers, forwarding
- `behavior`: replaceable driver policy and evacuation lifecycle
- `routing`: versioned route cache and backend-independent route execution
- `environment`: road, shelter, and hazard state shared by all vehicles
- `events`: immutable records and future-event queue
- `metrics`: online counters and buffered JSONL/Parquet output
- `simulation`: context, multi-rate scheduler, seed streams, and libsumo adapter
- `compat`: optional read-only snapshots of legacy agent objects

## Unit tests

```bash
python3 -m pytest -q tests/ijdrr_evacuations
```

## libsumo smoke run

The runner never falls back to TraCI:

```bash
python3 scenarios/IJDRR/runners/test_run.py \
  --vehicles 8 \
  --max-time 600 \
  --seed 23423
```

If the local SUMO build has no generated Python binding, install a matching
`libsumo` package or rebuild SUMO after installing SWIG.  The executable SUMO
binary and the Python libsumo binding should use the same SUMO version.
