"""Compare two traces written by tools/compat/run_seeded.py (Step 2).

Usage:
    python tools/compat/compare_traces.py A/trace.json B/trace.json [--events-only|--arrivals-only]

Exit code 0 when the compared parts are identical, 1 otherwise.
Compared parts:
  - arrival time of every SUMO vehicle ID (arrival_time_by_vehID)
  - all route-change events (successful and failed; every field)
  - route-change counters
"""
from __future__ import annotations

import json
import sys


def load(path: str) -> dict:
    with open(path) as f:
        return json.load(f)


def summarize(trace: dict) -> str:
    arrivals = trace["arrival_time_by_vehID"]
    events = trace["route_change_events"]
    ok = [e for e in events if e.get("success")]
    times = list(arrivals.values())
    return (
        f"arrived={len(arrivals)} completion={max(times) if times else None} "
        f"mean_arrival={sum(times)/len(times) if times else None:.3f} "
        f"events={len(events)} success={len(ok)} counters={trace.get('counters')}"
    )


def main() -> int:
    args = [a for a in sys.argv[1:] if not a.startswith("--")]
    flags = {a for a in sys.argv[1:] if a.startswith("--")}
    a, b = load(args[0]), load(args[1])
    print("A:", summarize(a))
    print("B:", summarize(b))

    same = True
    if "--events-only" not in flags:
        aa, ba = a["arrival_time_by_vehID"], b["arrival_time_by_vehID"]
        diff_ids = sorted(
            k for k in set(aa) | set(ba) if aa.get(k) != ba.get(k)
        )
        print(f"arrival times identical: {not diff_ids} (differing vehicles: {len(diff_ids)})")
        for k in diff_ids[:10]:
            print(f"   {k}: {aa.get(k)} vs {ba.get(k)}")
        same &= not diff_ids
    if "--arrivals-only" not in flags:
        ea, eb = a["route_change_events"], b["route_change_events"]
        n_same = 0
        for x, y in zip(ea, eb):
            if x != y:
                break
            n_same += 1
        events_same = ea == eb
        print(
            f"route-change events identical: {events_same} "
            f"(A={len(ea)}, B={len(eb)}, identical prefix={n_same})"
        )
        if not events_same and n_same < min(len(ea), len(eb)):
            x, y = ea[n_same], eb[n_same]
            keys = sorted(k for k in set(x) | set(y) if x.get(k) != y.get(k))
            print(f"   first difference at index {n_same}: fields {keys}")
            for k in keys[:6]:
                print(f"     {k}: {x.get(k)!r} vs {y.get(k)!r}")
        counters_same = a.get("counters") == b.get("counters")
        print(f"counters identical: {counters_same}")
        same &= events_same and counters_same
    print("RESULT:", "IDENTICAL" if same else "DIFFERENT")
    return 0 if same else 1


if __name__ == "__main__":
    sys.exit(main())
