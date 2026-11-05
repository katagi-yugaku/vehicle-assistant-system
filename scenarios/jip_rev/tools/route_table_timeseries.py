#!/usr/bin/env python3
"""
避難所が共有している「経路ごとの平均旅行時間の表」を、各回の raw.json から時刻ごとに再現する(M2・R2-1)。

    python tools/route_table_timeseries.py runs/<実験ID> [--out DIR] [--step 10] [--window 200] [--include-uturn]

避難所の計算と同じ定義:
  - 経路は到着時の edge 列で区別する(前方で経路を変えた車は変更後の経路に入る)
  - 旅行時間 = 到着時刻 − 出発時刻 − travel_time_offset(manifest の設定。修正版は 0)
  - 時刻 t の値 = t までに到着した車の累積平均(時間窓なし)
  - U ターンした車は再生成時刻から測った「U ターン地点 → 避難所」の時間で、別の経路として入る
    (既定では出さない。--include-uturn で出す)
比較用に、--window を付けると「直近 window 秒に着いた車だけの平均」も出す(実装では使っていない値)。

出力(--out、既定は results/<実験ID>/route_table):
  route_table_runs.csv    回ごと:condition, variant, seed, t, route, avg_time, n_vehicles[, window_avg, window_n]
  route_table_mean.csv    条件 × 版 × 経路 × 時刻ごとの、シード平均(その時刻に値がある回だけで平均)
  route_table_<条件>.png / .pdf   経路 1・経路 2 の累積平均の推移(版ごとに線種を変える)
"""
from __future__ import annotations

import argparse
import csv
import json
import statistics
from collections import defaultdict
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
ORIGIN_EDGE = "E0"


def route_label(route: tuple[str, ...]) -> str:
    if not route:
        return ""
    if route[0] == ORIGIN_EDGE:
        return {"E16": "route1_E0-E16", "E13": "route2_E0-E13"}.get(route[-1], f"E0-{route[-1]}")
    return f"uturn_{route[0]}-{route[-1]}"


def load_manifest(run_root: Path) -> dict:
    return json.loads((run_root / "manifest.json").read_text())


def offset_for(manifest: dict, variant: str) -> float:
    flags = (manifest.get("variants") or {}).get(variant) or {}
    extra = manifest.get("extra_flags") or {}
    return float(extra.get("travel_time_offset", flags.get("travel_time_offset", 100.0)))


def runs_of(run_root: Path):
    with open(run_root / "tasks.tsv") as f:
        for t in csv.DictReader(f, delimiter="\t"):
            d = Path(t["run_dir"])
            if (d / "raw.json").exists():
                yield t["layout"], t["condition"], t["variant"], int(t["seed"]), d


def series_for_run(raw: dict, offset: float, step: float, window: float | None, include_uturn: bool):
    recs = []
    for v in (raw.get("vehicles") or {}).values():
        r = tuple(v.get("arrival_route") or ())
        if v.get("arrival_time") is None or not r:
            continue
        label = route_label(r)
        if label.startswith("uturn_") and not include_uturn:
            continue
        tt = float(v["arrival_time"]) - float(v["arrival_sumo_depart_time"]) - offset
        recs.append((float(v["arrival_time"]), label, tt))
    recs.sort()
    end = float(raw.get("end_simulation_time") or (recs[-1][0] if recs else 0))
    out = []
    i, acc = 0, defaultdict(lambda: [0.0, 0])
    t = step
    while t <= end + 1e-9:
        while i < len(recs) and recs[i][0] <= t:
            _, label, tt = recs[i]
            acc[label][0] += tt
            acc[label][1] += 1
            i += 1
        for label, (s, n) in acc.items():
            row = {"t": t, "route": label, "avg_time": s / n, "n_vehicles": n}
            if window:
                w = [tt for (ta, lb, tt) in recs if lb == label and t - window < ta <= t]
                row["window_avg"] = statistics.fmean(w) if w else None
                row["window_n"] = len(w)
            out.append(row)
        t += step
    return out


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("run_root")
    ap.add_argument("--out")
    ap.add_argument("--step", type=float, default=10.0)
    ap.add_argument("--window", type=float, default=None)
    ap.add_argument("--include-uturn", action="store_true")
    args = ap.parse_args()
    run_root = Path(args.run_root).resolve()
    manifest = load_manifest(run_root)
    out = Path(args.out) if args.out else ROOT / "results" / manifest["experiment_id"] / "route_table"
    out.mkdir(parents=True, exist_ok=True)

    rows = []
    for layout, cond, variant, seed, d in runs_of(run_root):
        raw = json.loads((d / "raw.json").read_text())
        for r in series_for_run(raw, offset_for(manifest, variant), args.step, args.window, args.include_uturn):
            rows.append({"layout": layout, "condition": cond, "variant": variant, "seed": seed, **r})
    if not rows:
        print("no raw.json found")
        return 1
    fields = list(dict.fromkeys(k for r in rows for k in r))
    with open(out / "route_table_runs.csv", "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=fields, lineterminator="\n")
        w.writeheader()
        for r in rows:
            w.writerow({k: (round(v, 3) if isinstance(v, float) else v) for k, v in r.items()})

    groups = defaultdict(list)
    for r in rows:
        groups[(r["layout"], r["condition"], r["variant"], r["route"], r["t"])].append(r)
    mean_rows = []
    for (layout, cond, variant, route, t), rs in sorted(groups.items()):
        rec = {"layout": layout, "condition": cond, "variant": variant, "route": route, "t": t,
               "n_runs": len(rs), "avg_time_mean": statistics.fmean(r["avg_time"] for r in rs),
               "n_vehicles_mean": statistics.fmean(r["n_vehicles"] for r in rs)}
        if args.window:
            wv = [r["window_avg"] for r in rs if r.get("window_avg") is not None]
            rec["window_avg_mean"] = statistics.fmean(wv) if wv else None
        mean_rows.append(rec)
    with open(out / "route_table_mean.csv", "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(mean_rows[0]), lineterminator="\n")
        w.writeheader()
        for r in mean_rows:
            w.writerow({k: (round(v, 3) if isinstance(v, float) else v) for k, v in r.items()})

    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except Exception:
        return 0
    colors = {"route1_E0-E16": "#1f77b4", "route2_E0-E13": "#d62728"}
    styles = {"v2v_dtn": "-", "v2v_only": "--", "none": ":"}
    for (layout, cond) in sorted({(r["layout"], r["condition"]) for r in mean_rows}):
        fig, ax = plt.subplots(figsize=(8, 4.5))
        for (variant, route) in sorted({(r["variant"], r["route"]) for r in mean_rows
                                        if r["layout"] == layout and r["condition"] == cond}):
            if route not in colors:
                continue
            rs = [r for r in mean_rows if r["layout"] == layout and r["condition"] == cond
                  and r["variant"] == variant and r["route"] == route]
            ax.plot([r["t"] for r in rs], [r["avg_time_mean"] for r in rs], color=colors[route],
                    linestyle=styles.get(variant, "-."), linewidth=1.8,
                    label=f"{route.split('_')[0]} cumulative ({variant})")
            if args.window:
                ax.plot([r["t"] for r in rs], [r.get("window_avg_mean") for r in rs], color=colors[route],
                        linestyle=styles.get(variant, "-."), linewidth=0.8, alpha=0.5,
                        label=f"{route.split('_')[0]} last {args.window:g} s ({variant})")
        ax.set_xlabel("Simulation time [s]")
        ax.set_ylabel("Average travel time shared by the shelter [s]")
        ax.set_title(f"{layout} {cond} (mean over seeds)", fontsize=10)
        ax.grid(alpha=0.3)
        ax.legend(fontsize=7)
        fig.tight_layout()
        for ext in ("png", "pdf"):
            fig.savefig(out / f"route_table_{cond}.{ext}", dpi=150)
        plt.close(fig)
    print(f"[route_table] {len(rows)} rows -> {out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
