#!/usr/bin/env python3
"""
判断の記録(log_decisions = true で runner が書く decisions.jsonl.gz)を、条件 × 版ごとにまとめる。

    python3 tools/analyze_decisions.py runs/<実験ID> --out results/<実験ID>/decisions

見るもの
  分岐前(E0/E1):表(経路 A・B の平均旅行時間)を持って判断した車・持たないまま通過した車の台数、
                 変更した車の短縮時間と θ、変更しなかった車の最後の判断の短縮時間と θ、情報の古さ、
                 時刻ごとの表の値(経路 A・B の平均と差)
  分岐後       :承認された U ターンの回数・短縮時間・θ
  θ < 0 の運転者の数
出力: decisions_summary.csv、decisions_timeline.csv、decisions.md
"""
from __future__ import annotations

import argparse
import csv
import gzip
import json
import statistics
from collections import defaultdict
from pathlib import Path

TIME_BIN = 300


def med(xs):
    xs = [x for x in xs if x is not None]
    return round(statistics.median(xs), 1) if xs else None


def mean(xs):
    xs = [x for x in xs if x is not None]
    return round(sum(xs) / len(xs), 1) if xs else None


def find_runs(root: Path) -> list[dict]:
    tasks = root / "tasks.tsv"
    if tasks.is_file():
        with open(tasks) as f:
            rows = list(csv.DictReader(f, delimiter="\t"))
        return [r for r in rows if (Path(r["run_dir"]) / "decisions.jsonl.gz").is_file()]
    return [{"layout": "", "condition": p.parent.parent.name, "variant": p.parent.name, "seed": p.name,
             "run_dir": str(p)} for p in sorted(q.parent for q in root.rglob("decisions.jsonl.gz"))]


def is_route_a(route_key: str) -> bool:
    return route_key.endswith("E16")


def analyze_run(run_dir: Path) -> dict:
    with gzip.open(run_dir / "decisions.jsonl.gz", "rt", encoding="utf-8") as f:
        recs = [json.loads(line) for line in f]
    theta = {}
    pre = defaultdict(list)
    noinfo = set()
    uturns = []
    timeline = defaultdict(list)
    for d in recs:
        theta.setdefault(d["veh"], d.get("theta"))
        if d["kind"] == "pre_x":
            pre[d["veh"]].append(d)
            a = next((v for k, v in d["info"].items() if is_route_a(k)), None)
            b = next((v for k, v in d["info"].items() if not is_route_a(k)), None)
            timeline[int(d["t"] // TIME_BIN) * TIME_BIN].append((a, b, d["t"] - d["info_time"]))
        elif d["kind"] == "pre_x_no_info":
            noinfo.add(d["veh"])
        elif d["kind"] == "post_x" and d.get("approved") and d.get("candidates"):
            if d["candidates"][0]["type"] == "uturn":
                uturns.append(d)
    changed = [next(x for x in v if x["approved"]) for v in pre.values() if any(x["approved"] for x in v)]
    kept = [v[-1] for v in pre.values() if not any(x["approved"] for x in v)]
    return {
        "theta_negative": sum(1 for t in theta.values() if t is not None and t < 0),
        "prex_with_info": len(pre),
        "prex_without_info": len(noinfo - set(pre)),
        "prex_changed": len(changed),
        "prex_changed_gain": [x["gain"] for x in changed],
        "prex_changed_theta": [x["theta"] for x in changed],
        "prex_changed_time": [x["t"] for x in changed],
        "prex_kept_gain": [x["gain"] for x in kept],
        "prex_kept_theta": [x["theta"] for x in kept],
        "prex_info_age": [x["t"] - x["info_time"] for v in pre.values() for x in v],
        "uturn": len(uturns),
        "uturn_gain": [x["gain"] for x in uturns],
        "uturn_theta": [x["theta"] for x in uturns],
        "uturn_theta_negative": sum(1 for x in uturns if x["theta"] < 0),
        "timeline": timeline,
    }


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("root")
    ap.add_argument("--out", required=True)
    a = ap.parse_args()
    root, out = Path(a.root).resolve(), Path(a.out)
    out.mkdir(parents=True, exist_ok=True)
    runs = find_runs(root)
    if not runs:
        print(f"[decisions] {root} に decisions.jsonl.gz がありません")
        return 0

    groups: dict[tuple, list[dict]] = defaultdict(list)
    for r in runs:
        groups[(r["layout"], r["condition"], r["variant"])].append(analyze_run(Path(r["run_dir"])))

    summary, timeline_rows = [], []
    for (layout, cond, var), rs in sorted(groups.items()):
        cat = lambda k: [x for r in rs for x in r[k]]
        summary.append({
            "layout": layout, "condition": cond, "variant": var, "n_runs": len(rs),
            "theta_negative_per_run": mean([r["theta_negative"] for r in rs]),
            "prex_with_info_per_run": mean([r["prex_with_info"] for r in rs]),
            "prex_without_info_per_run": mean([r["prex_without_info"] for r in rs]),
            "prex_changed_per_run": mean([r["prex_changed"] for r in rs]),
            "prex_changed_gain_median": med(cat("prex_changed_gain")),
            "prex_changed_theta_median": med(cat("prex_changed_theta")),
            "prex_changed_time_median": med(cat("prex_changed_time")),
            "prex_kept_gain_median": med(cat("prex_kept_gain")),
            "prex_kept_theta_median": med(cat("prex_kept_theta")),
            "prex_info_age_median": med(cat("prex_info_age")),
            "uturn_per_run": mean([r["uturn"] for r in rs]),
            "uturn_gain_median": med(cat("uturn_gain")),
            "uturn_theta_median": med(cat("uturn_theta")),
            "uturn_theta_negative_total": sum(r["uturn_theta_negative"] for r in rs),
        })
        bins = defaultdict(list)
        for r in rs:
            for k, v in r["timeline"].items():
                bins[k].extend(v)
        for k in sorted(bins):
            A = [x[0] for x in bins[k]]
            B = [x[1] for x in bins[k]]
            timeline_rows.append({
                "layout": layout, "condition": cond, "variant": var, "t_from": k, "t_to": k + TIME_BIN,
                "n_decisions": len(bins[k]), "route_a_median": med(A), "route_b_median": med(B),
                "gain_median": med([x - y for x, y in zip(A, B) if x is not None and y is not None]),
                "info_age_median": med([x[2] for x in bins[k]]),
            })

    for name, rows in (("decisions_summary.csv", summary), ("decisions_timeline.csv", timeline_rows)):
        with open(out / name, "w", newline="") as f:
            w = csv.DictWriter(f, fieldnames=list(rows[0]))
            w.writeheader()
            w.writerows(rows)

    lines = ["## 判断の記録(1 回あたりの平均台数・回数、値は中央値)", "",
             "| 条件 | 版 | 回数 | 分岐前で表あり | 表なしで通過 | 分岐前で変更 | 変更時の短縮 / θ | 見送り時の短縮 / θ | 情報の古さ | U ターン | U ターンの短縮 / θ | θ<0 の人 |",
             "| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |"]
    for s in summary:
        lines.append(
            f"| {s['condition']} | {s['variant']} | {s['n_runs']} | {s['prex_with_info_per_run']} | "
            f"{s['prex_without_info_per_run']} | {s['prex_changed_per_run']} | "
            f"{s['prex_changed_gain_median']} / {s['prex_changed_theta_median']} | "
            f"{s['prex_kept_gain_median']} / {s['prex_kept_theta_median']} | {s['prex_info_age_median']} s | "
            f"{s['uturn_per_run']} | {s['uturn_gain_median']} / {s['uturn_theta_median']} | "
            f"{s['theta_negative_per_run']} |")
    lines += ["", "- 分岐前の判断:経路 A の平均 − 経路 B の平均(短縮)> θ なら経路 B へ。U ターン:T_cur − U ターンの見積もり > θ。",
              "- 見送り時 = 分岐前で表を持って判断したが一度も変えなかった車の、最後の判断の値。",
              f"- 時刻ごとの表の値は decisions_timeline.csv({TIME_BIN} s ごと)。"]
    (out / "decisions.md").write_text("\n".join(lines) + "\n")
    print(f"[decisions] {len(runs)} runs -> {out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
