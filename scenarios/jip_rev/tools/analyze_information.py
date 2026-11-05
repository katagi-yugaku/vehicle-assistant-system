#!/usr/bin/env python3
"""
情報伝達の分析(M3:V2V only と DTN で、情報が届くまでにどれだけ遅れるか)。

    python tools/analyze_information.py RUN_DIR [RUN_DIR ...] --out OUT_DIR

RUN_DIR は information.jsonl.gz(log_information = true で runner が書く)と raw.json がある
1 回分のディレクトリ。runs/<実験ID> を渡すと、その下の r*/ をすべて使う。

見るもの(分岐前 = E0, E1, E20 にいて通信できる車両):
  1. 2 経路分の経路情報を持っている割合(時刻ごと)
  2. 持っている情報の古さ(時刻 − 情報の元の時刻)
  3. そのラウンドで、避難所から直接受け取った車と同じ連結成分にいる割合(= その場で最新情報が届く)
  4. 車両ごとの待ち時間:E0 に入った時刻(情報がまだ誰にもない時期なら、最初に避難所が
     2 経路分の情報を出した時刻)から、2 経路分の情報を持つまで。分岐点 X を通過するまでに
     受け取れなかった車も数える。
出力:information_summary.csv、information_timeseries.csv、information.png / .pdf、information.md
"""
from __future__ import annotations

import argparse
import csv
import gzip
import json
import statistics
from collections import defaultdict
from pathlib import Path

PRE_X = {"E0", "E1", "E20"}


def find_runs(paths):
    out = []
    for p in map(Path, paths):
        if (p / "information.jsonl.gz").exists():
            out.append(p)
        else:
            out.extend(sorted(q.parent for q in p.rglob("information.jsonl.gz")))
    return out


def label_of(run: Path) -> tuple[str, str, str]:
    """(条件, 通信方式, シード)。条件は run_experiment.py の配置 <構成>/<条件>/<版>/rNNN から取る。"""
    raw = json.loads((run / "raw.json").read_text())
    flags = raw.get("flags") or {}
    mode = flags.get("communication_mode") or "?"
    condition = run.parent.parent.name if run.name.startswith("r") else "-"
    return condition, mode, str(raw.get("seed"))


def analyze(run: Path):
    rows_t = []
    enter_e0: dict[str, float] = {}
    got_info: dict[str, float] = {}
    left_prex: dict[str, float] = {}
    t_avail = None
    last_seen_prex: dict[str, float] = {}
    with gzip.open(run / "information.jsonl.gz", "rt") as f:
        for line in f:
            rec = json.loads(line)
            t = rec["t"]
            direct_comps = set()
            any_direct2 = False
            for vid, edge, lp, x, y, comm, info_t, n, direct, comp in rec["v"]:
                if direct and n >= 2:
                    any_direct2 = True
                    if comp is not None:
                        direct_comps.add(comp)
            if t_avail is None:
                # round 単位でなければ(legacy)、誰かが 2 経路分を持った最初の時刻
                if any_direct2 or (rec.get("mode") == "legacy"
                                   and any(r[7] >= 2 for r in rec["v"])):
                    t_avail = t
            n_pre = n_have = n_conn = 0
            ages = []
            for vid, edge, lp, x, y, comm, info_t, n, direct, comp in rec["v"]:
                if edge in PRE_X:
                    last_seen_prex[vid] = t
                    if edge == "E0" and vid not in enter_e0:
                        enter_e0[vid] = t
                    if not comm:
                        continue
                    n_pre += 1
                    if n >= 2:
                        n_have += 1
                        got_info.setdefault(vid, t)
                        if info_t is not None:
                            ages.append(t - info_t)
                    if comp is not None and comp in direct_comps:
                        n_conn += 1
                else:
                    if vid in last_seen_prex and vid not in left_prex:
                        left_prex[vid] = t
            rows_t.append({
                "t": t, "n_prex": n_pre,
                "frac_have": n_have / n_pre if n_pre else None,
                "mean_age": statistics.fmean(ages) if ages else None,
                "frac_connected": n_conn / n_pre if n_pre else None,
                "n_components": len(rec.get("comps") or []),
            })
    waits, missed = [], 0
    for vid, t0 in enter_e0.items():
        if t_avail is None:
            continue
        start = max(t0, t_avail)
        t_got = got_info.get(vid)
        t_left = left_prex.get(vid)
        if t_left is not None and t_left <= start:
            continue           # 情報が出る前に X を通過した車は対象外
        if t_got is None or (t_left is not None and t_got > t_left):
            missed += 1        # X を通過するまでに受け取れなかった
            continue
        waits.append(max(0.0, t_got - start))
    return rows_t, waits, missed, t_avail


def _fmean(vals):
    vals = [v for v in vals if v is not None]
    return statistics.fmean(vals) if vals else None


def pct(vals, q):
    if not vals:
        return None
    s = sorted(vals)
    k = (len(s) - 1) * q
    lo, hi = int(k), min(int(k) + 1, len(s) - 1)
    return s[lo] + (s[hi] - s[lo]) * (k - lo)


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("runs", nargs="+")
    ap.add_argument("--out", required=True)
    args = ap.parse_args()
    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)

    summary, series = [], defaultdict(list)
    for run in find_runs(args.runs):
        condition, mode, seed = label_of(run)
        rows_t, waits, missed, t_avail = analyze(run)
        for r in rows_t:
            series[(condition, mode)].append({**r, "seed": seed})
        late = [r for r in rows_t if t_avail is not None and r["t"] >= t_avail and r["n_prex"]]
        summary.append({
            "run": str(run), "condition": condition, "mode": mode, "seed": seed, "t_info_available": t_avail,
            "n_vehicles_eval": len(waits) + missed,
            "missed_before_X": missed,
            "wait_mean": statistics.fmean(waits) if waits else None,
            "wait_median": pct(waits, 0.5), "wait_p90": pct(waits, 0.9), "wait_max": max(waits) if waits else None,
            "frac_have_prex_mean": _fmean([r["frac_have"] for r in late]),
            "age_prex_mean": _fmean([r["mean_age"] for r in late]),
            "frac_connected_prex_mean": _fmean([r["frac_connected"] for r in late]),
        })

    with open(out / "information_summary.csv", "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(summary[0]), lineterminator="\n")
        w.writeheader()
        w.writerows(summary)
    with open(out / "information_timeseries.csv", "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=["condition", "mode", "seed", "t", "n_prex", "frac_have", "mean_age",
                                          "frac_connected", "n_components"], lineterminator="\n")
        w.writeheader()
        for (condition, mode), rs in series.items():
            for r in rs:
                w.writerow({"condition": condition, "mode": mode, **r})

    def f1(x):
        return "" if x is None else f"{x:.1f}"

    def mean_of(rows, key, scale=1.0):
        vals = [r[key] for r in rows if r[key] is not None]
        return statistics.fmean(vals) * scale if vals else None

    lines = ["# 情報伝達の比較(分岐前 = E0/E1/E20 の通信可能な車両)", "",
             "条件 × 方式ごとに、各回の値を平均したもの(回ごとの値は information_summary.csv)。", "",
             "| 条件 | 方式 | 回数 | X までに受け取れず(台) | 対象(台) | 待ち 平均 | 待ち 中央 | 待ち 90% | 保有率 | 情報の古さ | 直結率 |",
             "| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |"]
    groups = defaultdict(list)
    for s_ in summary:
        groups[(s_["condition"], s_["mode"])].append(s_)
    for (condition, mode), rows in sorted(groups.items()):
        lines.append(f"| {condition} | {mode} | {len(rows)} | {f1(mean_of(rows, 'missed_before_X'))} | "
                     f"{f1(mean_of(rows, 'n_vehicles_eval'))} | {f1(mean_of(rows, 'wait_mean'))} | "
                     f"{f1(mean_of(rows, 'wait_median'))} | {f1(mean_of(rows, 'wait_p90'))} | "
                     f"{f1(mean_of(rows, 'frac_have_prex_mean', 100))}% | {f1(mean_of(rows, 'age_prex_mean'))} s | "
                     f"{f1(mean_of(rows, 'frac_connected_prex_mean', 100))}% |")
    lines += ["", "- 待ち:E0 に入った時刻(または避難所が最初に 2 経路分の情報を出した時刻の遅い方)から、"
              "2 経路分の情報を持つまで [s]。",
              "- 保有率・情報の古さ・直結率:情報が出た後の各ラウンドの平均。直結率 = そのラウンドに避難所から"
              "直接受け取った車と同じ連結成分にいる割合。"]
    (out / "information.md").write_text("\n".join(lines) + "\n")
    print("\n".join(lines))

    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except Exception:
        return 0
    styles = {"v2v_dtn": ("-", "Vehicular DTN"), "v2v_only": ("--", "V2V only"), "legacy": (":", "DTN (legacy, paper v1)")}
    keys = [("frac_have", "Share holding 2-route info"), ("mean_age", "Mean info age [s]"),
            ("frac_connected", "Share linked to a\nshelter-direct vehicle")]
    conditions = sorted({c for c, _ in series})
    for condition in conditions:
        fig, axes = plt.subplots(3, 1, figsize=(8, 9), sharex=True)
        for (c, mode), rs in sorted(series.items()):
            if c != condition:
                continue
            ls, lab = styles.get(mode, ("-.", mode))
            by_seed = defaultdict(list)
            for r in rs:
                by_seed[r["seed"]].append(r)
            # シードが多いと読めないので、最初の 3 シードだけ描く(数値は CSV にすべてある)
            for i, (seed, srs) in enumerate(sorted(by_seed.items(), key=lambda kv: int(kv[0]))[:3]):
                for ax, (k, _) in zip(axes, keys):
                    xs = [r["t"] for r in srs if r[k] is not None]
                    ys = [r[k] for r in srs if r[k] is not None]
                    ax.plot(xs, ys, linestyle=ls, color=f"C{i}", linewidth=1.3,
                            label=f"{lab} (seed {seed})")
        for ax, (_, title) in zip(axes, keys):
            ax.set_ylabel(title)
            ax.grid(alpha=0.3)
        axes[0].legend(fontsize=7, ncol=2)
        axes[-1].set_xlabel("Time [s]")
        fig.suptitle(f"Vehicles before divergence X (E0, E1, E20) — {condition}", fontsize=10)
        fig.tight_layout()
        suffix = "" if len(conditions) == 1 else f"_{condition}"
        for ext in ("png", "pdf"):
            fig.savefig(out / f"information{suffix}.{ext}", dpi=150)
        plt.close(fig)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
