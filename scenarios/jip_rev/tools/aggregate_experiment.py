#!/usr/bin/env python3
"""
実験の集計(JIP 26-S015f 改訂)。

    python tools/aggregate_experiment.py runs/<実験ID> [--out results/<実験ID>]

出力(--out、既定は scenarios/jip_rev/results/<実験ID>):
  runs.csv      1 回ごとの値(完了時刻、平均到着時刻、経路変更の内訳、E0 再生成回数など)
  summary.csv   構成 × 条件 × 版ごとの平均・標準偏差・95% 信頼区間(t 分布)
  paired.csv    [compare] がある場合:同じシードの組で treatment − baseline
  decision.json / decision.md   E4 の判断材料(差の大きさ、μθ の単調性、P の効果の向き)
  figs/completion_time.{png,pdf}
  manifest.json 実験の manifest に集計結果を追記したもの
"""
from __future__ import annotations

import argparse
import csv
import re
import datetime as dt
import json
import math
import statistics
import sys
from collections import defaultdict
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent


def t_quantile_975(df: int) -> float:
    try:
        from scipy.stats import t  # type: ignore
        return float(t.ppf(0.975, df))
    except Exception:
        pass
    z = 1.959963984540054   # Cornish–Fisher 展開(df >= 3 で誤差 < 1e-3)
    g1 = (z**3 + z) / 4
    g2 = (5 * z**5 + 16 * z**3 + 3 * z) / 96
    g3 = (3 * z**7 + 19 * z**5 + 17 * z**3 - 15 * z) / 384
    g4 = (79 * z**9 + 776 * z**7 + 1482 * z**5 - 1920 * z**3 - 945 * z) / 92160
    return z + g1 / df + g2 / df**2 + g3 / df**3 + g4 / df**4


def describe(values) -> dict:
    vals = [v for v in values if v is not None and math.isfinite(v)]
    n = len(vals)
    if n == 0:
        return {"n": 0, "mean": None, "sd": None, "ci95_low": None, "ci95_high": None}
    mean = statistics.fmean(vals)
    if n < 2:
        return {"n": n, "mean": mean, "sd": None, "ci95_low": None, "ci95_high": None}
    sd = statistics.stdev(vals)
    half = t_quantile_975(n - 1) * sd / math.sqrt(n)
    return {"n": n, "mean": mean, "sd": sd, "ci95_low": mean - half, "ci95_high": mean + half}


def fmt(x, nd=1):
    return "" if x is None else (f"{x:.{nd}f}" if isinstance(x, float) else str(x))


def read_status(path: Path) -> dict:
    out = {}
    if path.exists():
        for line in path.read_text().splitlines():
            k, _, v = line.partition("=")
            out[k] = v
    return out


def load_runs(run_root: Path, manifest: dict) -> list[dict]:
    rows = []
    with open(run_root / "tasks.tsv") as f:
        for t in csv.DictReader(f, delimiter="\t"):
            run_dir = Path(t["run_dir"])
            st = read_status(run_dir / "status.txt")
            cond = manifest["conditions"].get(t["condition"], {})
            row = {
                "task_id": int(t["task_id"]), "layout": t["layout"], "condition": t["condition"],
                "mu_theta": cond.get("mu_theta"), "p_follow": cond.get("p_follow"),
                "sigma_theta": cond.get("sigma_theta"),
                "variant": t["variant"], "seed": int(t["seed"]),
                "status": st.get("status", "MISSING"),
                "elapsed_s": float(st["elapsed_s"]) if st.get("elapsed_s") else None,
                "host": st.get("host", ""),
                "run_dir": t["run_dir"],
            }
            raw_path = run_dir / "raw.json"
            if row["status"] == "SUCCESS" and raw_path.exists():
                raw = json.loads(raw_path.read_text())
                s = raw.get("route_change_summary", {})
                by_reason, by_type = s.get("success_by_reason", {}), s.get("success_by_type", {})
                row.update({
                    "arrived": raw.get("arrived_count"), "num_vehicles": raw.get("num_vehicles"),
                    "completion_time": raw.get("completion_time"),
                    "mean_arrival_time": raw.get("mean_arrival_time"),
                    "route_changes": s.get("success_total", 0),
                    "rc_normalcy": by_reason.get("normalcy", 0),
                    "rc_majority": by_reason.get("majority", 0),
                    "rc_shelter_full": by_reason.get("shelter_full", 0),
                    "rc_uturn": by_type.get("uturn", 0),
                    "rc_forward": by_type.get("route_change", 0),
                    "rc_failed": s.get("failed_total", 0),
                    "e0_regeneration": s.get("e0_regeneration_count", 0),
                    "rc_on_e0": s.get("success_by_from_edge", {}).get("E0", 0),
                    "raw_seed": raw.get("seed"),
                })
                if row["raw_seed"] != row["seed"]:
                    row["status"] = "SEED_MISMATCH"
                if row["arrived"] != row["num_vehicles"]:
                    row["note"] = f"arrived {row['arrived']}/{row['num_vehicles']}"
            rows.append(row)
    return rows


# 論文(aggregate_schubert_logs.py)と同じ車両 ID の正規化: init/newveh/ped の派生 ID を元の番号にまとめる
ORIGINAL_VEH_ID_RE = re.compile(r"^(?:ped_)?(?:init|newveh)_Shelter[A-Za-z]+_\d+_(\d+)(?:_\d+)*$")


def original_vehicle_id(raw_id: str) -> str:
    m = ORIGINAL_VEH_ID_RE.match(str(raw_id).strip())
    return m.group(1) if m else str(raw_id)


def load_vehicle_arrivals(run_dir: Path) -> dict[str, list[float]]:
    """1 回分の到着時刻を、元の車両番号ごとにまとめる(論文の集計と同じ扱い)。"""
    raw = json.loads((run_dir / "raw.json").read_text())
    out: dict[str, list[float]] = defaultdict(list)
    for sumo_id, t in (raw.get("arrival_time_by_sumo_id") or {}).items():
        out[original_vehicle_id(sumo_id)].append(float(t))
    return out


METRICS = ["completion_time", "mean_arrival_time", "route_changes", "rc_normalcy", "rc_majority",
           "rc_uturn", "rc_forward", "e0_regeneration", "rc_on_e0"]


def paper_completion(rs) -> tuple[float | None, float | None]:
    """
    論文の Fig.8–11 と同じ定義:車両ごとに全回の到着時刻を平均し、その最大値(と平均)。
    各回の最後の到着時刻の平均(completion_time)より小さくなる。
    """
    acc: dict[str, list[float]] = defaultdict(list)
    for r in rs:
        if r["status"] != "SUCCESS" or not r.get("run_dir"):
            continue
        for vid, vals in load_vehicle_arrivals(Path(r["run_dir"])).items():
            acc[vid].extend(vals)
    means = [statistics.fmean(v) for v in acc.values() if v]
    if not means:
        return None, None
    return max(means), statistics.fmean(means)


def summarize(rows):
    groups = defaultdict(list)
    for r in rows:
        groups[(r["layout"], r["condition"], r["variant"])].append(r)
    out = []
    for (layout, cond, variant), rs in sorted(groups.items()):
        ok = [r for r in rs if r["status"] == "SUCCESS"]
        rec = {"layout": layout, "condition": cond, "mu_theta": rs[0]["mu_theta"],
               "sigma_theta": rs[0].get("sigma_theta"),
               "p_follow": rs[0]["p_follow"], "variant": variant, "n_planned": len(rs),
               "n_success": len(ok), "n_not_all_arrived": sum(1 for r in ok if r.get("note"))}
        rec["completion_time_paperdef"], rec["mean_arrival_time_paperdef"] = paper_completion(ok)
        for key in METRICS:
            d = describe([r.get(key) for r in ok])
            rec[f"{key}_mean"] = d["mean"]
            if key in ("completion_time", "mean_arrival_time"):
                rec[f"{key}_sd"] = d["sd"]
                rec[f"{key}_ci95_low"] = d["ci95_low"]
                rec[f"{key}_ci95_high"] = d["ci95_high"]
        out.append(rec)
    return out


def paired(rows, baseline, treatment, threshold):
    idx = {(r["layout"], r["condition"], r["variant"], r["seed"]): r for r in rows if r["status"] == "SUCCESS"}
    out = []
    for layout, cond in sorted({(r["layout"], r["condition"]) for r in rows}):
        meta = next(r for r in rows if r["layout"] == layout and r["condition"] == cond)
        seeds = sorted({r["seed"] for r in rows if r["layout"] == layout and r["condition"] == cond})
        pairs = [(idx.get((layout, cond, baseline, s)), idx.get((layout, cond, treatment, s))) for s in seeds]
        pairs = [(a, b) for a, b in pairs if a and b]
        rec = {"layout": layout, "condition": cond, "mu_theta": meta["mu_theta"],
               "p_follow": meta["p_follow"], "n_pairs": len(pairs)}
        for key in ("completion_time", "mean_arrival_time", "route_changes", "rc_uturn", "rc_majority"):
            d = describe([b[key] - a[key] for a, b in pairs if a.get(key) is not None and b.get(key) is not None])
            rec[f"diff_{key}_mean"] = d["mean"]
            rec[f"diff_{key}_ci95_low"] = d["ci95_low"]
            rec[f"diff_{key}_ci95_high"] = d["ci95_high"]
        m = rec["diff_completion_time_mean"]
        rec["exceeds_e4_threshold"] = bool(m is not None and abs(m) >= threshold)
        out.append(rec)
    return out


def direction_checks(summary, variants):
    table = {(s["layout"], s["mu_theta"], s["p_follow"], s["variant"]): s["completion_time_mean"] for s in summary}
    mus = sorted({s["mu_theta"] for s in summary if s["mu_theta"] is not None})
    ps = sorted({s["p_follow"] for s in summary if s["p_follow"] is not None})
    res = {}
    for layout in sorted({s["layout"] for s in summary}):
        for v in variants:
            mu_effect, p_effect = {}, {}
            for p in ps:
                vals = [table.get((layout, mu, p, v)) for mu in mus]
                if None not in vals and len(vals) >= 2:
                    mu_effect[f"P={p}"] = {"values": vals, "increasing": all(a < b for a, b in zip(vals, vals[1:]))}
            for mu in mus:
                vals = [table.get((layout, mu, p, v)) for p in ps]
                if None not in vals and len(vals) >= 2:
                    diff = vals[-1] - vals[0]
                    p_effect[f"mu={mu}"] = {"values": vals, "diff_high_minus_low_P": diff,
                                            "sign": "+" if diff > 0 else ("-" if diff < 0 else "0")}
            res[f"{layout}/{v}"] = {"mu_effect": mu_effect, "p_effect": p_effect}
    return res


def decide(prs, checks, baseline, treatment, threshold):
    reasons = [f"{r['layout']} μθ={r['mu_theta']:g}, P={r['p_follow']:g}: 完了時刻の平均が "
               f"{r['diff_completion_time_mean']:+.1f} s 動いた(基準 {threshold:.0f} s)"
               for r in prs if r["exceeds_e4_threshold"]]
    for layout in sorted({k.split("/")[0] for k in checks}):
        b, t = checks.get(f"{layout}/{baseline}"), checks.get(f"{layout}/{treatment}")
        if not b or not t:
            continue
        for key, info in b["mu_effect"].items():
            if key in t["mu_effect"] and info["increasing"] != t["mu_effect"][key]["increasing"]:
                reasons.append(f"{layout} {key}: μθ に対する単調性が変わった")
        for key, info in b["p_effect"].items():
            if key in t["p_effect"] and info["sign"] != t["p_effect"][key]["sign"]:
                reasons.append(f"{layout} {key}: P の効果の向きが変わった({info['sign']} → {t['p_effect'][key]['sign']})")
    return {"e4_recommended": bool(reasons), "reasons": reasons}


def make_figures(summary, out: Path, title: str):
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except Exception as exc:
        print(f"[aggregate] matplotlib unavailable: {exc}")
        return []
    figs = out / "figs"
    figs.mkdir(parents=True, exist_ok=True)
    layouts = sorted({s["layout"] for s in summary})
    variants = sorted({s["variant"] for s in summary})
    conds = sorted({(s["mu_theta"], s["p_follow"], s["condition"]) for s in summary})
    fig, axes = plt.subplots(1, len(layouts), figsize=(5.2 * len(layouts), 4.0), squeeze=False, sharey=True)
    markers = ["o", "s", "^", "D"]
    for ax, layout in zip(axes[0], layouts):
        for vi, v in enumerate(variants):
            xs, ys, lo, hi = [], [], [], []
            for ci, (mu, p, cond) in enumerate(conds):
                s = next((s for s in summary if s["layout"] == layout and s["condition"] == cond and s["variant"] == v), None)
                if not s or s["completion_time_mean"] is None:
                    continue
                m = s["completion_time_mean"]
                l = s["completion_time_ci95_low"] if s["completion_time_ci95_low"] is not None else m
                h = s["completion_time_ci95_high"] if s["completion_time_ci95_high"] is not None else m
                xs.append(ci + (vi - (len(variants) - 1) / 2) * 0.18)
                ys.append(m); lo.append(m - l); hi.append(h - m)
            ax.errorbar(xs, ys, yerr=[lo, hi], fmt=markers[vi % len(markers)], capsize=3, label=v)
        ax.set_xticks(range(len(conds)))
        sig = {s["condition"]: s.get("sigma_theta") for s in summary}
        ax.set_xticklabels([f"μθ={mu:g}\nP={p:g}" + (f"\nσθ={sig[c]:g}" if sig.get(c) is not None else "")
                            for mu, p, c in conds], fontsize=7 if len(conds) > 5 else 10)
        ax.set_title({"1-1-2": "1-1-2 (2 routes)", "1-1-4": "1-1-4 (4 routes)"}.get(layout, layout))
        ax.grid(axis="y", alpha=0.3)
    axes[0][0].set_ylabel("Evacuation completion time [s]\n(mean ± 95% CI)")
    axes[0][-1].legend(title="variant")
    fig.suptitle(title, fontsize=9)
    fig.tight_layout()
    paths = []
    for ext in ("png", "pdf"):
        p = figs / f"completion_time.{ext}"
        fig.savefig(p, dpi=150)
        paths.append(str(p))
    plt.close(fig)
    return paths


def write_csv(path: Path, rows):
    if not rows:
        path.write_text("")
        return
    fields = list(dict.fromkeys(k for r in rows for k in r))
    with open(path, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=fields, lineterminator="\n")
        w.writeheader()
        for r in rows:
            w.writerow({k: (round(v, 3) if isinstance(v, float) else v) for k, v in r.items()})


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("run_root")
    ap.add_argument("--out")
    args = ap.parse_args()
    run_root = Path(args.run_root).resolve()
    manifest = json.loads((run_root / "manifest.json").read_text())
    out = Path(args.out).resolve() if args.out else ROOT / "results" / manifest["experiment_id"]
    out.mkdir(parents=True, exist_ok=True)

    rows = load_runs(run_root, manifest)
    summary = summarize(rows)
    write_csv(out / "runs.csv", rows)
    write_csv(out / "summary.csv", summary)

    comp = manifest.get("compare") or {}
    decision = None
    if comp.get("baseline") and comp.get("treatment"):
        threshold = float(comp.get("e4_threshold_s", 40.0))
        prs = paired(rows, comp["baseline"], comp["treatment"], threshold)
        write_csv(out / "paired.csv", prs)
        checks = direction_checks(summary, [comp["baseline"], comp["treatment"]])
        decision = decide(prs, checks, comp["baseline"], comp["treatment"], threshold)
        decision.update({"checks": checks, "paired": prs})
        (out / "decision.json").write_text(json.dumps(decision, indent=2, ensure_ascii=False))

    n_ok = sum(r["status"] == "SUCCESS" for r in rows)
    failed = [r for r in rows if r["status"] != "SUCCESS"]
    lines = [f"# {manifest['experiment_id']}", "",
             f"- 成功 {n_ok} / 予定 {manifest['planned_runs']}",
             f"- コード {manifest['code_commit'][:7]}{' (dirty)' if manifest.get('code_dirty') else ''}", "",
             "## 完了時刻(平均 [95% CI]、s)と経路変更(平均回数)", "",
             "完了時刻 = 各回の最後の到着時刻の平均。論文定義 = 車両ごとに到着時刻を平均した値の最大値"
             "(論文の Fig.8–11 と同じ集計。各回の値がないので信頼区間は出せない)。", "",
             "| 構成 | 条件 | 版 | n | 完了時刻 | 論文定義 | 平均到着 | 経路変更 | 同調 | U ターン | E0 再生成 |",
             "| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |"]
    for s in summary:
        lines.append(
            f"| {s['layout']} | μθ={s['mu_theta']:g}, P={s['p_follow']:g} | {s['variant']} | {s['n_success']} | "
            f"{fmt(s['completion_time_mean'])} [{fmt(s['completion_time_ci95_low'])}, {fmt(s['completion_time_ci95_high'])}] | "
            f"{fmt(s['completion_time_paperdef'])} | {fmt(s['mean_arrival_time_mean'])} | {fmt(s['route_changes_mean'])} | {fmt(s['rc_majority_mean'])} | "
            f"{fmt(s['rc_uturn_mean'])} | {fmt(s['e0_regeneration_mean'])} |")
    if decision is not None:
        lines += ["", f"## {comp['treatment']} − {comp['baseline']}(同じシードの組)", "",
                  "| 構成 | 条件 | 組 | 完了時刻の差 [95% CI] | 基準超え |", "| --- | --- | --- | --- | --- |"]
        for r in decision["paired"]:
            lines.append(
                f"| {r['layout']} | μθ={r['mu_theta']:g}, P={r['p_follow']:g} | {r['n_pairs']} | "
                f"{fmt(r['diff_completion_time_mean'])} [{fmt(r['diff_completion_time_ci95_low'])}, "
                f"{fmt(r['diff_completion_time_ci95_high'])}] | {'yes' if r['exceeds_e4_threshold'] else ''} |")
        lines += ["", "論文定義の完了時刻の差(treatment − baseline):"]
        pd = {(s["layout"], s["condition"], s["variant"]): s["completion_time_paperdef"] for s in summary}
        for r in decision["paired"]:
            a = pd.get((r["layout"], r["condition"], comp["baseline"]))
            b = pd.get((r["layout"], r["condition"], comp["treatment"]))
            if a is not None and b is not None:
                lines.append(f"- {r['layout']} μθ={r['mu_theta']:g}, P={r['p_follow']:g}: {b - a:+.1f} s")
        lines += ["", "## E4(全条件の再実行)の判断材料(最終判断は著者)", ""]
        lines += [f"- {x}" for x in decision["reasons"]] or ["- 基準に当たった項目はなし"]
    if failed:
        lines += ["", "## 失敗した実行", ""] + [
            f"- task {r['task_id']}: {r['layout']} {r['condition']} {r['variant']} seed {r['seed']} → {r['status']}"
            for r in failed[:50]]
    (out / "decision.md").write_text("\n".join(lines) + "\n")

    figs = make_figures(summary, out, manifest["experiment_id"])
    manifest["aggregate"] = {"aggregated_at": dt.datetime.now().isoformat(timespec="seconds"),
                             "n_success": n_ok, "n_failed": len(failed), "results_dir": str(out),
                             "figures": figs,
                             "e4_recommended": decision["e4_recommended"] if decision else None}
    (out / "manifest.json").write_text(json.dumps(manifest, indent=2, ensure_ascii=False))
    print("\n".join(lines))
    return 0 if not failed else 3


if __name__ == "__main__":
    sys.exit(main())
