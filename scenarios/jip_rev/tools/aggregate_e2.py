#!/usr/bin/env python3
"""
E2(通信方式の比較、M3 / Fig.12 の差し替え)の集計。

    python tools/aggregate_e2.py runs/<実験ID> [--out results/<実験ID>]

実験定義(experiments/E2.toml)に aggregator = "aggregate_e2.py" と書くと、
finalize_experiment.sh が aggregate_experiment.py の代わりにこれを呼ぶ。

aggregate_experiment.py との違い:
  - E4 の判定はしない(40 s 基準や「E4 推奨」は出さない)
  - [compare] pairs に書いた複数の組について、同じシードどうしの差(後 − 前)を出す
      pairs = [["v2v_only", "v2v_dtn"], ["none", "v2v_dtn"], ["none", "v2v_only"]]
    pairs がなければ baseline / treatment の 1 組として扱う
  - 組ごとに「全条件で同じ向きか」をまとめる(論文の「DTN がすべての設定で短い」の確認)
  - Fig.12 の下書き:車両ごとに全シードの到着時刻を平均した値の CDF(論文 Fig.12 と同じ定義)

出力(--out、既定は scenarios/jip_rev/results/<実験ID>):
  runs.csv / summary.csv          aggregate_experiment.py と同じ
  paired.csv                      組 × 構成 × 条件ごとの差(平均・95% CI)
  vehicle_mean_arrival.csv        図を描き直すための値(車両ごとの平均到着時刻)
  decision.json / decision.md     E2 の結果のまとめ(メール本文にも使われる)
  figs/completion_time.{png,pdf}  完了時刻(平均 ± 95% CI)
  figs/arrival_cdf.{png,pdf}      Fig.12 の下書き(全条件を 1 枚に重ねたもの)
  figs/arrival_cdf_by_condition.{png,pdf}  条件ごとに分けたもの
  manifest.json                   実験の manifest に集計結果を追記したもの
"""
from __future__ import annotations

import argparse
import datetime as dt
import json
import statistics
import sys
from collections import defaultdict
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from aggregate_experiment import (  # noqa: E402  共通部分は E3 と同じものを使う
    ROOT,
    describe,
    fmt,
    load_runs,
    load_vehicle_arrivals,
    make_figures,
    summarize,
    write_csv,
)

PAIR_METRICS = ("completion_time", "mean_arrival_time", "route_changes", "rc_majority", "rc_uturn")
LINESTYLES = {"v2v_dtn": "-", "dtn": "-", "v2v_only": "--", "none": ":", "no_comm": ":"}
COND_COLORS = ["#1f77b4", "#ff7f0e", "#2ca02c", "#d62728", "#9467bd", "#8c564b",
               "#e377c2", "#7f7f7f", "#bcbd22", "#17becf"]


# ---------------------------------------------------------------- 比べる組
def resolve_pairs(comp: dict, variants: list[str]) -> list[tuple[str, str]]:
    pairs = comp.get("pairs")
    if not pairs and comp.get("baseline") and comp.get("treatment"):
        pairs = [[comp["baseline"], comp["treatment"]]]
    out = []
    for p in pairs or []:
        if len(p) != 2:
            sys.exit(f"ERROR: [compare] pairs の要素は 2 つの版名にしてください: {p!r}")
        a, b = str(p[0]), str(p[1])
        missing = [v for v in (a, b) if v not in variants]
        if missing:
            # --variants で版を絞った試験では組の片方がないことがある。その組だけ飛ばす。
            print(f"[aggregate_e2] 組 {a} → {b} は版 {missing} がないので飛ばします(variants={variants})")
            continue
        out.append((a, b))
    return out


def cond_label(rec) -> str:
    """条件の表示名。σθ が分かれば入れる(μθ が同じで σθ が違う条件を区別するため)。"""
    text = f"μθ={rec['mu_theta']:g}"
    if rec.get("sigma_theta") is not None:
        text += f", σθ={rec['sigma_theta']:g}"
    return text + f", P={rec['p_follow']:g}"


def label_of(variant: str, labels: dict) -> str:
    return str(labels.get(variant, variant))


# ---------------------------------------------------------------- 同じシードの差
def verdict(lo, hi) -> str:
    if lo is None or hi is None:
        return "不明(組が足りない)"
    if hi < 0:
        return "短い"
    if lo > 0:
        return "長い"
    return "差ははっきりしない"


def paired_multi(rows, pairs, labels):
    idx = {(r["layout"], r["condition"], r["variant"], r["seed"]): r for r in rows if r["status"] == "SUCCESS"}
    out = []
    for base, treat in pairs:
        for layout, cond in sorted({(r["layout"], r["condition"]) for r in rows}):
            meta = next(r for r in rows if r["layout"] == layout and r["condition"] == cond)
            seeds = sorted({r["seed"] for r in rows if r["layout"] == layout and r["condition"] == cond})
            prs = [(idx.get((layout, cond, base, s)), idx.get((layout, cond, treat, s))) for s in seeds]
            prs = [(a, b) for a, b in prs if a and b]
            rec = {"pair": f"{treat} - {base}", "baseline": base, "treatment": treat,
                   "label": f"{label_of(treat, labels)} − {label_of(base, labels)}",
                   "layout": layout, "condition": cond, "mu_theta": meta["mu_theta"],
                   "sigma_theta": meta.get("sigma_theta"),
                   "p_follow": meta["p_follow"], "n_pairs": len(prs),
                   # 到着が全員そろわなかった回を含む組の数(終了時刻で打ち切られた可能性)
                   "n_pairs_incomplete": sum(1 for a, b in prs if a.get("note") or b.get("note"))}
            for key in PAIR_METRICS:
                d = describe([b[key] - a[key] for a, b in prs
                              if a.get(key) is not None and b.get(key) is not None])
                rec[f"diff_{key}_mean"] = d["mean"]
                rec[f"diff_{key}_ci95_low"] = d["ci95_low"]
                rec[f"diff_{key}_ci95_high"] = d["ci95_high"]
            rec["completion_time_verdict"] = verdict(rec["diff_completion_time_ci95_low"],
                                                     rec["diff_completion_time_ci95_high"])
            out.append(rec)
    return out


def consistency(prs, summary, pairs):
    """組 × 構成ごとに、完了時刻の差の向きが全条件でそろっているか。"""
    pd = {(s["layout"], s["condition"], s["variant"]): s["completion_time_paperdef"] for s in summary}
    res = []
    for base, treat in pairs:
        for layout in sorted({r["layout"] for r in prs}):
            rs = [r for r in prs if r["baseline"] == base and r["treatment"] == treat and r["layout"] == layout]
            means = [r["diff_completion_time_mean"] for r in rs]
            paper = []
            for r in rs:
                a, b = pd.get((layout, r["condition"], base)), pd.get((layout, r["condition"], treat))
                paper.append(None if a is None or b is None else b - a)
            known = [m for m in means if m is not None]
            res.append({
                "pair": f"{treat} - {base}", "layout": layout, "n_conditions": len(rs),
                "all_shorter_mean": bool(known) and len(known) == len(rs) and all(m < 0 for m in known),
                "all_shorter_ci": bool(rs) and all(r["completion_time_verdict"] == "短い" for r in rs),
                "all_shorter_paperdef": bool(rs) and all(p is not None and p < 0 for p in paper),
                "verdicts": {r["condition"]: r["completion_time_verdict"] for r in rs},
            })
    return res


# ---------------------------------------------------------------- Fig.12 の下書き
def vehicle_means(rows):
    """(構成, 条件, 版) ごとに、車両ごとに全シードの到着時刻を平均する(論文 Fig.12 と同じ定義)。"""
    acc: dict[tuple, dict[str, list[float]]] = defaultdict(lambda: defaultdict(list))
    for r in rows:
        if r["status"] != "SUCCESS":
            continue
        for vid, vals in load_vehicle_arrivals(Path(r["run_dir"])).items():
            acc[(r["layout"], r["condition"], r["variant"])][vid].extend(vals)
    out = {}
    for key, by_vid in acc.items():
        out[key] = {vid: (statistics.fmean(v), len(v)) for vid, v in by_vid.items() if v}
    return out


def make_cdf_figures(vm, summary, out: Path, title: str, labels: dict, variants: list[str]):
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        import numpy as np
    except Exception as exc:
        print(f"[aggregate_e2] matplotlib unavailable: {exc}")
        return []
    figs = out / "figs"
    figs.mkdir(parents=True, exist_ok=True)
    conds = sorted({(s["mu_theta"], s["p_follow"], s["condition"]) for s in summary})
    clabel = {s["condition"]: cond_label(s) for s in summary}
    color = {c[2]: COND_COLORS[i % len(COND_COLORS)] for i, c in enumerate(conds)}
    other_styles = iter(["-.", (0, (1, 1)), (0, (5, 1))])
    style = {v: LINESTYLES.get(v) or next(other_styles, "-") for v in variants}
    layouts = sorted({s["layout"] for s in summary})
    paths = []

    def curve(ax, layout, cond, v, mu, p, with_cond_label):
        vals = vm.get((layout, cond, v))
        if not vals:
            return
        xs = np.sort(np.array([m for m, _ in vals.values()]))
        ys = np.arange(1, xs.size + 1) / xs.size
        lab = label_of(v, labels)
        if with_cond_label:
            lab = f"{clabel[cond]} / {lab}"
        ax.plot(xs, ys, color=color[cond], linestyle=style[v], linewidth=2.0, label=lab)

    # 1) 全条件を 1 枚に重ねる(論文 Fig.12 と同じ形)
    fig, axes = plt.subplots(1, len(layouts), figsize=(7.5 * len(layouts), 4.8), squeeze=False)
    for ax, layout in zip(axes[0], layouts):
        for mu, p, cond in conds:
            for v in variants:
                curve(ax, layout, cond, v, mu, p, True)
        ax.set_xlabel("Arrival time [s] (per-vehicle mean over seeds)")
        ax.set_ylabel("CDF")
        ax.set_ylim(0, 1.0)
        ax.grid(alpha=0.3)
        ax.set_title(layout)
        ax.legend(fontsize=7, ncol=1 if len(conds) * len(variants) <= 8 else 2)
    fig.suptitle(title, fontsize=9)
    fig.tight_layout()
    for ext in ("png", "pdf"):
        pth = figs / f"arrival_cdf.{ext}"
        fig.savefig(pth, dpi=150)
        paths.append(str(pth))
    plt.close(fig)

    # 2) 条件ごとに分ける(方式の違いを読みやすくする)
    ncol = min(len(conds), 2)
    nrow = (len(conds) + ncol - 1) // ncol
    for layout in layouts:
        fig, axes = plt.subplots(nrow, ncol, figsize=(5.2 * ncol, 3.8 * nrow), squeeze=False,
                                 sharex=True, sharey=True)
        for i, (mu, p, cond) in enumerate(conds):
            ax = axes[i // ncol][i % ncol]
            for v in variants:
                curve(ax, layout, cond, v, mu, p, False)
            ax.set_title(clabel[cond], fontsize=10)
            ax.grid(alpha=0.3)
            ax.set_ylim(0, 1.0)
            ax.legend(fontsize=8)
        for j in range(len(conds), nrow * ncol):
            axes[j // ncol][j % ncol].axis("off")
        for ax in axes[-1]:
            ax.set_xlabel("Arrival time [s]")
        for row in axes:
            row[0].set_ylabel("CDF")
        fig.suptitle(f"{title}  [{layout}]", fontsize=9)
        fig.tight_layout()
        suffix = "" if len(layouts) == 1 else f"_{layout}"
        for ext in ("png", "pdf"):
            pth = figs / f"arrival_cdf_by_condition{suffix}.{ext}"
            fig.savefig(pth, dpi=150)
            paths.append(str(pth))
        plt.close(fig)
    return paths


# ---------------------------------------------------------------- main
def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("run_root")
    ap.add_argument("--out")
    args = ap.parse_args()
    run_root = Path(args.run_root).resolve()
    manifest = json.loads((run_root / "manifest.json").read_text())
    out = Path(args.out).resolve() if args.out else ROOT / "results" / manifest["experiment_id"]
    out.mkdir(parents=True, exist_ok=True)

    comp = manifest.get("compare") or {}
    labels = comp.get("labels") or {}
    variants = list(manifest["variants"].keys())
    pairs = resolve_pairs(comp, variants)

    rows = load_runs(run_root, manifest)
    summary = summarize(rows)
    write_csv(out / "runs.csv", rows)
    write_csv(out / "summary.csv", summary)

    prs = paired_multi(rows, pairs, labels)
    write_csv(out / "paired.csv", prs)
    cons = consistency(prs, summary, pairs)

    vm = vehicle_means(rows)
    write_csv(out / "vehicle_mean_arrival.csv", [
        {"layout": k[0], "condition": k[1], "variant": k[2], "vehicle": vid,
         "mean_arrival_time": m, "n_arrivals": n}
        for k, by_vid in sorted(vm.items()) for vid, (m, n) in sorted(by_vid.items(), key=lambda x: x[1][0])])

    n_ok = sum(r["status"] == "SUCCESS" for r in rows)
    failed = [r for r in rows if r["status"] != "SUCCESS"]
    incomplete = [s for s in summary if s["n_not_all_arrived"]]
    end_times = sorted({str(f.get("end_simulation_time")) for f in manifest["variants"].values()}
                       | ({str(manifest["extra_flags"]["end_simulation_time"])}
                          if "end_simulation_time" in (manifest.get("extra_flags") or {}) else set()))

    run_args = manifest.get("run_args") or {}

    def vdesc(v):
        ra = run_args.get(v, {})
        mode = (manifest["variants"].get(v) or {}).get("communication_mode", "legacy")
        rate = ra.get("v2v_rate", manifest.get("v2v_rate"))
        return f"{v} = {label_of(v, labels)}(communication_mode={mode}, v2v_rate={rate})"
    vlabel = "; ".join(vdesc(v) for v in variants)
    lines = [f"# {manifest['experiment_id']}(E2:通信方式の比較)", "",
             f"- 成功 {n_ok} / 予定 {manifest['planned_runs']}",
             f"- コード {manifest['code_commit'][:7]}{' (dirty)' if manifest.get('code_dirty') else ''}",
             f"- 版: {vlabel}",
             f"- 終了時刻(設定): {', '.join(end_times)}", ""]
    if incomplete:
        lines += ["**注意:到着がそろわなかった回があります(終了時刻で打ち切られた可能性)。"
                  "その回の完了時刻は実際より短く出ます。**", ""]
        lines += [f"- {s['layout']} {cond_label(s)} {s['variant']}: "
                  f"{s['n_not_all_arrived']} / {s['n_success']} 回" for s in incomplete]
        lines += [""]
    lines += ["## 完了時刻(平均 [95% CI]、s)と経路変更(平均回数)", "",
              "完了時刻 = 各回の最後の到着時刻の平均。論文定義 = 車両ごとに全シードの到着時刻を平均した値の最大値"
              "(論文 Fig.12 の CDF の右端と同じ)。", "",
              "| 構成 | 条件 | 版 | n | 完了時刻 | 論文定義 | 平均到着 | 経路変更 | 同調 | U ターン |",
              "| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |"]
    for s in summary:
        lines.append(
            f"| {s['layout']} | {cond_label(s)} | {label_of(s['variant'], labels)} | "
            f"{s['n_success']} | {fmt(s['completion_time_mean'])} [{fmt(s['completion_time_ci95_low'])}, "
            f"{fmt(s['completion_time_ci95_high'])}] | {fmt(s['completion_time_paperdef'])} | "
            f"{fmt(s['mean_arrival_time_mean'])} | {fmt(s['route_changes_mean'])} | {fmt(s['rc_majority_mean'])} | "
            f"{fmt(s['rc_uturn_mean'])} |")

    pd = {(s["layout"], s["condition"], s["variant"]): s["completion_time_paperdef"] for s in summary}
    for base, treat in pairs:
        rs = [r for r in prs if r["baseline"] == base and r["treatment"] == treat]
        lab = f"{label_of(treat, labels)} − {label_of(base, labels)}"
        lines += ["", f"## {lab}(同じシードの組。負なら {label_of(treat, labels)} の方が短い)", "",
                  "| 構成 | 条件 | 組 | 完了時刻の差 [95% CI] | 判定 | 論文定義の差 | 平均到着の差 [95% CI] |",
                  "| --- | --- | --- | --- | --- | --- | --- |"]
        for r in rs:
            a, b = pd.get((r["layout"], r["condition"], base)), pd.get((r["layout"], r["condition"], treat))
            lines.append(
                f"| {r['layout']} | {cond_label(r)} | {r['n_pairs']} | "
                f"{fmt(r['diff_completion_time_mean'])} [{fmt(r['diff_completion_time_ci95_low'])}, "
                f"{fmt(r['diff_completion_time_ci95_high'])}] | {r['completion_time_verdict']} | "
                f"{'' if a is None or b is None else f'{b - a:+.1f}'} | "
                f"{fmt(r['diff_mean_arrival_time_mean'])} [{fmt(r['diff_mean_arrival_time_ci95_low'])}, "
                f"{fmt(r['diff_mean_arrival_time_ci95_high'])}] |")

    lines += ["", "## 全条件で同じ向きか(論文 5.3.2 節・6 章の「DTN がすべての設定で短い」の確認)", ""]
    for c in cons:
        a, b = c["pair"].split(" - ")[1], c["pair"].split(" - ")[0]
        lines.append(
            f"- {c['layout']} {label_of(b, labels)} − {label_of(a, labels)}({c['n_conditions']} 条件): "
            f"平均で全条件短い = {'yes' if c['all_shorter_mean'] else 'no'}、"
            f"95% CI で全条件短い = {'yes' if c['all_shorter_ci'] else 'no'}、"
            f"論文定義で全条件短い = {'yes' if c['all_shorter_paperdef'] else 'no'}")
    if failed:
        lines += ["", "## 失敗した実行", ""] + [
            f"- task {r['task_id']}: {r['layout']} {r['condition']} {r['variant']} seed {r['seed']} → {r['status']}"
            for r in failed[:50]]
    (out / "decision.md").write_text("\n".join(lines) + "\n")

    decision = {"kind": "E2", "pairs": [list(p) for p in pairs], "labels": labels,
                "consistency": cons, "paired": prs,
                "incomplete_groups": [{k: s[k] for k in ("layout", "condition", "variant", "n_not_all_arrived")}
                                      for s in incomplete]}
    (out / "decision.json").write_text(json.dumps(decision, indent=2, ensure_ascii=False))

    title = manifest["experiment_id"]
    figs = make_figures(summary, out, title) + make_cdf_figures(vm, summary, out, title, labels, variants)
    manifest["aggregate"] = {"aggregated_at": dt.datetime.now().isoformat(timespec="seconds"),
                             "aggregator": "aggregate_e2.py",
                             "n_success": n_ok, "n_failed": len(failed), "results_dir": str(out),
                             "figures": figs, "e4_recommended": None}
    (out / "manifest.json").write_text(json.dumps(manifest, indent=2, ensure_ascii=False))
    print("\n".join(lines))
    return 0 if not failed else 3


if __name__ == "__main__":
    sys.exit(main())
