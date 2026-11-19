#!/usr/bin/env python3
"""
μθ × P の格子実験の補助図(集計結果の CSV から作る。シミュレーションは使わない)。

    python3 tools/plot_grid_figures.py results/<実験ID>

作るもの(構成ごと。μθ と P がそれぞれ 2 値以上あるときだけ)
  figs/grid_completion_lines  完了時刻の平均 ± 95% CI。横軸 μθ、線 = P、パネル = 版
  figs/grid_cdf               到着時刻の CDF(車両ごとに全シードで平均。論文 Fig.12 と同じ定義)を
                              ヒートマップと同じ配置(列 = μθ、行 = P、上が大きい P)で。各パネルに全版
  figs/grid_information       分岐前の車への伝達(X までに受け取れなかった台数、待ち時間の平均)。
                              横軸 μθ、線 = 版(P はまとめて平均、誤差棒は回の間の 95% CI)
入力: summary.csv、vehicle_mean_arrival.csv、information/information_summary.csv(あるものだけ使う)
"""
from __future__ import annotations

import argparse
import csv
import math
from collections import defaultdict
from pathlib import Path

STYLES = {"v2v_only": "--", "v2v_dtn": "-", "none": ":"}


def fnum(x):
    try:
        return float(x)
    except (TypeError, ValueError):
        return None


def read_csv(path: Path) -> list[dict]:
    if not path.is_file():
        return []
    with open(path) as f:
        return list(csv.DictReader(f))


def ci95(xs):
    xs = [x for x in xs if x is not None]
    if not xs:
        return None, None
    m = sum(xs) / len(xs)
    if len(xs) < 2:
        return m, 0.0
    sd = math.sqrt(sum((x - m) ** 2 for x in xs) / (len(xs) - 1))
    return m, 1.96 * sd / math.sqrt(len(xs))


def save(fig, path: Path):
    for ext in ("png", "pdf"):
        fig.savefig(path.with_suffix(f".{ext}"), dpi=150)


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("results")
    a = ap.parse_args()
    res = Path(a.results)
    summary = read_csv(res / "summary.csv")
    if not summary:
        print(f"[grid] {res}/summary.csv がありません")
        return 0
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    figs = res / "figs"
    figs.mkdir(parents=True, exist_ok=True)
    vehicles = read_csv(res / "vehicle_mean_arrival.csv")
    info = read_csv(res / "information" / "information_summary.csv")
    made = 0
    for layout in sorted({r.get("layout", "") for r in summary}):
        srows = [r for r in summary if r.get("layout", "") == layout]
        mus = sorted({fnum(r["mu_theta"]) for r in srows})
        ps = sorted({fnum(r["p_follow"]) for r in srows})
        if len(mus) < 2 or len(ps) < 2:
            print(f"[grid] {layout}: 格子でないので作りません(μθ {len(mus)} 値 × P {len(ps)} 値)")
            continue
        suffix = "" if layout in ("", "1-1-2") else f"_{layout}"
        variants = list(dict.fromkeys(r["variant"] for r in srows))
        cond_mp = {r["condition"]: (fnum(r["mu_theta"]), fnum(r["p_follow"])) for r in srows}
        cmap = plt.get_cmap("viridis")
        pcolor = {p: cmap(i / max(1, len(ps) - 1)) for i, p in enumerate(ps)}

        # 1. 完了時刻の折れ線
        fig, axes = plt.subplots(1, len(variants), figsize=(5.0 * len(variants), 3.9), squeeze=False, sharey=True)
        for ax, v in zip(axes[0], variants):
            for p in ps:
                pts = sorted((fnum(r["mu_theta"]), fnum(r["completion_time_mean"]),
                              fnum(r.get("completion_time_ci95_low")), fnum(r.get("completion_time_ci95_high")))
                             for r in srows if r["variant"] == v and fnum(r["p_follow"]) == p)
                xs = [x[0] for x in pts]
                ys = [x[1] for x in pts]
                err = [[(y - lo) if lo is not None else 0 for _, y, lo, _ in pts],
                       [(hi - y) if hi is not None else 0 for _, y, _, hi in pts]]
                ax.errorbar(xs, ys, yerr=err, color=pcolor[p], marker="o", ms=3, capsize=2, label=f"P={p:g}")
            ax.set_title(v)
            ax.set_xlabel("μθ [s]")
            ax.grid(alpha=0.3)
        axes[0][0].set_ylabel("Evacuation completion time [s]\n(mean ± 95% CI)")
        axes[0][-1].legend(fontsize=7)
        fig.suptitle(f"{layout}  completion time vs μθ", fontsize=10)
        fig.tight_layout()
        save(fig, figs / f"grid_completion_lines{suffix}")
        plt.close(fig)
        made += 1

        # 2. CDF を格子の配置で
        if vehicles:
            by = defaultdict(list)
            for r in vehicles:
                if r.get("layout", "") == layout and r["condition"] in cond_mp:
                    t = fnum(r.get("mean_arrival_time"))
                    if t is not None:
                        by[(r["condition"], r["variant"])].append(t)
            fig, axes = plt.subplots(len(ps), len(mus), figsize=(2.6 * len(mus), 2.2 * len(ps)),
                                     squeeze=False, sharex=True, sharey=True)
            for cond, (mu, p) in cond_mp.items():
                ax = axes[len(ps) - 1 - ps.index(p)][mus.index(mu)]
                for v in variants:
                    xs = sorted(by.get((cond, v), []))
                    if xs:
                        ax.plot(xs, [(i + 1) / len(xs) for i in range(len(xs))],
                                linestyle=STYLES.get(v, "-."), linewidth=1.3, label=v)
                ax.set_title(f"μθ={mu:g}, P={p:g}", fontsize=8)
                ax.grid(alpha=0.3)
            for ax in axes[-1]:
                ax.set_xlabel("Arrival time [s]", fontsize=8)
            for row in axes:
                row[0].set_ylabel("CDF", fontsize=8)
            axes[0][-1].legend(fontsize=6)
            fig.suptitle(f"{layout}  arrival-time CDF (per-vehicle mean over seeds)", fontsize=10)
            fig.tight_layout()
            save(fig, figs / f"grid_cdf{suffix}")
            plt.close(fig)
            made += 1

        # 3. 伝達の要約
        if info:
            vals = defaultdict(lambda: defaultdict(list))
            for r in info:
                if r.get("condition") not in cond_mp:
                    continue
                mu, _ = cond_mp[r["condition"]]
                vals[(r["mode"], mu)]["missed"].append(fnum(r.get("missed_before_X")))
                vals[(r["mode"], mu)]["wait"].append(fnum(r.get("wait_mean")))
            modes = list(dict.fromkeys(r["mode"] for r in info))
            fig, axes = plt.subplots(1, 2, figsize=(10, 3.8), squeeze=False)
            for k, (key, ylab) in enumerate([("missed", "Vehicles passing X without\nroute info (per run)"),
                                             ("wait", "Mean wait for route info\nbefore X [s]")]):
                ax = axes[0][k]
                for m in modes:
                    pts = [(mu, *ci95(vals[(m, mu)][key])) for mu in mus if vals.get((m, mu))]
                    pts = [x for x in pts if x[1] is not None]
                    ax.errorbar([x[0] for x in pts], [x[1] for x in pts], yerr=[x[2] for x in pts],
                                linestyle=STYLES.get(m, "-."), marker="o", ms=3, capsize=2, label=m)
                ax.set_xlabel("μθ [s]")
                ax.set_ylabel(ylab)
                ax.grid(alpha=0.3)
            axes[0][0].legend(fontsize=8)
            fig.suptitle(f"{layout}  information delivery before intersection X (all P pooled, mean ± 95% CI)",
                         fontsize=10)
            fig.tight_layout()
            save(fig, figs / f"grid_information{suffix}")
            plt.close(fig)
            made += 1
    print(f"[grid] {made} 枚 -> {figs}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
