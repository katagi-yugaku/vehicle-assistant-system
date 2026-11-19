#!/usr/bin/env python3
"""
μθ × P のヒートマップを作る(aggregate_e2.py の summary.csv / paired.csv から。シミュレーションは使わない)。

    python3 tools/plot_heatmaps.py results/<実験ID>

作るもの(構成ごと。μθ と P がそれぞれ 2 値以上あるときだけ)
  figs/heatmap_completion.{png,pdf}   完了時刻の平均。版ごと(色の範囲は共通)+ 組ごとの差(後 − 前)
  figs/heatmap_mean_arrival.{png,pdf} 平均到着時刻。同上
  figs/heatmap_route_changes.{png,pdf} 経路変更の平均回数(版ごと)と差
  heatmaps.md                          同じ値の表(メール本文用)
差のセルには 95% 信頼区間が 0 をまたがないとき * を付ける。
"""
from __future__ import annotations

import argparse
import csv
import json
from collections import defaultdict
from pathlib import Path


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


def grid_axes(rows):
    mus = sorted({fnum(r["mu_theta"]) for r in rows if fnum(r.get("mu_theta")) is not None})
    ps = sorted({fnum(r["p_follow"]) for r in rows if fnum(r.get("p_follow")) is not None})
    return mus, ps


def to_grid(rows, mus, ps, key):
    g = [[None] * len(mus) for _ in ps]
    for r in rows:
        mu, p, v = fnum(r.get("mu_theta")), fnum(r.get("p_follow")), fnum(r.get(key))
        if mu in mus and p in ps:
            g[ps.index(p)][mus.index(mu)] = v
    return g


def sig_grid(rows, mus, ps, metric):
    g = [[False] * len(mus) for _ in ps]
    for r in rows:
        mu, p = fnum(r.get("mu_theta")), fnum(r.get("p_follow"))
        lo, hi = fnum(r.get(f"diff_{metric}_ci95_low")), fnum(r.get(f"diff_{metric}_ci95_high"))
        if mu in mus and p in ps and lo is not None and hi is not None:
            g[ps.index(p)][mus.index(mu)] = (lo > 0) or (hi < 0)
    return g


def label_of(labels, name):
    return labels.get(name, name)


def draw(fig_path: Path, title: str, panels: list[dict], mus, ps):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    n = len(panels)
    fig, axes = plt.subplots(1, n, figsize=(4.2 * n + 0.6, 3.9), squeeze=False)
    seq = [p for p in panels if not p.get("diverging")]
    vals = [v for p in seq for row in p["grid"] for v in row if v is not None]
    vmin, vmax = (min(vals), max(vals)) if vals else (0, 1)
    for ax, pnl in zip(axes[0], panels):
        g = [[float("nan") if v is None else v for v in row] for row in pnl["grid"]]
        if pnl.get("diverging"):
            m = max([abs(v) for row in pnl["grid"] for v in row if v is not None] or [1.0])
            im = ax.imshow(g, origin="lower", cmap="RdBu_r", vmin=-m, vmax=m, aspect="auto")
        else:
            im = ax.imshow(g, origin="lower", cmap="viridis", vmin=vmin, vmax=vmax, aspect="auto")
        for i, row in enumerate(pnl["grid"]):
            for j, v in enumerate(row):
                if v is None:
                    continue
                star = "*" if pnl.get("sig") and pnl["sig"][i][j] else ""
                txt = f"{v:+.0f}{star}" if pnl.get("diverging") else f"{v:.0f}"
                r, gg, b, _ = im.cmap(im.norm(v))
                ax.text(j, i, txt, ha="center", va="center", fontsize=8,
                        color="black" if 0.299 * r + 0.587 * gg + 0.114 * b > 0.5 else "white")
        ax.set_xticks(range(len(mus)), [f"{m:g}" for m in mus])
        ax.set_yticks(range(len(ps)), [f"{p:g}" for p in ps])
        ax.set_xlabel("μθ [s]")
        ax.set_ylabel("P_base follow")
        ax.set_title(pnl["title"], fontsize=10)
        fig.colorbar(im, ax=ax, fraction=0.046, pad=0.04)
    fig.suptitle(title, fontsize=11)
    fig.tight_layout()
    for ext in ("png", "pdf"):
        fig.savefig(fig_path.with_suffix(f".{ext}"), dpi=150)
    plt.close(fig)


def md_table(title, grid, mus, ps, sig=None, signed=False):
    lines = [f"**{title}**", "", "| P \\ μθ | " + " | ".join(f"{m:g}" for m in mus) + " |",
             "| --- |" + " --- |" * len(mus)]
    for i in reversed(range(len(ps))):
        cells = []
        for j in range(len(mus)):
            v = grid[i][j]
            if v is None:
                cells.append("–")
            else:
                cells.append((f"{v:+.0f}" if signed else f"{v:.0f}") + ("*" if sig and sig[i][j] else ""))
        lines.append(f"| {ps[i]:g} | " + " | ".join(cells) + " |")
    return lines + [""]


# (ファイル名, 列, 表の名前, 図の名前。計算機の matplotlib に日本語フォントがない場合に備えて図は英語)
METRICS = [("completion", "completion_time", "完了時刻の平均 [s]", "Evacuation completion time [s]"),
           ("mean_arrival", "mean_arrival_time", "平均到着時刻 [s]", "Mean arrival time [s]"),
           ("route_changes", "route_changes", "経路変更の平均回数", "Route changes per run")]


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("results")
    a = ap.parse_args()
    res = Path(a.results)
    summary, paired = read_csv(res / "summary.csv"), read_csv(res / "paired.csv")
    if not summary:
        print(f"[heatmap] {res}/summary.csv がありません")
        return 0
    labels = {}
    man = res / "manifest.json"
    if man.is_file():
        labels = (json.loads(man.read_text()).get("compare") or {}).get("labels") or {}
    figs = res / "figs"
    figs.mkdir(parents=True, exist_ok=True)
    md = ["## ヒートマップ(μθ × P)", ""]
    made = 0
    for layout in sorted({r.get("layout", "") for r in summary}):
        srows = [r for r in summary if r.get("layout", "") == layout]
        mus, ps = grid_axes(srows)
        if len(mus) < 2 or len(ps) < 2:
            print(f"[heatmap] {layout}: μθ {len(mus)} 値 × P {len(ps)} 値なので作りません")
            continue
        variants = list(dict.fromkeys(r["variant"] for r in srows))
        prows = [r for r in paired if r.get("layout", "") == layout]
        pairs = list(dict.fromkeys((r["baseline"], r["treatment"]) for r in prows))
        for tag, key, name, fig_name in METRICS:
            panels, mdl = [], []
            for v in variants:
                g = to_grid([r for r in srows if r["variant"] == v], mus, ps, f"{key}_mean")
                panels.append({"title": label_of(labels, v), "grid": g})
                mdl += md_table(f"{name}:{label_of(labels, v)}", g, mus, ps)
            for base, treat in pairs:
                rr = [r for r in prows if r["baseline"] == base and r["treatment"] == treat]
                if not rr or f"diff_{key}_mean" not in rr[0]:
                    continue
                g = to_grid(rr, mus, ps, f"diff_{key}_mean")
                sig = sig_grid(rr, mus, ps, key)
                t = f"{label_of(labels, treat)} - {label_of(labels, base)}"
                panels.append({"title": t + "  (* 95% CI excludes 0)", "grid": g, "sig": sig, "diverging": True})
                mdl += md_table(f"{name}の差:{t}(* = 95% CI が 0 をまたがない)", g, mus, ps, sig, signed=True)
            draw(figs / f"heatmap_{tag}{'' if layout in ('', '1-1-2') else '_' + layout}",
                 f"{layout}  {fig_name}", panels, mus, ps)
            if tag != "route_changes":
                md += mdl
            made += 1
    if made:
        (res / "heatmaps.md").write_text("\n".join(md) + "\n")
        print(f"[heatmap] {made} 枚 -> {figs}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
