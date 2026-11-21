#!/usr/bin/env python3
"""
E4h 本体と追加分(E4h-ext)の集計結果をつないで、μθ × P の格子の図を作り直す(シミュレーションは使わない)。
あわせて、μθ=150 の「切断なし(本体 mu150_*)」と「切断あり(追加 mu150tn_*)」を同じシードの組で比べる。

    python3 tools/merge_grid_results.py \
        --base results/20261119-2008_E4h-w300_4520e42 \
        --ext  results/<日時>_E4h-ext-w300_<commit> \
        --out  results/E4h-w300-merged
    (--mu150 ext で、格子の μθ=150 に切断ありの方を使う。既定は base = 本体)

入力(各結果フォルダ。aggregate_e2.py / analyze_information.py / analyze_decisions.py の出力)
  summary.csv, paired.csv, runs.csv, vehicle_mean_arrival.csv,
  information/information_summary.csv, decisions/decisions_summary.csv(あるものだけ)

出力(--out)
  上の CSV をつないだもの(各行に theta_dist = normal / trunc0 と source = base / ext を追加)
  figs/heatmap_*.{png,pdf}, heatmaps.md, figs/grid_*.{png,pdf}(plot_heatmaps.py / plot_grid_figures.py)
  truncation_check.csv / truncation_check.md   μθ=150 の切断あり − 切断なし(同じシード・同じ版、95% CI)
  merge_info.md                                  どの条件をどちらから使ったか
"""
from __future__ import annotations

import argparse
import csv
import subprocess
import sys
from collections import defaultdict
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
from aggregate_experiment import describe, write_csv  # noqa: E402

FILES = ["summary.csv", "paired.csv", "runs.csv", "vehicle_mean_arrival.csv",
         "information/information_summary.csv", "decisions/decisions_summary.csv"]
TN_TAG = "tn_"          # 追加分の確認用条件(mu150tn_p010 など)
METRICS = ["completion_time", "mean_arrival_time"]


def read_csv(path: Path) -> list[dict]:
    if not path.is_file() or path.stat().st_size == 0:
        return []
    with open(path, newline="") as f:
        return list(csv.DictReader(f))


def fnum(x):
    try:
        return float(x)
    except (TypeError, ValueError):
        return None


def find(res: Path, rel: str) -> Path:
    """decisions_summary.csv は置き場所が版によって違うので、見つからなければ下を探す。"""
    p = res / rel
    if p.is_file():
        return p
    hits = sorted(res.rglob(Path(rel).name))
    return hits[0] if hits else p


def cond_of(row: dict) -> str | None:
    return row.get("condition")


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--base", required=True, help="E4h 本体の結果フォルダ(切断なし)")
    ap.add_argument("--ext", required=True, help="E4h-ext の結果フォルダ(切断あり)")
    ap.add_argument("--out", required=True)
    ap.add_argument("--mu150", choices=["base", "ext"], default="base",
                    help="格子の μθ=150 にどちらを使うか(既定 base = 本体、切断なし)")
    ap.add_argument("--no-plots", action="store_true")
    a = ap.parse_args()
    base, ext, out = Path(a.base), Path(a.ext), Path(a.out)
    out.mkdir(parents=True, exist_ok=True)

    bsum, esum = read_csv(base / "summary.csv"), read_csv(ext / "summary.csv")
    if not bsum or not esum:
        print("summary.csv がありません", file=sys.stderr)
        return 1
    bconds = {r["condition"] for r in bsum}
    econds = {r["condition"] for r in esum}
    e_new = {c for c in econds if TN_TAG not in c}            # μθ=50/100
    e_tn = {c for c in econds if TN_TAG in c}                 # μθ=150 切断あり
    clash = bconds & econds
    if clash:
        print(f"条件名が本体と追加で重なっています: {sorted(clash)}", file=sys.stderr)
        return 1
    # 格子に使う条件
    if a.mu150 == "ext":
        drop = {c.replace("tn_", "_") for c in e_tn}         # mu150tn_p010 → mu150_p010
        use_base = bconds - drop
        use_ext = e_new | e_tn
    else:
        use_base = bconds
        use_ext = e_new
    # 格子に同じ (μθ, P) が 2 つ入らないことを確かめる
    keys = defaultdict(list)
    for src, rows, use in (("base", bsum, use_base), ("ext", esum, use_ext)):
        for r in rows:
            if r["condition"] in use:
                keys[(r.get("layout"), r["mu_theta"], r["p_follow"], r["variant"])].append((src, r["condition"]))
    dup = {k: v for k, v in keys.items() if len(v) > 1}
    if dup:
        print(f"格子の同じセルに 2 つの条件があります: {dup}", file=sys.stderr)
        return 1

    # つなぐ
    for rel in FILES:
        rows = []
        for src, res, use, dist in (("base", base, use_base, "normal"), ("ext", ext, use_ext, "trunc0")):
            for r in read_csv(find(res, rel)):
                if cond_of(r) in use:
                    rows.append({**r, "theta_dist": dist, "source": src})
        if rows:
            (out / rel).parent.mkdir(parents=True, exist_ok=True)
            write_csv(out / rel, rows)
            print(f"[merge] {rel}: {len(rows)} 行")

    # μθ=150 の切断あり − 切断なし(同じ版・同じシード)
    check_rows, md = [], []
    if e_tn:
        bruns, eruns = read_csv(base / "runs.csv"), read_csv(ext / "runs.csv")
        idx = {}
        for src, rows in (("base", bruns), ("ext", eruns)):
            for r in rows:
                if r.get("status", "SUCCESS") == "SUCCESS":
                    idx[(src, r.get("layout"), r["condition"], r["variant"], r["seed"])] = r
        variants = sorted({r["variant"] for r in eruns})
        layouts = sorted({r.get("layout") for r in eruns})
        for layout in layouts:
            for ce in sorted(e_tn):
                cb = ce.replace("tn_", "_")
                if cb not in bconds:
                    continue
                seeds = sorted({k[4] for k in idx if k[0] == "ext" and k[1] == layout and k[2] == ce}, key=int)
                for v in variants:
                    rec = {"layout": layout, "condition": cb, "variant": v}
                    for m in METRICS:
                        d = [fnum(idx[("ext", layout, ce, v, s)][m]) - fnum(idx[("base", layout, cb, v, s)][m])
                             for s in seeds if ("base", layout, cb, v, s) in idx and ("ext", layout, ce, v, s) in idx
                             and fnum(idx[("ext", layout, ce, v, s)][m]) is not None
                             and fnum(idx[("base", layout, cb, v, s)][m]) is not None]
                        st = describe(d)
                        rec.update({f"{m}_n": st["n"], f"{m}_diff": st["mean"],
                                    f"{m}_ci95_low": st["ci95_low"], f"{m}_ci95_high": st["ci95_high"]})
                    check_rows.append(rec)
                # 方式の差(DTN − V2V only)が切断で変わるか:差の差
                if len(variants) == 2:
                    v0, v1 = ("v2v_only", "v2v_dtn") if set(variants) == {"v2v_only", "v2v_dtn"} else variants
                    rec = {"layout": layout, "condition": cb, "variant": f"({v1} - {v0})"}
                    for m in METRICS:
                        d = []
                        for s in seeds:
                            try:
                                tn = fnum(idx[("ext", layout, ce, v1, s)][m]) - fnum(idx[("ext", layout, ce, v0, s)][m])
                                nm = fnum(idx[("base", layout, cb, v1, s)][m]) - fnum(idx[("base", layout, cb, v0, s)][m])
                                d.append(tn - nm)
                            except (KeyError, TypeError):
                                pass
                        st = describe(d)
                        rec.update({f"{m}_n": st["n"], f"{m}_diff": st["mean"],
                                    f"{m}_ci95_low": st["ci95_low"], f"{m}_ci95_high": st["ci95_high"]})
                    check_rows.append(rec)
        write_csv(out / "truncation_check.csv", check_rows)

        def f(x):
            return "-" if x is None else f"{x:+.1f}"

        def star(lo, hi):
            return "*" if lo is not None and hi is not None and (lo > 0 or hi < 0) else ""
        md = ["# μθ=150:切断あり(θ ≥ 0)− 切断なし(同じシード・同じ版)",
              "",
              "正なら切断ありの方が長い。* = 95% CI が 0 をまたがない。"
              "「(v2v_dtn - v2v_only)」の行は方式の差(DTN − V2V only)が切断でどれだけ変わったか(差の差)。",
              "",
              "| 条件 | 版 | n | 完了時刻の差 [95% CI] | 平均到着の差 [95% CI] |",
              "| --- | --- | --- | --- | --- |"]
        for r in check_rows:
            md.append(f"| {r['condition']} | {r['variant']} | {r['completion_time_n']} | "
                      f"{f(r['completion_time_diff'])} [{f(r['completion_time_ci95_low'])}, {f(r['completion_time_ci95_high'])}]"
                      f"{star(r['completion_time_ci95_low'], r['completion_time_ci95_high'])} | "
                      f"{f(r['mean_arrival_time_diff'])} [{f(r['mean_arrival_time_ci95_low'])}, {f(r['mean_arrival_time_ci95_high'])}]"
                      f"{star(r['mean_arrival_time_ci95_low'], r['mean_arrival_time_ci95_high'])} |")
        (out / "truncation_check.md").write_text("\n".join(md) + "\n")
        print("\n".join(md))

    info = ["# 統合の内容", "",
            f"- 本体(切断なし): {base}", f"- 追加(θ を 0 で切断): {ext}",
            f"- 格子の μθ=150: {'追加(切断あり)' if a.mu150 == 'ext' else '本体(切断なし)'}",
            f"- 本体から使った条件: {', '.join(sorted(use_base))}",
            f"- 追加から使った条件: {', '.join(sorted(use_ext))}",
            "- 図の注記: μθ≤100 は θ ~ TN(μθ, σθ², 0, ∞)、μθ≥" + ("200" if a.mu150 == "ext" else "150") + " は θ ~ N(μθ, σθ²)"]
    (out / "merge_info.md").write_text("\n".join(info) + "\n")
    print("\n".join(info))

    if not a.no_plots:
        for tool in ("plot_heatmaps.py", "plot_grid_figures.py"):
            subprocess.run([sys.executable, str(HERE / tool), str(out)], check=False)
    return 0


if __name__ == "__main__":
    sys.exit(main())
