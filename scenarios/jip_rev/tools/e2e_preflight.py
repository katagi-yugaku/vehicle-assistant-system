#!/usr/bin/env python3
"""
実験定義の静的チェック(SUMO も git も使わない。投入前に Mac でも計算機でも回せる)。

    python3 tools/e2e_preflight.py                       # experiments/ 以下の全 *.toml
    python3 tools/e2e_preflight.py experiments/E2.toml   # 指定したものだけ

確かめること
  - 条件の設定ファイルがあり、expect の値と一致する
  - 版のフラグが revision_options.RevisionOptions の項目(+ early_rate / v2v_rate)だけ
  - フラグが元の設定ファイルのキーと重ならない(run_experiment.py が止まる条件)
  - communication_mode が構成の runner で使える(1-1-4 は legacy のみ)
  - fix_alternative_from_table = true のとき travel_time_offset = 0
  - [compare] の版名が定義にある、aggregator が tools/ にある
  - 実験ごとの修正フラグの一覧(E4 で使う一式と違う実験を目立たせる)
終了コード: エラーがあれば 1。
"""
from __future__ import annotations

import argparse
import ast
import re
import sys
from pathlib import Path

if sys.version_info >= (3, 11):
    import tomllib as toml_loader
else:
    import tomli as toml_loader

ROOT = Path(__file__).resolve().parent.parent       # scenarios/jip_rev
CODE = ROOT / "code"
RUN_ARG_KEYS = {"early_rate", "v2v_rate"}
# E4 で使う修正一式(2026-11-18 時点の方針。変えたらここも直す)
FINAL_FIXES = {"fix_origin_edge": True, "fix_alternative_from_table": True, "travel_time_offset": 0.0,
               "vehicle_max_speed": 9.0, "free_flow_speed": 9.0}
FIX_KEYS = ["fix_origin_edge", "fix_pre_x_alternatives", "fix_alternative_from_table",
            "travel_time_offset", "vehicle_max_speed", "free_flow_speed", "communication_mode",
            "end_simulation_time"]


def option_fields() -> set[str]:
    """revision_options.py の RevisionOptions のフィールド名を import せずに読む。"""
    src = (CODE / "evacsim/sim/revision_options.py").read_text()
    tree = ast.parse(src)
    for node in tree.body:
        if isinstance(node, ast.ClassDef) and node.name == "RevisionOptions":
            return {n.target.id for n in node.body if isinstance(n, ast.AnnAssign)}
    raise SystemExit("RevisionOptions が見つかりません")


def supported_modes(layout: str) -> tuple[str, ...]:
    runner = CODE / "scenarios/JIP" / layout / "simulation/runner_simulator.py"
    if not runner.is_file():
        return ()
    text = runner.read_text()
    m = re.search(r"bootstrap\(sys\.argv,\s*supported_modes=\(([^)]*)\)", text)
    if not m:
        return ("legacy", "v2v_dtn", "v2v_only")
    names = {"_rev.LEGACY": "legacy", "_rev.V2V_DTN": "v2v_dtn", "_rev.V2V_ONLY": "v2v_only"}
    return tuple(names.get(x.strip(), x.strip()) for x in m.group(1).split(",") if x.strip())


def check(spec_path: Path, fields: set[str]) -> tuple[list[str], list[str], dict]:
    err: list[str] = []
    warn: list[str] = []
    spec = toml_loader.loads(spec_path.read_text())
    for k in ("name", "seed_start", "seed_end", "layouts", "conditions", "variants"):
        if k not in spec:
            err.append(f"{k} がありません")
    if err:
        return err, warn, {}
    agg = spec.get("aggregator", "aggregate_experiment.py")
    if not (ROOT / "tools" / agg).is_file():
        err.append(f"aggregator {agg} が tools/ にありません")

    seeds = spec["seed_end"] - spec["seed_start"] + 1
    for layout in spec["layouts"]:
        if not (CODE / "scenarios/JIP" / layout / "simulation/runner_simulator.py").is_file():
            err.append(f"layout {layout} の runner がありません")

    base_keys: dict[str, set] = {}
    for cond, info in spec["conditions"].items():
        p = CODE / info["config"]
        if not p.is_file():
            err.append(f"[{cond}] 設定 {info['config']} がありません")
            continue
        cfg = toml_loader.loads(p.read_text())
        base_keys[cond] = set(cfg)
        for k, v in (info.get("expect") or {}).items():
            if cfg.get(k) != v:
                err.append(f"[{cond}] {k}={cfg.get(k)!r}(expect {v!r})")

    for var, flags in spec["variants"].items():
        unknown = set(flags) - fields - RUN_ARG_KEYS
        if unknown:
            err.append(f"[{var}] 未知のフラグ {sorted(unknown)}(綴りを確認)")
        for cond, keys in base_keys.items():
            overlap = (set(flags) - RUN_ARG_KEYS) & keys
            if overlap:
                err.append(f"[{var}] {cond} の設定と重なるキー {sorted(overlap)}")
        mode = str(flags.get("communication_mode", "legacy")).lower()
        for layout in spec["layouts"]:
            modes = supported_modes(layout)
            if modes and mode not in modes:
                err.append(f"[{var}] {layout} の runner は communication_mode={mode} に対応していません({modes})")
        if flags.get("fix_alternative_from_table") and float(flags.get("travel_time_offset", 100.0)) != 0.0:
            warn.append(f"[{var}] fix_alternative_from_table と travel_time_offset≠0 の組み合わせ(表の値から差し引かれる)")
        for k in RUN_ARG_KEYS:
            if k in flags and not 0.0 <= float(flags[k]) <= 1.0:
                err.append(f"[{var}] {k}={flags[k]} は 0〜1")
        if not flags.get("save_raw"):
            warn.append(f"[{var}] save_raw がない(集計に raw.json が必要)")
        missing_final = [k for k, v in FINAL_FIXES.items() if flags.get(k, None) != v]
        if missing_final and var not in ("off", "before"):
            warn.append(f"[{var}] E4 の修正一式と違う: {', '.join(missing_final)}")

    comp = spec.get("compare") or {}
    names = set(spec["variants"])
    pairs = comp.get("pairs") or ([[comp["baseline"], comp["treatment"]]] if comp.get("baseline") else [])
    for p in pairs:
        for v in p:
            if v not in names:
                err.append(f"[compare] 版 {v} が定義にありません")

    n = len(spec["layouts"]) * len(spec["conditions"]) * len(spec["variants"]) * seeds
    summary = {"name": spec["name"], "tasks": n,
               "flags": {var: {k: flags.get(k, "-") for k in FIX_KEYS} for var, flags in spec["variants"].items()}}
    return err, warn, summary


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("specs", nargs="*")
    a = ap.parse_args()
    specs = [Path(s) for s in a.specs] or sorted((ROOT / "experiments").rglob("*.toml"))
    fields = option_fields()
    total_err = 0
    rows = []
    for sp in specs:
        err, warn, summ = check(sp.resolve(), fields)
        rel = sp.resolve().relative_to(ROOT) if sp.resolve().is_relative_to(ROOT) else sp
        status = "NG" if err else ("OK(注意あり)" if warn else "OK")
        print(f"== {rel}: {status}" + (f"  タスク {summ['tasks']}" if summ else ""))
        for e in err:
            print(f"  ERROR {e}")
        for w in warn:
            print(f"  WARN  {w}")
        total_err += len(err)
        if summ:
            for var, fl in summ["flags"].items():
                rows.append((summ["name"], var, *[str(fl[k]) for k in FIX_KEYS]))
    print("\n修正フラグの一覧(- は未指定 = 元の動作)")
    head = ("実験", "版", "origin_edge", "pre_x", "alt_table", "offset", "vmax", "v_free", "comm", "end")
    print(" | ".join(head))
    for r in rows:
        print(" | ".join(r))
    print(f"\n{'NG' if total_err else 'OK'}: エラー {total_err} 件")
    return 1 if total_err else 0


if __name__ == "__main__":
    sys.exit(main())
