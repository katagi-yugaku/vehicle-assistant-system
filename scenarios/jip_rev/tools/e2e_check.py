#!/usr/bin/env python3
"""
e2e テストの判定(実験が終わった後に回す。SUMO は使わない)。

    python3 tools/e2e_check.py runs/<実験ID>                         # 1 回分の実験を判定
    python3 tools/e2e_check.py runs/<実験ID> --compare-to runs/<別ID>  # 同じ設定・シードの再実行と完全一致するか(再現性)
    python3 tools/e2e_check.py runs/<実験ID> --results results/<実験ID> --report e2e_report.md

判定項目(FAIL があれば終了コード 1)
  P1 manifest / tasks.tsv があり、予定数 = タスク数、凍結コードが dirty でない(dirty は WARN)
  P2 全タスクが status=SUCCESS、run.err に Traceback がない
  R1 raw.json の schema・seed がタスクと一致
  R2 raw.json の flags が実験定義の版のフラグと一致(設定が実際に効いている)
  R3 run.out の [RUN_CONFIG] 行に seed とフラグが出ている
  R4 全車両が到着(end_simulation_time >= 3000 のとき。短縮した試験では WARN)、完了時刻 <= 終了時刻
  R5 経路変更の内訳の合計が一致(理由別 = 種類別 = 合計)
  F1 fix_origin_edge = true なら E0 上の再生成 0 回
  F2 fix_origin_edge = false(元の動作)の 2 経路・4 経路では E0 上の再生成が 1 回以上ある(不具合が再現できている。WARN)
  F3 v2v_rate = 0(通信なし)なら情報による経路変更 0 回(WARN)
  F4 log_information = true なら information.jsonl.gz がある、log_sections = true なら sections が空でない
  A1 集計結果(summary.csv、decision.md、[compare] があれば paired.csv、figs/*.png)がある
  A2 summary.csv の成功数 = raw.json のある回数
  D1 --compare-to:同じ (構成, 条件, 版, シード) の到着時刻と経路変更イベントが完全一致
"""
from __future__ import annotations

import argparse
import csv
import gzip
import json
import re
import sys
from collections import defaultdict
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent   # scenarios/jip_rev
# 実験定義の版に書けるが raw.json の flags には入らないキー
NOT_IN_FLAGS = {"early_rate", "v2v_rate"}
LAYOUTS_WITH_E0_BUG = {"1-1-2", "1-1-4"}


class Report:
    def __init__(self) -> None:
        self.items: list[tuple[str, str, str, str]] = []   # (level, id, scope, message)

    def add(self, level: str, cid: str, scope: str, msg: str) -> None:
        self.items.append((level, cid, scope, msg))

    def ok(self, cid, scope, msg=""):
        self.add("PASS", cid, scope, msg)

    def fail(self, cid, scope, msg):
        self.add("FAIL", cid, scope, msg)

    def warn(self, cid, scope, msg):
        self.add("WARN", cid, scope, msg)

    @property
    def n_fail(self) -> int:
        return sum(1 for i in self.items if i[0] == "FAIL")

    def summary_by_check(self) -> list[tuple[str, int, int, int]]:
        agg: dict[str, list[int]] = defaultdict(lambda: [0, 0, 0])
        for lvl, cid, _, _ in self.items:
            agg[cid][{"PASS": 0, "WARN": 1, "FAIL": 2}[lvl]] += 1
        return [(cid, *v) for cid, v in sorted(agg.items())]


def read_status(path: Path) -> dict:
    out = {}
    if path.is_file():
        for line in path.read_text().splitlines():
            if "=" in line:
                k, v = line.split("=", 1)
                out[k] = v
    return out


def load_raw(run_dir: Path) -> dict | None:
    for name in ("raw.json", "raw.json.gz"):
        p = run_dir / name
        if p.is_file():
            opener = gzip.open if name.endswith(".gz") else open
            with opener(p, "rt") as f:
                return json.load(f)
    return None


def same(a, b) -> bool:
    try:
        return abs(float(a) - float(b)) < 1e-9
    except (TypeError, ValueError):
        return str(a).lower() == str(b).lower()


def check_run(rep: Report, task: dict, variant_flags: dict, run_dir: Path) -> dict | None:
    scope = f"{task['layout']}/{task['condition']}/{task['variant']}/r{int(task['seed']):03d}"
    st = read_status(run_dir / "status.txt")
    if st.get("status") != "SUCCESS":
        rep.fail("P2", scope, f"status={st.get('status', 'MISSING')} {st.get('reason', '')}".strip())
        return None
    err = run_dir / "run.err"
    if err.is_file() and "Traceback" in err.read_text(errors="replace"):
        rep.fail("P2", scope, "run.err に Traceback")
    else:
        rep.ok("P2", scope)

    raw = load_raw(run_dir)
    if raw is None:
        rep.fail("R1", scope, "raw.json がない")
        return None
    if raw.get("schema") != "evacsim-raw/1":
        rep.fail("R1", scope, f"schema={raw.get('schema')!r}")
    elif raw.get("seed") != int(task["seed"]):
        rep.fail("R1", scope, f"raw seed={raw.get('seed')} task seed={task['seed']}")
    else:
        rep.ok("R1", scope)

    flags = raw.get("flags") or {}
    bad = [f"{k}: 定義 {v!r} / 実際 {flags.get(k)!r}"
           for k, v in variant_flags.items() if k not in NOT_IN_FLAGS and not same(flags.get(k), v)]
    rep.fail("R2", scope, "; ".join(bad)) if bad else rep.ok("R2", scope)

    out = run_dir / "run.out"
    line = ""
    if out.is_file():
        line = next((l for l in out.read_text(errors="replace").splitlines() if "[RUN_CONFIG]" in l), "")
    if not line:
        rep.fail("R3", scope, "run.out に [RUN_CONFIG] がない")
    else:
        miss = []
        if f"seed={task['seed']} " not in line + " ":
            miss.append("seed")
        for k, v in variant_flags.items():
            m = re.search(rf"\b{k}=(\S+)", line)
            if m and not same(m.group(1), v):
                miss.append(f"{k}={m.group(1)}(定義 {v})")
        rep.fail("R3", scope, ", ".join(miss)) if miss else rep.ok("R3", scope)

    end = float(flags.get("end_simulation_time") or 3000)
    arrived, n = raw.get("arrived_count"), raw.get("num_vehicles")
    ct = raw.get("completion_time")
    if arrived != n:
        (rep.fail if end >= 3000 else rep.warn)("R4", scope, f"到着 {arrived}/{n}(終了 {end:g} s)")
    elif ct is not None and ct > end + 1e-6:
        rep.fail("R4", scope, f"完了時刻 {ct} > 終了 {end}")
    else:
        rep.ok("R4", scope)

    s = raw.get("route_change_summary") or {}
    tot = s.get("success_total", 0)
    sr, sty = sum((s.get("success_by_reason") or {}).values()), sum((s.get("success_by_type") or {}).values())
    sfe = sum((s.get("success_by_from_edge") or {}).values())
    if not (tot == sr == sty == sfe):
        rep.fail("R5", scope, f"合計 {tot} / 理由別 {sr} / 種類別 {sty} / edge 別 {sfe}")
    else:
        rep.ok("R5", scope)

    e0 = s.get("e0_regeneration_count", 0)
    if flags.get("fix_origin_edge"):
        rep.fail("F1", scope, f"E0 再生成 {e0} 回") if e0 else rep.ok("F1", scope)
    elif task["layout"] in LAYOUTS_WITH_E0_BUG and str(flags.get("communication_mode", "legacy")) == "legacy":
        rep.ok("F2", scope, f"E0 再生成 {e0} 回") if e0 else rep.warn("F2", scope, "元の動作なのに E0 再生成が 0 回")

    if float(task.get("v2v_rate", 1.0)) == 0.0:
        normalcy = (s.get("success_by_reason") or {}).get("normalcy", 0)
        rep.warn("F3", scope, f"通信なしで情報による経路変更 {normalcy} 回") if normalcy else rep.ok("F3", scope)

    if flags.get("log_information"):
        info = Path(flags.get("information_output") or run_dir / "information.jsonl.gz")
        if not info.is_absolute():
            info = run_dir / info
        cand = [info, run_dir / "information.jsonl.gz"]
        rep.ok("F4", scope) if any(c.is_file() and c.stat().st_size > 0 for c in cand) else \
            rep.fail("F4", scope, "information.jsonl.gz がない")
    if flags.get("log_sections"):
        rep.ok("F4", scope) if raw.get("sections") else rep.fail("F4", scope, "sections が空")
    return raw


def check_results(rep: Report, results: Path, manifest: dict, n_raw: int) -> None:
    scope = str(results)
    need = ["summary.csv", "decision.md"]
    comp = manifest.get("compare") or {}
    if comp.get("baseline") or comp.get("pairs"):
        need.append("paired.csv")
    missing = [f for f in need if not (results / f).is_file() or (results / f).stat().st_size == 0]
    if not list((results / "figs").glob("*.png")):
        missing.append("figs/*.png")
    rep.fail("A1", scope, "ない: " + ", ".join(missing)) if missing else rep.ok("A1", scope)
    sp = results / "summary.csv"
    if sp.is_file():
        with open(sp) as f:
            rows = list(csv.DictReader(f))
        n_succ = sum(int(float(r.get("n_success") or 0)) for r in rows)
        rep.ok("A2", scope, f"{n_succ}") if n_succ == n_raw else \
            rep.fail("A2", scope, f"summary の成功数 {n_succ} / raw.json {n_raw}")


def compare_runs(rep: Report, a_root: Path, b_root: Path) -> None:
    def index(root: Path) -> dict:
        with open(root / "tasks.tsv") as f:
            tasks = list(csv.DictReader(f, delimiter="\t"))
        return {(t["layout"], t["condition"], t["variant"], t["seed"]): Path(t["run_dir"]) for t in tasks}
    ia, ib = index(a_root), index(b_root)
    common = sorted(set(ia) & set(ib))
    if not common:
        rep.fail("D1", "-", "共通する (構成, 条件, 版, シード) がない")
        return
    for key in common:
        scope = "/".join(key[:3]) + f"/r{int(key[3]):03d}"
        ra, rb = load_raw(ia[key]), load_raw(ib[key])
        if ra is None or rb is None:
            rep.fail("D1", scope, "raw.json がない")
            continue
        diffs = [k for k in ("arrival_time_by_sumo_id", "route_change_events", "completion_time", "counters")
                 if ra.get(k) != rb.get(k)]
        rep.fail("D1", scope, "不一致: " + ", ".join(diffs)) if diffs else rep.ok("D1", scope)


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("run_root")
    ap.add_argument("--results", help="既定: scenarios/jip_rev/results/<実験ID>")
    ap.add_argument("--compare-to", help="再現性の確認に使う別の実験(同じ設定・シード)")
    ap.add_argument("--report", help="Markdown の報告を書く先(既定: <results>/e2e_report.md)")
    ap.add_argument("--verbose", action="store_true", help="PASS も 1 件ずつ表示する")
    a = ap.parse_args()

    run_root = Path(a.run_root).resolve()
    rep = Report()
    mpath, tpath = run_root / "manifest.json", run_root / "tasks.tsv"
    if not (mpath.is_file() and tpath.is_file()):
        print(f"FAIL P1: {run_root} に manifest.json / tasks.tsv がない")
        return 1
    manifest = json.loads(mpath.read_text())
    with open(tpath) as f:
        tasks = list(csv.DictReader(f, delimiter="\t"))
    if manifest.get("planned_runs") != len(tasks):
        rep.fail("P1", "manifest", f"planned_runs {manifest.get('planned_runs')} / tasks {len(tasks)}")
    else:
        rep.ok("P1", "manifest", f"{len(tasks)} タスク")
    if manifest.get("code_dirty"):
        rep.warn("P1", "manifest", "凍結コードが dirty(--allow-dirty)。本番前にコミットすること")

    variants = manifest.get("variants") or {}
    extra = manifest.get("extra_flags") or {}
    raws = 0
    for t in tasks:
        vflags = {**variants.get(t["variant"], {}), **extra}
        if check_run(rep, t, vflags, Path(t["run_dir"])) is not None:
            raws += 1

    results = Path(a.results).resolve() if a.results else ROOT / "results" / manifest["experiment_id"]
    if results.is_dir():
        check_results(rep, results, manifest, raws)
    else:
        rep.fail("A1", str(results), "集計結果のディレクトリがない(finalize が走ったか確認)")
    if a.compare_to:
        compare_runs(rep, run_root, Path(a.compare_to).resolve())

    lines = [f"# e2e 判定: {manifest['experiment_id']}", "",
             f"- コード: {manifest.get('code_commit', '')[:7]}{' (dirty)' if manifest.get('code_dirty') else ''}"
             f" / 投入: {manifest.get('submitted_at', '')} / ホスト: {manifest.get('host', '')}",
             f"- 結果: {'FAIL' if rep.n_fail else 'PASS'}(FAIL {rep.n_fail} 件)", "",
             "| 項目 | PASS | WARN | FAIL |", "| --- | --- | --- | --- |"]
    lines += [f"| {cid} | {p} | {w} | {fl} |" for cid, p, w, fl in rep.summary_by_check()]
    probs = [i for i in rep.items if i[0] != "PASS" or a.verbose]
    if probs:
        lines += ["", "| 判定 | 項目 | 対象 | 内容 |", "| --- | --- | --- | --- |"]
        lines += [f"| {lvl} | {cid} | {sc} | {msg} |" for lvl, cid, sc, msg in probs]
    text = "\n".join(lines) + "\n"
    print(text)
    out = Path(a.report) if a.report else (results / "e2e_report.md" if results.is_dir() else None)
    if out:
        out.write_text(text)
        print(f"[report] {out}")
    return 1 if rep.n_fail else 0


if __name__ == "__main__":
    sys.exit(main())
