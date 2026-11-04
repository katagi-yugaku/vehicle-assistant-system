#!/usr/bin/env python3
"""
実験を 1 コマンドで投入する(JIP 26-S015f 改訂。scenarios/jip_rev の中だけで完結)。

    cd scenarios/jip_rev
    tools/run_experiment.sh experiments/E3.toml                 # Slurm に投入
    tools/run_experiment.sh experiments/E3.toml --print-only    # 投入せずに sbatch のコマンドを表示
    tools/run_experiment.sh experiments/E3.toml --backend local --seeds 1-2 --set end_simulation_time=300

流れ:
  1. jip_rev の code / tools / experiments に未コミットの変更があれば止める(--allow-dirty で試験時のみ許可)
  2. 実験 ID を決める: YYYYmmdd-HHMM_<名前>_<コミット短縮形>
  3. code / tools / experiments を runs/<実験ID>/frozen に凍結する(git archive)
  4. 実行ごとの設定 toml、tasks.tsv、manifest.json を作る
  5. Slurm のジョブ配列(1 実験 1 配列、同時実行数 --max-parallel)と、
     全タスク終了後に走る集計ジョブ(--dependency=afterany)を投入する
"""
from __future__ import annotations

import argparse
import csv
import datetime as dt
import json
import os
import platform
import shlex
import shutil
import socket
import subprocess
import sys
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path

if sys.version_info >= (3, 11):
    import tomllib as toml_loader
else:
    import tomli as toml_loader

ROOT = Path(__file__).resolve().parent.parent          # .../scenarios/jip_rev
FROZEN_PATHS = ["code", "tools", "experiments"]
TASK_FIELDS = ["task_id", "layout", "condition", "variant", "seed", "config", "run_dir"]


def sh(cmd: list[str], **kw) -> str:
    return subprocess.run(cmd, check=True, capture_output=True, text=True, **kw).stdout.strip()


def toml_value(v) -> str:
    if isinstance(v, bool):
        return "true" if v else "false"
    if isinstance(v, (int, float)):
        return repr(v)
    return json.dumps(str(v), ensure_ascii=False)


def parse_seeds(text: str) -> list[int]:
    a, _, b = text.partition("-")
    return list(range(int(a), int(b or a) + 1))


def parse_set(items: list[str]) -> dict:
    out = {}
    for item in items:
        key, _, raw = item.partition("=")
        try:
            out[key.strip()] = toml_loader.loads(f"v = {raw.strip()}")["v"]
        except Exception:
            out[key.strip()] = raw.strip()
    return out


def git_state(ref: str | None, allow_dirty: bool) -> tuple[str, str, bool, str, str]:
    """
    凍結するコードのコミットを決める。

    --ref を指定した場合(計算機での通常の使い方):そのコミットの scenarios/jip_rev を
      git のオブジェクトから直接凍結する。作業ツリーの jip_rev は投入スクリプトとして使うだけ。
    --ref を省略した場合:HEAD に scenarios/jip_rev があれば HEAD を使い、
      jip_rev に未コミットの変更があれば止める(--allow-dirty なら作業ツリーをそのまま写す)。
    """
    top = sh(["git", "-C", str(ROOT), "rev-parse", "--show-toplevel"])
    prefix = os.path.relpath(ROOT, top)
    ref_name = ref or "HEAD"
    try:
        commit = sh(["git", "-C", top, "rev-parse", "--verify", f"{ref_name}^{{commit}}"])
    except subprocess.CalledProcessError:
        sys.exit(f"ERROR: git の参照 {ref_name!r} が見つかりません(git fetch origin を実行しましたか)")
    try:
        sh(["git", "-C", top, "cat-file", "-e", f"{commit}:{prefix}/code"])
    except subprocess.CalledProcessError:
        sys.exit(f"ERROR: {ref_name} ({commit[:7]}) に {prefix}/code がありません。"
                 "計算機では --ref origin/jip/revision-26S015f を指定してください。")
    dirty = False
    if ref is None:
        dirty_lines = sh(["git", "-C", top, "status", "--porcelain", "--untracked-files=all", "--",
                          *[f"{prefix}/{p}" for p in FROZEN_PATHS]])
        dirty = bool(dirty_lines)
        if dirty and not allow_dirty:
            sys.exit("ERROR: jip_rev に未コミットの変更があります。コミットしてから投入してください"
                     "(試験時のみ --allow-dirty)。\n" + dirty_lines)
    else:
        # 作業ツリーの投入スクリプトとコミットの内容が違えば知らせる(凍結はコミットから行う)
        differ = []
        for line in sh(["git", "-C", top, "ls-tree", "-r", commit, "--", f"{prefix}/tools"]).splitlines():
            meta, path = line.split("\t", 1)
            blob = meta.split()[2]
            local = Path(top) / path
            if not local.is_file() or sh(["git", "-C", top, "hash-object", str(local)]) != blob:
                differ.append(path)
        if differ:
            print(f"[warn] 作業ツリーの {prefix}/tools が {ref_name} と違います。"
                  f"実行には {ref_name} のコードを使います。", file=sys.stderr)
    return commit, commit[:7], dirty, prefix, ref_name


def freeze(dest: Path, commit: str, dirty: bool, prefix: str) -> None:
    dest.mkdir(parents=True)
    if not dirty:
        top = sh(["git", "-C", str(ROOT), "rev-parse", "--show-toplevel"])
        # パス指定はリポジトリ直下から解釈させる(サブディレクトリで実行すると失敗するため)
        archive = subprocess.Popen(
            ["git", "-C", top, "archive", f"{commit}:{prefix}", *FROZEN_PATHS],
            stdout=subprocess.PIPE,
        )
        subprocess.run(["tar", "-x", "-C", str(dest)], stdin=archive.stdout, check=True)
        if archive.wait() != 0:
            sys.exit("ERROR: git archive failed")
        return
    ignore = shutil.ignore_patterns("__pycache__", "*.pyc")
    for p in FROZEN_PATHS:
        shutil.copytree(ROOT / p, dest / p, ignore=ignore)


def tool_versions(python: str) -> dict:
    code = (
        "import json,sys\n"
        "d={'python':sys.version.split()[0]}\n"
        "for m in ('traci','sumolib','numpy','tomli'):\n"
        "    try:\n"
        "        mod=__import__(m); d[m]=getattr(mod,'__version__',None) or 'installed'\n"
        "    except Exception as e: d[m]=f'missing ({e.__class__.__name__})'\n"
        "print(json.dumps(d))\n"
    )
    try:
        info = json.loads(sh([python, "-c", code]))
    except Exception as exc:
        info = {"error": str(exc)}
    sumo_home = os.environ.get("SUMO_HOME", "")
    sumo_bin = shutil.which("sumo") or (os.path.join(sumo_home, "bin", "sumo") if sumo_home else "")
    try:
        info["sumo"] = sh([sumo_bin, "--version"]).splitlines()[0]
    except Exception:
        info["sumo"] = f"unknown (SUMO_HOME={sumo_home or 'unset'})"
    return info


def build(args) -> tuple[Path, dict]:
    spec_path = Path(args.spec).resolve()
    spec = toml_loader.loads(spec_path.read_text())
    name = args.name or spec["name"]
    commit, short, dirty, prefix, ref_name = git_state(args.ref, args.allow_dirty)
    stamp = dt.datetime.now().strftime("%Y%m%d-%H%M")
    exp_id = f"{stamp}_{name}_{short}{'-dirty' if dirty else ''}"
    run_root = (ROOT / args.runs_root / exp_id).resolve()
    if run_root.exists():
        sys.exit(f"ERROR: {run_root} already exists")

    seeds = parse_seeds(args.seeds) if args.seeds else list(range(spec["seed_start"], spec["seed_end"] + 1))
    layouts = args.layouts.split(",") if args.layouts else spec["layouts"]
    extra = parse_set(args.set or [])

    frozen = run_root / "frozen"
    freeze(frozen, commit, dirty, prefix)
    cfg_dir = run_root / "configs"
    cfg_dir.mkdir()
    shutil.copy2(spec_path, cfg_dir / f"spec_{spec_path.name}")

    tasks: list[dict] = []
    condition_meta = {}
    for layout in layouts:
        for cond, cinfo in spec["conditions"].items():
            base_text = (frozen / "code" / cinfo["config"]).read_text()
            base_cfg = toml_loader.loads(base_text)
            for key, val in (cinfo.get("expect") or {}).items():
                if base_cfg.get(key) != val:
                    sys.exit(f"ERROR: {cinfo['config']}: {key}={base_cfg.get(key)!r}, expected {val!r}")
            condition_meta[cond] = {
                "config": cinfo["config"],
                "mu_theta": base_cfg.get("active_route_change_threshold_center"),
                "p_follow": base_cfg.get("p_follow"),
            }
            for variant, flags in spec["variants"].items():
                merged = {**flags, **extra}
                overlap = set(merged) & set(base_cfg)
                if overlap:
                    sys.exit(f"ERROR: flags already defined in {cinfo['config']}: {sorted(overlap)}")
                cfg_file = cfg_dir / f"{layout}__{cond}__{variant}.toml"
                lines = [base_text.rstrip("\n"), "", f"# ---- {name} / {variant} (run_experiment.py) ----"]
                lines += [f"{k} = {toml_value(v)}" for k, v in merged.items()]
                cfg_file.write_text("\n".join(lines) + "\n")
                for seed in seeds:
                    tasks.append({
                        "task_id": len(tasks) + 1, "layout": layout, "condition": cond,
                        "variant": variant, "seed": seed, "config": str(cfg_file),
                        "run_dir": str(run_root / layout / cond / variant / f"r{seed:03d}"),
                    })

    with open(run_root / "tasks.tsv", "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=TASK_FIELDS, delimiter="\t", lineterminator="\n")
        w.writeheader()
        w.writerows(tasks)

    manifest = {
        "experiment_id": exp_id,
        "name": name,
        "description": spec.get("description", ""),
        "spec": os.path.relpath(spec_path, ROOT),
        "code_ref": ref_name,
        "code_commit": commit,
        "code_dirty": dirty,
        "code_path": str(frozen / "code"),
        "submit_command": " ".join(shlex.quote(a) for a in sys.argv),
        "backend": args.backend,
        "layouts": layouts,
        "conditions": condition_meta,
        "variants": spec["variants"],
        "extra_flags": extra,
        "seeds": [seeds[0], seeds[-1]] if seeds else [],
        "early_rate": spec.get("early_rate", 1.0),
        "v2v_rate": spec.get("v2v_rate", 1.0),
        "compare": spec.get("compare", {}),
        "planned_runs": len(tasks),
        "host": socket.gethostname(),
        "submitted_at": dt.datetime.now().isoformat(timespec="seconds"),
        "versions": tool_versions(args.python),
        "platform": platform.platform(),
    }
    (run_root / "manifest.json").write_text(json.dumps(manifest, indent=2, ensure_ascii=False))
    (run_root / "slurm").mkdir()
    return run_root, manifest


def submit_slurm(run_root: Path, manifest: dict, args) -> None:
    n = manifest["planned_runs"]
    name = manifest["name"]
    tools = run_root / "frozen" / "tools"     # 凍結したスクリプトを使う
    env = f"ALL,EVAC_PYTHON={args.python}"
    array = [
        "sbatch", "--parsable", f"--job-name=va_{name}", f"--partition={args.partition}",
        f"--array=1-{n}%{args.max_parallel}", "--cpus-per-task=1", f"--mem={args.mem}",
        f"--time={args.time_limit}",
        f"--output={run_root}/slurm/task_%A_%a.out", f"--error={run_root}/slurm/task_%A_%a.out",
        f"--export={env}", str(tools / "slurm/run_task.sh"), str(run_root),
    ]
    finalize = [
        "sbatch", "--parsable", f"--job-name=va_{name}_finalize", f"--partition={args.partition}",
        "--dependency=afterany:{ARRAY_JOB_ID}", "--cpus-per-task=1", "--mem=4G", "--time=02:00:00",
        f"--output={run_root}/slurm/finalize_%j.out", f"--error={run_root}/slurm/finalize_%j.out",
        f"--export={env},EVAC_JIP_REV={ROOT}", str(tools / "finalize_experiment.sh"), str(run_root),
    ] + (["--dry-run-mail"] if args.dry_run_mail else [])
    cmd_file = run_root / "slurm" / "submit_commands.sh"
    cmd_file.write_text("#!/usr/bin/env bash\n# run_experiment.py が組み立てた投入コマンド\n"
                        + " ".join(shlex.quote(c) for c in array) + "\n"
                        + " ".join(shlex.quote(c) for c in finalize) + "\n")
    print(f"[experiment] {manifest['experiment_id']}  runs={n}")
    print(" ".join(shlex.quote(c) for c in array))
    print(" ".join(shlex.quote(c) for c in finalize))
    if args.print_only:
        print(f"[print-only] 投入していません({cmd_file})。不要なら {run_root} を削除してください。")
        return
    array_id = sh(array).split(";")[0]
    finalize = [c.replace("{ARRAY_JOB_ID}", array_id) for c in finalize]
    fin_id = sh(finalize).split(";")[0]
    m = json.loads((run_root / "manifest.json").read_text())
    m["slurm"] = {"array_job_id": array_id, "finalize_job_id": fin_id}
    (run_root / "manifest.json").write_text(json.dumps(m, indent=2, ensure_ascii=False))
    print(f"[submitted] array job {array_id} ({n} tasks, max {args.max_parallel} parallel), finalize job {fin_id}")
    print(f"  状況: squeue -u $USER   /   tools/experiment_status.sh {run_root}")
    print(f"  取消: scancel {array_id} {fin_id}")


def run_local(run_root: Path, manifest: dict, args) -> int:
    n = manifest["planned_runs"]
    tools = run_root / "frozen" / "tools"
    env = dict(os.environ, EVAC_PYTHON=args.python, EVAC_JIP_REV=str(ROOT))

    def one(i: int) -> int:
        with open(run_root / "slurm" / f"task_local_{i}.out", "w") as out:
            return subprocess.run([str(tools / "slurm/run_task.sh"), str(run_root), str(i)],
                                  stdout=out, stderr=subprocess.STDOUT, env=env).returncode

    print(f"[local] {manifest['experiment_id']}  runs={n}  parallel={args.max_parallel}")
    if args.print_only:
        return 0
    with ThreadPoolExecutor(max_workers=args.max_parallel) as ex:
        codes = list(ex.map(one, range(1, n + 1)))
    print(f"[local] finished: {codes.count(0)}/{n} exit 0")
    fin = [str(tools / "finalize_experiment.sh"), str(run_root)] + (["--dry-run-mail"] if args.dry_run_mail else [])
    return subprocess.run(fin, env=env).returncode


def main() -> int:
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("spec", help="experiments/*.toml")
    p.add_argument("--name", help="実験名(既定は spec の name)")
    p.add_argument("--backend", choices=["slurm", "local"], default="slurm")
    p.add_argument("--print-only", action="store_true", help="投入せずにコマンドを表示する")
    p.add_argument("--ref", help="凍結するコードの git 参照(計算機では origin/jip/revision-26S015f)")
    p.add_argument("--allow-dirty", action="store_true", help="未コミットの変更を許す(試験用)")
    p.add_argument("--seeds", help="例: 1-2(試験用にシード範囲を上書き)")
    p.add_argument("--layouts", help="例: 1-1-2(試験用に構成を絞る)")
    p.add_argument("--set", action="append", help="全実行に追加する設定 KEY=VALUE(試験用)")
    p.add_argument("--max-parallel", type=int, default=int(os.environ.get("EVAC_MAX_PARALLEL", "32")))
    p.add_argument("--partition", default=os.environ.get("EVAC_PARTITION", "ubuntu"))
    p.add_argument("--mem", default="4G")
    p.add_argument("--time-limit", default="02:00:00")
    p.add_argument("--runs-root", default="runs")
    p.add_argument("--python", default=os.environ.get("EVAC_PYTHON", "python3"))
    p.add_argument("--dry-run-mail", action="store_true", help="メールを送らずに件名と本文を表示する")
    args = p.parse_args()
    run_root, manifest = build(args)
    print(f"[prepared] {run_root}")
    if args.backend == "slurm":
        submit_slurm(run_root, manifest, args)
        return 0
    return run_local(run_root, manifest, args)


if __name__ == "__main__":
    sys.exit(main())
