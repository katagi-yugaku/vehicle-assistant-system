from __future__ import annotations
from dataclasses import dataclass
from pathlib import Path
import argparse
import os
from concurrent.futures import ProcessPoolExecutor, as_completed
import subprocess
import re
import ast
import statistics
from collections import defaultdict
import sys
import json
import matplotlib.pyplot as plt
import numpy as np
import datetime
from pathlib import Path
from datetime import datetime
import json
import numpy as np
import matplotlib.pyplot as plt
import git
from typing import Any, Dict, List, Optional

_ID_RE = re.compile(r'^(?:init|newveh)_ShelterA_1_(\d+)(?:_\d+)?$')

# --- ユーザー設定 ---
# 実行するシミュレーションスクリプトのモジュール名
SIM_SCRIPTS = [
    "its102.map_one.simulation.runner",
    "its102.map_one.simulation.runner_nosystem"
]

# シミュレーションの固定引数
STATIC_ARGS = ["--nogui", "5.0", "30"]

# 結果を保存するJSONファイル名
OUTPUT_JSON_FILE = "simulation_averages.json"
NUM_RUNS = 2
early_rate_list = [0.1, 0.5, 0.9]
vehicle_interval = 5.0

# ---- 収集対象キーをここで集中管理 ----
COUNT_KEYS = [
    "obtain_info_lane_change_count",
    "elapsed_time_lane_change_count",
    "normalcy_bias_count",
    "negative_majority_bias_count",
    "positive_majority_bias_count",
    "lane_changed_vehicle_count",
]
DICT_KEYS = [
    "arrival_time_by_vehID_dict",   # ここに辞書で受けたいキーを追加
]

# --- TOML loader（Py3.11+ は tomllib、Py3.10 は tomli を使用） ---
def load_toml(path: Path) -> dict:
    if sys.version_info >= (3, 11):
        import tomllib
        with path.open("rb") as f:
            return tomllib.load(f)
    else:
        import tomli
        with path.open("rb") as f:
            return tomli.load(f)

# --- 必要なら型を付けて扱いやすく ---
@dataclass(frozen=True)
class SimulationConfig:
    scenario_id: int
    num_simulations: int
    comm_range: float
    num_vehicles: int
    vehicle_interval: float
    active_route_change_mean: float
    active_route_change_var: float
    cautious_route_change_mean: float
    cautious_route_change_var: float
    shelter_capacity_threshold: float
    tsunami_sign_start_time: float
    tsunami_sign_end_time: float
    motivation_threshold_start: float
    motivation_threshold_end: float
    min_motivation_start: float
    min_motivation_end: float
    positive_lanechange_start: float
    positive_lanechange_end: float
    negative_lanechange_start: float
    negative_lanechange_end: float
    driver_visibility_distance: float
    positive_majority_bias: float
    negative_majority_bias: float

    @staticmethod
    def from_dict(d: dict) -> "SimulationConfig":
        return SimulationConfig(**d)

# ===== Git branch helpers =====
def get_current_branch_name(repo: git.Repo) -> Optional[str]:
    try:
        return repo.active_branch.name
    except Exception:
        return None  # detached HEAD など

def branch_exists_local(repo: git.Repo, name: str) -> bool:
    return any(h.name.split("/")[-1] == name for h in repo.heads)

def branch_exists_remote(repo: git.Repo, name: str, remote_name: str = "origin") -> bool:
    try:
        remote = repo.remote(remote_name)
        remote.fetch(prune=True)
        return any(ref.name.split("/")[-1] == name for ref in remote.refs)
    except Exception:
        return False

def make_unique_branch_name(repo: git.Repo, base_prefix: str = "sim") -> str:
    """
    例: sim/2025-11-10_18-22-05_001
    既存と被れば 002, 003 ... を自動付与。
    """
    ts = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
    idx = 1
    while True:
        candidate = f"{base_prefix}/{ts}_{idx:03d}"
        if not branch_exists_local(repo, candidate) and not branch_exists_remote(repo, candidate):
            return candidate
        idx += 1

def ensure_on_unique_work_branch(repo: git.Repo,
                                 remote_name: str = "origin",
                                 base_branch: str = "main",
                                 prefix: str = "sim") -> str:
    """
    - 現在 main にいたら pull --rebase --autostash
    - main をベースに一意の作業ブランチを作成して checkout
    - 作業ブランチ名を返す
    """
    cur = get_current_branch_name(repo)
    if cur == base_branch:
        try:
            repo.git.pull("--rebase", "--autostash", remote_name, base_branch)
            print(f"[OK] git pull --rebase --autostash {remote_name} {base_branch}")
        except Exception as e:
            print(f"[WARN] pull 失敗（続行）: {e}")

    # 未コミットがあると checkout できないので自動 stash（安全にしたいなら手動に変更可）
    if repo.is_dirty(untracked_files=True):
        try:
            repo.git.stash("push", "-u", "-m", "auto-stash before sim branch checkout")
            print("[OK] auto-stash 完了")
        except Exception as e:
            print(f"[WARN] auto-stash 失敗（続行）: {e}")

    work_branch = make_unique_branch_name(repo, base_prefix=prefix)

    # ベースを main に固定してブランチ作成
    try:
        repo.git.checkout(base_branch)
    except Exception:
        pass
    repo.git.checkout("-b", work_branch)
    print(f"[OK] 作業ブランチ作成: {work_branch} (base={base_branch})")
    return work_branch

def push_current_branch(repo: git.Repo, remote_name: str, branch: Optional[str] = None) -> None:
    """
    現在のブランチ（または指定ブランチ）を push。
    初回は -u で upstream 設定。2回目以降は通常 push。
    """
    if branch is None:
        branch = get_current_branch_name(repo)
    if not branch:
        print("[ERROR] push するブランチが特定できません（detached HEAD?）")
        return

    remote = repo.remote(remote_name)

    # 変更がなければスキップ
    if not repo.is_dirty(untracked_files=True):
        print("[INFO] 変更なし: push スキップ")
        return

    # upstream 未設定なら -u で設定
    try:
        tracking = repo.active_branch.tracking_branch()
    except Exception:
        tracking = None

    try:
        if tracking is None:
            # 初回
            repo.git.push("-u", remote_name, branch)
        else:
            # 2回目以降
            remote.push(branch)
    except Exception as e:
        print(f"[ERROR] push 失敗: {e}")
        return
    print(f"[OK] push 完了: {remote_name}/{branch}")

def git_add_commit_push(repo: git.Repo, paths: List[Path], remote_name: str, branch: Optional[str], message: Optional[str]):
    if not paths:
        print("[INFO] No files to commit; skip git operations.")
        return

    # リポジトリルート
    if not getattr(repo, "working_tree_dir", None):
        raise RuntimeError("Repo has no working_tree_dir (bare repo?)")
    repo_root = Path(repo.working_tree_dir).resolve()

    # すべて絶対化 & リポジトリ配下チェック
    abs_paths = [Path(p).resolve() for p in paths]
    for p in abs_paths:
        if repo_root not in p.parents and p != repo_root:
            raise ValueError(f"{p} is outside the repo root: {repo_root}")

    # 相対パスに揃えて add
    rels = [str(p.relative_to(repo_root)) for p in abs_paths]
    if not rels:
        print("[INFO] No relative paths; skip git operations.")
        return
    repo.index.add(rels)

    # 変更がなければコミットスキップ
    if not repo.is_dirty(untracked_files=True):
        print("[INFO] Nothing to commit; working tree clean.")
        return

    commit_msg = message or f"Add {len(paths)} file(s) at {datetime.now():%Y-%m-%d %H:%M}"
    repo.index.commit(commit_msg)
    print(f"[OK] commit: {commit_msg}")

    # push
    origin = repo.remote(name=remote_name)
    try:
        if branch:
            push_res = origin.push(refspec=f"HEAD:{branch}")
        else:
            push_res = origin.push()
    except Exception as e:
        print(f"[ERROR] push failed: {e}")
        return

    # 失敗検知
    try:
        from git.remote import PushInfo
        for r in push_res:
            if r.flags & PushInfo.ERROR:
                print(f"[ERROR] push failed: {r.summary}")
                return
    except Exception:
        # 古いGitPythonなどで PushInfo がない場合は簡易チェック
        pass

    print("[OK] push complete.")

def git_pull(repo: git.Repo, remote_name: str = "origin", branch: Optional[str] = None) -> None:
    """
    実行前の最新化。デタッチHEAD等に配慮しつつ --rebase --autostash 相当を実施。
    """
    # 可能なら現在のブランチ名を取得
    active_branch_name = None
    try:
        active_branch_name = repo.active_branch.name
    except Exception:
        pass

    target_branch = branch or active_branch_name
    try:
        # rebase/autostashはGitPythonの高レベルAPIにないため低レベルで実行
        if target_branch:
            repo.git.pull("--rebase", "--autostash", remote_name, target_branch)
        else:
            # ブランチ不明なら通常pull
            repo.remotes[remote_name].pull()
        print(f"[OK] git pull ({remote_name}/{target_branch or 'current'})")
    except Exception as e:
        print(f"git pull 失敗（処理続行）: {e}")

def get_repo() -> git.Repo:
    """
    現在のスクリプトのパスから、最も近いGitリポジトリを自動検出して返す。
    """
    current_path = Path(__file__).resolve()
    try:
        repo = git.Repo(current_path, search_parent_directories=True)
        print(f"使用中のGitリポジトリ: {repo.working_tree_dir}")
        return repo
    except git.exc.InvalidGitRepositoryError:
        raise RuntimeError("このスクリプトはGitリポジトリ内で実行されていません。")

def ensure_dir(p: Path) -> None:
    p.mkdir(parents=True, exist_ok=True)

def build_cdf_input_for_scenario(scn: dict) -> dict[float, list[float]]:
    """
    - system: 0.1, 0.5, 0.9 をそのまま
    - nosystem: 0.5 を 0.0 にマップ（描画スタイルで破線に）
    """
    out: dict[float, list[float]] = {}
    system_arr = scn.get("system", {}).get("average_arrival_times", {})
    nosys_arr = scn.get("nosystem", {}).get("average_arrival_times", {})

    # system 側
    if 0.1 in system_arr: out[0.1] = list(system_arr[0.1])
    if 0.5 in system_arr: out[0.5] = list(system_arr[0.5])
    if 0.9 in system_arr: out[0.9] = list(system_arr[0.9])

    # nosystem 側 0.5 → 0.0
    if 0.5 in nosys_arr:
        out[0.0] = list(nosys_arr[0.5])  # 破線で描くためのキー変換
    return out

def plot_cdfs_to_path(data_dict: dict[float, list[float]], save_path: Path) -> None:
    import numpy as np
    import matplotlib.pyplot as plt

    plt.figure(figsize=(10, 6))

    def label_for(key: float) -> str:
        # 表示ラベルは明確に
        if key == 0.1: return "system 0.1"
        if key == 0.5: return "system 0.5"
        if key == 0.9: return "system 0.9"
        if key == 0.0: return "nosystem 0.5"
        return f"key={key}"

    for key, value in data_dict.items():
        arr = np.sort(np.array(value, dtype=float))
        if arr.size == 0: 
            continue
        cdf = np.arange(1, arr.size + 1) / arr.size

        # あなたのスタイル規則を踏襲
        if key == 0.1:
            plt.plot(arr, cdf, label=label_for(key), color='r', linestyle='-')
        elif key == 0.5:
            plt.plot(arr, cdf, label=label_for(key), color='blue', linestyle='-')
        elif key == 0.9:
            plt.plot(arr, cdf, label=label_for(key), color='g', linestyle='-')
        elif key == 0.0:
            plt.plot(arr, cdf, label=label_for(key), color='blue', linestyle='--')
        else:
            plt.plot(arr, cdf, label=label_for(key))

    plt.xlim(200, 1200)
    plt.ylim(0.05, 1.0)
    plt.xticks(ticks=np.arange(200, 1310, 200), fontsize=20, fontweight='roman')
    plt.yticks(ticks=np.arange(0.1, 1.1, 0.1), fontsize=20, fontweight='roman')
    plt.legend(loc="lower right")
    plt.tight_layout()
    plt.savefig(save_path)
    print(f"Saved figure as: {save_path}")
    plt.close()


def write_log_for_scenario(scenario_id: str, scenario_obj: dict, out_dir: Path) -> None:
    ensure_dir(out_dir)
    log_path = out_dir / f"log_{scenario_id}.txt"

    # 構成
    ts = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
    system_arr = scenario_obj.get("system", {}).get("average_arrival_times", {})
    nosys_arr = scenario_obj.get("nosystem", {}).get("average_arrival_times", {})
    system_cnt = scenario_obj.get("system", {}).get("average_count_metrics", {})
    nosys_cnt = scenario_obj.get("nosystem", {}).get("average_count_metrics", {})

    lines = []
    lines.append(f"[RUN SUMMARY] scenario_id={scenario_id}")
    lines.append(f"timestamp: {ts}")
    # 推定で early_rate 一覧（system 側のキー＋nosystem 側のキーの和集合）
    er_set = sorted(set(system_arr.keys()) | set(nosys_arr.keys()))
    lines.append(f"early_rates_present: {er_set}")
    lines.append("")

    lines.append("[SYSTEM] average_arrival_times:")
    for k in sorted(system_arr.keys()):
        lines.append(f"  early_rate={k}: {system_arr[k]}")
    lines.append("")

    lines.append("[SYSTEM] average_count_metrics:")
    for k in sorted(system_cnt.keys()):
        lines.append(f"  early_rate={k}: {system_cnt[k]}")
    lines.append("")

    lines.append("[NOSYSTEM] average_arrival_times:")
    for k in sorted(nosys_arr.keys()):
        lines.append(f"  early_rate={k}: {nosys_arr[k]}")
    if 0.5 not in nosys_arr:
        lines.append("  (note) nosystem early_rate=0.5 が見つからないため CDF の key=0.0 は出力しません。")
    lines.append("")

    lines.append("[NOSYSTEM] average_count_metrics:")
    for k in sorted(nosys_cnt.keys()):
        lines.append(f"  early_rate={k}: {nosys_cnt[k]}")
    lines.append("")

    # 機械可読の原データ（all_results の該当シナリオ部）を最後にJSONで併記
    lines.append("[RAW_JSON]")
    lines.append(json.dumps(scenario_obj, ensure_ascii=False, indent=2))
    lines.append("note: CDFでは nosystem(early_rate=0.5) を key=0.0 にマップし、青の破線で描画しています。")

    with open(log_path, "w", encoding="utf-8") as f:
        f.write("\n".join(lines))

    print(f"Wrote log: {log_path}")

def save_outputs(all_results: dict) -> None:
    """
    all_results から各シナリオIDの結果を
    - log_<id>.txt
    - cdf_<id>.pdf
    として results/<id>/ に保存。
    """
    for scenario_id, scenario_obj in all_results.items():
        out_dir = RESULTS_ROOT / scenario_id
        ensure_dir(out_dir)

        # 1) ログ
        write_log_for_scenario(scenario_id, scenario_obj, out_dir)

        # 2) CDF 用データを組み立て＆保存
        cdf_data = build_cdf_input_for_scenario(scenario_obj)
        cdf_path = out_dir / f"cdf_{scenario_id}.pdf"
        if len(cdf_data) == 0:
            print(f"⚠️ CDF data is empty for scenario {scenario_id}; skip PDF.")
        else:
            plot_cdfs_to_path(cdf_data, cdf_path)

def parse_args():
    parser = argparse.ArgumentParser(description="Load a TOML and print variables")
    return parser.parse_args()

# ---- 共通ユーティリティ ----
def _parse_num(s: str):
    s = s.strip()
    try:
        return int(s)
    except ValueError:
        try:
            return float(s)
        except ValueError:
            return s  # 将来の拡張用に非数値も許容

def parse_stdout(stdout: str, dict_keys: list[str], count_keys: list[str]):
    """標準出力から、辞書・カウンタ・スカラーを抽出して返す共通関数。"""
    dict_results   = {key_name: {}    for key_name in dict_keys}
    count_results  = {count_key: 0    for count_key in count_keys}

    for raw_line in stdout.splitlines():
        line = raw_line.strip()
        if not line or ":" not in line:
            continue
        key_name, val_str = line.split(":", 1)
        key_name = key_name.strip()
        val_str = val_str.strip()

        if key_name in dict_results:
            try:
                dict_results[key_name] = ast.literal_eval(val_str)
            except Exception as e:
                print(f"Error parsing dict for {key_name}: {e}")
        elif key_name in count_results:
            try:
                count_results[key_name] = _parse_num(val_str)
            except Exception as e:
                print(f"Error parsing count for {key_name}: {e}")
            except Exception as e:
                print(f"Error parsing scalar for {key_name}: {e}")
    return dict_results, count_results

# 関数シグネチャを変更（サフィックスを付けられるように）
def run_simulation_with_system(script_name: str, config_path: str, early_rate: float, log_suffix: str = ""):
    # python3 -m scenarios.its102.map_one.simulation.runner_simulator --nogui scenarios/its102/configs/1.toml 0.5
    config_path = str(config_path)
    command = [
        "python3", "-m", script_name,
        "--nogui", config_path, str(early_rate)
        ]
    print(f"  実行中: {' '.join(command)}")
    try:
        result = subprocess.run(
            command,
            capture_output=True,
            text=True,
            check=True,
            encoding='utf-8'
        )
    except subprocess.CalledProcessError as e:
        print(f"  [致命的エラー] コマンド実行に失敗しました: {' '.join(command)}", file=sys.stderr)
        print(f"  Return Code: {e.returncode}", file=sys.stderr)
        print(f"  Stderr: {e.stderr}", file=sys.stderr)
        return None
    except Exception as e:
        print(f"  [致命的エラー] パース中に予期せぬエラーが発生しました: {e}", file=sys.stderr)
        print(f"  コマンド: {' '.join(command)}", file=sys.stderr)
        return None

    dict_res, count_res = parse_stdout(result.stdout, DICT_KEYS, COUNT_KEYS)
    arrival_time_by_target_vehID_dict = dict_res.get("arrival_time_by_vehID_dict", {})

    log_filename = f"log_{early_rate}{log_suffix}.txt"
    try:
        with open(log_filename, "w", encoding="utf-8") as f:
            f.write(f"{arrival_time_by_target_vehID_dict}\n")
            f.write(f"{count_res}\n")
    except Exception as e:
        print(f"Error writing to log file: {e}")

    return arrival_time_by_target_vehID_dict, count_res

def run_simulation_with_nosystem(script_name: str, config_path: str, early_rate: float, log_suffix: str = ""):
    config_path = str(config_path)
    command = [
        "python3", "-m", script_name,
        "--nogui", config_path, str(early_rate)
    ]
    print(f"  実行中: {' '.join(command)}")
    try:
        result = subprocess.run(
            command,
            capture_output=True,
            text=True,
            check=True,
            encoding="utf-8"
        )
    except subprocess.CalledProcessError as e:
        print(f"  [致命的エラー] コマンド実行に失敗しました: {' '.join(command)}", file=sys.stderr)
        print(f"  Return Code: {e.returncode}", file=sys.stderr)
        print(f"  Stderr: {e.stderr}", file=sys.stderr)
        return None
    except Exception as e:
        print(f"  [致命的エラー] パース中に予期せぬエラーが発生しました: {e}", file=sys.stderr)
        print(f"  コマンド: {' '.join(command)}", file=sys.stderr)
        return None

    # stdout から辞書・カウントを抽出（system と同じキー前提）
    dict_results, count_results = parse_stdout(result.stdout, DICT_KEYS, COUNT_KEYS)
    arrival_time_by_target_vehID_dict = dict_results.get("arrival_time_by_vehID_dict", {})

    # ログ（_nosystem を明示）
    log_filename = f"log_{early_rate}_nosystem{log_suffix}.txt"
    try:
        with open(log_filename, "w", encoding="utf-8") as f:
            f.write(f"{arrival_time_by_target_vehID_dict}\n")
            f.write(f"{count_results}\n")
    except Exception as e:
        print(f"Error writing to log file: {e}")

    return arrival_time_by_target_vehID_dict, count_results


def _run_once(mode: str, config_path: str, early_rate: float, run_index: int, script_name: str):
    if mode == "system":
        result_tuple = run_simulation_with_system(script_name, config_path, early_rate, log_suffix=f"_{run_index}")
    else:
        result_tuple = run_simulation_with_nosystem(script_name, config_path, early_rate, log_suffix=f"_{run_index}")
    return (mode, config_path, early_rate, run_index, result_tuple)

def compute_average_arrival_times(runvehID_arrival_time_dict_per_run_lists):
    """
    runvehID_arrival_time_dict_per_run_lists: 各シミュレーション(run)の
      {vehID(str): arrival_time(float)} を要素にもつリスト

    返り値: {基底ID(int): 平均到着時間(float)} をID昇順で並べたdict
    """
    times_by_base_id = defaultdict(list)

    for one_run in runvehID_arrival_time_dict_per_run_lists:
        for veh_key, arrival_time in one_run.items():
            match = _ID_RE.match(veh_key)
            if not match:
                # もし別フォーマットが混じっていたらスキップ（必要ならraiseに変更）
                continue
            base_id = int(match.group(1))  # 56や120など
            times_by_base_id[base_id].append(float(arrival_time))

    avg_by_id = {base_id: sum(times)/len(times) for base_id, times in times_by_base_id.items()}
    # IDで昇順ソートして辞書化
    return dict(sorted(avg_by_id.items(), key=lambda kv: kv[0]))

# __main__ の system 側ループを “並列化” 版に置き換え
if __name__ == "__main__":
    #  python3 -m scenarios.its102.map_one.simulation.runner_simulator --nogui scenarios/its102/configs/1.toml 0.5
    # Git リポジトリ検出＆最新化＋作業ブランチへ
    try:
        repo = get_repo()
        work_branch = ensure_on_unique_work_branch(
            repo=repo,
            remote_name="origin",
            base_branch="main",
            prefix="sim"  # 好みで "runs", "results" などに変更可
        )
        print(f"[INFO] 現在の作業ブランチ: {work_branch}")
    except Exception as e:
        print(f"[WARN] Git 初期化ステップをスキップ: {e}")
        repo = None
        work_branch = None

    # シナリオの TOML 一覧（例: its102/configs/*.toml）
    scenario_name = sys.argv[1]  # 例: "its102"
    script_name_with_system = f"scenarios.{scenario_name}.map_one.simulation.runner_simulator" ###実際にはmap_oneは飛ばしたい
    SCENARIO_DIR = Path(f"scenarios/{scenario_name}/configs")   # プロジェクト直下からの相対
    RESULTS_ROOT = Path(f"scenarios/{scenario_name}/map_one/results")
    scenario_config_paths = sorted(SCENARIO_DIR.glob("*.toml"))
    all_results = {}  # {scenario_name: {...既存のavg_* 構造...}}
    
    # 並列ワーカー数
    MAX_WORKERS = int(os.environ.get("MAX_WORKERS", max(1, (os.cpu_count() or 2) - 1)))

    for config_path in scenario_config_paths:
        print(f"=== [Scenario={config_path.name}] system 全ジョブを並列実行（workers={MAX_WORKERS}） ===")
        # 結果バッファ（early_rate ごとに veh/chg/cnt を貯める）
        system_runs = {
            early_rate: {"vehicle_results": [], "count_metrics_list": []}
            for early_rate in early_rate_list
        }

        with ProcessPoolExecutor(max_workers=MAX_WORKERS) as executor:
            futures = []
            for early_rate in early_rate_list:
                for run_index in range(NUM_RUNS):
                    futures.append(
                        executor.submit(_run_once, "system", config_path, early_rate, run_index, script_name_with_system)
                    )
            for future in as_completed(futures):
                mode, returned_config_path, returned_early_rate, returned_run_index, result_tuple = future.result()
                if result_tuple is None:
                    # エラーまたは中断時のスキップ（必要ならログ化可能）
                    continue

                vehicle_arrival_time_dict, count_metrics_dict = result_tuple
                system_runs[returned_early_rate]["vehicle_results"].append(vehicle_arrival_time_dict)
                system_runs[returned_early_rate]["count_metrics_list"].append(count_metrics_dict)
        # print(f"system 全ジョブ完了")
        # === system 集計（平均化） ===
        average_arrival_times_by_early_rate: dict[float, list[float]] = {}
        average_changed_vehicle_counts_by_early_rate: dict[float, float] = {}
        average_count_metrics_by_early_rate: dict[float, dict[str, float]] = {
            early_rate: {count_key: 0.0 for count_key in COUNT_KEYS}
            for early_rate in early_rate_list
        }

        for early_rate in early_rate_list:
            vehicle_runs_per_early_rate = system_runs[early_rate]["vehicle_results"]
            count_metrics_per_early_rate = system_runs[early_rate]["count_metrics_list"]

            # 各 early_rate の全試行で平均到着時間を計算
            if vehicle_runs_per_early_rate:
                averaged_arrival_time_dict = compute_average_arrival_times(vehicle_runs_per_early_rate)
                average_arrival_times_by_early_rate[early_rate] = list(averaged_arrival_time_dict.values())
            else:
                average_arrival_times_by_early_rate[early_rate] = []

            # 各カウント系メトリクスの平均値を算出
            if count_metrics_per_early_rate:
                for count_key in COUNT_KEYS:
                    count_values = [metrics.get(count_key, 0.0) for metrics in count_metrics_per_early_rate]
                    average_count_metrics_by_early_rate[early_rate][count_key] = (
                        sum(count_values) / len(count_values)
                    ) if count_values else 0.0
                # === nosystem も同じ early_rate_list × NUM_RUNS で並列実行 ===
        script_name_with_nosystem = f"scenarios.{scenario_name}.map_one.simulation.runner_simulator_nosystem"

        print(f"=== [Scenario={config_path.name}] nosystem 全ジョブを並列実行（workers={MAX_WORKERS}） ===")
        nosystem_runs_by_early_rate = {
            early_rate_value: {
                "vehicle_results": [],
                "count_metrics_list": [],
            }
            for early_rate_value in early_rate_list
        }

        with ProcessPoolExecutor(max_workers=MAX_WORKERS) as executor:
            futures_nosystem = []
            for early_rate_value in early_rate_list:
                for run_index in range(NUM_RUNS):
                    futures_nosystem.append(
                        executor.submit(
                            _run_once,
                            "nosystem",
                            config_path,
                            early_rate_value,
                            run_index,
                            script_name_with_nosystem,
                        )
                    )

            for future in as_completed(futures_nosystem):
                (
                    returned_mode,
                    returned_config_path,
                    returned_early_rate,
                    returned_run_index,
                    result_tuple,
                ) = future.result()

                if result_tuple is None:
                    continue

                vehicle_arrival_time_dict, count_metrics_dict = result_tuple
                nosystem_runs_by_early_rate[returned_early_rate]["vehicle_results"].append(
                    vehicle_arrival_time_dict
                )
                nosystem_runs_by_early_rate[returned_early_rate]["count_metrics_list"].append(
                    count_metrics_dict
                )

        # === nosystem 集計（平均化） ===
        average_arrival_times_nosystem_by_early_rate: dict[float, list[float]] = {}
        average_count_metrics_nosystem_by_early_rate: dict[float, dict[str, float]] = {
            early_rate_value: {count_key: 0.0 for count_key in COUNT_KEYS}
            for early_rate_value in early_rate_list
        }

        for early_rate_value in early_rate_list:
            vehicle_runs_for_nosystem = nosystem_runs_by_early_rate[early_rate_value]["vehicle_results"]
            count_metrics_runs_for_nosystem = nosystem_runs_by_early_rate[early_rate_value]["count_metrics_list"]

            if vehicle_runs_for_nosystem:
                averaged_arrival_time_dict_nosystem = compute_average_arrival_times(
                    vehicle_runs_for_nosystem
                )
                average_arrival_times_nosystem_by_early_rate[early_rate_value] = list(
                    averaged_arrival_time_dict_nosystem.values()
                )
            else:
                average_arrival_times_nosystem_by_early_rate[early_rate_value] = []

            if count_metrics_runs_for_nosystem:
                for count_key in COUNT_KEYS:
                    values_for_key = [m.get(count_key, 0.0) for m in count_metrics_runs_for_nosystem]
                    average_count_metrics_nosystem_by_early_rate[early_rate_value][count_key] = (
                        sum(values_for_key) / len(values_for_key)
                    ) if values_for_key else 0.0

        # === このシナリオの結果を統合して格納（system / nosystem分離で上書き防止） ===
        all_results[config_path.stem] = {
            "system": {
                "average_arrival_times": average_arrival_times_by_early_rate,
                "average_count_metrics": average_count_metrics_by_early_rate,
            },
            "nosystem": {
                "average_arrival_times": average_arrival_times_nosystem_by_early_rate,
                "average_count_metrics": average_count_metrics_nosystem_by_early_rate,
            },
        }
    print(f"=== system シナリオ完了 ===")
    print(f"all_results: {all_results}")    
    save_outputs(all_results)
    # === results/<scenario_id>/ 以下の生成物だけを選択コミット＆push ===
    try:
        if repo is None:
            raise RuntimeError("Git repo 未取得のためスキップ")

        results_root = RESULTS_ROOT.resolve()
        scenario_ids = list(all_results.keys())

        paths: List[Path] = []
        for sid in scenario_ids:
            d = results_root / sid
            paths.extend(d.glob("log_*.txt"))
            paths.extend(d.glob("cdf_*.pdf"))

        # add & commit（現在の作業ブランチに対して）
        git_add_commit_push(
            repo=repo,
            paths=paths,
            remote_name="origin",
            branch=None,  # None の場合は HEAD を push（後段の push_current_branch が制御）
            message=f"results: add logs & CDF for {', '.join(scenario_ids)}"
        )

        # 上の関数は commit まで。push は tracking の有無で挙動を分けたいので別関数で。
        push_current_branch(repo=repo, remote_name="origin")
    except Exception as e:
        print(f"git add/commit/push skipped: {e}")


