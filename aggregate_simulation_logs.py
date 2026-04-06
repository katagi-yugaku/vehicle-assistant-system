#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
slurm の .out ログを条件別に集計し、平均値と CDF を出力するスクリプト。

主な機能:
- .out ファイルのみを対象に集計（.err は無視）
- ファイル名から scenario / early_rate / v2v_rate / run_id / jobid を自動抽出
- 条件 (scenario, early_rate, v2v_rate) ごとに集計
- ped_init_..._末尾番号 を init_... に正規化
- count 系の平均値を算出
- arrival_time_by_vehID_dict の vehicle ごとの平均到着時刻を算出
- 条件ごとに CDF グラフ (PNG) を出力
- vehicle_abandant_time / walking_distance の平均を算出
- 欠損 run / パース失敗 run を記録
- 結果を JSON に保存

実行例:
    python aggregate_simulation_logs.py ./logs
    python aggregate_simulation_logs.py ./logs --output-dir ./aggregated
    python aggregate_simulation_logs.py ./logs --expected-max-run-id 50
"""

from __future__ import annotations

import argparse
import ast
import json
import math
import os
import re
import sys
from collections import defaultdict
from dataclasses import dataclass
from typing import Any, DefaultDict, Dict, Iterable, List, Optional, Tuple

import matplotlib.pyplot as plt


# =========================================
# 拡張しやすい定数定義
# =========================================

OUTPUT_JSON_FILE = "simulation_averages.json"

# 想定 run_id の上限。必要に応じて CLI 引数で上書き可能。
DEFAULT_EXPECTED_MAX_RUN_ID = 50

# 集計対象の count 系キー
# ユーザー指定のキーをベースにしつつ、実ログ例で登場したキーも追加している。
COUNT_KEYS = [
    "obtain_info_lane_change_count",
    "elapsed_time_lane_change_count",
    "normalcy_bias_count",
    "negative_majority_bias_count",
    "positive_majority_bias_count",
    "lane_changed_vehicle_count",
    "route_changed_vehicle_count",
    "normalcy_bias_route_change_count",
    "majority_bias_route_change_count",
    "shelter_congestion_count",
    "shelter_capacity_full_count",
    "pedestrian_count",
    # 実ログ例との互換性のため追加
    "majority_bias_lanechange_count",
    "normalcy_bias_lanechange_count",
]

# 集計対象の dict 系キー
DICT_KEYS = [
    "arrival_time_by_vehID_dict",
    "vehicle_abandant_time_by_pedestrianID_dict",
    "walking_distance_by_pedestrianID_dict",
]

# canonical key -> alias 一覧
# 先頭が canonical 名で、後続が別名
ALIAS_MAP = {
    "obtain_info_lane_change_count": [
        "obtain_info_lane_change_count",
        "info_obtained_lanechange_count",
        "info_obtained_lane_change_count",
        "obtain_info_lanechange_count",
    ],
    "elapsed_time_lane_change_count": [
        "elapsed_time_lane_change_count",
        "elapsed_time_lanechange_count",
    ],
    "normalcy_bias_count": [
        "normalcy_bias_count",
        "normalcy_bias_lanechange_count",
    ],
    "negative_majority_bias_count": [
        "negative_majority_bias_count",
    ],
    "positive_majority_bias_count": [
        "positive_majority_bias_count",
    ],
    "lane_changed_vehicle_count": [
        "lane_changed_vehicle_count",
    ],
    "route_changed_vehicle_count": [
        "route_changed_vehicle_count",
    ],
    "normalcy_bias_route_change_count": [
        "normalcy_bias_route_change_count",
    ],
    "majority_bias_route_change_count": [
        "majority_bias_route_change_count",
    ],
    "shelter_congestion_count": [
        "shelter_congestion_count",
    ],
    "shelter_capacity_full_count": [
        "shelter_capacity_full_count",
    ],
    "pedestrian_count": [
        "pedestrian_count",
    ],
    "majority_bias_lanechange_count": [
        "majority_bias_lanechange_count",
    ],
    "normalcy_bias_lanechange_count": [
        "normalcy_bias_lanechange_count",
    ],
    "arrival_time_by_vehID_dict": [
        "arrival_time_by_vehID_dict",
    ],
    "vehicle_abandant_time_by_pedestrianID_dict": [
        "vehicle_abandant_time_by_pedestrianID_dict",
        "vehicle_abandon_time_by_pedestrianID_dict",
    ],
    "walking_distance_by_pedestrianID_dict": [
        "walking_distance_by_pedestrianID_dict",
    ],
}

# .out ファイル名規則
OUT_FILENAME_RE = re.compile(
    r"^va_s(?P<scenario>\d+)_e(?P<early_rate>\d+(?:\.\d+)?)_v(?P<v2v_rate>\d+(?:\.\d+)?)_r(?P<run_id>\d+)_(?P<jobid>\d+)\.out$"
)

# ped ID の正規化用
PED_ID_RE = re.compile(r"^ped_(init_.+)_\d+$")


# =========================================
# データ構造
# =========================================

@dataclass(frozen=True)
class LogFileInfo:
    """ファイル名から抽出したメタ情報"""
    path: str
    filename: str
    scenario: int
    early_rate: float
    v2v_rate: float
    run_id: int
    jobid: int

    @property
    def condition_key(self) -> str:
        return build_condition_key(self.scenario, self.early_rate, self.v2v_rate)


# =========================================
# 汎用ユーティリティ
# =========================================

def warn(message: str) -> None:
    """warning を標準出力に出す"""
    print(f"[WARNING] {message}", file=sys.stdout)


def info(message: str) -> None:
    """通常情報を標準出力に出す"""
    print(f"[INFO] {message}", file=sys.stdout)


def build_condition_key(scenario: int, early_rate: float, v2v_rate: float) -> str:
    """条件キー文字列を生成する"""
    return f"s{scenario}_e{early_rate}_v{v2v_rate}"


def safe_mean(values: Iterable[float]) -> Optional[float]:
    """空配列なら None、そうでなければ平均を返す"""
    values = list(values)
    if not values:
        return None
    return sum(values) / len(values)


def sort_numeric_strings_as_numbers(keys: Iterable[str]) -> List[str]:
    """ID 等のキーを文字列として安定ソートする"""
    return sorted(keys)


def ensure_output_dir(path: str) -> None:
    """出力ディレクトリを作成する"""
    os.makedirs(path, exist_ok=True)


def normalize_vehicle_id(raw_id: str) -> str:
    """
    ped_init_..._末尾番号 を init_... に正規化する。
    例:
        ped_init_ShelterA_1_286_3 -> init_ShelterA_1_286
        init_ShelterA_1_286       -> init_ShelterA_1_286
    """
    raw_id = raw_id.strip()
    match = PED_ID_RE.match(raw_id)
    if match:
        return match.group(1)
    return raw_id


def normalize_id_value_dict(raw_dict: Dict[Any, Any]) -> Dict[str, float]:
    """
    dict のキーを vehicle ID に正規化し、値を float 化する。
    同一 run 内で正規化後に同じ key が衝突した場合は後勝ちにする。
    """
    normalized: Dict[str, float] = {}
    for key, value in raw_dict.items():
        normalized_key = normalize_vehicle_id(str(key))
        try:
            normalized_value = float(value)
        except (TypeError, ValueError):
            continue
        normalized[normalized_key] = normalized_value
    return normalized


# =========================================
# ファイル探索・ファイル名解析
# =========================================

def list_out_files(log_dir: str) -> List[str]:
    """ログディレクトリ内の .out ファイル一覧を取得する"""
    if not os.path.isdir(log_dir):
        raise FileNotFoundError(f"ログディレクトリが存在しません: {log_dir}")

    out_files: List[str] = []
    for entry in os.listdir(log_dir):
        if entry.endswith(".out"):
            out_files.append(os.path.join(log_dir, entry))
    out_files.sort()
    return out_files


def parse_out_filename(path: str) -> Optional[LogFileInfo]:
    """ファイル名から条件情報を抽出する"""
    filename = os.path.basename(path)
    match = OUT_FILENAME_RE.match(filename)
    if not match:
        warn(f"ファイル名規則に一致しないためスキップします: {filename}")
        return None

    try:
        return LogFileInfo(
            path=path,
            filename=filename,
            scenario=int(match.group("scenario")),
            early_rate=float(match.group("early_rate")),
            v2v_rate=float(match.group("v2v_rate")),
            run_id=int(match.group("run_id")),
            jobid=int(match.group("jobid")),
        )
    except ValueError as exc:
        warn(f"ファイル名の数値変換に失敗したためスキップします: {filename} ({exc})")
        return None


def deduplicate_by_condition_and_run(files: List[LogFileInfo]) -> Dict[str, List[LogFileInfo]]:
    """
    同一 (condition, run_id) に複数ファイルがある場合は jobid が最大のものを採用する。
    """
    selected: Dict[Tuple[str, int], LogFileInfo] = {}

    for file_info in files:
        key = (file_info.condition_key, file_info.run_id)
        if key not in selected:
            selected[key] = file_info
            continue

        existing = selected[key]
        if file_info.jobid > existing.jobid:
            warn(
                f"同一条件・同一 run_id の重複を検出しました。"
                f" jobid={existing.jobid} より jobid={file_info.jobid} を優先します: "
                f"{file_info.condition_key}, run_id={file_info.run_id}"
            )
            selected[key] = file_info
        else:
            warn(
                f"同一条件・同一 run_id の重複を検出しました。"
                f" jobid={existing.jobid} を維持し、jobid={file_info.jobid} は無視します: "
                f"{file_info.condition_key}, run_id={file_info.run_id}"
            )

    grouped: DefaultDict[str, List[LogFileInfo]] = defaultdict(list)
    for file_info in selected.values():
        grouped[file_info.condition_key].append(file_info)

    for condition_key in grouped:
        grouped[condition_key].sort(key=lambda x: x.run_id)

    return dict(grouped)


# =========================================
# ログ本文解析
# =========================================

def read_text_file(path: str) -> Optional[str]:
    """テキストファイルを安全に読み込む"""
    try:
        with open(path, "r", encoding="utf-8", errors="replace") as f:
            return f.read()
    except OSError as exc:
        warn(f"ファイル読み込みに失敗しました: {path} ({exc})")
        return None


def build_alias_lookup(alias_map: Dict[str, List[str]]) -> Dict[str, str]:
    """
    alias -> canonical の逆引きマップを生成する。
    使わないが、拡張時に便利なので残している。
    """
    lookup: Dict[str, str] = {}
    for canonical, aliases in alias_map.items():
        for alias in aliases:
            lookup[alias] = canonical
    return lookup


def extract_scalar_by_aliases(text: str, aliases: List[str]) -> Optional[float]:
    """
    alias 群のいずれかについて、
    例: key_name:123
    のような 1 行値を抽出して float に変換する。
    """
    for alias in aliases:
        pattern = re.compile(rf"^\s*{re.escape(alias)}\s*:\s*(.+?)\s*$", re.MULTILINE)
        match = pattern.search(text)
        if not match:
            continue

        raw_value = match.group(1).strip()
        try:
            return float(raw_value)
        except ValueError:
            warn(f"数値パースに失敗しました: key={alias}, value={raw_value}")
            return None
    return None


def find_braced_literal_after_key(text: str, key_name: str) -> Optional[str]:
    """
    key_name:{...} の {...} 部分を、波括弧の対応を見ながら抽出する。
    複数行 dict にも対応する。
    """
    key_pattern = re.compile(rf"{re.escape(key_name)}\s*:\s*\{{", re.MULTILINE)
    match = key_pattern.search(text)
    if not match:
        return None

    brace_start = match.end() - 1
    depth = 0

    for idx in range(brace_start, len(text)):
        ch = text[idx]
        if ch == "{":
            depth += 1
        elif ch == "}":
            depth -= 1
            if depth == 0:
                return text[brace_start:idx + 1]

    return None


def extract_dict_by_aliases(text: str, aliases: List[str]) -> Optional[Dict[Any, Any]]:
    """
    alias 群のいずれかについて dict 文字列を抽出し、ast.literal_eval で dict 化する。
    """
    for alias in aliases:
        literal_text = find_braced_literal_after_key(text, alias)
        if literal_text is None:
            continue

        try:
            parsed = ast.literal_eval(literal_text)
        except (ValueError, SyntaxError) as exc:
            warn(f"辞書パースに失敗しました: key={alias} ({exc})")
            return None

        if not isinstance(parsed, dict):
            warn(f"dict ではない値が見つかりました: key={alias}")
            return None

        return parsed

    return None


def parse_log_content(text: str) -> Tuple[Dict[str, Optional[float]], Dict[str, Dict[str, float]], List[str]]:
    """
    ログ本文から集計対象を抽出する。

    戻り値:
        count_values: canonical count key -> float or None
        dict_values : canonical dict key -> normalized dict
        messages    : warning 相当の補足メッセージ
    """
    count_values: Dict[str, Optional[float]] = {}
    dict_values: Dict[str, Dict[str, float]] = {}
    messages: List[str] = []

    # count 系
    for canonical_key in COUNT_KEYS:
        aliases = ALIAS_MAP.get(canonical_key, [canonical_key])
        value = extract_scalar_by_aliases(text, aliases)
        count_values[canonical_key] = value
        if value is None:
            messages.append(f"count key が見つかりませんでした: {canonical_key}")

    # dict 系
    for canonical_key in DICT_KEYS:
        aliases = ALIAS_MAP.get(canonical_key, [canonical_key])
        raw_dict = extract_dict_by_aliases(text, aliases)
        if raw_dict is None:
            dict_values[canonical_key] = {}
            messages.append(f"dict key が見つかりませんでした: {canonical_key}")
        else:
            dict_values[canonical_key] = normalize_id_value_dict(raw_dict)

    return count_values, dict_values, messages


# =========================================
# 条件別集計
# =========================================

def aggregate_condition(
    condition_key: str,
    files: List[LogFileInfo],
    output_dir: str,
    expected_max_run_id: int,
) -> Dict[str, Any]:
    """
    1 条件分の集計を実施する。
    """
    if not files:
        raise ValueError(f"files が空です: {condition_key}")

    scenario = files[0].scenario
    early_rate = files[0].early_rate
    v2v_rate = files[0].v2v_rate

    expected_run_ids = list(range(1, expected_max_run_id + 1))
    found_run_ids = sorted({f.run_id for f in files})
    missing_run_ids = sorted(set(expected_run_ids) - set(found_run_ids))

    parsed_run_ids: List[int] = []
    failed_run_ids: List[int] = []

    # count 系: key ごとに run 横断で値を蓄積
    count_accumulator: DefaultDict[str, List[float]] = defaultdict(list)

    # arrival_time: vehicle_id ごとに run 横断で arrival time を蓄積
    arrival_time_accumulator: DefaultDict[str, List[float]] = defaultdict(list)

    # abandonment: 全イベント平均 + vehicle ごと平均
    abandon_time_all_events: List[float] = []
    walking_distance_all_events: List[float] = []
    abandon_time_by_vehicle: DefaultDict[str, List[float]] = defaultdict(list)
    walking_distance_by_vehicle: DefaultDict[str, List[float]] = defaultdict(list)

    for file_info in files:
        text = read_text_file(file_info.path)
        if text is None:
            failed_run_ids.append(file_info.run_id)
            continue

        count_values, dict_values, messages = parse_log_content(text)

        # 何も取れなかった場合は失敗扱い
        has_any_count = any(value is not None for value in count_values.values())
        has_any_dict = any(bool(value_dict) for value_dict in dict_values.values())
        if not (has_any_count or has_any_dict):
            failed_run_ids.append(file_info.run_id)
            warn(
                f"必要なキーを抽出できなかったため失敗扱いにします: "
                f"{file_info.filename}"
            )
            for message in messages:
                warn(f"{file_info.filename}: {message}")
            continue

        parsed_run_ids.append(file_info.run_id)

        # 部分的な欠損は warning のみ
        for message in messages:
            warn(f"{file_info.filename}: {message}")

        # count 系集計
        for key, value in count_values.items():
            if value is not None and not math.isnan(value):
                count_accumulator[key].append(value)

        # arrival_time 集計
        arrival_dict = dict_values.get("arrival_time_by_vehID_dict", {})
        for vehicle_id, arrival_time in arrival_dict.items():
            arrival_time_accumulator[vehicle_id].append(arrival_time)

        # abandonment time 集計
        abandon_dict = dict_values.get("vehicle_abandant_time_by_pedestrianID_dict", {})
        for vehicle_id, abandon_time in abandon_dict.items():
            abandon_time_all_events.append(abandon_time)
            abandon_time_by_vehicle[vehicle_id].append(abandon_time)

        # walking distance 集計
        walking_dict = dict_values.get("walking_distance_by_pedestrianID_dict", {})
        for vehicle_id, walking_distance in walking_dict.items():
            walking_distance_all_events.append(walking_distance)
            walking_distance_by_vehicle[vehicle_id].append(walking_distance)

    # 重複 run の可能性は事前に除外済みだが、念のためソート & unique 化
    parsed_run_ids = sorted(set(parsed_run_ids))
    failed_run_ids = sorted(set(failed_run_ids))

    # count 平均
    count_averages: Dict[str, Optional[float]] = {}
    for key in COUNT_KEYS:
        count_averages[key] = safe_mean(count_accumulator.get(key, []))

    # arrival_time の vehicle ごと平均
    vehicle_mean_arrival_time: Dict[str, float] = {}
    for vehicle_id, values in arrival_time_accumulator.items():
        mean_value = safe_mean(values)
        if mean_value is not None:
            vehicle_mean_arrival_time[vehicle_id] = mean_value

    # abandonment の vehicle ごと平均
    vehicle_mean_abandon_time: Dict[str, float] = {}
    for vehicle_id, values in abandon_time_by_vehicle.items():
        mean_value = safe_mean(values)
        if mean_value is not None:
            vehicle_mean_abandon_time[vehicle_id] = mean_value

    vehicle_mean_walking_distance: Dict[str, float] = {}
    for vehicle_id, values in walking_distance_by_vehicle.items():
        mean_value = safe_mean(values)
        if mean_value is not None:
            vehicle_mean_walking_distance[vehicle_id] = mean_value

    # CDF 出力
    cdf_plot_file = create_cdf_plot(
        condition_key=condition_key,
        values=list(vehicle_mean_arrival_time.values()),
        output_dir=output_dir,
    )

    result: Dict[str, Any] = {
        "scenario": scenario,
        "early_rate": early_rate,
        "v2v_rate": v2v_rate,
        "run_status": {
            "expected_run_ids": expected_run_ids,
            "found_run_ids": found_run_ids,
            "missing_run_ids": missing_run_ids,
            "parsed_run_ids": parsed_run_ids,
            "failed_run_ids": failed_run_ids,
        },
        "count_averages": count_averages,
        "arrival_time": {
            "vehicle_mean_arrival_time": {
                vehicle_id: vehicle_mean_arrival_time[vehicle_id]
                for vehicle_id in sort_numeric_strings_as_numbers(vehicle_mean_arrival_time.keys())
            },
            "cdf_plot_file": cdf_plot_file,
        },
        "abandonment": {
            "mean_abandon_time": safe_mean(abandon_time_all_events),
            "mean_walking_distance": safe_mean(walking_distance_all_events),
            "vehicle_mean_abandon_time": {
                vehicle_id: vehicle_mean_abandon_time[vehicle_id]
                for vehicle_id in sort_numeric_strings_as_numbers(vehicle_mean_abandon_time.keys())
            },
            "vehicle_mean_walking_distance": {
                vehicle_id: vehicle_mean_walking_distance[vehicle_id]
                for vehicle_id in sort_numeric_strings_as_numbers(vehicle_mean_walking_distance.keys())
            },
        },
    }

    return result


# =========================================
# CDF 出力
# =========================================

def compute_cdf_points(values: List[float]) -> Tuple[List[float], List[float]]:
    """
    CDF 用の x, y を作る。
    y = i / n (1 始まり)
    """
    if not values:
        return [], []

    sorted_values = sorted(values)
    n = len(sorted_values)
    x = sorted_values
    y = [(i + 1) / n for i in range(n)]
    return x, y


def create_cdf_plot(condition_key: str, values: List[float], output_dir: str) -> Optional[str]:
    """
    条件ごとに CDF グラフを PDF 出力する。
    値が空なら None を返す。
    """
    x, y = compute_cdf_points(values)
    if not x:
        warn(f"CDF を作成する値が存在しません: {condition_key}")
        return None

    filename = f"cdf_{condition_key}.pdf"
    output_path = os.path.join(output_dir, filename)

    plt.figure(figsize=(8, 6))
    plt.plot(x, y, marker="o", linestyle="-")
    plt.xlabel("Mean arrival time")
    plt.ylabel("Cumulative distribution")
    plt.title(f"CDF of Mean Arrival Time ({condition_key})")
    plt.grid(True)
    plt.tight_layout()
    plt.savefig(output_path, dpi=150)
    plt.close()

    return filename


# =========================================
# JSON 出力
# =========================================

def write_json(data: Dict[str, Any], output_path: str) -> None:
    """結果 JSON を保存する"""
    with open(output_path, "w", encoding="utf-8") as f:
        json.dump(data, f, ensure_ascii=False, indent=2, sort_keys=True)


# =========================================
# 全体処理
# =========================================

def aggregate_all_conditions(
    log_dir: str,
    output_dir: str,
    expected_max_run_id: int,
    json_filename: str,
) -> Dict[str, Any]:
    """全条件を集計し、JSON と CDF を出力する"""
    ensure_output_dir(output_dir)

    out_paths = list_out_files(log_dir)
    info(f".out ファイル数: {len(out_paths)}")

    parsed_files: List[LogFileInfo] = []
    for path in out_paths:
        file_info = parse_out_filename(path)
        if file_info is not None:
            parsed_files.append(file_info)

    info(f"ファイル名の解析に成功した .out ファイル数: {len(parsed_files)}")

    grouped = deduplicate_by_condition_and_run(parsed_files)
    info(f"検出された条件数: {len(grouped)}")

    result: Dict[str, Any] = {}
    for condition_key in sorted(grouped.keys()):
        info(f"集計中: {condition_key}")
        condition_result = aggregate_condition(
            condition_key=condition_key,
            files=grouped[condition_key],
            output_dir=output_dir,
            expected_max_run_id=expected_max_run_id,
        )
        result[condition_key] = condition_result

    json_output_path = os.path.join(output_dir, json_filename)
    write_json(result, json_output_path)
    info(f"JSON を出力しました: {json_output_path}")

    return result

def build_output_dir_from_scenario_name(scenario_name: str) -> str:
    """
    aggregate_simulation_logs.py が置かれている
    vehicle-assistant-system/ を基準にして、
    scenarios/{scenario_name}/map_one/results を返す
    """
    repo_root = os.path.dirname(os.path.abspath(__file__))
    return os.path.join(
        repo_root,
        "scenarios",
        scenario_name,
        "map_one",
        "results",
    )

# =========================================
# CLI
# =========================================

def parse_args() -> argparse.Namespace:
    """コマンドライン引数を解析する"""
    parser = argparse.ArgumentParser(
        description="slurm の .out シミュレーションログを条件別に集計し、平均値と CDF を出力する"
    )
    parser.add_argument(
        "log_dir",
        help="ログディレクトリのパス (.out / .err が混在していてよい)",
    )
    parser.add_argument(
        "--output-dir",
        default=None,
        help="出力先ディレクトリ。未指定時は log_dir 配下に出力する",
    )
    parser.add_argument(
        "--expected-max-run-id",
        type=int,
        default=DEFAULT_EXPECTED_MAX_RUN_ID,
        help=f"想定する最大 run_id。既定値: {DEFAULT_EXPECTED_MAX_RUN_ID}",
    )
    parser.add_argument(
        "--json-file",
        default=OUTPUT_JSON_FILE,
        help=f"出力 JSON ファイル名。既定値: {OUTPUT_JSON_FILE}",
    )
    parser.add_argument(
    "--scenario-name",
    default=None,
    help="scenarios/{scenario_name}/map_one/results を出力先として使うためのシナリオ名。例: its105",
    )
    return parser.parse_args()

def main() -> int:
    """エントリポイント"""
    args = parse_args()

    log_dir = os.path.abspath(args.log_dir)

    if args.scenario_name:
        output_dir = build_output_dir_from_scenario_name(args.scenario_name)
    elif args.output_dir is not None:
        output_dir = os.path.abspath(args.output_dir)
    else:
        output_dir = log_dir

    try:
        aggregate_all_conditions(
            log_dir=log_dir,
            output_dir=output_dir,
            expected_max_run_id=args.expected_max_run_id,
            json_filename=args.json_file,
        )
    except Exception as exc:
        print(f"[ERROR] {exc}", file=sys.stderr)
        return 1

    return 0

if __name__ == "__main__":
    sys.exit(main())