#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
slurm の .out ログを条件別に集計し、平均値と CDF を出力するスクリプト。

主な機能:
- .out ファイルのみを対象に集計（.err は無視）
- ファイル名から scenario / early_rate / v2v_rate / run_id / jobid を自動抽出
- 条件 (scenario, early_rate, v2v_rate) ごとに集計
- ped_init_..._末尾番号 を init_... に正規化
- 今回の Simlation Result Summary に含まれる scalar 系キーの平均値を算出
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
from datetime import datetime
from typing import Any, DefaultDict, Dict, Iterable, List, Optional, Tuple

import matplotlib.pyplot as plt

OUTPUT_JSON_FILE = "output.json"

COMPARISON_SYSTEM_EARLY_RATES = [0.1, 0.5, 0.9]
COMPARISON_SYSTEM_V2V_RATE = 1.0
COMPARISON_NOSYSTEM_EARLY_RATE = 0.5
COMPARISON_NOSYSTEM_V2V_RATE = 0.0

# output.json の average_count_metrics に出すキー。
# runner_simulator.py 側の以下の print 出力名に合わせる。
SUMMARY_COUNT_KEYS = [
    "avg_congestion_duration",
    "pedestrian_count",
    "route_changed_vehicle_count",
    "wrong_way_driving_count",
    "rate_vehicle_abandonment",  # 追加: 乗り捨て率
    "vehicle_abandonment_count",
    "normalcy_bias_route_change_count",
    "majority_bias_route_change_count",
    "lane_changed_vehicle_count",
    "info_obtained_lanechange_count",
    "elapsed_time_lanechange_count",
    "majority_bias_lanechange_count",
]


WALKING_DISTANCE_BIN_WIDTH = 50
WALKING_DISTANCE_BIN_EDGES = [
    200, 250, 300, 350, 400, 450, 500, 550,
    600, 650, 700, 750, 800, 850, 900, 950, 1000,
]

ABANDON_TIME_BIN_EDGES = [
    800, 900, 1000, 1100, 1200, 1300, 1400, 1500,
    1600, 1700, 1800, 1900, 2000, 2100, 2200, 2300,
]

def is_close_float(a: float, b: float, tol: float = 1e-9) -> bool:
    return abs(a - b) < tol

def build_scenario_output_dir(base_output_dir: str, scenario: int) -> str:
    return os.path.join(base_output_dir, f"scenario{scenario}")

def build_output_dir_from_scenario_name(scenario_name: str) -> str:
    repo_root = os.path.dirname(os.path.abspath(__file__))
    return os.path.join(repo_root, "scenarios", scenario_name, "map_one", "results")

# =========================================
# 拡張しやすい定数定義
# =========================================


# 想定 run_id の上限。必要に応じて CLI 引数で上書き可能。
DEFAULT_EXPECTED_MAX_RUN_ID = 50

# 集計対象の scalar 系キー。
# 今回のログ末尾の "===== Simlation Result Summary =====" で出力している
# print(...) のキー名を canonical 名として扱う。
COUNT_KEYS = [
    "avg_congestion_duration",
    "pedestrian_count",
    "route_changed_vehicle_count",
    "wrong_way_driving_count",
    "vehicle_abandonment_count",
    "rate_vehicle_abandonment",  # 追加: 乗り捨て率
    "normalcy_bias_route_change_count",
    "majority_bias_route_change_count",
    "lane_changed_vehicle_count",
    "info_obtained_lanechange_count",
    "elapsed_time_lanechange_count",
    "majority_bias_lanechange_count",
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
    # scalar 系
    "avg_congestion_duration": [
        "avg_congestion_duration",
        "average_congestion_duration",
    ],
    "pedestrian_count": [
        "pedestrian_count",
    ],
    "route_changed_vehicle_count": [
        "route_changed_vehicle_count",
    ],
    "wrong_way_driving_count": [
        "wrong_way_driving_count",
        "wrong_way_success_count",
    ],
    "rate_vehicle_abandonment": [
        "rate_vehicle_abandonment",
    ],
    
    "vehicle_abandonment_count": [
        "vehicle_abandonment_count",
        "vehicle_abandonment_success_count",
    ],
    "normalcy_bias_route_change_count": [
        "normalcy_bias_route_change_count",
    ],
    "majority_bias_route_change_count": [
        "majority_bias_route_change_count",
    ],
    "lane_changed_vehicle_count": [
        "lane_changed_vehicle_count",
    ],
    "info_obtained_lanechange_count": [
        "info_obtained_lanechange_count",
        # 旧ログ名との互換用
        "obtain_info_lane_change_count",
        "info_obtained_lane_change_count",
        "obtain_info_lanechange_count",
    ],
    "elapsed_time_lanechange_count": [
        "elapsed_time_lanechange_count",
        # 旧ログ名との互換用
        "elapsed_time_lane_change_count",
    ],
    "majority_bias_lanechange_count": [
        "majority_bias_lanechange_count",
        # 旧ログ名との互換用
        "positive_majority_bias_count",
    ],

    # dict 系
    "arrival_time_by_vehID_dict": [
        "arrival_time_by_vehID_dict",
    ],
    "vehicle_abandant_time_by_pedestrianID_dict": [
        "vehicle_abandant_time_by_pedestrianID_dict",
        # typo 修正後の名前にも対応
        "vehicle_abandon_time_by_pedestrianID_dict",
    ],
    "walking_distance_by_pedestrianID_dict": [
        "walking_distance_by_pedestrianID_dict",
    ],
}

# .out ファイル名規則
OUT_FILENAME_RE = re.compile(
    r"^va_s(?P<scenario>\d+)"
    r"(?:_(?P<mode>system|nosystem))?"
    r"_e(?P<early_rate>\d+(?:\.\d+)?)"
    r"_v(?P<v2v_rate>\d+(?:\.\d+)?)"
    r"_r(?P<run_id>\d+)"
    r"_(?P<jobid>\d+)\.out$"
)


# =========================================
# データ構造
# =========================================

@dataclass(frozen=True)
class LogFileInfo:
    """ファイル名から抽出したメタ情報"""
    path: str
    filename: str
    scenario: int
    mode: Optional[str]
    early_rate: float
    v2v_rate: float
    run_id: int
    jobid: int

    @property
    def condition_key(self) -> str:
        return build_condition_key(
            self.scenario,
            self.early_rate,
            self.v2v_rate,
            self.mode,
        )

# =========================================
# 汎用ユーティリティ
# =========================================

def warn(message: str) -> None:
    """warning を標準出力に出す"""
    print(f"[WARNING] {message}", file=sys.stdout)


def info(message: str) -> None:
    """通常情報を標準出力に出す"""
    print(f"[INFO] {message}", file=sys.stdout)


def build_condition_key(
    scenario: int,
    early_rate: float,
    v2v_rate: float,
    mode: Optional[str] = None,
) -> str:
    """条件キー文字列を生成する"""
    mode_part = f"_{mode}" if mode else ""
    return f"s{scenario}{mode_part}_e{early_rate}_v{v2v_rate}"


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


ORIGINAL_VEH_ID_RE = re.compile(
    r"^(?:ped_)?(?:init|newveh)_Shelter[A-Za-z]+_\d+_(\d+)(?:_\d+)*$"
)

def normalize_vehicle_id(raw_id: str) -> str:
    """
    車両ID・歩行者IDから元の車両番号を取り出す。

    examples:
        init_ShelterA_1_267 -> 267
        newveh_ShelterA_1_267_138 -> 267
        ped_init_ShelterA_1_197_44 -> 197
        newveh_ShelterA_1_197_45 -> 197
        ped_newveh_ShelterA_1_98_91_47 -> 98
    """
    raw_id = raw_id.strip()

    match = ORIGINAL_VEH_ID_RE.match(raw_id)
    if match is None:
        raise ValueError(f"Unexpected vehicle/pedestrian ID format: {raw_id}")

    return match.group(1)

def normalize_id_value_dict(raw_dict: Dict[Any, Any]) -> Dict[str, float]:
    normalized: Dict[str, float] = {}

    for key, value in raw_dict.items():
        normalized_key = normalize_vehicle_id(str(key))

        try:
            normalized_value = float(value)
        except (TypeError, ValueError):
            continue

        if normalized_key in normalized:
            warn(
                f"同一 run 内で元車両IDが重複しました: "
                f"original_id={normalized_key}, "
                f"old={normalized[normalized_key]}, new={normalized_value}, raw_id={key}"
            )

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
            mode=match.group("mode"),
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
        count_values: canonical scalar key -> float or None
        dict_values : canonical dict key -> normalized dict
        messages    : warning 相当の補足メッセージ
    """
    count_values: Dict[str, Optional[float]] = {}
    dict_values: Dict[str, Dict[str, float]] = {}
    messages: List[str] = []

    # scalar 系
    for canonical_key in COUNT_KEYS:
        aliases = ALIAS_MAP.get(canonical_key, [canonical_key])
        value = extract_scalar_by_aliases(text, aliases)
        count_values[canonical_key] = value
        if value is None:
            messages.append(f"scalar key が見つかりませんでした: {canonical_key}")

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


def build_histogram_from_values(values: List[float], bin_edges: List[float]) -> Dict[str, Any]:
    """
    指定した bin_edges に従って度数分布を作る。
    区間は [left, right) とし、最後の bin のみ right を含む。
    bin 範囲外の値は無視する。
    """
    if len(bin_edges) < 2:
        raise ValueError("bin_edges には 2 つ以上の境界が必要です")

    counts = [0] * (len(bin_edges) - 1)

    for value in values:
        for index in range(len(bin_edges) - 1):
            left = bin_edges[index]
            right = bin_edges[index + 1]
            is_last_bin = index == len(bin_edges) - 2
            if (left <= value < right) or (is_last_bin and left <= value <= right):
                counts[index] += 1
                break

    counted_total = sum(counts)
    if counted_total > 0:
        ratios = [round(count / counted_total, 4) for count in counts]
    else:
        ratios = [0.0 for _ in counts]

    bin_width = bin_edges[1] - bin_edges[0]

    return {
        "bin_width": bin_width,
        "bin_edges": list(bin_edges),
        "counts": counts,
        "ratios": ratios,
    }


def get_requested_output_key(
    early_rate: float,
    v2v_rate: float,
    mode: Optional[str] = None,
) -> Optional[str]:
    """
    出力 JSON 用のキーを返す。

    system:
        early_rate をそのまま "0.1", "0.5", "0.9", "1.0" などにする
    nosystem:
        "nosystem" にする
    """
    if mode == "nosystem":
        return "nosystem"

    if (
        is_close_float(early_rate, COMPARISON_NOSYSTEM_EARLY_RATE)
        and is_close_float(v2v_rate, COMPARISON_NOSYSTEM_V2V_RATE)
    ):
        return "nosystem"

    if mode == "system" or is_close_float(v2v_rate, COMPARISON_SYSTEM_V2V_RATE):
        return f"{early_rate:.1f}"

    return None


def build_requested_output_for_scenario(
    scenario: int,
    scenario_results: Dict[str, Dict[str, Any]],
) -> Dict[str, Any]:
    """
    ユーザー指定形式の output.json を構築する。
    0.1 / 0.5 / 0.9 / nosystem ごとに
    all_abandon_time_events を含めて出力する。
    """
    if scenario == 1:
        ordered_keys = ["0.1", "0.5", "0.9", "nosystem"]
    else:
        ordered_keys = ["1.0"]

    selected_results: Dict[str, Dict[str, Any]] = {}
    for _, condition_result in scenario_results.items():
        early_rate = float(condition_result["early_rate"])
        v2v_rate = float(condition_result["v2v_rate"])
        mode = condition_result.get("mode")
        output_key = get_requested_output_key(early_rate, v2v_rate, mode)
        if output_key is not None:
            selected_results[output_key] = condition_result

    vehicle_mean_evacuation_time: Dict[str, Dict[str, float]] = {}
    cdf_source: Dict[str, List[float]] = {}
    arrival_time_cdf: Dict[str, Dict[str, List[float]]] = {}
    average_count_metrics: Dict[str, Dict[str, float]] = {}

    all_abandon_time_events: Dict[str, List[float]] = {}

    abandon_time_distribution: Dict[str, Dict[str, Any]] = {}
    walking_distance_distribution: Dict[str, Dict[str, Any]] = {}

    for output_key in ordered_keys:
        condition_result = selected_results.get(output_key)

        if condition_result is None:
            cdf_source[output_key] = []
            arrival_time_cdf[output_key] = {"x": [], "y": []}
            average_count_metrics[output_key] = {}
            vehicle_mean_evacuation_time[output_key] = {}
            all_abandon_time_events[output_key] = []

            abandon_time_distribution[output_key] = build_histogram_from_values(
                [],
                ABANDON_TIME_BIN_EDGES,
            )
            walking_distance_distribution[output_key] = build_histogram_from_values(
                [],
                WALKING_DISTANCE_BIN_EDGES,
            )
            continue

        arrival_values = [
            float(value)
            for value in condition_result.get("arrival_time", {}).get("arrival_time_list", [])
        ]
        arrival_values = sorted(arrival_values)
        cdf_source[output_key] = arrival_values

        vehicle_mean_arrival_time_map = (
            condition_result
            .get("arrival_time", {})
            .get("vehicle_mean_arrival_time", {})
        )

        vehicle_mean_evacuation_time[output_key] = {
            str(vehicle_id): float(mean_time)
            for vehicle_id, mean_time in sorted(
                vehicle_mean_arrival_time_map.items(),
                key=lambda item: int(item[0]) if str(item[0]).isdigit() else str(item[0]),
            )
        }

        x_values, y_values = compute_cdf_points(arrival_values)
        arrival_time_cdf[output_key] = {
            "x": x_values,
            "y": y_values,
        }

        count_averages = condition_result.get("count_averages", {})
        average_count_metrics[output_key] = {
            key: float(count_averages[key])
            for key in SUMMARY_COUNT_KEYS
            if count_averages.get(key) is not None
        }

        abandonment = condition_result.get("abandonment", {})

        # 追加: 全乗り捨てイベント時刻
        abandon_time_values = [
            float(value)
            for value in abandonment.get("all_abandon_time_events", [])
        ]
        all_abandon_time_events[output_key] = abandon_time_values

        # 既存の histogram も残したいならこれでOK
        abandon_time_distribution[output_key] = build_histogram_from_values(
            abandon_time_values,
            ABANDON_TIME_BIN_EDGES,
        )

        walking_distance_map = abandonment.get("vehicle_mean_walking_distance", {})
        walking_distance_values = [float(value) for value in walking_distance_map.values()]
        walking_distance_distribution[output_key] = build_histogram_from_values(
            walking_distance_values,
            WALKING_DISTANCE_BIN_EDGES,
        )

    return {
        "scenario": scenario,
        "timestamp": datetime.now().strftime("%Y-%m-%d %H:%M:%S"),
        "average_count_metrics": average_count_metrics,
        "vehicle_mean_evacuation_time": vehicle_mean_evacuation_time,
        "cdf_source": cdf_source,
        "arrival_time_cdf": arrival_time_cdf,
        "all_abandon_time_events": all_abandon_time_events,

        "abandon_time_distribution": abandon_time_distribution,
        "walking_distance_distribution": walking_distance_distribution,
    }

def plot_cdfs_to_path(data_dict: Dict[float, List[float]], save_path: str) -> None:
    plt.figure(figsize=(10, 6))

    def label_for(key: float) -> str:
        """
        CDF グラフの凡例名を返す。

        key=0.0 は nosystem を表す。
        それ以外は system の early_rate を表す。
        """
        if is_close_float(key, 0.0):
            return "nosystem 0.5"
        if is_close_float(key, 1.0):
            return "system 1.0"
        return f"system {key:.1f}"

    def style_for(key: float) -> Dict[str, str]:
        """
        条件ごとの線の色・線種を返す。
        既存の色設定をなるべく維持する。
        """
        if is_close_float(key, 0.0):
            return {"color": "blue", "linestyle": "--"}
        if is_close_float(key, 0.1):
            return {"color": "r", "linestyle": "-"}
        if is_close_float(key, 0.5):
            return {"color": "blue", "linestyle": "-"}
        if is_close_float(key, 0.9):
            return {"color": "g", "linestyle": "-"}
        if is_close_float(key, 1.0):
            return {"color": "black", "linestyle": "-"}

        # 想定外の early_rate が来ても描画できるようにする
        return {"linestyle": "-"}

    plotted = False

    for key in sorted(data_dict.keys()):
        values = data_dict.get(key, [])
        arr = sorted(float(v) for v in values)

        if not arr:
            continue

        cdf = [(i + 1) / len(arr) for i in range(len(arr))]

        plt.plot(
            arr,
            cdf,
            label=label_for(key),
            **style_for(key),
        )
        plotted = True

    plt.xlim(200, 3000)
    plt.ylim(0.05, 1.0)
    plt.xticks(ticks=list(range(200, 3000, 100)), fontsize=14, fontweight="semibold")
    plt.yticks(ticks=[i / 10 for i in range(1, 11)], fontsize=14, fontweight="semibold")
    plt.xlabel("Mean arrival time")
    plt.ylabel("Cumulative distribution")

    if plotted:
        plt.legend(loc="lower right")

    plt.grid(True)
    plt.tight_layout()
    plt.savefig(save_path, dpi=150)
    plt.close()

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

    count_accumulator: DefaultDict[str, List[float]] = defaultdict(list)
    arrival_time_accumulator: DefaultDict[str, List[float]] = defaultdict(list)

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

        for message in messages:
            warn(f"{file_info.filename}: {message}")

        for key, value in count_values.items():
            if value is not None and not math.isnan(value):
                count_accumulator[key].append(value)

        arrival_dict = dict_values.get("arrival_time_by_vehID_dict", {})
        for vehicle_id, arrival_time in arrival_dict.items():
            arrival_time_accumulator[vehicle_id].append(arrival_time)

        abandon_dict = dict_values.get("vehicle_abandant_time_by_pedestrianID_dict", {})
        for vehicle_id, abandon_time in abandon_dict.items():
            abandon_time_all_events.append(abandon_time)
            abandon_time_by_vehicle[vehicle_id].append(abandon_time)

        walking_dict = dict_values.get("walking_distance_by_pedestrianID_dict", {})
        for vehicle_id, walking_distance in walking_dict.items():
            walking_distance_all_events.append(walking_distance)
            walking_distance_by_vehicle[vehicle_id].append(walking_distance)

    parsed_run_ids = sorted(set(parsed_run_ids))
    failed_run_ids = sorted(set(failed_run_ids))

    count_averages: Dict[str, Optional[float]] = {}
    for key in COUNT_KEYS:
        count_averages[key] = safe_mean(count_accumulator.get(key, []))

    vehicle_mean_arrival_time: Dict[str, float] = {}
    for vehicle_id, values in arrival_time_accumulator.items():
        mean_value = safe_mean(values)
        if mean_value is not None:
            vehicle_mean_arrival_time[vehicle_id] = mean_value

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

    # CDF の元データとして使いやすいように list でも保持
    arrival_time_list = sorted(vehicle_mean_arrival_time.values())

    result: Dict[str, Any] = {
    "scenario": scenario,
    "mode": files[0].mode,
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
        "arrival_time_list": arrival_time_list,
        "vehicle_mean_arrival_time": {
            vehicle_id: vehicle_mean_arrival_time[vehicle_id]
            for vehicle_id in sort_numeric_strings_as_numbers(vehicle_mean_arrival_time.keys())
        },
    },
    "abandonment": {
        "mean_abandon_time": safe_mean(abandon_time_all_events),
        "mean_walking_distance": safe_mean(walking_distance_all_events),

        "all_abandon_time_events": sorted(float(v) for v in abandon_time_all_events),

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
    """
    結果 JSON を保存する。

    top-level は改行する。
    top-level の値が dict の場合，その直下の key ごとに改行する。
    その下の dict は 1 行で出力する。
    """
    with open(output_path, "w", encoding="utf-8") as f:
        f.write("{\n")

        top_items = list(data.items())

        for top_index, (top_key, top_value) in enumerate(top_items):
            is_last_top = top_index == len(top_items) - 1
            top_comma = "" if is_last_top else ","

            if not isinstance(top_value, dict):
                dumped_value = json.dumps(
                    top_value,
                    ensure_ascii=False,
                    separators=(", ", ": "),
                    sort_keys=False,
                )
                f.write(f'  "{top_key}": {dumped_value}{top_comma}\n')
                continue

            f.write(f'  "{top_key}": {{\n')

            child_items = list(top_value.items())

            for child_index, (child_key, child_value) in enumerate(child_items):
                is_last_child = child_index == len(child_items) - 1
                child_comma = "" if is_last_child else ","

                dumped_child_value = json.dumps(
                    child_value,
                    ensure_ascii=False,
                    separators=(", ", ": "),
                    sort_keys=False,
                )

                f.write(
                    f'    "{child_key}": {dumped_child_value}{child_comma}\n'
                )

            f.write(f"  }}{top_comma}\n")

        f.write("}\n")
        

# =========================================
# 全体処理
# =========================================

def aggregate_all_conditions(
    log_dir: str,
    output_dir: str,
    expected_max_run_id: int,
    json_filename: str,
) -> Dict[str, Any]:
    """全条件を集計し、scenario ごとに JSON と比較 CDF を出力する"""
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

    all_results_by_scenario: Dict[int, Dict[str, Any]] = defaultdict(dict)

    for condition_key in sorted(grouped.keys()):
        info(f"集計中: {condition_key}")
        condition_result = aggregate_condition(
            condition_key=condition_key,
            files=grouped[condition_key],
            output_dir=output_dir,
            expected_max_run_id=expected_max_run_id,
        )
        scenario = int(condition_result["scenario"])
        all_results_by_scenario[scenario][condition_key] = condition_result

    final_output: Dict[str, Any] = {}

    for scenario, scenario_results in sorted(all_results_by_scenario.items()):
        scenario_output_dir = build_scenario_output_dir(output_dir, scenario)
        ensure_output_dir(scenario_output_dir)

        comparison_plot_filename = f"cdf_compare_s{scenario}.pdf"
        comparison_plot_path = os.path.join(
            scenario_output_dir,
            comparison_plot_filename,
        )

        comparison_data = build_comparison_cdf_data_for_scenario(scenario_results)
        if comparison_data:
            plot_cdfs_to_path(comparison_data, comparison_plot_path)
        else:
            warn(f"比較 CDF 用データが不足しているためスキップします: scenario={scenario}")
            comparison_plot_filename = None

        scenario_json_data = build_requested_output_for_scenario(scenario, scenario_results)

        json_output_path = os.path.join(scenario_output_dir, json_filename)
        write_json(scenario_json_data, json_output_path)
        info(f"JSON を出力しました: {json_output_path}")

        final_output[f"scenario{scenario}"] = scenario_json_data

    return final_output


def build_comparison_cdf_data_for_scenario(
    scenario_results: Dict[str, Dict[str, Any]]
) -> Dict[float, List[float]]:
    comparison_data: Dict[float, List[float]] = {}

    for _, condition_result in scenario_results.items():
        early_rate = float(condition_result["early_rate"])
        v2v_rate = float(condition_result["v2v_rate"])
        mode = condition_result.get("mode")
        arrival_values = list(condition_result["arrival_time"].get("arrival_time_list", []))

        if not arrival_values:
            continue

        if mode == "nosystem" or (
            is_close_float(early_rate, COMPARISON_NOSYSTEM_EARLY_RATE)
            and is_close_float(v2v_rate, COMPARISON_NOSYSTEM_V2V_RATE)
        ):
            comparison_data[0.0] = arrival_values
            continue

        if mode == "system" or is_close_float(v2v_rate, COMPARISON_SYSTEM_V2V_RATE):
            comparison_data[early_rate] = arrival_values

    return comparison_data

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