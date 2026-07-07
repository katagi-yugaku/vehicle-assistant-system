#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Aggregate Slurm .out logs for vehicle-assistant-system simulations.

Features:
- Reads only .out files and ignores .err files.
- Parses filenames of the form:
    va_s{scenario}_e{early_rate}_v{v2v_rate}_r{run_id}_{jobid}.out
- Groups logs by scenario / early_rate / v2v_rate.
- If multiple logs exist for the same condition and run_id, uses the largest jobid.
- Parses scalar metrics and dictionary metrics from "Simlation Result Summary".
- Aggregates run_id = 1..N, records missing and failed runs.
- Normalizes derived vehicle IDs such as init_..., newveh_..., and ped_... to the original ID.
- Does not drop duplicated normalized IDs; values are accumulated as lists and averaged later.
- Handles rate_vehicle_abandonment values with a trailing percent sign, such as 0.00%.
- Writes output.json and a comparison CDF PDF for each scenario.

Examples:
    python aggregate_simulation_logs.py ./logs --expected-max-run-id 50
    python aggregate_simulation_logs.py ./logs --scenario-name JIP/1-1-2 --expected-max-run-id 50
    python aggregate_simulation_logs.py ./logs --output-dir ./aggregated --expected-max-run-id 50
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

OUTPUT_JSON_FILE = "output.json"
DEFAULT_EXPECTED_MAX_RUN_ID = 50
SUMMARY_MARKER = "===== Simlation Result Summary ====="
JSON_CHUNK_SIZE = 100
JSON_INDENT_SIZE = 2

# Scalar keys printed by runner_simulator.py.
COUNT_KEYS = [
    "avg_congestion_duration",
    "pedestrian_count",
    "route_changed_vehicle_count",
    "rate_vehicle_abandonment",
    "wrong_way_driving_count",
    "vehicle_abandonment_count",
    "normalcy_bias_route_change_count",
    "majority_bias_route_change_count",
    "lane_changed_vehicle_count",
    "info_obtained_lanechange_count",
    "elapsed_time_lanechange_count",
    "majority_bias_lanechange_count",
]

# Dict keys printed by runner_simulator.py.
DICT_KEYS = [
    "arrival_time_by_vehID_dict",
    "vehicle_abandant_time_by_pedestrianID_dict",
    "walking_distance_by_pedestrianID_dict",
    "route_change_time_by_vehID_dict",
]

# Canonical key -> acceptable aliases.
ALIAS_MAP: Dict[str, List[str]] = {
    "avg_congestion_duration": [
        "avg_congestion_duration",
        "average_congestion_duration",
    ],
    "pedestrian_count": ["pedestrian_count"],
    "route_changed_vehicle_count": ["route_changed_vehicle_count"],
    "rate_vehicle_abandonment": ["rate_vehicle_abandonment"],
    "wrong_way_driving_count": [
        "wrong_way_driving_count",
        "wrong_way_success_count",
    ],
    "vehicle_abandonment_count": [
        "vehicle_abandonment_count",
        "vehicle_abandonment_success_count",
    ],
    "normalcy_bias_route_change_count": ["normalcy_bias_route_change_count"],
    "majority_bias_route_change_count": ["majority_bias_route_change_count"],
    "lane_changed_vehicle_count": ["lane_changed_vehicle_count"],
    "info_obtained_lanechange_count": [
        "info_obtained_lanechange_count",
        "obtain_info_lane_change_count",
        "info_obtained_lane_change_count",
        "obtain_info_lanechange_count",
    ],
    "elapsed_time_lanechange_count": [
        "elapsed_time_lanechange_count",
        "elapsed_time_lane_change_count",
    ],
    "majority_bias_lanechange_count": [
        "majority_bias_lanechange_count",
        "positive_majority_bias_count",
    ],
    "arrival_time_by_vehID_dict": ["arrival_time_by_vehID_dict"],
    "vehicle_abandant_time_by_pedestrianID_dict": [
        "vehicle_abandant_time_by_pedestrianID_dict",
        "vehicle_abandon_time_by_pedestrianID_dict",
    ],
    "walking_distance_by_pedestrianID_dict": ["walking_distance_by_pedestrianID_dict"],
    "route_change_time_by_vehID_dict": ["route_change_time_by_vehID_dict"],
}

OUT_FILENAME_RE = re.compile(
    r"^va_s(?P<scenario>\d+)"
    r"(?:_(?P<mode>system|nosystem))?"
    r"_e(?P<early_rate>\d+(?:\.\d+)?)"
    r"_v(?P<v2v_rate>\d+(?:\.\d+)?)"
    r"_r(?P<run_id>\d+)"
    r"_(?P<jobid>\d+)\.out$"
)

ORIGINAL_VEH_ID_RE = re.compile(
    r"^(?:ped_)?(?:init|newveh)_Shelter[A-Za-z]+_\d+_(\d+)(?:_\d+)*$"
)


@dataclass(frozen=True)
class LogFileInfo:
    path: str
    filename: str
    scenario: int
    early_rate: float
    early_rate_text: str
    v2v_rate: float
    v2v_rate_text: str
    run_id: int
    jobid: int
    mode: Optional[str] = None

    @property
    def condition_key(self) -> str:
        # The requested key format does not include mode.
        return build_condition_key(
            scenario=self.scenario,
            early_rate_text=self.early_rate_text,
            v2v_rate_text=self.v2v_rate_text,
        )


def warn(message: str) -> None:
    print(f"[WARNING] {message}", file=sys.stdout)


def info(message: str) -> None:
    print(f"[INFO] {message}", file=sys.stdout)


def is_close_float(a: float, b: float, tol: float = 1e-9) -> bool:
    return abs(a - b) < tol


def safe_mean(values: Iterable[float]) -> Optional[float]:
    values_list = [float(v) for v in values]
    if not values_list:
        return None
    return sum(values_list) / len(values_list)


def build_condition_key(
    scenario: int,
    early_rate_text: str,
    v2v_rate_text: str,
) -> str:
    return f"s{scenario}_e{early_rate_text}_v{v2v_rate_text}"


def sort_numeric_strings_as_numbers(keys: Iterable[str]) -> List[str]:
    def sort_key(value: str) -> Tuple[int, Any]:
        try:
            return (0, int(value))
        except ValueError:
            try:
                return (1, float(value))
            except ValueError:
                return (2, value)

    return sorted(keys, key=sort_key)


def ensure_output_dir(path: str) -> None:
    os.makedirs(path, exist_ok=True)


def find_repo_root(start_path: Optional[str] = None) -> str:
    """
    Find the repository root by walking upward until a .git directory is found.

    The current working directory is tried first because the script may not be
    placed directly under the repository root. If that fails, the directory of
    this script is also tried.
    """
    candidates: List[str] = []
    if start_path is not None:
        candidates.append(os.path.abspath(start_path))
    candidates.append(os.path.abspath(os.getcwd()))
    candidates.append(os.path.dirname(os.path.abspath(__file__)))

    checked: set[str] = set()
    for candidate in candidates:
        current = candidate
        while True:
            if current in checked:
                break
            checked.add(current)

            if os.path.isdir(os.path.join(current, ".git")):
                return current

            parent = os.path.dirname(current)
            if parent == current:
                break
            current = parent

    raise FileNotFoundError("リポジトリルート .git が見つかりませんでした")


def build_output_dir_from_scenario_name(scenario_name: str) -> str:
    normalized_name = scenario_name.strip().strip("/")
    if normalized_name.startswith("scenarios/"):
        normalized_name = normalized_name[len("scenarios/") :]

    repo_root = find_repo_root()
    return os.path.join(repo_root, "scenarios", normalized_name, "results")


def resolve_output_dir(args: argparse.Namespace) -> str:
    """
    Priority:
      1. --output-dir
      2. --scenario-name -> scenarios/{scenario_name}/results
      3. log_dir
    """
    if args.output_dir is not None:
        return os.path.abspath(args.output_dir)
    if args.scenario_name is not None:
        return build_output_dir_from_scenario_name(args.scenario_name)
    return os.path.abspath(args.log_dir)


def build_scenario_output_dir(base_output_dir: str, scenario: int) -> str:
    return os.path.join(base_output_dir, f"scenario{scenario}")


def normalize_vehicle_id(raw_id: str) -> str:
    """
    Normalize vehicle/pedestrian IDs to the original vehicle ID.

    Examples:
        init_ShelterA_1_267 -> 267
        newveh_ShelterA_1_267_138 -> 267
        ped_init_ShelterA_1_197_44 -> 197
        newveh_ShelterA_1_197_45 -> 197
        ped_newveh_ShelterA_1_98_91_47 -> 98
    """
    text = str(raw_id).strip()

    match = ORIGINAL_VEH_ID_RE.match(text)
    if match is not None:
        return match.group(1)

    # If the simulator already emits a plain numeric ID, accept it.
    if re.fullmatch(r"\d+", text):
        return text

    # Fallback: use the raw ID rather than aborting the whole log.
    # This keeps parsing robust when a new ID format appears.
    return text


def normalize_id_value_dict_to_lists(raw_dict: Dict[Any, Any]) -> Dict[str, List[float]]:
    """
    Normalize keys and accumulate values as lists.

    This intentionally does not warn when the same normalized ID appears more
    than once in the same run, because init/newveh/ped derived IDs can coexist.
    """
    normalized: DefaultDict[str, List[float]] = defaultdict(list)

    for raw_key, raw_value in raw_dict.items():
        normalized_key = normalize_vehicle_id(str(raw_key))
        try:
            normalized_value = float(raw_value)
        except (TypeError, ValueError):
            continue
        normalized[normalized_key].append(normalized_value)

    return dict(normalized)


def list_out_files(log_dir: str) -> List[str]:
    if not os.path.isdir(log_dir):
        raise FileNotFoundError(f"ログディレクトリが存在しません: {log_dir}")

    paths = [
        os.path.join(log_dir, entry)
        for entry in os.listdir(log_dir)
        if entry.endswith(".out")
    ]
    return sorted(paths)


def parse_out_filename(path: str) -> Optional[LogFileInfo]:
    filename = os.path.basename(path)
    match = OUT_FILENAME_RE.match(filename)
    if match is None:
        warn(f"ファイル名規則に一致しないためスキップします: {filename}")
        return None

    try:
        early_rate_text = match.group("early_rate")
        v2v_rate_text = match.group("v2v_rate")
        return LogFileInfo(
            path=path,
            filename=filename,
            scenario=int(match.group("scenario")),
            early_rate=float(early_rate_text),
            early_rate_text=early_rate_text,
            v2v_rate=float(v2v_rate_text),
            v2v_rate_text=v2v_rate_text,
            run_id=int(match.group("run_id")),
            jobid=int(match.group("jobid")),
            mode=match.group("mode"),
        )
    except ValueError as exc:
        warn(f"ファイル名の数値変換に失敗したためスキップします: {filename} ({exc})")
        return None


def deduplicate_by_condition_and_run(
    files: List[LogFileInfo],
) -> Dict[str, List[LogFileInfo]]:
    selected: Dict[Tuple[str, int], LogFileInfo] = {}

    for file_info in files:
        key = (file_info.condition_key, file_info.run_id)
        existing = selected.get(key)
        if existing is None or file_info.jobid > existing.jobid:
            if existing is not None:
                warn(
                    "同一条件・同一 run_id の重複を検出しました。"
                    f" jobid={existing.jobid} より jobid={file_info.jobid} を優先します: "
                    f"{file_info.condition_key}, run_id={file_info.run_id}"
                )
            selected[key] = file_info

    grouped: DefaultDict[str, List[LogFileInfo]] = defaultdict(list)
    for file_info in selected.values():
        grouped[file_info.condition_key].append(file_info)

    return {
        condition_key: sorted(condition_files, key=lambda item: item.run_id)
        for condition_key, condition_files in grouped.items()
    }


def read_text_file(path: str) -> Optional[str]:
    try:
        with open(path, "r", encoding="utf-8", errors="replace") as file:
            return file.read()
    except OSError as exc:
        warn(f"ファイル読み込みに失敗しました: {path} ({exc})")
        return None


def get_summary_text(text: str) -> str:
    marker_index = text.rfind(SUMMARY_MARKER)
    if marker_index == -1:
        return text
    return text[marker_index:]


def parse_scalar_value(raw_value: str) -> Optional[float]:
    value = raw_value.strip()
    if value.endswith("%"):
        value = value[:-1].strip()
    value = value.replace(",", "")

    try:
        parsed = float(value)
    except ValueError:
        return None

    if math.isnan(parsed):
        return None
    return parsed


def extract_scalar_by_aliases(text: str, aliases: List[str]) -> Optional[float]:
    for alias in aliases:
        pattern = re.compile(rf"^\s*{re.escape(alias)}\s*:\s*(.+?)\s*$", re.MULTILINE)
        match = pattern.search(text)
        if match is None:
            continue

        raw_value = match.group(1)
        value = parse_scalar_value(raw_value)
        if value is None:
            warn(f"数値パースに失敗しました: key={alias}, value={raw_value.strip()}")
        return value

    return None


def find_braced_literal_after_key(text: str, key_name: str) -> Optional[str]:
    key_pattern = re.compile(rf"{re.escape(key_name)}\s*:\s*\{{", re.MULTILINE)
    match = key_pattern.search(text)
    if match is None:
        return None

    brace_start = match.end() - 1
    depth = 0
    in_string: Optional[str] = None
    escaped = False

    for index in range(brace_start, len(text)):
        char = text[index]

        if in_string is not None:
            if escaped:
                escaped = False
            elif char == "\\":
                escaped = True
            elif char == in_string:
                in_string = None
            continue

        if char in {"'", '"'}:
            in_string = char
            continue
        if char == "{":
            depth += 1
        elif char == "}":
            depth -= 1
            if depth == 0:
                return text[brace_start : index + 1]

    return None


def extract_dict_by_aliases(text: str, aliases: List[str]) -> Optional[Dict[Any, Any]]:
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


def parse_log_content(
    text: str,
) -> Tuple[Dict[str, Optional[float]], Dict[str, Dict[str, List[float]]]]:
    summary_text = get_summary_text(text)

    count_values: Dict[str, Optional[float]] = {}
    dict_values: Dict[str, Dict[str, List[float]]] = {}

    for canonical_key in COUNT_KEYS:
        aliases = ALIAS_MAP.get(canonical_key, [canonical_key])
        count_values[canonical_key] = extract_scalar_by_aliases(summary_text, aliases)

    for canonical_key in DICT_KEYS:
        aliases = ALIAS_MAP.get(canonical_key, [canonical_key])
        raw_dict = extract_dict_by_aliases(summary_text, aliases)
        if raw_dict is None:
            dict_values[canonical_key] = {}
        else:
            dict_values[canonical_key] = normalize_id_value_dict_to_lists(raw_dict)

    return count_values, dict_values


def extend_accumulator(
    accumulator: DefaultDict[str, List[float]],
    values_by_id: Dict[str, List[float]],
) -> None:
    for vehicle_id, values in values_by_id.items():
        accumulator[vehicle_id].extend(values)


def flatten_values(values_by_id: Dict[str, List[float]]) -> List[float]:
    return [value for values in values_by_id.values() for value in values]


def mean_by_id(accumulator: DefaultDict[str, List[float]]) -> Dict[str, float]:
    result: Dict[str, float] = {}
    for vehicle_id, values in accumulator.items():
        mean_value = safe_mean(values)
        if mean_value is not None:
            result[vehicle_id] = mean_value
    return result


def aggregate_condition(
    condition_key: str,
    files: List[LogFileInfo],
    expected_max_run_id: int,
) -> Dict[str, Any]:
    if not files:
        raise ValueError(f"files が空です: {condition_key}")

    first_file = files[0]
    expected_run_ids = list(range(1, expected_max_run_id + 1))
    found_run_ids = sorted(file_info.run_id for file_info in files)
    missing_run_ids = sorted(set(expected_run_ids) - set(found_run_ids))

    parsed_run_ids: List[int] = []
    failed_run_ids: List[int] = []

    count_accumulator: DefaultDict[str, List[float]] = defaultdict(list)
    arrival_time_accumulator: DefaultDict[str, List[float]] = defaultdict(list)
    abandon_time_by_vehicle: DefaultDict[str, List[float]] = defaultdict(list)
    walking_distance_by_vehicle: DefaultDict[str, List[float]] = defaultdict(list)
    route_change_time_by_vehicle: DefaultDict[str, List[float]] = defaultdict(list)

    abandon_time_all_events: List[float] = []
    walking_distance_all_events: List[float] = []

    for file_info in files:
        text = read_text_file(file_info.path)
        if text is None:
            failed_run_ids.append(file_info.run_id)
            continue

        count_values, dict_values = parse_log_content(text)

        has_any_count = any(value is not None for value in count_values.values())
        has_any_dict = any(bool(values) for values in dict_values.values())
        if not has_any_count and not has_any_dict:
            failed_run_ids.append(file_info.run_id)
            warn(f"必要なキーを抽出できなかったため失敗扱いにします: {file_info.filename}")
            continue

        parsed_run_ids.append(file_info.run_id)

        for key, value in count_values.items():
            if value is not None:
                count_accumulator[key].append(value)

        arrival_dict = dict_values.get("arrival_time_by_vehID_dict", {})
        extend_accumulator(arrival_time_accumulator, arrival_dict)

        abandon_dict = dict_values.get("vehicle_abandant_time_by_pedestrianID_dict", {})
        extend_accumulator(abandon_time_by_vehicle, abandon_dict)
        abandon_time_all_events.extend(flatten_values(abandon_dict))

        walking_dict = dict_values.get("walking_distance_by_pedestrianID_dict", {})
        extend_accumulator(walking_distance_by_vehicle, walking_dict)
        walking_distance_all_events.extend(flatten_values(walking_dict))

        route_change_dict = dict_values.get("route_change_time_by_vehID_dict", {})
        extend_accumulator(route_change_time_by_vehicle, route_change_dict)

    parsed_run_ids = sorted(set(parsed_run_ids))
    failed_run_ids = sorted(set(failed_run_ids))

    count_averages: Dict[str, Optional[float]] = {
        key: safe_mean(count_accumulator.get(key, []))
        for key in COUNT_KEYS
    }

    vehicle_mean_arrival_time = mean_by_id(arrival_time_accumulator)
    vehicle_mean_abandon_time = mean_by_id(abandon_time_by_vehicle)
    vehicle_mean_walking_distance = mean_by_id(walking_distance_by_vehicle)
    vehicle_mean_route_change_time = mean_by_id(route_change_time_by_vehicle)

    sorted_arrival_time_list = sorted(float(value) for value in vehicle_mean_arrival_time.values())

    result: Dict[str, Any] = {
        "scenario": first_file.scenario,
        "mode": first_file.mode,
        "early_rate": first_file.early_rate,
        "v2v_rate": first_file.v2v_rate,
        "condition_key": condition_key,
        "run_status": {
            "expected_run_ids": expected_run_ids,
            "found_run_ids": found_run_ids,
            "missing_run_ids": missing_run_ids,
            "parsed_run_ids": parsed_run_ids,
            "failed_run_ids": failed_run_ids,
        },
        "count_averages": count_averages,
        "arrival_time": {
            "arrival_time_list": sorted_arrival_time_list,
            "vehicle_mean_arrival_time": {
                vehicle_id: vehicle_mean_arrival_time[vehicle_id]
                for vehicle_id in sort_numeric_strings_as_numbers(vehicle_mean_arrival_time.keys())
            },
        },
        "abandonment": {
            "mean_abandon_time": safe_mean(abandon_time_all_events),
            "mean_walking_distance": safe_mean(walking_distance_all_events),
            "all_abandon_time_events": sorted(float(value) for value in abandon_time_all_events),
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

    # Parsed for future use. It is not required by the current output spec, but
    # keeping it here makes it visible without changing the parser later.
    if vehicle_mean_route_change_time:
        result["route_change_time"] = {
            "vehicle_mean_route_change_time": {
                vehicle_id: vehicle_mean_route_change_time[vehicle_id]
                for vehicle_id in sort_numeric_strings_as_numbers(vehicle_mean_route_change_time.keys())
            }
        }

    return result


def get_requested_output_key(
    early_rate: float,
    v2v_rate: float,
    mode: Optional[str] = None,
) -> Optional[str]:
    if mode == "nosystem" or is_close_float(v2v_rate, 0.0):
        return "nosystem"
    if mode == "system" or is_close_float(v2v_rate, 1.0):
        return f"{early_rate:.1f}"
    return None


def build_requested_output_for_scenario(
    scenario: int,
    scenario_results: Dict[str, Dict[str, Any]],
) -> Dict[str, Any]:
    """
    Build the requested top-level JSON structure for a scenario.

    Main order is stable: "1.0" first and "nosystem" second.
    Additional system early_rate keys, if present, are appended in numeric order.
    """
    del scenario  # The scenario value is already stored in each condition result.

    selected_results: Dict[str, Dict[str, Any]] = {}

    for condition_result in scenario_results.values():
        output_key = get_requested_output_key(
            early_rate=float(condition_result["early_rate"]),
            v2v_rate=float(condition_result["v2v_rate"]),
            mode=condition_result.get("mode"),
        )
        if output_key is not None:
            selected_results[output_key] = condition_result

    ordered_keys = ["1.0", "nosystem"]
    additional_keys = [
        key for key in selected_results.keys()
        if key not in ordered_keys
    ]

    def additional_sort_key(key: str) -> Tuple[int, Any]:
        try:
            return (0, float(key))
        except ValueError:
            return (1, key)

    ordered_keys.extend(sorted(additional_keys, key=additional_sort_key))

    return {
        key: selected_results[key]
        for key in ordered_keys
        if key in selected_results
    }


def build_comparison_cdf_data_for_scenario(
    scenario_results: Dict[str, Dict[str, Any]],
) -> Dict[str, List[float]]:
    comparison_data: Dict[str, List[float]] = {}

    for condition_result in scenario_results.values():
        output_key = get_requested_output_key(
            early_rate=float(condition_result["early_rate"]),
            v2v_rate=float(condition_result["v2v_rate"]),
            mode=condition_result.get("mode"),
        )
        if output_key is None:
            continue

        arrival_values = list(condition_result["arrival_time"].get("arrival_time_list", []))
        if arrival_values:
            comparison_data[output_key] = [float(value) for value in arrival_values]

    ordered = build_requested_output_for_scenario(0, {
        key: value for key, value in scenario_results.items()
    })
    ordered_keys = list(ordered.keys())

    # Keep the same order as the JSON where possible.
    return {
        key: comparison_data[key]
        for key in ordered_keys
        if key in comparison_data
    }


def compute_cdf_points(values: List[float]) -> Tuple[List[float], List[float]]:
    if not values:
        return [], []

    sorted_values = sorted(float(value) for value in values)
    n = len(sorted_values)
    cdf = [(index + 1) / n for index in range(n)]
    return sorted_values, cdf


def label_for_output_key(output_key: str) -> str:
    if output_key == "nosystem":
        return "nosystem: e1.0, v0.0"
    return f"system: e{output_key}, v1.0"


def plot_comparison_cdf_to_path(
    comparison_data: Dict[str, List[float]],
    save_path: str,
) -> Optional[str]:
    if not comparison_data:
        return None

    plotted = False
    plt.figure(figsize=(8, 6))

    for output_key, values in comparison_data.items():
        x_values, y_values = compute_cdf_points(values)
        if not x_values:
            continue
        plt.plot(x_values, y_values, label=label_for_output_key(output_key))
        plotted = True

    if not plotted:
        plt.close()
        return None

    plt.xlabel("Mean arrival time [sec.]")
    plt.ylabel("Cumulative distribution")
    plt.grid(True)
    plt.legend(loc="lower right")
    plt.tight_layout()
    plt.savefig(save_path, format="pdf")
    plt.close()
    return save_path


def json_indent(indent: int) -> str:
    return " " * indent


def is_json_scalar(value: Any) -> bool:
    return value is None or isinstance(value, (str, int, float, bool))


def dumps_json_scalar(value: Any) -> str:
    return json.dumps(value, ensure_ascii=False, allow_nan=False)


def format_json_key(key: Any) -> str:
    # json.dump() coerces non-string dict keys to strings.
    # Do the same here so that the output remains valid JSON.
    return json.dumps(str(key), ensure_ascii=False, allow_nan=False)


def format_json_value(value: Any, indent: int = 0, chunk_size: int = JSON_CHUNK_SIZE) -> str:
    if isinstance(value, dict):
        return format_json_dict_compact(value, indent=indent, chunk_size=chunk_size)

    if isinstance(value, list):
        return format_json_list_compact(value, indent=indent, chunk_size=chunk_size)

    if isinstance(value, tuple):
        return format_json_list_compact(list(value), indent=indent, chunk_size=chunk_size)

    return dumps_json_scalar(value)


def format_json_list_compact(
    values: List[Any],
    indent: int = 0,
    chunk_size: int = JSON_CHUNK_SIZE,
) -> str:
    if not values:
        return "[]"

    if all(is_json_scalar(value) for value in values):
        dumped_values = [dumps_json_scalar(value) for value in values]

        if len(dumped_values) <= chunk_size:
            return "[" + ", ".join(dumped_values) + "]"

        child_indent = indent + JSON_INDENT_SIZE
        lines = ["["]
        for start in range(0, len(dumped_values), chunk_size):
            chunk = dumped_values[start : start + chunk_size]
            is_last_chunk = start + chunk_size >= len(dumped_values)
            comma = "" if is_last_chunk else ","
            lines.append(f"{json_indent(child_indent)}{', '.join(chunk)}{comma}")
        lines.append(f"{json_indent(indent)}]")
        return "\n".join(lines)

    child_indent = indent + JSON_INDENT_SIZE
    lines = ["["]
    for index, value in enumerate(values):
        comma = "" if index == len(values) - 1 else ","
        formatted_value = format_json_value(
            value,
            indent=child_indent,
            chunk_size=chunk_size,
        )
        lines.append(f"{json_indent(child_indent)}{formatted_value}{comma}")
    lines.append(f"{json_indent(indent)}]")
    return "\n".join(lines)


def format_json_dict_compact(
    values: Dict[str, Any],
    indent: int = 0,
    chunk_size: int = JSON_CHUNK_SIZE,
) -> str:
    if not values:
        return "{}"

    items = list(values.items())
    child_indent = indent + JSON_INDENT_SIZE

    if all(is_json_scalar(value) for _, value in items):
        dumped_entries = [
            f"{format_json_key(key)}: {dumps_json_scalar(value)}"
            for key, value in items
        ]

        if len(dumped_entries) <= chunk_size:
            return "{ " + ", ".join(dumped_entries) + " }"

        lines = ["{"]
        for start in range(0, len(dumped_entries), chunk_size):
            chunk = dumped_entries[start : start + chunk_size]
            is_last_chunk = start + chunk_size >= len(dumped_entries)
            comma = "" if is_last_chunk else ","
            lines.append(f"{json_indent(child_indent)}{', '.join(chunk)}{comma}")
        lines.append(f"{json_indent(indent)}}}")
        return "\n".join(lines)

    lines = ["{"]
    for index, (key, value) in enumerate(items):
        comma = "" if index == len(items) - 1 else ","
        formatted_value = format_json_value(
            value,
            indent=child_indent,
            chunk_size=chunk_size,
        )
        lines.append(
            f"{json_indent(child_indent)}{format_json_key(key)}: {formatted_value}{comma}"
        )
    lines.append(f"{json_indent(indent)}}}")
    return "\n".join(lines)


def write_json(data: Dict[str, Any], output_path: str) -> None:
    formatted_json = format_json_value(
        data,
        indent=0,
        chunk_size=JSON_CHUNK_SIZE,
    )
    with open(output_path, "w", encoding="utf-8") as file:
        file.write(formatted_json)
        file.write("\n")


def aggregate_all_conditions(
    log_dir: str,
    output_dir: str,
    expected_max_run_id: int,
    json_filename: str,
) -> Dict[str, Any]:
    ensure_output_dir(output_dir)

    out_paths = list_out_files(log_dir)
    info(f".out ファイル数: {len(out_paths)}")

    parsed_files = [
        file_info
        for path in out_paths
        if (file_info := parse_out_filename(path)) is not None
    ]
    info(f"ファイル名の解析に成功した .out ファイル数: {len(parsed_files)}")

    grouped = deduplicate_by_condition_and_run(parsed_files)
    info(f"検出された条件数: {len(grouped)}")

    all_results_by_scenario: DefaultDict[int, Dict[str, Dict[str, Any]]] = defaultdict(dict)
    for condition_key in sorted(grouped.keys()):
        info(f"集計中: {condition_key}")
        condition_result = aggregate_condition(
            condition_key=condition_key,
            files=grouped[condition_key],
            expected_max_run_id=expected_max_run_id,
        )
        scenario = int(condition_result["scenario"])
        all_results_by_scenario[scenario][condition_key] = condition_result

    final_output: Dict[str, Any] = {}
    scenario_count = len(all_results_by_scenario)

    for scenario, scenario_results in sorted(all_results_by_scenario.items()):
        scenario_output_dir = build_scenario_output_dir(output_dir, scenario)
        ensure_output_dir(scenario_output_dir)

        scenario_json_data = build_requested_output_for_scenario(scenario, scenario_results)
        json_output_path = os.path.join(scenario_output_dir, json_filename)
        write_json(scenario_json_data, json_output_path)
        info(f"JSON を出力しました: {json_output_path}")

        comparison_data = build_comparison_cdf_data_for_scenario(scenario_results)
        cdf_output_path = os.path.join(scenario_output_dir, f"cdf_compare_s{scenario}.pdf")
        if plot_comparison_cdf_to_path(comparison_data, cdf_output_path) is not None:
            info(f"CDF を出力しました: {cdf_output_path}")
        else:
            warn(f"比較 CDF 用データが不足しているためスキップします: scenario={scenario}")

        # If the log set contains only one scenario, also mirror outputs directly
        # under output_dir so that scenarios/{scenario_name}/results/output.json
        # is available for the common single-scenario workflow.
        if scenario_count == 1:
            mirror_json_path = os.path.join(output_dir, json_filename)
            write_json(scenario_json_data, mirror_json_path)
            info(f"JSON を出力しました: {mirror_json_path}")

            mirror_cdf_path = os.path.join(output_dir, f"cdf_compare_s{scenario}.pdf")
            if comparison_data:
                plot_comparison_cdf_to_path(comparison_data, mirror_cdf_path)
                info(f"CDF を出力しました: {mirror_cdf_path}")

        final_output[f"scenario{scenario}"] = scenario_json_data

    return final_output


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Slurm の .out シミュレーションログを条件別に集計し、output.json と CDF を出力する"
    )
    parser.add_argument(
        "log_dir",
        help="ログディレクトリのパス。.out のみを集計し、.err は無視する。",
    )
    parser.add_argument(
        "--output-dir",
        default=None,
        help="出力先ディレクトリ。指定された場合は --scenario-name より優先する。",
    )
    parser.add_argument(
        "--scenario-name",
        default=None,
        help="scenarios/{scenario_name}/results を出力先にする。例: JIP/1-1-2",
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
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    log_dir = os.path.abspath(args.log_dir)

    try:
        output_dir = resolve_output_dir(args)
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
