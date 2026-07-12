#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Aggregate schubert_logs results for vehicle-assistant-system simulations.

Expected input layout:

    schubert_logs/
    ├── va_s1_e1.0_v1.0_r1/
    │   ├── status.txt
    │   ├── tripinfo.xml
    │   ├── va_s1_e1.0_v1.0_r1.out
    │   └── va_s1_e1.0_v1.0_r1.err
    ├── va_s1_e1.0_v1.0_r2/
    │   └── ...
    └── ...

Features:
- Reads experiment directories named:
    va_s{scenario}_e{early_rate}_v{v2v_rate}_r{run_id}
- Ignores backup directories such as:
    va_s1_e1.0_v1.0_r1.backup_YYYYmmdd_HHMMSS
- Uses status.txt to distinguish successful and failed runs.
- Reads only each experiment's .out file and ignores .err.
- Parses scalar metrics and dictionary metrics from
  "===== Simlation Result Summary =====".
- Supports the corrected marker
  "===== Simulation Result Summary =====" as well.
- Groups results by scenario / early_rate / v2v_rate.
- Aggregates run_id = 1..N and records missing and failed runs.
- Normalizes derived vehicle IDs such as init_..., newveh_..., and ped_...
  to the original vehicle ID.
- Does not drop duplicated normalized IDs. Values are accumulated as lists
  and averaged later.
- Handles rate_vehicle_abandonment values with a trailing percent sign.
- Writes output.json using the same structure as the existing Slurm
  aggregation implementation.
- Writes a comparison CDF PDF for each scenario.

Examples:

    # 50 runs
    python aggregate_schubert_simulation_logs.py \
        ./schubert_logs \
        --expected-max-run-id 50

    # Write to scenarios/JIP/1-1-4/results
    python aggregate_schubert_simulation_logs.py \
        ./schubert_logs \
        --scenario-name JIP/1-1-4 \
        --expected-max-run-id 50

    # 1250 runs
    python aggregate_schubert_simulation_logs.py \
        ./schubert_logs \
        --scenario-name JIP/1-1-4 \
        --expected-max-run-id 1250

    # Disable CDF generation
    python aggregate_schubert_simulation_logs.py \
        ./schubert_logs \
        --expected-max-run-id 50 \
        --no-cdf
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
from pathlib import Path
from typing import Any, DefaultDict, Dict, Iterable, List, Optional, Tuple


OUTPUT_JSON_FILE = "output.json"
DEFAULT_EXPECTED_MAX_RUN_ID = 50

SUMMARY_MARKERS = (
    "===== Simlation Result Summary =====",
    "===== Simulation Result Summary =====",
)

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


# Dictionary keys printed by runner_simulator.py.
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
    "pedestrian_count": [
        "pedestrian_count",
    ],
    "route_changed_vehicle_count": [
        "route_changed_vehicle_count",
    ],
    "rate_vehicle_abandonment": [
        "rate_vehicle_abandonment",
    ],
    "wrong_way_driving_count": [
        "wrong_way_driving_count",
        "wrong_way_success_count",
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
    "route_change_time_by_vehID_dict": [
        "route_change_time_by_vehID_dict",
    ],
}


NUMBER_PATTERN = r"[+-]?(?:\d+(?:\.\d*)?|\.\d+)(?:[eE][+-]?\d+)?"


RUN_DIR_RE = re.compile(
    rf"^va_s(?P<scenario>\d+)"
    rf"(?:_(?P<mode>system|nosystem))?"
    rf"_e(?P<early_rate>{NUMBER_PATTERN})"
    rf"_v(?P<v2v_rate>{NUMBER_PATTERN})"
    rf"_r(?P<run_id>\d+)$"
)


ORIGINAL_VEH_ID_RE = re.compile(
    r"^(?:ped_)?(?:init|newveh)_Shelter[A-Za-z]+_\d+_(\d+)(?:_\d+)*$"
)


@dataclass(frozen=True)
class RunDirectoryInfo:
    path: Path
    directory_name: str
    scenario: int
    early_rate: float
    early_rate_text: str
    v2v_rate: float
    v2v_rate_text: str
    run_id: int
    mode: Optional[str] = None

    @property
    def condition_key(self) -> str:
        return build_condition_key(
            scenario=self.scenario,
            early_rate_text=self.early_rate_text,
            v2v_rate_text=self.v2v_rate_text,
        )

    @property
    def status_path(self) -> Path:
        return self.path / "status.txt"

    @property
    def out_path(self) -> Path:
        expected_path = self.path / f"{self.directory_name}.out"
        if expected_path.is_file():
            return expected_path

        # Fallback for manually renamed directories.
        out_candidates = sorted(self.path.glob("*.out"))
        if len(out_candidates) == 1:
            return out_candidates[0]

        return expected_path


def warn(message: str) -> None:
    print(f"[WARNING] {message}", file=sys.stdout)


def info(message: str) -> None:
    print(f"[INFO] {message}", file=sys.stdout)


def safe_mean(values: Iterable[float]) -> Optional[float]:
    values_list = [float(value) for value in values]
    if not values_list:
        return None
    return sum(values_list) / len(values_list)


def is_close_float(a: float, b: float, tolerance: float = 1e-9) -> bool:
    return abs(a - b) < tolerance


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


def ensure_output_dir(path: Path) -> None:
    path.mkdir(parents=True, exist_ok=True)


def find_repo_root(start_path: Optional[Path] = None) -> Path:
    """
    Find the repository root by walking upward until a .git entry is found.
    """
    candidates: List[Path] = []

    if start_path is not None:
        candidates.append(start_path.resolve())

    candidates.append(Path.cwd().resolve())
    candidates.append(Path(__file__).resolve().parent)

    checked: set[Path] = set()

    for candidate in candidates:
        current = candidate

        if current.is_file():
            current = current.parent

        while True:
            if current in checked:
                break

            checked.add(current)

            if (current / ".git").exists():
                return current

            if current.parent == current:
                break

            current = current.parent

    raise FileNotFoundError(
        "リポジトリルートを示す .git が見つかりませんでした"
    )


def build_output_dir_from_scenario_name(scenario_name: str) -> Path:
    normalized_name = scenario_name.strip().strip("/")

    if normalized_name.startswith("scenarios/"):
        normalized_name = normalized_name[len("scenarios/") :]

    repo_root = find_repo_root()

    return repo_root / "scenarios" / normalized_name / "results"


def resolve_output_dir(args: argparse.Namespace) -> Path:
    """
    Priority:
      1. --output-dir
      2. --scenario-name -> scenarios/{scenario_name}/results
      3. log_dir
    """
    if args.output_dir is not None:
        return args.output_dir.resolve()

    if args.scenario_name is not None:
        return build_output_dir_from_scenario_name(args.scenario_name)

    return args.log_dir.resolve()


def build_scenario_output_dir(
    base_output_dir: Path,
    scenario: int,
) -> Path:
    return base_output_dir / f"scenario{scenario}"


def normalize_vehicle_id(raw_id: str) -> str:
    """
    Normalize vehicle/pedestrian IDs to the original vehicle ID.

    Examples:
        init_ShelterA_1_267            -> 267
        newveh_ShelterA_1_267_138      -> 267
        ped_init_ShelterA_1_197_44     -> 197
        newveh_ShelterA_1_197_45       -> 197
        ped_newveh_ShelterA_1_98_91_47 -> 98
    """
    text = str(raw_id).strip()

    match = ORIGINAL_VEH_ID_RE.match(text)
    if match is not None:
        return match.group(1)

    if re.fullmatch(r"\d+", text):
        return text

    # Preserve unknown formats instead of aborting all aggregation.
    return text


def normalize_id_value_dict_to_lists(
    raw_dict: Dict[Any, Any],
) -> Dict[str, List[float]]:
    """
    Normalize keys and accumulate values as lists.

    Derived init/newveh/ped IDs may normalize to the same original ID.
    Those values are intentionally retained and averaged later.
    """
    normalized: DefaultDict[str, List[float]] = defaultdict(list)

    for raw_key, raw_value in raw_dict.items():
        normalized_key = normalize_vehicle_id(str(raw_key))

        try:
            normalized_value = float(raw_value)
        except (TypeError, ValueError):
            continue

        if not math.isfinite(normalized_value):
            continue

        normalized[normalized_key].append(normalized_value)

    return dict(normalized)


def parse_status_file(path: Path) -> Dict[str, str]:
    values: Dict[str, str] = {}

    if not path.is_file():
        return values

    try:
        with path.open("r", encoding="utf-8", errors="replace") as file:
            for raw_line in file:
                line = raw_line.strip()

                if not line or "=" not in line:
                    continue

                key, value = line.split("=", 1)
                values[key.strip()] = value.strip()

    except OSError as error:
        warn(f"status.txt の読み込みに失敗しました: {path} ({error})")

    return values


def list_run_directories(log_dir: Path) -> List[Path]:
    if not log_dir.is_dir():
        raise FileNotFoundError(
            f"ログディレクトリが存在しません: {log_dir}"
        )

    return sorted(
        entry
        for entry in log_dir.iterdir()
        if entry.is_dir()
    )


def parse_run_directory(path: Path) -> Optional[RunDirectoryInfo]:
    directory_name = path.name
    match = RUN_DIR_RE.fullmatch(directory_name)

    if match is None:
        # aggregate/, scenario1/, and backup directories are silently ignored.
        if directory_name.startswith("va_"):
            warn(
                "実験ディレクトリ名の規則に一致しないためスキップします: "
                f"{directory_name}"
            )
        return None

    try:
        early_rate_text = match.group("early_rate")
        v2v_rate_text = match.group("v2v_rate")

        return RunDirectoryInfo(
            path=path,
            directory_name=directory_name,
            scenario=int(match.group("scenario")),
            early_rate=float(early_rate_text),
            early_rate_text=early_rate_text,
            v2v_rate=float(v2v_rate_text),
            v2v_rate_text=v2v_rate_text,
            run_id=int(match.group("run_id")),
            mode=match.group("mode"),
        )

    except ValueError as error:
        warn(
            "実験ディレクトリ名の数値変換に失敗したためスキップします: "
            f"{directory_name} ({error})"
        )
        return None


def group_by_condition(
    runs: List[RunDirectoryInfo],
) -> Dict[str, List[RunDirectoryInfo]]:
    """
    Group by condition.

    Ordinarily, one exact directory exists for each condition/run_id.
    If a duplicate is somehow detected, the directory whose status.txt has the
    newest modification time is used.
    """
    selected: Dict[Tuple[str, int], RunDirectoryInfo] = {}

    for run_info in runs:
        key = (run_info.condition_key, run_info.run_id)
        existing = selected.get(key)

        if existing is None:
            selected[key] = run_info
            continue

        existing_mtime = (
            existing.status_path.stat().st_mtime
            if existing.status_path.exists()
            else existing.path.stat().st_mtime
        )
        candidate_mtime = (
            run_info.status_path.stat().st_mtime
            if run_info.status_path.exists()
            else run_info.path.stat().st_mtime
        )

        if candidate_mtime > existing_mtime:
            warn(
                "同一条件・同一 run_id の重複を検出しました。"
                f"新しいディレクトリを優先します: "
                f"{run_info.condition_key}, run_id={run_info.run_id}"
            )
            selected[key] = run_info

    grouped: DefaultDict[str, List[RunDirectoryInfo]] = defaultdict(list)

    for run_info in selected.values():
        grouped[run_info.condition_key].append(run_info)

    return {
        condition_key: sorted(
            condition_runs,
            key=lambda item: item.run_id,
        )
        for condition_key, condition_runs in grouped.items()
    }


def read_text_file(path: Path) -> Optional[str]:
    try:
        with path.open("r", encoding="utf-8", errors="replace") as file:
            return file.read()
    except OSError as error:
        warn(f"ファイル読み込みに失敗しました: {path} ({error})")
        return None


def get_summary_text(text: str) -> str:
    marker_index = -1

    for marker in SUMMARY_MARKERS:
        marker_index = max(marker_index, text.rfind(marker))

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

    if not math.isfinite(parsed):
        return None

    return parsed


def extract_scalar_by_aliases(
    text: str,
    aliases: List[str],
) -> Optional[float]:
    for alias in aliases:
        pattern = re.compile(
            rf"^\s*{re.escape(alias)}\s*:\s*(.+?)\s*$",
            re.MULTILINE,
        )
        match = pattern.search(text)

        if match is None:
            continue

        raw_value = match.group(1)
        value = parse_scalar_value(raw_value)

        if value is None:
            warn(
                "数値パースに失敗しました: "
                f"key={alias}, value={raw_value.strip()}"
            )

        return value

    return None


def find_braced_literal_after_key(
    text: str,
    key_name: str,
) -> Optional[str]:
    key_pattern = re.compile(
        rf"{re.escape(key_name)}\s*:\s*\{{",
        re.MULTILINE,
    )
    match = key_pattern.search(text)

    if match is None:
        return None

    brace_start = match.end() - 1
    depth = 0
    in_string: Optional[str] = None
    escaped = False

    for index in range(brace_start, len(text)):
        character = text[index]

        if in_string is not None:
            if escaped:
                escaped = False
            elif character == "\\":
                escaped = True
            elif character == in_string:
                in_string = None
            continue

        if character in {"'", '"'}:
            in_string = character
            continue

        if character == "{":
            depth += 1
        elif character == "}":
            depth -= 1

            if depth == 0:
                return text[brace_start : index + 1]

    return None


def extract_dict_by_aliases(
    text: str,
    aliases: List[str],
) -> Optional[Dict[Any, Any]]:
    for alias in aliases:
        literal_text = find_braced_literal_after_key(text, alias)

        if literal_text is None:
            continue

        try:
            parsed = ast.literal_eval(literal_text)
        except (ValueError, SyntaxError) as error:
            warn(
                f"辞書パースに失敗しました: key={alias} ({error})"
            )
            return None

        if not isinstance(parsed, dict):
            warn(f"dictではない値が見つかりました: key={alias}")
            return None

        return parsed

    return None


def parse_log_content(
    text: str,
) -> Tuple[
    Dict[str, Optional[float]],
    Dict[str, Dict[str, List[float]]],
]:
    summary_text = get_summary_text(text)

    count_values: Dict[str, Optional[float]] = {}
    dict_values: Dict[str, Dict[str, List[float]]] = {}

    for canonical_key in COUNT_KEYS:
        aliases = ALIAS_MAP.get(canonical_key, [canonical_key])
        count_values[canonical_key] = extract_scalar_by_aliases(
            summary_text,
            aliases,
        )

    for canonical_key in DICT_KEYS:
        aliases = ALIAS_MAP.get(canonical_key, [canonical_key])
        raw_dict = extract_dict_by_aliases(summary_text, aliases)

        if raw_dict is None:
            dict_values[canonical_key] = {}
        else:
            dict_values[canonical_key] = (
                normalize_id_value_dict_to_lists(raw_dict)
            )

    return count_values, dict_values


def extend_accumulator(
    accumulator: DefaultDict[str, List[float]],
    values_by_id: Dict[str, List[float]],
) -> None:
    for vehicle_id, values in values_by_id.items():
        accumulator[vehicle_id].extend(values)


def flatten_values(
    values_by_id: Dict[str, List[float]],
) -> List[float]:
    return [
        value
        for values in values_by_id.values()
        for value in values
    ]


def mean_by_id(
    accumulator: DefaultDict[str, List[float]],
) -> Dict[str, float]:
    result: Dict[str, float] = {}

    for vehicle_id, values in accumulator.items():
        mean_value = safe_mean(values)

        if mean_value is not None:
            result[vehicle_id] = mean_value

    return result


def aggregate_condition(
    condition_key: str,
    runs: List[RunDirectoryInfo],
    expected_max_run_id: int,
    allow_missing_status: bool,
) -> Dict[str, Any]:
    if not runs:
        raise ValueError(f"runs が空です: {condition_key}")

    first_run = runs[0]

    expected_run_ids = list(range(1, expected_max_run_id + 1))
    found_run_ids = sorted(run_info.run_id for run_info in runs)
    missing_run_ids = sorted(
        set(expected_run_ids) - set(found_run_ids)
    )

    parsed_run_ids: List[int] = []
    failed_run_ids: List[int] = []

    count_accumulator: DefaultDict[str, List[float]] = defaultdict(list)
    arrival_time_accumulator: DefaultDict[str, List[float]] = (
        defaultdict(list)
    )
    abandon_time_by_vehicle: DefaultDict[str, List[float]] = (
        defaultdict(list)
    )
    walking_distance_by_vehicle: DefaultDict[str, List[float]] = (
        defaultdict(list)
    )
    route_change_time_by_vehicle: DefaultDict[str, List[float]] = (
        defaultdict(list)
    )

    abandon_time_all_events: List[float] = []
    walking_distance_all_events: List[float] = []

    for run_info in runs:
        status_values = parse_status_file(run_info.status_path)
        status = status_values.get("status")

        if status != "SUCCESS":
            if status is None and allow_missing_status:
                warn(
                    "status.txtまたはstatusがありませんが、"
                    f"--allow-missing-status により解析します: "
                    f"{run_info.directory_name}"
                )
            else:
                failed_run_ids.append(run_info.run_id)
                warn(
                    "SUCCESSではないため集計対象から除外します: "
                    f"{run_info.directory_name}, "
                    f"status={status or 'MISSING'}"
                )
                continue

        out_path = run_info.out_path
        text = read_text_file(out_path)

        if text is None:
            failed_run_ids.append(run_info.run_id)
            continue

        count_values, dict_values = parse_log_content(text)

        has_any_count = any(
            value is not None
            for value in count_values.values()
        )
        has_any_dict = any(
            bool(values)
            for values in dict_values.values()
        )

        if not has_any_count and not has_any_dict:
            failed_run_ids.append(run_info.run_id)
            warn(
                "必要なキーを抽出できなかったため失敗扱いにします: "
                f"{out_path}"
            )
            continue

        parsed_run_ids.append(run_info.run_id)

        for key, value in count_values.items():
            if value is not None:
                count_accumulator[key].append(value)

        arrival_dict = dict_values.get(
            "arrival_time_by_vehID_dict",
            {},
        )
        extend_accumulator(
            arrival_time_accumulator,
            arrival_dict,
        )

        abandon_dict = dict_values.get(
            "vehicle_abandant_time_by_pedestrianID_dict",
            {},
        )
        extend_accumulator(
            abandon_time_by_vehicle,
            abandon_dict,
        )
        abandon_time_all_events.extend(
            flatten_values(abandon_dict)
        )

        walking_dict = dict_values.get(
            "walking_distance_by_pedestrianID_dict",
            {},
        )
        extend_accumulator(
            walking_distance_by_vehicle,
            walking_dict,
        )
        walking_distance_all_events.extend(
            flatten_values(walking_dict)
        )

        route_change_dict = dict_values.get(
            "route_change_time_by_vehID_dict",
            {},
        )
        extend_accumulator(
            route_change_time_by_vehicle,
            route_change_dict,
        )

    parsed_run_ids = sorted(set(parsed_run_ids))
    failed_run_ids = sorted(set(failed_run_ids))

    count_averages: Dict[str, Optional[float]] = {
        key: safe_mean(count_accumulator.get(key, []))
        for key in COUNT_KEYS
    }

    vehicle_mean_arrival_time = mean_by_id(
        arrival_time_accumulator
    )
    vehicle_mean_abandon_time = mean_by_id(
        abandon_time_by_vehicle
    )
    vehicle_mean_walking_distance = mean_by_id(
        walking_distance_by_vehicle
    )
    vehicle_mean_route_change_time = mean_by_id(
        route_change_time_by_vehicle
    )

    sorted_arrival_time_list = sorted(
        float(value)
        for value in vehicle_mean_arrival_time.values()
    )

    result: Dict[str, Any] = {
        "scenario": first_run.scenario,
        "mode": first_run.mode,
        "early_rate": first_run.early_rate,
        "v2v_rate": first_run.v2v_rate,
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
                for vehicle_id in sort_numeric_strings_as_numbers(
                    vehicle_mean_arrival_time.keys()
                )
            },
        },
        "abandonment": {
            "mean_abandon_time": safe_mean(
                abandon_time_all_events
            ),
            "mean_walking_distance": safe_mean(
                walking_distance_all_events
            ),
            "all_abandon_time_events": sorted(
                float(value)
                for value in abandon_time_all_events
            ),
            "vehicle_mean_abandon_time": {
                vehicle_id: vehicle_mean_abandon_time[vehicle_id]
                for vehicle_id in sort_numeric_strings_as_numbers(
                    vehicle_mean_abandon_time.keys()
                )
            },
            "vehicle_mean_walking_distance": {
                vehicle_id: vehicle_mean_walking_distance[vehicle_id]
                for vehicle_id in sort_numeric_strings_as_numbers(
                    vehicle_mean_walking_distance.keys()
                )
            },
        },
    }

    if vehicle_mean_route_change_time:
        result["route_change_time"] = {
            "vehicle_mean_route_change_time": {
                vehicle_id: vehicle_mean_route_change_time[vehicle_id]
                for vehicle_id in sort_numeric_strings_as_numbers(
                    vehicle_mean_route_change_time.keys()
                )
            }
        }

    return result


def canonical_rate_key(value: float) -> str:
    """
    Format rate values without losing precision.

    Examples:
        1.0  -> "1.0"
        0.5  -> "0.5"
        0.25 -> "0.25"
    """
    text = format(value, ".15g")

    if "e" not in text.lower() and "." not in text:
        text += ".0"

    return text


def get_requested_output_key(
    early_rate: float,
    v2v_rate: float,
    mode: Optional[str] = None,
) -> Optional[str]:
    if mode == "nosystem" or is_close_float(v2v_rate, 0.0):
        return "nosystem"

    if mode == "system" or is_close_float(v2v_rate, 1.0):
        return canonical_rate_key(early_rate)

    return None


def build_requested_output_for_scenario(
    scenario_results: Dict[str, Dict[str, Any]],
) -> Dict[str, Any]:
    """
    Build the same top-level JSON structure as the Slurm aggregator.

    Preferred order:
      1. "1.0"
      2. "nosystem"
      3. Other numeric early-rate keys
    """
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
        key
        for key in selected_results.keys()
        if key not in ordered_keys
    ]

    def additional_sort_key(key: str) -> Tuple[int, Any]:
        try:
            return (0, float(key))
        except ValueError:
            return (1, key)

    ordered_keys.extend(
        sorted(additional_keys, key=additional_sort_key)
    )

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

        arrival_values = list(
            condition_result["arrival_time"].get(
                "arrival_time_list",
                [],
            )
        )

        if arrival_values:
            comparison_data[output_key] = [
                float(value)
                for value in arrival_values
            ]

    ordered_output = build_requested_output_for_scenario(
        scenario_results
    )
    ordered_keys = list(ordered_output.keys())

    return {
        key: comparison_data[key]
        for key in ordered_keys
        if key in comparison_data
    }


def compute_cdf_points(
    values: List[float],
) -> Tuple[List[float], List[float]]:
    if not values:
        return [], []

    sorted_values = sorted(float(value) for value in values)
    count = len(sorted_values)

    cdf = [
        (index + 1) / count
        for index in range(count)
    ]

    return sorted_values, cdf


def label_for_output_key(output_key: str) -> str:
    if output_key == "nosystem":
        return "nosystem: e1.0, v0.0"

    return f"system: e{output_key}, v1.0"


def plot_comparison_cdf_to_path(
    comparison_data: Dict[str, List[float]],
    save_path: Path,
) -> Optional[Path]:
    if not comparison_data:
        return None

    try:
        import matplotlib.pyplot as plt
    except ModuleNotFoundError as error:
        raise ModuleNotFoundError(
            "CDFを出力するにはmatplotlibが必要です。"
            " `python -m pip install matplotlib` を実行するか、"
            " `--no-cdf`を指定してください。"
        ) from error

    plotted = False
    plt.figure(figsize=(8, 6))

    for output_key, values in comparison_data.items():
        x_values, y_values = compute_cdf_points(values)

        if not x_values:
            continue

        plt.plot(
            x_values,
            y_values,
            label=label_for_output_key(output_key),
        )
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
    return (
        value is None
        or isinstance(value, (str, int, float, bool))
    )


def dumps_json_scalar(value: Any) -> str:
    return json.dumps(
        value,
        ensure_ascii=False,
        allow_nan=False,
    )


def format_json_key(key: Any) -> str:
    return json.dumps(
        str(key),
        ensure_ascii=False,
        allow_nan=False,
    )


def format_json_value(
    value: Any,
    indent: int = 0,
    chunk_size: int = JSON_CHUNK_SIZE,
) -> str:
    if isinstance(value, dict):
        return format_json_dict_compact(
            value,
            indent=indent,
            chunk_size=chunk_size,
        )

    if isinstance(value, list):
        return format_json_list_compact(
            value,
            indent=indent,
            chunk_size=chunk_size,
        )

    if isinstance(value, tuple):
        return format_json_list_compact(
            list(value),
            indent=indent,
            chunk_size=chunk_size,
        )

    return dumps_json_scalar(value)


def format_json_list_compact(
    values: List[Any],
    indent: int = 0,
    chunk_size: int = JSON_CHUNK_SIZE,
) -> str:
    if not values:
        return "[]"

    if all(is_json_scalar(value) for value in values):
        dumped_values = [
            dumps_json_scalar(value)
            for value in values
        ]

        if len(dumped_values) <= chunk_size:
            return "[" + ", ".join(dumped_values) + "]"

        child_indent = indent + JSON_INDENT_SIZE
        lines = ["["]

        for start in range(
            0,
            len(dumped_values),
            chunk_size,
        ):
            chunk = dumped_values[
                start : start + chunk_size
            ]
            is_last_chunk = (
                start + chunk_size
                >= len(dumped_values)
            )
            comma = "" if is_last_chunk else ","

            lines.append(
                f"{json_indent(child_indent)}"
                f"{', '.join(chunk)}{comma}"
            )

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

        lines.append(
            f"{json_indent(child_indent)}"
            f"{formatted_value}{comma}"
        )

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

    if all(
        is_json_scalar(value)
        for _, value in items
    ):
        dumped_entries = [
            f"{format_json_key(key)}: "
            f"{dumps_json_scalar(value)}"
            for key, value in items
        ]

        if len(dumped_entries) <= chunk_size:
            return "{ " + ", ".join(dumped_entries) + " }"

        lines = ["{"]

        for start in range(
            0,
            len(dumped_entries),
            chunk_size,
        ):
            chunk = dumped_entries[
                start : start + chunk_size
            ]
            is_last_chunk = (
                start + chunk_size
                >= len(dumped_entries)
            )
            comma = "" if is_last_chunk else ","

            lines.append(
                f"{json_indent(child_indent)}"
                f"{', '.join(chunk)}{comma}"
            )

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
            f"{json_indent(child_indent)}"
            f"{format_json_key(key)}: "
            f"{formatted_value}{comma}"
        )

    lines.append(f"{json_indent(indent)}}}")
    return "\n".join(lines)


def write_json(
    data: Dict[str, Any],
    output_path: Path,
) -> None:
    formatted_json = format_json_value(
        data,
        indent=0,
        chunk_size=JSON_CHUNK_SIZE,
    )

    with output_path.open(
        "w",
        encoding="utf-8",
    ) as file:
        file.write(formatted_json)
        file.write("\n")


def aggregate_all_conditions(
    log_dir: Path,
    output_dir: Path,
    expected_max_run_id: int,
    json_filename: str,
    allow_missing_status: bool,
    generate_cdf: bool,
) -> Dict[str, Any]:
    ensure_output_dir(output_dir)

    directory_paths = list_run_directories(log_dir)

    parsed_runs = [
        run_info
        for path in directory_paths
        if (run_info := parse_run_directory(path)) is not None
    ]

    info(
        "解析対象の実験ディレクトリ数: "
        f"{len(parsed_runs)}"
    )

    grouped = group_by_condition(parsed_runs)

    info(f"検出された条件数: {len(grouped)}")

    all_results_by_scenario: DefaultDict[
        int,
        Dict[str, Dict[str, Any]],
    ] = defaultdict(dict)

    for condition_key in sorted(grouped.keys()):
        info(f"集計中: {condition_key}")

        condition_result = aggregate_condition(
            condition_key=condition_key,
            runs=grouped[condition_key],
            expected_max_run_id=expected_max_run_id,
            allow_missing_status=allow_missing_status,
        )

        scenario = int(condition_result["scenario"])

        all_results_by_scenario[scenario][
            condition_key
        ] = condition_result

    final_output: Dict[str, Any] = {}
    scenario_count = len(all_results_by_scenario)

    for scenario, scenario_results in sorted(
        all_results_by_scenario.items()
    ):
        scenario_output_dir = build_scenario_output_dir(
            output_dir,
            scenario,
        )
        ensure_output_dir(scenario_output_dir)

        scenario_json_data = (
            build_requested_output_for_scenario(
                scenario_results
            )
        )

        json_output_path = (
            scenario_output_dir / json_filename
        )
        write_json(
            scenario_json_data,
            json_output_path,
        )
        info(f"JSONを出力しました: {json_output_path}")

        comparison_data = (
            build_comparison_cdf_data_for_scenario(
                scenario_results
            )
        )

        if generate_cdf:
            cdf_output_path = (
                scenario_output_dir
                / f"cdf_compare_s{scenario}.pdf"
            )

            if plot_comparison_cdf_to_path(
                comparison_data,
                cdf_output_path,
            ) is not None:
                info(f"CDFを出力しました: {cdf_output_path}")
            else:
                warn(
                    "比較CDF用データが不足しているため"
                    f"スキップします: scenario={scenario}"
                )

        # A single-scenario input also writes output.json directly under
        # output_dir, matching the common workflow of the existing script.
        if scenario_count == 1:
            mirror_json_path = output_dir / json_filename

            write_json(
                scenario_json_data,
                mirror_json_path,
            )
            info(f"JSONを出力しました: {mirror_json_path}")

            if generate_cdf and comparison_data:
                mirror_cdf_path = (
                    output_dir
                    / f"cdf_compare_s{scenario}.pdf"
                )

                plot_comparison_cdf_to_path(
                    comparison_data,
                    mirror_cdf_path,
                )
                info(f"CDFを出力しました: {mirror_cdf_path}")

        final_output[f"scenario{scenario}"] = (
            scenario_json_data
        )

    return final_output


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "schubert_logs配下の実験ディレクトリを条件別に集計し、"
            "output.jsonとCDFを出力する"
        )
    )

    parser.add_argument(
        "log_dir",
        type=Path,
        help=(
            "schubert_logsディレクトリ。"
            "各va_s...ディレクトリ内の.outを集計し、"
            ".errは無視する。"
        ),
    )

    parser.add_argument(
        "--output-dir",
        type=Path,
        default=None,
        help=(
            "出力先ディレクトリ。"
            "指定された場合は--scenario-nameより優先する。"
        ),
    )

    parser.add_argument(
        "--scenario-name",
        default=None,
        help=(
            "scenarios/{scenario_name}/resultsを出力先にする。"
            "例: JIP/1-1-4"
        ),
    )

    parser.add_argument(
        "--expected-max-run-id",
        type=int,
        default=DEFAULT_EXPECTED_MAX_RUN_ID,
        help=(
            "想定する最大run_id。"
            f"既定値: {DEFAULT_EXPECTED_MAX_RUN_ID}"
        ),
    )

    parser.add_argument(
        "--json-file",
        default=OUTPUT_JSON_FILE,
        help=(
            "出力JSONファイル名。"
            f"既定値: {OUTPUT_JSON_FILE}"
        ),
    )

    parser.add_argument(
        "--allow-missing-status",
        action="store_true",
        help=(
            "status.txtが存在しない実験についても"
            ".outの解析を試みる。"
            "status=FAILEDまたはRUNNINGは対象外。"
        ),
    )

    parser.add_argument(
        "--no-cdf",
        action="store_true",
        help="CDF PDFを生成せず、JSONだけを出力する。",
    )

    args = parser.parse_args()

    if args.expected_max_run_id < 1:
        parser.error(
            "--expected-max-run-idは1以上で指定してください"
        )

    return args


def main() -> int:
    args = parse_args()

    log_dir = args.log_dir.resolve()

    try:
        output_dir = resolve_output_dir(args)

        aggregate_all_conditions(
            log_dir=log_dir,
            output_dir=output_dir,
            expected_max_run_id=args.expected_max_run_id,
            json_filename=args.json_file,
            allow_missing_status=args.allow_missing_status,
            generate_cdf=not args.no_cdf,
        )

    except Exception as error:
        print(f"[ERROR] {error}", file=sys.stderr)
        return 1

    return 0


if __name__ == "__main__":
    sys.exit(main())

