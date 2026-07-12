#!/usr/bin/env python3
from __future__ import annotations

import argparse
import csv
import json
import math
import re
import statistics
import xml.etree.ElementTree as ET
from collections import defaultdict
from pathlib import Path
from typing import Any, Iterable


JOB_NAME_RE = re.compile(
    r"^va_s(?P<scenario_id>\d+)"
    r"_e(?P<early_rate>[^_]+)"
    r"_v(?P<v2v_rate>[^_]+)"
    r"_r(?P<run_id>\d+)$"
)

RUN_SUMMARY_FIELDS = [
    "job_name", "scenario_id", "early_rate", "v2v_rate", "run_id",
    "status", "exit_code", "hostname", "start_time", "end_time",
    "config", "run_dir", "tripinfo_exists", "tripinfo_parse_error",
    "vehicle_count", "completion_time", "arrival_time_mean",
    "arrival_time_median", "arrival_time_p95", "duration_mean",
    "duration_median", "waiting_time_mean", "time_loss_mean",
    "route_length_mean",
]


def parse_status_file(status_path: Path) -> dict[str, str]:
    """status.txt の key=value を辞書として読み込む。"""
    result: dict[str, str] = {}
    if not status_path.is_file():
        return result

    with status_path.open("r", encoding="utf-8", errors="replace") as file:
        for raw_line in file:
            line = raw_line.strip()
            if not line or "=" not in line:
                continue
            key, value = line.split("=", 1)
            result[key.strip()] = value.strip()
    return result


def to_float(value: str | None) -> float | None:
    if value is None:
        return None
    try:
        number = float(value)
    except (TypeError, ValueError):
        return None
    return number if math.isfinite(number) else None


def percentile(values: list[float], percentage: float) -> float | None:
    """線形補間によるパーセンタイル。percentage は0.0から1.0。"""
    if not values:
        return None
    ordered = sorted(values)
    if len(ordered) == 1:
        return ordered[0]

    position = (len(ordered) - 1) * percentage
    lower_index = math.floor(position)
    upper_index = math.ceil(position)
    if lower_index == upper_index:
        return ordered[lower_index]

    fraction = position - lower_index
    return ordered[lower_index] + (
        ordered[upper_index] - ordered[lower_index]
    ) * fraction


def mean_or_none(values: Iterable[float]) -> float | None:
    items = list(values)
    return statistics.fmean(items) if items else None


def median_or_none(values: Iterable[float]) -> float | None:
    items = list(values)
    return statistics.median(items) if items else None


def aggregate_tripinfo(tripinfo_path: Path) -> dict[str, Any]:
    """
    SUMOのtripinfo.xmlから1試行分の指標を集計する。

    completion_timeはtripinfo要素のarrival最大値。
    """
    result: dict[str, Any] = {
        "tripinfo_exists": tripinfo_path.is_file(),
        "tripinfo_parse_error": "",
        "vehicle_count": 0,
        "completion_time": None,
        "arrival_time_mean": None,
        "arrival_time_median": None,
        "arrival_time_p95": None,
        "duration_mean": None,
        "duration_median": None,
        "waiting_time_mean": None,
        "time_loss_mean": None,
        "route_length_mean": None,
    }

    if not tripinfo_path.is_file():
        result["tripinfo_parse_error"] = "tripinfo.xml not found"
        return result
    if tripinfo_path.stat().st_size == 0:
        result["tripinfo_parse_error"] = "tripinfo.xml is empty"
        return result

    arrivals: list[float] = []
    durations: list[float] = []
    waiting_times: list[float] = []
    time_losses: list[float] = []
    route_lengths: list[float] = []

    try:
        for _, element in ET.iterparse(tripinfo_path, events=("end",)):
            if element.tag != "tripinfo":
                continue

            result["vehicle_count"] += 1
            arrival = to_float(element.get("arrival"))
            duration = to_float(element.get("duration"))
            waiting_time = to_float(element.get("waitingTime"))
            time_loss = to_float(element.get("timeLoss"))
            route_length = to_float(element.get("routeLength"))

            if arrival is not None and arrival >= 0:
                arrivals.append(arrival)
            if duration is not None and duration >= 0:
                durations.append(duration)
            if waiting_time is not None and waiting_time >= 0:
                waiting_times.append(waiting_time)
            if time_loss is not None and time_loss >= 0:
                time_losses.append(time_loss)
            if route_length is not None and route_length >= 0:
                route_lengths.append(route_length)
            element.clear()

    except ET.ParseError as error:
        result["tripinfo_parse_error"] = f"XML parse error: {error}"
        return result
    except OSError as error:
        result["tripinfo_parse_error"] = f"File read error: {error}"
        return result

    result.update({
        "completion_time": max(arrivals) if arrivals else None,
        "arrival_time_mean": mean_or_none(arrivals),
        "arrival_time_median": median_or_none(arrivals),
        "arrival_time_p95": percentile(arrivals, 0.95),
        "duration_mean": mean_or_none(durations),
        "duration_median": median_or_none(durations),
        "waiting_time_mean": mean_or_none(waiting_times),
        "time_loss_mean": mean_or_none(time_losses),
        "route_length_mean": mean_or_none(route_lengths),
    })
    return result


def round_float(value: Any, digits: int = 6) -> Any:
    return round(value, digits) if isinstance(value, float) else value


def collect_run_summary(run_dir: Path) -> dict[str, Any] | None:
    """1つのva_s...ディレクトリを集計する。"""
    match = JOB_NAME_RE.fullmatch(run_dir.name)
    if match is None:
        return None

    name_fields = match.groupdict()
    status = parse_status_file(run_dir / "status.txt")
    tripinfo = aggregate_tripinfo(run_dir / "tripinfo.xml")

    row: dict[str, Any] = {
        "job_name": run_dir.name,
        "scenario_id": int(name_fields["scenario_id"]),
        "early_rate": name_fields["early_rate"],
        "v2v_rate": name_fields["v2v_rate"],
        "run_id": int(name_fields["run_id"]),
        "status": status.get("status", "MISSING"),
        "exit_code": status.get("exit_code", ""),
        "hostname": status.get("hostname", ""),
        "start_time": status.get("start_time", ""),
        "end_time": status.get("end_time", ""),
        "config": status.get("config", ""),
        "run_dir": str(run_dir.resolve()),
    }
    row.update(tripinfo)
    return {key: round_float(value) for key, value in row.items()}


def numeric_values(rows: list[dict[str, Any]], field: str) -> list[float]:
    values: list[float] = []
    for row in rows:
        value = row.get(field)
        if isinstance(value, (int, float)) and not isinstance(value, bool):
            if math.isfinite(float(value)):
                values.append(float(value))
    return values


def summarize_group(rows: list[dict[str, Any]]) -> dict[str, Any]:
    successful_rows = [row for row in rows if row["status"] == "SUCCESS"]
    failed_rows = [row for row in rows if row["status"] == "FAILED"]
    running_rows = [row for row in rows if row["status"] == "RUNNING"]
    missing_rows = [row for row in rows if row["status"] == "MISSING"]

    completion_times = numeric_values(successful_rows, "completion_time")
    vehicle_counts = numeric_values(successful_rows, "vehicle_count")
    duration_means = numeric_values(successful_rows, "duration_mean")
    waiting_means = numeric_values(successful_rows, "waiting_time_mean")
    time_loss_means = numeric_values(successful_rows, "time_loss_mean")

    return {
        "total_runs": len(rows),
        "success_runs": len(successful_rows),
        "failed_runs": len(failed_rows),
        "running_runs": len(running_rows),
        "missing_status_runs": len(missing_rows),
        "success_rate": len(successful_rows) / len(rows) if rows else None,
        "completion_time": {
            "count": len(completion_times),
            "mean": mean_or_none(completion_times),
            "std_population": (
                statistics.pstdev(completion_times)
                if len(completion_times) >= 2
                else 0.0 if completion_times else None
            ),
            "median": median_or_none(completion_times),
            "min": min(completion_times) if completion_times else None,
            "max": max(completion_times) if completion_times else None,
            "p95": percentile(completion_times, 0.95),
        },
        "vehicle_count_mean": mean_or_none(vehicle_counts),
        "duration_mean_across_runs": mean_or_none(duration_means),
        "waiting_time_mean_across_runs": mean_or_none(waiting_means),
        "time_loss_mean_across_runs": mean_or_none(time_loss_means),
        "failed_jobs": [row["job_name"] for row in failed_rows],
        "tripinfo_error_jobs": [
            row["job_name"] for row in rows if row.get("tripinfo_parse_error")
        ],
    }


def aggregate_schubert_logs(
    log_root: Path,
    output_dir: Path | None = None,
) -> dict[str, Any]:
    """
    schubert_logs以下の実験結果を集計する。

    出力:
      run_summary.csv        1試行1行の詳細
      condition_summary.csv  scenario×early_rate×v2v_rate単位の集計
      aggregate_summary.json 全体と条件別の詳細
      failed_jobs.txt        失敗ジョブ名
    """
    log_root = log_root.resolve()
    if not log_root.is_dir():
        raise FileNotFoundError(f"schubert_logs directory not found: {log_root}")

    if output_dir is None:
        output_dir = log_root / "aggregate"
    output_dir = output_dir.resolve()
    output_dir.mkdir(parents=True, exist_ok=True)

    rows: list[dict[str, Any]] = []
    for child in sorted(log_root.iterdir()):
        if not child.is_dir():
            continue
        row = collect_run_summary(child)
        if row is not None:
            rows.append(row)

    rows.sort(key=lambda row: (
        row["scenario_id"], row["early_rate"],
        row["v2v_rate"], row["run_id"],
    ))

    run_summary_path = output_dir / "run_summary.csv"
    with run_summary_path.open("w", encoding="utf-8", newline="") as file:
        writer = csv.DictWriter(file, fieldnames=RUN_SUMMARY_FIELDS)
        writer.writeheader()
        writer.writerows(rows)

    grouped: dict[tuple[int, str, str], list[dict[str, Any]]] = defaultdict(list)
    for row in rows:
        grouped[(row["scenario_id"], row["early_rate"], row["v2v_rate"])].append(row)

    condition_rows: list[dict[str, Any]] = []
    condition_details: dict[str, Any] = {}

    for (scenario_id, early_rate, v2v_rate), group_rows in sorted(grouped.items()):
        summary = summarize_group(group_rows)
        condition_key = (
            f"scenario={scenario_id},early_rate={early_rate},v2v_rate={v2v_rate}"
        )
        condition_details[condition_key] = summary
        completion = summary["completion_time"]

        condition_rows.append({
            "scenario_id": scenario_id,
            "early_rate": early_rate,
            "v2v_rate": v2v_rate,
            "total_runs": summary["total_runs"],
            "success_runs": summary["success_runs"],
            "failed_runs": summary["failed_runs"],
            "running_runs": summary["running_runs"],
            "success_rate": round_float(summary["success_rate"]),
            "completion_time_count": completion["count"],
            "completion_time_mean": round_float(completion["mean"]),
            "completion_time_std_population": round_float(completion["std_population"]),
            "completion_time_median": round_float(completion["median"]),
            "completion_time_min": round_float(completion["min"]),
            "completion_time_max": round_float(completion["max"]),
            "completion_time_p95": round_float(completion["p95"]),
            "vehicle_count_mean": round_float(summary["vehicle_count_mean"]),
            "duration_mean_across_runs": round_float(summary["duration_mean_across_runs"]),
            "waiting_time_mean_across_runs": round_float(summary["waiting_time_mean_across_runs"]),
            "time_loss_mean_across_runs": round_float(summary["time_loss_mean_across_runs"]),
        })

    condition_fields = [
        "scenario_id", "early_rate", "v2v_rate", "total_runs",
        "success_runs", "failed_runs", "running_runs", "success_rate",
        "completion_time_count", "completion_time_mean",
        "completion_time_std_population", "completion_time_median",
        "completion_time_min", "completion_time_max", "completion_time_p95",
        "vehicle_count_mean", "duration_mean_across_runs",
        "waiting_time_mean_across_runs", "time_loss_mean_across_runs",
    ]

    condition_summary_path = output_dir / "condition_summary.csv"
    with condition_summary_path.open("w", encoding="utf-8", newline="") as file:
        writer = csv.DictWriter(file, fieldnames=condition_fields)
        writer.writeheader()
        writer.writerows(condition_rows)

    overall_summary = summarize_group(rows)
    aggregate_json = {
        "log_root": str(log_root),
        "output_dir": str(output_dir),
        "overall": overall_summary,
        "conditions": condition_details,
    }

    aggregate_summary_path = output_dir / "aggregate_summary.json"
    with aggregate_summary_path.open("w", encoding="utf-8") as file:
        json.dump(aggregate_json, file, ensure_ascii=False, indent=2)

    failed_jobs_path = output_dir / "failed_jobs.txt"
    failed_jobs = [row["job_name"] for row in rows if row["status"] == "FAILED"]
    failed_jobs_path.write_text(
        "".join(f"{job_name}\n" for job_name in failed_jobs),
        encoding="utf-8",
    )

    return {
        "run_count": len(rows),
        "run_summary_csv": run_summary_path,
        "condition_summary_csv": condition_summary_path,
        "aggregate_summary_json": aggregate_summary_path,
        "failed_jobs_txt": failed_jobs_path,
        "overall": overall_summary,
    }


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Aggregate status.txt and tripinfo.xml under schubert_logs."
    )
    parser.add_argument(
        "log_root", nargs="?", type=Path, default=Path("schubert_logs"),
        help="schubert_logs directory; default: ./schubert_logs",
    )
    parser.add_argument(
        "--output-dir", type=Path, default=None,
        help="Output directory; default: <log_root>/aggregate",
    )
    return parser


def main() -> int:
    args = build_parser().parse_args()
    try:
        result = aggregate_schubert_logs(args.log_root, args.output_dir)
    except (FileNotFoundError, PermissionError, OSError) as error:
        print(f"ERROR: {error}")
        return 1

    overall = result["overall"]
    print("=== Aggregate Result ===")
    print(f"run_count={result['run_count']}")
    print(f"success_runs={overall['success_runs']}")
    print(f"failed_runs={overall['failed_runs']}")
    print(f"running_runs={overall['running_runs']}")
    print(f"completion_time_mean={round_float(overall['completion_time']['mean'])}")
    print(
        "completion_time_std_population="
        f"{round_float(overall['completion_time']['std_population'])}"
    )
    print(f"run_summary_csv={result['run_summary_csv']}")
    print(f"condition_summary_csv={result['condition_summary_csv']}")
    print(f"aggregate_summary_json={result['aggregate_summary_json']}")
    print(f"failed_jobs_txt={result['failed_jobs_txt']}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

