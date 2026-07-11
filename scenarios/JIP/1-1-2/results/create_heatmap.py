#!/usr/bin/env python3
from __future__ import annotations

import argparse
import csv
import json
import math
import re
import warnings
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import matplotlib.pyplot as plt

try:
    import tomllib  # Python 3.11+
except ModuleNotFoundError:
    import tomli as tomllib  # Python 3.10 or older


@dataclass
class HeatmapTable:
    row_values: list[float]
    col_values: list[float]
    values: list[list[float]]


@dataclass
class AnnotationTable:
    row_values: list[float]
    col_values: list[float]
    annotations: list[list[str]]


def load_toml(path: Path) -> dict[str, Any]:
    with path.open("rb") as f:
        return tomllib.load(f)


def extract_scenario_number_from_dir(path: Path) -> int:
    match = re.fullmatch(r"scenario(\d+)", path.name)
    if not match:
        raise ValueError(f"Invalid scenario directory name: {path.name}")
    return int(match.group(1))


def load_output_json(path: Path) -> dict[str, Any]:
    with path.open("r", encoding="utf-8") as f:
        return json.load(f)


def find_config_path(config_dir: Path, scenario_id: int) -> Path:
    return config_dir / f"config_scenario_{scenario_id}.toml"


def warn_and_skip(message: str) -> None:
    warnings.warn(message, RuntimeWarning)


def collect_records(
    results_dir: Path,
    config_dir: Path,
    scenario_ids: list[int],
) -> list[dict[str, Any]]:
    records: list[dict[str, Any]] = []

    scenario_dirs = [
        results_dir / f"scenario{scenario_id}"
        for scenario_id in scenario_ids
    ]

    for scenario_dir in scenario_dirs:
        if not scenario_dir.exists():
            warn_and_skip(f"Skip {scenario_dir.name}: directory not found")
            continue

        if not scenario_dir.is_dir():
            warn_and_skip(f"Skip {scenario_dir.name}: not a directory")
            continue

        try:
            scenario_dir_number = extract_scenario_number_from_dir(scenario_dir)
        except ValueError as e:
            warn_and_skip(f"Skip {scenario_dir.name}: {e}")
            continue

        output_json_path = scenario_dir / "output.json"

        if not output_json_path.exists():
            warn_and_skip(f"Skip {scenario_dir.name}: output.json not found")
            continue

        try:
            output_data = load_output_json(output_json_path)
        except Exception as e:
            warn_and_skip(f"Skip {scenario_dir.name}: failed to load output.json: {e}")
            continue

        if "1.0" not in output_data:
            warn_and_skip(f"Skip {scenario_dir.name}: key '1.0' not found")
            continue

        condition_data = output_data["1.0"]

        try:
            scenario_id = int(condition_data["scenario"])
        except KeyError:
            warn_and_skip(f"Skip {scenario_dir.name}: '1.0.scenario' not found")
            continue
        except Exception as e:
            warn_and_skip(f"Skip {scenario_dir.name}: invalid scenario id: {e}")
            continue

        if scenario_id != scenario_dir_number:
            warn_and_skip(
                f"{scenario_dir.name}: directory scenario number "
                f"({scenario_dir_number}) and output.json scenario id "
                f"({scenario_id}) are different. Use output.json scenario id."
            )

        config_path = find_config_path(config_dir, scenario_id)

        if not config_path.exists():
            warn_and_skip(
                f"Skip {scenario_dir.name}: config file not found: {config_path}"
            )
            continue

        try:
            config = load_toml(config_path)
        except Exception as e:
            warn_and_skip(f"Skip {scenario_dir.name}: failed to load config: {e}")
            continue

        try:
            p_follow = float(config["p_follow"])
            threshold = float(config["active_route_change_threshold_center"])
        except KeyError as e:
            warn_and_skip(f"Skip {scenario_dir.name}: missing config key: {e}")
            continue
        except Exception as e:
            warn_and_skip(f"Skip {scenario_dir.name}: invalid config value: {e}")
            continue

        try:
            arrival_time_list = condition_data["arrival_time"]["arrival_time_list"]
            if not arrival_time_list:
                raise ValueError("arrival_time_list is empty")
            evacuation_completion_time = float(arrival_time_list[-1])
        except KeyError as e:
            warn_and_skip(f"Skip {scenario_dir.name}: missing arrival_time key: {e}")
            continue
        except Exception as e:
            warn_and_skip(f"Skip {scenario_dir.name}: invalid arrival_time_list: {e}")
            continue

        try:
            count_averages = condition_data["count_averages"]

            route_changed_vehicle_count = float(
                count_averages["route_changed_vehicle_count"]
            )
            normalcy_bias_route_change_count = float(
                count_averages["normalcy_bias_route_change_count"]
            )
            majority_bias_route_change_count = float(
                count_averages["majority_bias_route_change_count"]
            )
        except KeyError as e:
            warn_and_skip(f"Skip {scenario_dir.name}: missing count_averages key: {e}")
            continue
        except Exception as e:
            warn_and_skip(
                f"Skip {scenario_dir.name}: invalid route change count value: {e}"
            )
            continue

        records.append(
            {
                "scenario_dir": scenario_dir.name,
                "scenario_id": scenario_id,
                "p_follow": p_follow,
                "active_route_change_threshold_center": threshold,
                "evacuation_completion_time": evacuation_completion_time,
                "route_changed_vehicle_count": route_changed_vehicle_count,
                "normalcy_bias_route_change_count": normalcy_bias_route_change_count,
                "majority_bias_route_change_count": majority_bias_route_change_count,
            }
        )

    return records


def build_table(
    records: list[dict[str, Any]],
    value_key: str,
) -> HeatmapTable:
    row_values = sorted(
        {
            float(record["active_route_change_threshold_center"])
            for record in records
        }
    )
    col_values = sorted({float(record["p_follow"]) for record in records})

    value_map: dict[tuple[float, float], float] = {}

    for record in records:
        row = float(record["active_route_change_threshold_center"])
        col = float(record["p_follow"])
        value = float(record[value_key])
        value_map[(row, col)] = value

    values: list[list[float]] = []

    for row in row_values:
        row_list: list[float] = []
        for col in col_values:
            row_list.append(value_map.get((row, col), math.nan))
        values.append(row_list)

    return HeatmapTable(
        row_values=row_values,
        col_values=col_values,
        values=values,
    )


def build_route_change_annotation_table(
    records: list[dict[str, Any]],
) -> AnnotationTable:
    row_values = sorted(
        {
            float(record["active_route_change_threshold_center"])
            for record in records
        }
    )
    col_values = sorted({float(record["p_follow"]) for record in records})

    annotation_map: dict[tuple[float, float], str] = {}

    for record in records:
        row = float(record["active_route_change_threshold_center"])
        col = float(record["p_follow"])

        total = float(record["route_changed_vehicle_count"])
        normalcy = float(record["normalcy_bias_route_change_count"])
        majority = float(record["majority_bias_route_change_count"])

        if total > 0.0:
            normalcy_ratio = normalcy / total * 100.0
            majority_ratio = majority / total * 100.0
        else:
            normalcy_ratio = 0.0
            majority_ratio = 0.0

        annotation = (
            f"{total:.1f}\n"
            f"({normalcy_ratio:.0f}:{majority_ratio:.0f})"
        )

        annotation_map[(row, col)] = annotation

    annotations: list[list[str]] = []

    for row in row_values:
        row_list: list[str] = []
        for col in col_values:
            row_list.append(annotation_map.get((row, col), ""))
        annotations.append(row_list)

    return AnnotationTable(
        row_values=row_values,
        col_values=col_values,
        annotations=annotations,
    )


def build_pivot_tables(
    records: list[dict[str, Any]],
) -> tuple[HeatmapTable, HeatmapTable, AnnotationTable]:
    completion_table = build_table(records, "evacuation_completion_time")
    route_changed_table = build_table(records, "route_changed_vehicle_count")
    route_changed_annotation_table = build_route_change_annotation_table(records)

    return completion_table, route_changed_table, route_changed_annotation_table


def save_table_csv(table: HeatmapTable, output_path: Path) -> None:
    with output_path.open("w", encoding="utf-8", newline="") as f:
        writer = csv.writer(f)

        header = ["active_route_change_threshold_center"]
        header.extend([f"{col:.2f}" for col in table.col_values])
        writer.writerow(header)

        for row_value, row_data in zip(table.row_values, table.values):
            row = [f"{row_value:.1f}"]
            for value in row_data:
                if math.isnan(value):
                    row.append("")
                else:
                    row.append(f"{value:.2f}")
            writer.writerow(row)


def save_annotation_csv(
    annotation_table: AnnotationTable,
    output_path: Path,
) -> None:
    with output_path.open("w", encoding="utf-8", newline="") as f:
        writer = csv.writer(f)

        header = ["active_route_change_threshold_center"]
        header.extend([f"{col:.2f}" for col in annotation_table.col_values])
        writer.writerow(header)

        for row_value, row_data in zip(
            annotation_table.row_values,
            annotation_table.annotations,
        ):
            row = [f"{row_value:.1f}"]
            row.extend(row_data)
            writer.writerow(row)


def save_heatmap(
    table: HeatmapTable,
    output_path: Path,
    title: str,
    colorbar_label: str,
    annotation_table: AnnotationTable | None = None,
    annotation_fontsize: int = 9,
) -> None:
    if not table.row_values or not table.col_values:
        warn_and_skip(f"Skip saving {output_path.name}: table is empty")
        return

    fig_width = max(8.0, 1.2 * len(table.col_values))
    fig_height = max(6.0, 0.8 * len(table.row_values))

    fig, ax = plt.subplots(figsize=(fig_width, fig_height))

    im = ax.imshow(table.values, aspect="auto")

    cbar = fig.colorbar(im, ax=ax)
    cbar.set_label(colorbar_label)

    ax.set_title(title)
    ax.set_xlabel("p_follow")
    ax.set_ylabel("active_route_change_threshold_center")

    ax.set_xticks(range(len(table.col_values)))
    ax.set_yticks(range(len(table.row_values)))

    ax.set_xticklabels([f"{x:.2f}" for x in table.col_values])
    ax.set_yticklabels([f"{y:.1f}" for y in table.row_values])

    for i, row_data in enumerate(table.values):
        for j, value in enumerate(row_data):
            if math.isnan(value):
                continue

            if annotation_table is not None:
                text = annotation_table.annotations[i][j]
            else:
                text = f"{value:.2f}"

            ax.text(
                j,
                i,
                text,
                ha="center",
                va="center",
                fontsize=annotation_fontsize,
            )

    fig.tight_layout()
    fig.savefig(output_path, format="pdf", bbox_inches="tight")
    plt.close(fig)


def resolve_default_results_dir() -> Path:
    return Path(__file__).resolve().parent


def resolve_default_config_dir(results_dir: Path) -> Path:
    return results_dir.resolve().parents[1] / "configs"


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Create heatmaps from scenario output.json files."
    )

    parser.add_argument(
        "--results-dir",
        type=Path,
        default=None,
        help="Path to scenarios/JIP/1-1-2/results",
    )

    parser.add_argument(
        "--config-dir",
        type=Path,
        default=None,
        help="Path to scenarios/JIP/configs",
    )

    parser.add_argument(
        "--scenario-ids",
        nargs="+",
        default=None,
        metavar="ID",
        help=(
            "Scenario IDs to load. Examples: "
            "--scenario-ids 1 3 5, "
            "--scenario-ids 1,3,5, "
            "--scenario-ids 31-36"
        ),
    )

    parser.add_argument(
        "--scenario-start",
        type=int,
        default=None,
        help="First scenario number for range selection. Default: 1",
    )

    parser.add_argument(
        "--scenario-end",
        type=int,
        default=None,
        help="Last scenario number for range selection. Default: 64",
    )

    return parser.parse_args()


def validate_scenario_range(scenario_start: int, scenario_end: int) -> None:
    if scenario_start <= 0:
        raise ValueError(
            f"scenario_start must be positive: scenario_start={scenario_start}"
        )

    if scenario_end <= 0:
        raise ValueError(
            f"scenario_end must be positive: scenario_end={scenario_end}"
        )

    if scenario_start > scenario_end:
        raise ValueError(
            "scenario_start must be less than or equal to scenario_end: "
            f"scenario_start={scenario_start}, scenario_end={scenario_end}"
        )


def parse_scenario_ids(raw_values: list[str]) -> list[int]:
    scenario_ids: list[int] = []

    for raw_value in raw_values:
        for token in raw_value.split(","):
            token = token.strip()
            if not token:
                continue

            token = re.sub(r"^scenario", "", token, flags=re.IGNORECASE)

            range_match = re.fullmatch(r"(\d+)-(?:scenario)?(\d+)", token, re.IGNORECASE)
            if range_match:
                range_start = int(range_match.group(1))
                range_end = int(range_match.group(2))
                validate_scenario_range(range_start, range_end)
                scenario_ids.extend(range(range_start, range_end + 1))
                continue

            if not token.isdigit():
                raise ValueError(
                    f"Invalid scenario ID specification: {raw_value!r}"
                )

            scenario_id = int(token)
            if scenario_id <= 0:
                raise ValueError(
                    f"Scenario ID must be positive: {scenario_id}"
                )
            scenario_ids.append(scenario_id)

    if not scenario_ids:
        raise ValueError("No scenario IDs were specified")

    # Remove duplicates while preserving the user-specified order.
    return list(dict.fromkeys(scenario_ids))


def resolve_scenario_ids(args: argparse.Namespace) -> list[int]:
    uses_explicit_ids = args.scenario_ids is not None
    uses_range = args.scenario_start is not None or args.scenario_end is not None

    if uses_explicit_ids and uses_range:
        raise ValueError(
            "--scenario-ids cannot be used together with "
            "--scenario-start or --scenario-end"
        )

    if uses_explicit_ids:
        return parse_scenario_ids(args.scenario_ids)

    scenario_start = args.scenario_start if args.scenario_start is not None else 1
    scenario_end = args.scenario_end if args.scenario_end is not None else 64
    validate_scenario_range(scenario_start, scenario_end)
    return list(range(scenario_start, scenario_end + 1))


def main() -> None:
    args = parse_args()
    scenario_ids = resolve_scenario_ids(args)

    results_dir = (
        args.results_dir.resolve()
        if args.results_dir
        else resolve_default_results_dir()
    )

    config_dir = (
        args.config_dir.resolve()
        if args.config_dir
        else resolve_default_config_dir(results_dir)
    )

    if not results_dir.exists():
        raise FileNotFoundError(f"results_dir not found: {results_dir}")

    if not config_dir.exists():
        raise FileNotFoundError(f"config_dir not found: {config_dir}")

    records = collect_records(
        results_dir=results_dir,
        config_dir=config_dir,
        scenario_ids=scenario_ids,
    )

    loaded_count = len(records)
    expected_count = len(scenario_ids)
    skipped_count = expected_count - loaded_count

    (
        completion_table,
        route_changed_table,
        route_changed_annotation_table,
    ) = build_pivot_tables(records)

    completion_pdf_path = results_dir / "heatmap_evacuation_completion_time.pdf"
    route_changed_pdf_path = results_dir / "heatmap_route_changed_vehicle_count.pdf"

    completion_csv_path = results_dir / "heatmap_evacuation_completion_time.csv"
    route_changed_csv_path = results_dir / "heatmap_route_changed_vehicle_count.csv"
    route_changed_annotation_csv_path = (
        results_dir / "heatmap_route_changed_vehicle_count_annotation.csv"
    )

    save_table_csv(completion_table, completion_csv_path)
    save_table_csv(route_changed_table, route_changed_csv_path)
    save_annotation_csv(
        route_changed_annotation_table,
        route_changed_annotation_csv_path,
    )

    save_heatmap(
        completion_table,
        completion_pdf_path,
        title="Evacuation Completion Time",
        colorbar_label="Completion Time [s]",
        annotation_table=None,
        annotation_fontsize=9,
    )

    save_heatmap(
        route_changed_table,
        route_changed_pdf_path,
        title="Route Changed Vehicle Count",
        colorbar_label="Route Changed Vehicle Count",
        annotation_table=route_changed_annotation_table,
        annotation_fontsize=8,
    )

    print(
        "Scenario IDs: "
        + ", ".join(f"scenario{scenario_id}" for scenario_id in scenario_ids)
    )
    print(f"Expected scenarios: {expected_count}")
    print(f"Loaded scenarios: {loaded_count}")
    print(f"Skipped scenarios: {skipped_count}")

    if completion_pdf_path.exists():
        print(f"Saved: {completion_pdf_path.name}")
    if route_changed_pdf_path.exists():
        print(f"Saved: {route_changed_pdf_path.name}")
    if completion_csv_path.exists():
        print(f"Saved: {completion_csv_path.name}")
    if route_changed_csv_path.exists():
        print(f"Saved: {route_changed_csv_path.name}")
    if route_changed_annotation_csv_path.exists():
        print(f"Saved: {route_changed_annotation_csv_path.name}")

if __name__ == "__main__":
    main()