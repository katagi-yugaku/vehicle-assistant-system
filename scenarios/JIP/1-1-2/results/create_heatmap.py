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


def collect_records(results_dir: Path, config_dir: Path) -> list[dict[str, Any]]:
    records: list[dict[str, Any]] = []

    scenario_dirs = [results_dir / f"scenario{i}" for i in range(1, 31)]

    for scenario_dir in scenario_dirs:
        if not scenario_dir.exists():
            warn_and_skip(f"Skip {scenario_dir.name}: directory not found")
            continue

        if not scenario_dir.is_dir():
            warn_and_skip(f"Skip {scenario_dir.name}: not a directory")
            continue

        try:
            extract_scenario_number_from_dir(scenario_dir)
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
            route_changed_vehicle_count = float(
                condition_data["count_averages"]["route_changed_vehicle_count"]
            )
        except KeyError as e:
            warn_and_skip(f"Skip {scenario_dir.name}: missing count_averages key: {e}")
            continue
        except Exception as e:
            warn_and_skip(
                f"Skip {scenario_dir.name}: invalid route_changed_vehicle_count: {e}"
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


def build_pivot_tables(records: list[dict[str, Any]]) -> tuple[HeatmapTable, HeatmapTable]:
    completion_table = build_table(records, "evacuation_completion_time")
    route_changed_table = build_table(records, "route_changed_vehicle_count")
    return completion_table, route_changed_table


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


def save_heatmap(
    table: HeatmapTable,
    output_path: Path,
    title: str,
    colorbar_label: str,
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

            ax.text(
                j,
                i,
                f"{value:.2f}",
                ha="center",
                va="center",
                fontsize=9,
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

    return parser.parse_args()


def main() -> None:
    args = parse_args()

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

    records = collect_records(results_dir, config_dir)

    loaded_count = len(records)
    skipped_count = 30 - loaded_count

    completion_table, route_changed_table = build_pivot_tables(records)

    completion_pdf_path = results_dir / "heatmap_evacuation_completion_time.pdf"
    route_changed_pdf_path = results_dir / "heatmap_route_changed_vehicle_count.pdf"

    completion_csv_path = results_dir / "heatmap_evacuation_completion_time.csv"
    route_changed_csv_path = results_dir / "heatmap_route_changed_vehicle_count.csv"

    save_table_csv(completion_table, completion_csv_path)
    save_table_csv(route_changed_table, route_changed_csv_path)

    save_heatmap(
        completion_table,
        completion_pdf_path,
        title="Evacuation Completion Time",
        colorbar_label="Completion Time [s]",
    )

    save_heatmap(
        route_changed_table,
        route_changed_pdf_path,
        title="Route Changed Vehicle Count",
        colorbar_label="Route Changed Vehicle Count",
    )

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


if __name__ == "__main__":
    main()