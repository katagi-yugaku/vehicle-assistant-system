import json
import sys
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np


ORDERED_CONDITIONS = ["0.1", "0.5", "0.9", "nosystem"]

METRIC_CONFIG = {
    "walking_distance_by_pedestrianID_dict": {
        "json_key": "walking_distance_distribution",
        "xlabel": "Walking distance [m]",
        "ylabel": "Count",
        "title": "Walking distance distribution",
        "filename_prefix": "walking_distance_distribution",
    },
    "vehicle_abandant_time_by_pedestrianID_dict": {
        "json_key": "abandon_time_distribution",
        "xlabel": "Vehicle abandonment time [s]",
        "ylabel": "Count",
        "title": "Vehicle abandonment time distribution",
        "filename_prefix": "vehicle_abandant_time_distribution",
    },
    # 念のため typo なし版も受ける
    "vehicle_abandon_time_by_pedestrianID_dict": {
        "json_key": "abandon_time_distribution",
        "xlabel": "Vehicle abandonment time [s]",
        "ylabel": "Count",
        "title": "Vehicle abandonment time distribution",
        "filename_prefix": "vehicle_abandant_time_distribution",
    },
}


def normalize_scenario_arg(arg: str) -> str:
    arg = arg.strip()
    if arg.isdigit():
        return f"scenario{arg}"
    return arg


def normalize_metric_arg(arg: str) -> str:
    arg = arg.strip()
    if arg not in METRIC_CONFIG:
        valid = ", ".join(METRIC_CONFIG.keys())
        raise ValueError(
            f"Unsupported metric: {arg}\n"
            f"Choose one of: {valid}"
        )
    return arg


def load_distribution_block(json_path: Path, json_key: str) -> dict:
    if not json_path.exists():
        raise FileNotFoundError(f"JSON file not found: {json_path}")

    with json_path.open("r", encoding="utf-8") as f:
        data = json.load(f)

    if json_key not in data:
        raise KeyError(f"'{json_key}' not found in {json_path}")

    return data[json_key]


def is_single_distribution(dist: dict) -> bool:
    required_keys = {"bin_width", "bin_edges", "counts", "ratios"}
    return required_keys.issubset(dist.keys())


def normalize_distribution_block(dist_block: dict) -> dict[str, dict]:
    """
    output.json が旧形式:
        "walking_distance_distribution": { "bin_width": ..., ... }
    の場合にも、
    新形式:
        "walking_distance_distribution": {
            "0.1": {...}, "0.5": {...}, ...
        }
    の場合にも対応する。
    """
    if is_single_distribution(dist_block):
        return {"all": dist_block}

    normalized = {}
    for key in ORDERED_CONDITIONS:
        if key in dist_block:
            normalized[key] = dist_block[key]

    # ORDERED_CONDITIONS にないキーも一応拾う
    for key, value in dist_block.items():
        if key not in normalized:
            normalized[key] = value

    return normalized


def plot_single_distribution(
    dist: dict,
    xlabel: str,
    ylabel: str,
    title: str,
    out_path: Path,
) -> None:
    bin_width = dist["bin_width"]
    bin_edges = dist["bin_edges"]
    counts = dist["counts"]

    left_edges = np.array(bin_edges[:-1], dtype=float)

    plt.figure(figsize=(10, 6))
    plt.bar(left_edges, counts, width=bin_width, align="edge", edgecolor="black")
    plt.xlabel(xlabel)
    plt.ylabel(ylabel)
    plt.title(title)
    plt.grid(True, axis="y", alpha=0.3)
    plt.savefig(out_path, dpi=300, bbox_inches="tight")
    plt.close()
    print(f"saved: {out_path}")


def plot_multi_condition_distribution(
    dist_map: dict[str, dict],
    xlabel: str,
    ylabel: str,
    title: str,
    out_path: Path,
) -> None:
    available_keys = [key for key in ORDERED_CONDITIONS if key in dist_map]
    if not available_keys:
        available_keys = list(dist_map.keys())

    first_dist = dist_map[available_keys[0]]
    bin_width = first_dist["bin_width"]
    bin_edges = first_dist["bin_edges"]
    centers = np.array(bin_edges[:-1], dtype=float) + bin_width / 2

    n_series = len(available_keys)
    bar_width = bin_width / max(n_series + 1, 2)

    plt.figure(figsize=(12, 6))

    for i, key in enumerate(available_keys):
        counts = dist_map[key]["counts"]
        offset = (i - (n_series - 1) / 2) * bar_width
        x = centers + offset
        plt.bar(x, counts, width=bar_width, label=key, edgecolor="black", alpha=0.85)

    plt.xlabel(xlabel)
    plt.ylabel(ylabel)
    plt.title(title)
    plt.legend()
    plt.grid(True, axis="y", alpha=0.3)
    plt.savefig(out_path, dpi=300, bbox_inches="tight")
    plt.close()
    print(f"saved: {out_path}")


def plot_distribution(
    dist_block: dict,
    scenario_name: str,
    metric_arg: str,
    base_dir: Path,
) -> None:
    config = METRIC_CONFIG[metric_arg]
    dist_map = normalize_distribution_block(dist_block)

    out_path = base_dir / f"{config['filename_prefix']}_{scenario_name}.pdf"
    title = f"{config['title']} ({scenario_name})"

    if list(dist_map.keys()) == ["all"]:
        plot_single_distribution(
            dist=dist_map["all"],
            xlabel=config["xlabel"],
            ylabel=config["ylabel"],
            title=title,
            out_path=out_path,
        )
    else:
        plot_multi_condition_distribution(
            dist_map=dist_map,
            xlabel=config["xlabel"],
            ylabel=config["ylabel"],
            title=title,
            out_path=out_path,
        )


if __name__ == "__main__":
    if len(sys.argv) < 3:
        print(
            "Usage:\n"
            "  python3 distribution_plot.py <scenario_id> <metric_key>\n\n"
            "Example:\n"
            "  python3 distribution_plot.py 1 walking_distance_by_pedestrianID_dict\n"
            "  python3 distribution_plot.py 1 vehicle_abandant_time_by_pedestrianID_dict"
        )
        sys.exit(1)

    scenario_name = normalize_scenario_arg(sys.argv[1])
    metric_arg = normalize_metric_arg(sys.argv[2])

    config = METRIC_CONFIG[metric_arg]

    base_dir = Path(__file__).resolve().parent
    json_path = base_dir / scenario_name / "output.json"

    dist_block = load_distribution_block(json_path, config["json_key"])
    plot_distribution(dist_block, scenario_name, metric_arg, base_dir)