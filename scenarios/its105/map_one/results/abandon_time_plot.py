import json
import sys
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np


def normalize_scenario_arg(arg: str) -> str:
    arg = arg.strip()
    if arg.isdigit():
        return f"scenario{int(arg)}"
    if arg.startswith("scenario") and arg[len("scenario"):].isdigit():
        return f"scenario{int(arg[len('scenario'):])}"
    return arg


def load_all_abandon_time_events(json_path: Path) -> dict[str, list[float]]:
    if not json_path.exists():
        raise FileNotFoundError(f"JSON file not found: {json_path}")

    with json_path.open("r", encoding="utf-8") as f:
        data = json.load(f)

    if "all_abandon_time_events" not in data:
        raise KeyError(f"'all_abandon_time_events' not found in {json_path}")

    all_events = data["all_abandon_time_events"]

    if not isinstance(all_events, dict):
        raise TypeError("'all_abandon_time_events' must be a dict")

    normalized: dict[str, list[float]] = {}
    for key, values in all_events.items():
        if not isinstance(values, list):
            raise TypeError(f"'all_abandon_time_events[{key}]' must be a list")
        normalized[key] = [float(v) for v in values]

    return normalized


def build_bin_edges(xmin: int, xmax: int, bin_width: int) -> np.ndarray:
    if xmax <= xmin:
        raise ValueError("xmax must be greater than xmin")
    if bin_width <= 0:
        raise ValueError("bin_width must be positive")

    # 例: xmin=1200, xmax=1800, bin_width=50
    # -> [1200, 1250, ..., 1800]
    return np.arange(xmin, xmax + bin_width, bin_width)


def count_events_in_bins(
    values: list[float],
    bin_edges: np.ndarray,
    xmin: int,
    xmax: int,
) -> np.ndarray:
    arr = np.array(values, dtype=float)
    arr = arr[(arr >= xmin) & (arr <= xmax)]
    counts, _ = np.histogram(arr, bins=bin_edges)
    return counts


def plot_abandon_time_grouped_bars(
    data_dict: dict[str, list[float]],
    save_path: Path,
    bin_width: int,
    xmin: int,
    xmax: int,
) -> None:
    plot_order = ["0.1", "0.5", "0.9", "nosystem"]
    style_map = {
        "0.1": {"color": "r", "label": "early_rate=0.1"},
        "0.5": {"color": "blue", "label": "early_rate=0.5"},
        "0.9": {"color": "g", "label": "early_rate=0.9"},
        "nosystem": {"color": "black", "label": "no system"},
    }

    bin_edges = build_bin_edges(xmin, xmax, bin_width)
    bin_lefts = bin_edges[:-1]
    bin_rights = bin_edges[1:]
    bin_labels = [f"{int(l)}-{int(r)}" for l, r in zip(bin_lefts, bin_rights)]

    group_x = np.arange(len(bin_labels))
    n_series = len(plot_order)
    total_group_width = 0.8
    bar_width = total_group_width / n_series

    plt.figure(figsize=(14, 6))

    max_count = 0
    for i, key in enumerate(plot_order):
        values = data_dict.get(key, [])
        counts = count_events_in_bins(values, bin_edges, xmin, xmax)
        max_count = max(max_count, int(counts.max()) if len(counts) > 0 else 0)

        offset = (i - (n_series - 1) / 2) * bar_width
        plt.bar(
            group_x + offset,
            counts,
            width=bar_width,
            label=style_map[key]["label"],
            color=style_map[key]["color"],
            align="center",
        )

    plt.xticks(group_x, bin_labels, rotation=45, ha="right", fontsize=12, fontweight="semibold")
    plt.yticks(fontsize=12, fontweight="semibold")
    plt.xlabel("Abandon time interval [s]", fontsize=14, fontweight="semibold")
    plt.ylabel("Count", fontsize=14, fontweight="semibold")
    plt.ylim(0, max_count + max(1, int(max_count * 0.1)))
    plt.legend(fontsize=12)
    plt.grid(axis="y", alpha=0.3)
    plt.tight_layout()

    plt.savefig(save_path, bbox_inches="tight")
    print(f"✅ Saved figure as: {save_path}")
    plt.close()


def main():
    if len(sys.argv) != 5:
        print("Usage: python3 abandon_time_plot.py <scenarioID> <bin_width> <xmin> <xmax>")
        print("Example: python3 abandon_time_plot.py scenario1 50 1200 1800")
        print("Example: python3 abandon_time_plot.py 1 50 1200 1800")
        sys.exit(1)

    scenario_id = normalize_scenario_arg(sys.argv[1])
    bin_width = int(sys.argv[2])
    xmin = int(sys.argv[3])
    xmax = int(sys.argv[4])

    base_dir = Path(__file__).resolve().parent
    scenario_dir = base_dir / scenario_id
    json_path = scenario_dir / "output.json"
    output_path = scenario_dir / f"abandon_time_count_{xmin}_{xmax}_bw{bin_width}.pdf"

    all_abandon_time_events = load_all_abandon_time_events(json_path)
    plot_abandon_time_grouped_bars(
        all_abandon_time_events,
        output_path,
        bin_width,
        xmin,
        xmax,
    )

    print(f"[INFO] Abandon time grouped-bar plot saved: {output_path}")


if __name__ == "__main__":
    main()