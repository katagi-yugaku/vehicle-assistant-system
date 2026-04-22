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


def get_base_style(early_rate: str) -> dict[str, str]:
    style_map = {
        "0.1": {"cmap": "Reds", "title": "early_rate=0.1"},
        "0.5": {"cmap": "Blues", "title": "early_rate=0.5"},
        "0.9": {"cmap": "Greens", "title": "early_rate=0.9"},
        "nosystem": {"cmap": "Greys", "title": "no system"},
    }
    return style_map.get(
        early_rate,
        {"cmap": "Greys", "title": f"early_rate={early_rate}"}
    )


def make_same_family_colors(early_rate: str, n: int) -> list:
    base_style = get_base_style(early_rate)
    cmap = plt.get_cmap(base_style["cmap"])

    if n <= 1:
        return [cmap(0.7)]

    # 薄すぎず濃すぎない範囲で同系色を並べる
    positions = np.linspace(0.45, 0.85, n)
    return [cmap(p) for p in positions]


def plot_compare_abandon_time_grouped_bars(
    scenario_to_values: dict[str, list[float]],
    early_rate: str,
    save_path: Path,
    bin_width: int,
    xmin: int,
    xmax: int,
) -> None:
    scenario_ids = list(scenario_to_values.keys())

    if len(scenario_ids) == 0:
        raise ValueError("No scenario data to plot")

    base_style = get_base_style(early_rate)
    colors = make_same_family_colors(early_rate, len(scenario_ids))

    bin_edges = build_bin_edges(xmin, xmax, bin_width)
    bin_lefts = bin_edges[:-1]
    bin_rights = bin_edges[1:]
    bin_labels = [f"{int(l)}-{int(r)}" for l, r in zip(bin_lefts, bin_rights)]

    group_x = np.arange(len(bin_labels))
    n_series = len(scenario_ids)
    total_group_width = 0.8
    bar_width = total_group_width / n_series

    plt.figure(figsize=(14, 6))

    max_count = 0
    for i, scenario_id in enumerate(scenario_ids):
        values = scenario_to_values[scenario_id]
        counts = count_events_in_bins(values, bin_edges, xmin, xmax)
        if len(counts) > 0:
            max_count = max(max_count, int(counts.max()))

        offset = (i - (n_series - 1) / 2) * bar_width
        plt.bar(
            group_x + offset,
            counts,
            width=bar_width,
            label=scenario_id,
            color=colors[i],
            align="center",
        )

    plt.xticks(
        group_x,
        bin_labels,
        rotation=45,
        ha="right",
        fontsize=12,
        fontweight="semibold",
    )
    plt.yticks(fontsize=12, fontweight="semibold")
    plt.xlabel("Abandon time interval [s]", fontsize=14, fontweight="semibold")
    plt.ylabel("Count", fontsize=14, fontweight="semibold")
    plt.title(f"Abandon time comparison ({base_style['title']})", fontsize=14, fontweight="semibold")

    upper = max_count + max(1, int(max_count * 0.1)) if max_count > 0 else 1
    plt.ylim(0, upper)

    plt.legend(fontsize=12)
    plt.grid(axis="y", alpha=0.3)
    plt.tight_layout()

    plt.savefig(save_path, bbox_inches="tight")
    print(f"✅ Saved figure as: {save_path}")
    plt.close()


def is_int_string(s: str) -> bool:
    try:
        int(s)
        return True
    except ValueError:
        return False


def main():
    raw_args = sys.argv[1:]

    if len(raw_args) < 2:
        print("Usage:")
        print("  python3 0000.py <scenarioID1> <scenarioID2> ... <early_rate>")
        print("  python3 0000.py <scenarioID1> <scenarioID2> ... <early_rate> <bin_width> <xmin> <xmax>")
        print("Example:")
        print("  python3 0000.py scenario1 scenario2 0.5")
        print("  python3 0000.py scenario1 scenario2 scenario3 0.5 50 1200 1800")
        sys.exit(1)

    # 後ろ3つが整数なら bin設定ありとみなす
    if len(raw_args) >= 5 and all(is_int_string(x) for x in raw_args[-3:]):
        bin_width = int(raw_args[-3])
        xmin = int(raw_args[-2])
        xmax = int(raw_args[-1])
        early_rate = raw_args[-4].strip()
        scenario_args = raw_args[:-4]
    else:
        bin_width = 50
        xmin = 800
        xmax = 1800
        early_rate = raw_args[-1].strip()
        scenario_args = raw_args[:-1]

    if len(scenario_args) < 1:
        raise ValueError("At least one scenarioID is required")

    scenario_ids = [normalize_scenario_arg(arg) for arg in scenario_args]

    base_dir = Path(__file__).resolve().parent
    output_dir = base_dir / "compared"
    output_dir.mkdir(parents=True, exist_ok=True)

    scenario_to_values: dict[str, list[float]] = {}

    for scenario_id in scenario_ids:
        scenario_dir = base_dir / scenario_id
        json_path = scenario_dir / "output.json"

        all_abandon_time_events = load_all_abandon_time_events(json_path)

        if early_rate not in all_abandon_time_events:
            raise KeyError(
                f"early_rate='{early_rate}' not found in all_abandon_time_events of {json_path}. "
                f"available keys = {list(all_abandon_time_events.keys())}"
            )

        values = all_abandon_time_events[early_rate]

        if not isinstance(values, list):
            raise TypeError(
                f"all_abandon_time_events['{early_rate}'] in {json_path} must be a list, "
                f"but got {type(values).__name__}"
            )

        scenario_to_values[scenario_id] = values

    scenario_part = "_vs_".join(scenario_ids)
    output_path = output_dir / (
        f"abandon_time_compare_{scenario_part}_early_rate_{early_rate}"
        f"_bw{bin_width}_{xmin}_{xmax}.pdf"
    )

    plot_compare_abandon_time_grouped_bars(
        scenario_to_values=scenario_to_values,
        early_rate=early_rate,
        save_path=output_path,
        bin_width=bin_width,
        xmin=xmin,
        xmax=xmax,
    )

    print(f"[INFO] Abandon time comparison plot saved: {output_path}")


if __name__ == "__main__":
    main()