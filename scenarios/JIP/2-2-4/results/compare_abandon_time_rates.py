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

    return {
        str(key): [float(v) for v in values]
        for key, values in all_events.items()
    }


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


def get_base_style(rate: str) -> dict[str, str]:
    style_map = {
        "0.1": {"cmap": "Reds", "label": "early_rate=0.1"},
        "0.5": {"cmap": "Blues", "label": "early_rate=0.5"},
        "0.9": {"cmap": "Greens", "label": "early_rate=0.9"},
        "nosystem": {"cmap": "Greys", "label": "no system"},
    }

    return style_map.get(
        rate,
        {"cmap": "Greys", "label": f"early_rate={rate}"},
    )


def get_rate_color(rate: str):
    """
    rate ごとの代表色を返す。

    nosystem は指定色 #6F94E6 に固定する。
    """
    if rate == "nosystem":
        return "#6F94E6"

    base_style = get_base_style(rate)
    cmap = plt.get_cmap(base_style["cmap"])
    return cmap(0.7)


def get_rate_label(rate: str, transparent_rates: set[str]) -> str:
    label = get_base_style(rate)["label"]

    if rate in transparent_rates:
        return f"{label} (transparent)"

    return label


def plot_compare_rates_grouped_bars(
    rate_to_values: dict[str, list[float]],
    scenario_id: str,
    save_path: Path,
    bin_width: int,
    xmin: int,
    xmax: int,
    transparent_rates: set[str],
) -> None:
    rate_ids = list(rate_to_values.keys())

    if len(rate_ids) == 0:
        raise ValueError("No early_rate data to plot")

    bin_edges = build_bin_edges(xmin, xmax, bin_width)
    bin_lefts = bin_edges[:-1]

    # 例: 700-, 800-, 900-
    bin_labels = [f"{int(l)}-" for l in bin_lefts]

    group_x = np.arange(len(bin_labels))
    n_series = len(rate_ids)

    total_group_width = 0.8
    bar_width = total_group_width / n_series

    plt.figure(figsize=(14, 6))

    colors = {
        rate_id: get_rate_color(rate_id)
        for rate_id in rate_ids
    }

    max_count = 0

    for i, rate_id in enumerate(rate_ids):
        values = rate_to_values[rate_id]
        counts = count_events_in_bins(values, bin_edges, xmin, xmax)

        if len(counts) > 0:
            max_count = max(max_count, int(counts.max()))

        offset = (i - (n_series - 1) / 2) * bar_width

        alpha = 0.25 if rate_id in transparent_rates else 1.0
        zorder = 1 if rate_id in transparent_rates else 3

        plt.bar(
            group_x + offset,
            counts,
            width=bar_width,
            label=get_rate_label(rate_id, transparent_rates),
            color=colors[rate_id],
            alpha=alpha,
            align="center",
            zorder=zorder,
        )

    plt.xticks(
        group_x,
        bin_labels,
        rotation=45,
        ha="right",
        fontsize=17,
        fontweight="semibold",
    )
    plt.yticks(fontsize=18, fontweight="semibold")

    upper = max_count + max(1, int(max_count * 0.1)) if max_count > 0 else 1
    plt.ylim(0, upper)

    plt.grid(axis="y", alpha=0.3, zorder=0)
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


def split_transparent_args(raw_args: list[str]) -> tuple[list[str], list[str]]:
    """
    --transparent 以降を透過対象 rate として分離する。

    例:
      scenario1 0.5 nosystem 100 700 1800 --transparent 0.1 0.9

    Returns:
      normal_args = ["scenario1", "0.5", "nosystem", "100", "700", "1800"]
      transparent_rates = ["0.1", "0.9"]
    """
    if "--transparent" not in raw_args:
        return raw_args, []

    idx = raw_args.index("--transparent")
    normal_args = raw_args[:idx]
    transparent_rates = raw_args[idx + 1:]

    if len(transparent_rates) == 0:
        raise ValueError("--transparent を指定した場合、透過対象の rate を1つ以上指定してください")

    return normal_args, transparent_rates


def main():
    raw_args = sys.argv[1:]
    normal_args, transparent_rate_args = split_transparent_args(raw_args)

    if len(normal_args) < 3:
        print("Usage:")
        print("  python3 compare_abandon_time_rates.py <scenarioID> <rate1> <rate2> [rate3 ...]")
        print("  python3 compare_abandon_time_rates.py <scenarioID> <rate1> <rate2> [rate3 ...] <bin_width> <xmin> <xmax>")
        print("  python3 compare_abandon_time_rates.py <scenarioID> <rate1> <rate2> <bin_width> <xmin> <xmax> --transparent <rateX> <rateY>")
        print("Example:")
        print("  python3 compare_abandon_time_rates.py scenario1 0.5 nosystem")
        print("  python3 compare_abandon_time_rates.py scenario1 0.5 nosystem 100 700 1800 --transparent 0.1 0.9")
        sys.exit(1)

    # 後ろ3つが整数なら bin 設定あり
    if len(normal_args) >= 6 and all(is_int_string(x) for x in normal_args[-3:]):
        bin_width = int(normal_args[-3])
        xmin = int(normal_args[-2])
        xmax = int(normal_args[-1])
        scenario_arg = normal_args[0]
        main_rate_args = normal_args[1:-3]
    else:
        bin_width = 50
        xmin = 800
        xmax = 1800
        scenario_arg = normal_args[0]
        main_rate_args = normal_args[1:]

    if len(main_rate_args) < 2:
        raise ValueError("At least two main rate keys are required")

    # 表示順:
    # まず目立たせたい main rates、その後に透明表示したい rates
    rate_args = []
    for rate in main_rate_args + transparent_rate_args:
        rate = rate.strip()
        if rate not in rate_args:
            rate_args.append(rate)

    transparent_rates = set(rate.strip() for rate in transparent_rate_args)

    scenario_id = normalize_scenario_arg(scenario_arg)

    base_dir = Path(__file__).resolve().parent
    scenario_dir = base_dir / scenario_id
    json_path = scenario_dir / "output.json"

    all_abandon_time_events = load_all_abandon_time_events(json_path)

    rate_to_values: dict[str, list[float]] = {}

    for rate in rate_args:
        if rate not in all_abandon_time_events:
            raise KeyError(
                f"early_rate='{rate}' not found in all_abandon_time_events of {json_path}. "
                f"available keys = {list(all_abandon_time_events.keys())}"
            )

        rate_to_values[rate] = all_abandon_time_events[rate]

    output_dir = base_dir / "compared"
    output_dir.mkdir(parents=True, exist_ok=True)

    main_rate_part = "_vs_".join(main_rate_args)

    transparent_part = ""
    if transparent_rate_args:
        transparent_part = "_transparent_" + "_".join(transparent_rate_args)

    output_path = output_dir / (
        f"abandon_time_compare_{scenario_id}_{main_rate_part}"
        f"{transparent_part}_bw{bin_width}_{xmin}_{xmax}.pdf"
    )

    plot_compare_rates_grouped_bars(
        rate_to_values=rate_to_values,
        scenario_id=scenario_id,
        save_path=output_path,
        bin_width=bin_width,
        xmin=xmin,
        xmax=xmax,
        transparent_rates=transparent_rates,
    )

    print(f"[INFO] Abandon time comparison plot saved: {output_path}")


if __name__ == "__main__":
    main()
