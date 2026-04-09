import json
import sys
from pathlib import Path

import numpy as np
import matplotlib.pyplot as plt


def normalize_scenario_arg(arg: str) -> str:
    """
    '1' が渡されても 'scenario1' に直す
    'scenario1' が渡されたらそのまま使う
    """
    arg = arg.strip()
    if arg.isdigit():
        return f"scenario{arg}"
    return arg


def load_cdf_source(json_path: Path) -> dict[str, list[float]]:
    """
    JSON から cdf_source を取り出す
    """
    if not json_path.exists():
        raise FileNotFoundError(f"JSON file not found: {json_path}")

    with json_path.open("r", encoding="utf-8") as f:
        obj = json.load(f)

    if "cdf_source" not in obj:
        raise KeyError("JSON に 'cdf_source' が存在しません。")

    return obj["cdf_source"]


def plot_cdfs(data_dict: dict[str, list[float]], save_path: Path):
    plt.figure(figsize=(10, 6))

    style_map = {
        "0.1": {"color": "r", "linestyle": "-", "label": "early_rate=0.1"},
        "0.5": {"color": "blue", "linestyle": "-", "label": "early_rate=0.5"},
        "0.9": {"color": "g", "linestyle": "-", "label": "early_rate=0.9"},
        "nosystem": {"color": "blue", "linestyle": "--", "label": "no system"},
    }

    plot_order = ["0.1", "0.5", "0.9", "nosystem"]

    for key in plot_order:
        if key not in data_dict:
            continue

        value = np.array(data_dict[key], dtype=float)
        if len(value) == 0:
            continue

        value = np.sort(value)
        cdf = np.arange(1, len(value) + 1) / len(value)

        style = style_map.get(
            key,
            {"color": "gray", "linestyle": "-", "label": key}
        )

        plt.plot(
            value,
            cdf,
            label=style["label"],
            color=style["color"],
            linestyle=style["linestyle"],
            linewidth=2,
        )
    start_time = 400
    end_time = 1800
    plt.xlim(start_time, end_time)
    plt.ylim(0.1, 1.0)
    plt.xticks(
        ticks=np.arange(start_time, end_time+51, 100),
        fontsize=12,
        fontweight="semibold"
    )
    plt.yticks(
        ticks=np.arange(0.1, 1.01, 0.1),
        fontsize=14,
        fontweight="semibold"
    )

    # plt.xlabel("Arrival time [s]", fontsize=20, fontweight="semibold")
    # plt.ylabel("CDF", fontsize=20, fontweight="semibold")
    # plt.legend(fontsize=14)
    # plt.grid(True, alpha=0.3)

    plt.savefig(save_path, bbox_inches="tight")
    print(f"✅ Saved figure as: {save_path}")

    plt.show()


if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("Usage: python3 cdf_plot.py <scenario_id|scenario_name> [label_suffix]")
        print("Example: python3 cdf_plot.py scenario1 30")
        sys.exit(1)

    scenario_name = normalize_scenario_arg(sys.argv[1])
    label_suffix = sys.argv[2] if len(sys.argv) >= 3 else ""

    base_dir = Path(__file__).resolve().parent
    json_path = base_dir / scenario_name / "output.json"
    cdf_source = load_cdf_source(json_path)

    suffix_part = f"_{label_suffix}" if label_suffix else ""
    save_path = base_dir / scenario_name / f"cdf_compare_{scenario_name}{suffix_part}.pdf"

    plot_cdfs(cdf_source, save_path)