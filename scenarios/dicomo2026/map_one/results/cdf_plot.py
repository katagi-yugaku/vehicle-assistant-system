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


def load_cdf_source(json_path: Path) -> dict:
    if not json_path.exists():
        raise FileNotFoundError(f"JSON file not found: {json_path}")

    with json_path.open("r", encoding="utf-8") as f:
        data = json.load(f)

    if "cdf_source" not in data:
        raise KeyError(f"'cdf_source' not found in {json_path}")

    cdf_source = data["cdf_source"]

    if not isinstance(cdf_source, dict):
        raise TypeError("'cdf_source' must be a dict")

    return cdf_source


def plot_cdfs(
    data_dict: dict[str, list[float]],
    save_path: Path,
    hidden_keys: set[str] | None = None,
) -> None:
    plt.figure(figsize=(10, 6))

    if hidden_keys is None:
        hidden_keys = set()

    style_map = {
        "0.1": {"color": "r", "linestyle": "-", "label": "early_rate=0.1"},
        "0.5": {"color": "blue", "linestyle": "-", "label": "early_rate=0.5"},
        "0.9": {"color": "g", "linestyle": "-", "label": "early_rate=0.9"},
        "nosystem": {"color": "blue", "linestyle": "--", "label": "no system"},
    }

    plot_order = ["0.1", "0.5", "0.9", "nosystem"]

    for key in plot_order:
        if key in hidden_keys:
            continue

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
    max_time = 2200
    min_time = 600
    ylim_min = 0.1
    plt.xlim(min_time, max_time)
    plt.ylim(ylim_min, 1.0)
    plt.xticks(np.arange(min_time, max_time+50, 200), fontsize=14, fontweight="semibold")
    plt.yticks(np.arange(ylim_min, 1.01, 0.1), fontsize=14, fontweight="semibold")

    # plt.xlabel("Arrival time [s]", fontsize=14, fontweight="semibold")
    # plt.ylabel("CDF", fontsize=14, fontweight="semibold")
    # plt.legend(fontsize=14)
    # plt.grid(True, alpha=0.3)

    plt.savefig(save_path, bbox_inches="tight")
    print(f"✅ Saved figure as: {save_path}")
    plt.close()


def main():
    if len(sys.argv) < 2:
        print("Usage: python3 cdf_plot.py <scenarioID> [hidden_key1 hidden_key2 ...]")
        print("Example: python3 cdf_plot.py scenario1")
        print("Example: python3 cdf_plot.py scenario1 0.5 0.1")
        print("Example: python3 cdf_plot.py 1 nosystem")
        sys.exit(1)

    scenario_id = normalize_scenario_arg(sys.argv[1])
    hidden_keys = set(arg.strip() for arg in sys.argv[2:])

    base_dir = Path(__file__).resolve().parent
    scenario_dir = base_dir / scenario_id
    json_path = scenario_dir / "output.json"
    output_path = scenario_dir / "lane_insight30_normalcy400700_result.pdf"

    cdf_source = load_cdf_source(json_path)
    plot_cdfs(cdf_source, output_path, hidden_keys)

    if hidden_keys:
        print(f"[INFO] Hidden keys: {sorted(hidden_keys)}")
    print(f"[INFO] CDF plot saved: {output_path}")


if __name__ == "__main__":
    main()