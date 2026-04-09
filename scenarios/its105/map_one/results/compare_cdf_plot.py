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


def plot_compare_cdfs(
    scenario_to_values: dict[str, list[float]],
    early_rate: str,
    save_path: Path,
) -> None:
    plt.figure(figsize=(10, 6))

    # 色は early_rate で決める
    style_map = {
        "0.1": {"color": "r", "label": "early_rate=0.1"},
        "0.5": {"color": "blue", "label": "early_rate=0.5"},
        "0.9": {"color": "g", "label": "early_rate=0.9"},
        "nosystem": {"color": "blue", "label": "no system"},
    }

    # 線種は scenario の順番で決める
    linestyle_list = ["-", "--", "-.", ":", (0, (3, 1, 1, 1)), (0, (5, 2))]

    base_style = style_map.get(
        early_rate,
        {"color": "gray", "label": f"early_rate={early_rate}"}
    )

    for i, (scenario_id, values) in enumerate(scenario_to_values.items()):
        value = np.array(values, dtype=float)
        if len(value) == 0:
            print(f"[WARN] {scenario_id} の early_rate={early_rate} は空です。スキップします。")
            continue

        value = np.sort(value)
        cdf = np.arange(1, len(value) + 1) / len(value)

        linestyle = linestyle_list[i % len(linestyle_list)]

        plt.plot(
            value,
            cdf,
            label=scenario_id,
            color=base_style["color"],
            linestyle=linestyle,
            linewidth=2,
        )

    max_time = 2200
    min_time = 0
    y_min = 0.0
    plt.xlim(min_time, max_time)
    plt.ylim(y_min, 1.0)
    plt.xticks(np.arange(min_time, max_time + 50, 200), fontsize=12, fontweight="semibold")
    plt.yticks(np.arange(y_min, 1.01, 0.1), fontsize=12, fontweight="semibold")

    plt.xlabel("Arrival time [s]", fontsize=14, fontweight="semibold")
    plt.ylabel("CDF", fontsize=14, fontweight="semibold")
    plt.title(f"CDF comparison ({base_style['label']})", fontsize=14, fontweight="semibold")
    plt.legend(fontsize=12)

    plt.savefig(save_path, bbox_inches="tight")
    print(f"✅ Saved figure as: {save_path}")
    plt.close()


def main():
    if len(sys.argv) < 3:
        print("Usage: python3 compare_cdf_plot.py <scenarioID1> <scenarioID2> ... <early_rate>")
        print("Example: python3 compare_cdf_plot.py scenario1 scenario2 0.1")
        print("Example: python3 compare_cdf_plot.py 1 2 3 0.5")
        sys.exit(1)

    raw_args = sys.argv[1:]
    early_rate = raw_args[-1].strip()
    scenario_ids = [normalize_scenario_arg(arg) for arg in raw_args[:-1]]

    base_dir = Path(__file__).resolve().parent
    output_dir = base_dir / "compared"
    output_dir.mkdir(parents=True, exist_ok=True)

    scenario_to_values = {}

    for scenario_id in scenario_ids:
        scenario_dir = base_dir / scenario_id
        json_path = scenario_dir / "output.json"

        cdf_source = load_cdf_source(json_path)

        if early_rate not in cdf_source:
            raise KeyError(
                f"early_rate='{early_rate}' not found in cdf_source of {json_path}. "
                f"available keys = {list(cdf_source.keys())}"
            )

        values = cdf_source[early_rate]

        if not isinstance(values, list):
            raise TypeError(
                f"cdf_source['{early_rate}'] in {json_path} must be a list, "
                f"but got {type(values).__name__}"
            )

        scenario_to_values[scenario_id] = values

    scenario_part = "_vs_".join(scenario_ids)
    output_path = output_dir / f"cdf_compare_{scenario_part}_early_rate_{early_rate}.pdf"

    plot_compare_cdfs(scenario_to_values, early_rate, output_path)

    print(f"[INFO] CDF comparison plot saved: {output_path}")


if __name__ == "__main__":
    main()