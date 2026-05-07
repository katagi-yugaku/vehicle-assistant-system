import json
import sys
from pathlib import Path
import matplotlib.pyplot as plt
import numpy as np


EARLY_RATE_KEYS = {"0.1", "0.5", "0.9", "nosystem"}


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
    scenario_to_rate_values: dict[str, dict[str, list[float]]],
    early_rates: list[str],
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

    for scenario_id, rate_to_values in scenario_to_rate_values.items():

        for early_rate in early_rates:
            # system は実線，nosystem は点線
            linestyle = "--" if early_rate == "nosystem" else "-"

            values = rate_to_values[early_rate]

            value = np.array(values, dtype=float)
            if len(value) == 0:
                print(f"[WARN] {scenario_id} の early_rate={early_rate} は空です。スキップします。")
                continue

            value = np.sort(value)
            cdf = np.arange(1, len(value) + 1) / len(value)

            base_style = style_map.get(
                early_rate,
                {"color": "gray", "label": f"early_rate={early_rate}"}
            )

            plt.plot(
                value,
                cdf,
                label=f"{scenario_id} / {base_style['label']}",
                color=base_style["color"],
                linestyle=linestyle,
                linewidth=2,
            )

    max_time = 2200
    min_time = 600
    y_min = 0.1

    plt.xlim(min_time, max_time)
    plt.ylim(y_min, 1.0)
    plt.xticks(np.arange(min_time, max_time + 50, 200), fontsize=12, fontweight="semibold")
    plt.yticks(np.arange(y_min, 1.01, 0.1), fontsize=12, fontweight="semibold")

    # plt.xlabel("Arrival time [s]", fontsize=14, fontweight="semibold")
    # plt.ylabel("CDF", fontsize=14, fontweight="semibold")

    early_rate_part = ", ".join(early_rates)

    plt.savefig(save_path, bbox_inches="tight")
    print(f"✅ Saved figure as: {save_path}")
    plt.close()


def main():
    if len(sys.argv) < 3:
        print("Usage: python3 compare_cdf_plot.py <scenarioID1> <scenarioID2> ... <early_rate1> <early_rate2> ...")
        print("Example: python3 compare_cdf_plot.py scenario1 scenario2 0.5 nosystem")
        print("Example: python3 compare_cdf_plot.py 1 2 3 0.1 0.5")
        sys.exit(1)

    raw_args = [arg.strip() for arg in sys.argv[1:]]

    early_rates = [arg for arg in raw_args if arg in EARLY_RATE_KEYS]
    scenario_args = [arg for arg in raw_args if arg not in EARLY_RATE_KEYS]

    if not scenario_args:
        raise ValueError("scenarioID が指定されていません。")

    if not early_rates:
        raise ValueError(
            f"early_rate が指定されていません。指定可能: {sorted(EARLY_RATE_KEYS)}"
        )

    scenario_ids = [normalize_scenario_arg(arg) for arg in scenario_args]

    base_dir = Path(__file__).resolve().parent
    output_dir = base_dir / "compared"
    output_dir.mkdir(parents=True, exist_ok=True)

    scenario_to_rate_values = {}

    for scenario_id in scenario_ids:
        scenario_dir = base_dir / scenario_id
        json_path = scenario_dir / "output.json"

        cdf_source = load_cdf_source(json_path)

        scenario_to_rate_values[scenario_id] = {}

        for early_rate in early_rates:
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

            scenario_to_rate_values[scenario_id][early_rate] = values

    scenario_part = "_vs_".join(scenario_ids)
    early_rate_part = "_".join(early_rates)

    output_path = output_dir / f"cdf_compare_{scenario_part}_early_rate_{early_rate_part}.pdf"

    plot_compare_cdfs(scenario_to_rate_values, early_rates, output_path)

    print(f"[INFO] CDF comparison plot saved: {output_path}")


if __name__ == "__main__":
    main()