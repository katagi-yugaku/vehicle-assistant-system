import json
import sys
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np


DEFAULT_CONDITION_KEY = "1.0"


def normalize_scenario_arg(arg: str) -> str:
    """
    scenario 引数を正規化する。

    例:
        "21"         -> "scenario21"
        "scenario21" -> "scenario21"
    """
    arg = arg.strip()

    if arg.isdigit():
        return f"scenario{int(arg)}"

    if arg.startswith("scenario") and arg[len("scenario"):].isdigit():
        return f"scenario{int(arg[len('scenario'):])}"

    return arg


def load_arrival_time_list(
    json_path: Path,
    condition_key: str = DEFAULT_CONDITION_KEY,
) -> list[float]:
    """
    output.json から arrival_time_list を取得する。

    主な想定形式:
    {
      "1.0": {
        "arrival_time": {
          "arrival_time_list": [...]
        }
      },
      "nosystem": {
        "arrival_time": {
          "arrival_time_list": [...]
        }
      }
    }

    condition_key を指定しない場合は "1.0" を使う。
    """

    if not json_path.exists():
        raise FileNotFoundError(f"JSON file not found: {json_path}")

    with json_path.open("r", encoding="utf-8") as f:
        data = json.load(f)

    if not isinstance(data, dict):
        raise TypeError(f"Root JSON object must be a dict: {json_path}")

    # 形式1:
    # {
    #   "arrival_time": {
    #     "arrival_time_list": [...]
    #   }
    # }
    if "arrival_time" in data:
        arrival_time = data["arrival_time"]

        if not isinstance(arrival_time, dict):
            raise TypeError(f"'arrival_time' must be a dict in {json_path}")

        values = arrival_time.get("arrival_time_list")

        if values is None:
            raise KeyError(f"'arrival_time_list' not found in {json_path}")

        if not isinstance(values, list):
            raise TypeError(
                f"'arrival_time_list' in {json_path} must be a list, "
                f"but got {type(values).__name__}"
            )

        return values

    # 形式2:
    # {
    #   "1.0": {
    #     "arrival_time": {
    #       "arrival_time_list": [...]
    #     }
    #   },
    #   "nosystem": {
    #     "arrival_time": {
    #       "arrival_time_list": [...]
    #     }
    #   }
    # }
    if condition_key in data:
        condition_data = data[condition_key]

        if not isinstance(condition_data, dict):
            raise TypeError(
                f"data['{condition_key}'] must be a dict in {json_path}"
            )

        arrival_time = condition_data.get("arrival_time")

        if not isinstance(arrival_time, dict):
            raise KeyError(
                f"'arrival_time' not found under condition_key='{condition_key}' "
                f"in {json_path}"
            )

        values = arrival_time.get("arrival_time_list")

        if values is None:
            raise KeyError(
                f"'arrival_time_list' not found under condition_key='{condition_key}' "
                f"in {json_path}"
            )

        if not isinstance(values, list):
            raise TypeError(
                f"arrival_time_list under condition_key='{condition_key}' "
                f"in {json_path} must be a list, "
                f"but got {type(values).__name__}"
            )

        print(f"[INFO] {json_path}: using condition key '{condition_key}'")
        return values

    # 指定された condition_key が存在しない場合、候補を表示する
    candidate_keys = []

    for key, value in data.items():
        if not isinstance(value, dict):
            continue

        arrival_time = value.get("arrival_time")
        if not isinstance(arrival_time, dict):
            continue

        if "arrival_time_list" in arrival_time:
            candidate_keys.append(key)

    raise KeyError(
        f"condition_key='{condition_key}' not found in {json_path}. "
        f"available arrival_time_list keys = {candidate_keys}"
    )


def plot_compare_arrival_time_cdfs(
    scenario_to_values: dict[str, list[float]],
    save_path: Path,
    no_legend: bool = False,
) -> None:
    """
    arrival_time_list を元に CDF をプロットして PDF 保存する。
    フォントサイズや軸設定は、既存の実装をベースにしている。
    """

    plt.figure(figsize=(10, 6))

    # scenario_color_map = {
    #     "scenario21": "r",
    #     "scenario43": "blue",
    #     "scenario57": "g",
    #     "scenario2": "r",
    #     "scenario3": "blue",
    #     "scenario4": "g",
    #     "scenario5": "orange",
    #     "scenario27": "r",
    #     "scenario29": "blue",
    # }

    fallback_colors = [
        "r",
        "blue",
        "g",
        "orange",
        "magenta",
        "cyan",
        "black",
    ]

    plotted_count = 0

    for index, (scenario_id, values) in enumerate(scenario_to_values.items()):
        value = np.array(values, dtype=float)

        if len(value) == 0:
            print(f"[WARN] {scenario_id} の arrival_time_list は空です。スキップします。")
            continue

        value = np.sort(value)
        cdf = np.arange(1, len(value) + 1) / len(value)

        # color = scenario_color_map.get(
        #     scenario_id,
        #     fallback_colors[index % len(fallback_colors)],
        # )

        plt.plot(
            value,
            cdf,
            label=scenario_id,
            # color=color,
            linestyle="-",
            linewidth=2,
        )

        plotted_count += 1

    if plotted_count == 0:
        raise ValueError("プロット対象の arrival_time_list がありません。")

    max_time = 2300
    min_time = 100
    y_min = 0.0

    plt.xlim(min_time, max_time)
    plt.ylim(y_min, 1.0)

    plt.xticks(
        np.arange(min_time, max_time + 50, 200),
        fontsize=14,
        fontweight="semibold",
    )
    plt.yticks(
        np.arange(y_min, 1.01, 0.1),
        fontsize=14,
        fontweight="semibold",
    )

    if not no_legend:
        plt.legend(fontsize=12)

    plt.savefig(save_path, bbox_inches="tight")
    print(f"✅ Saved figure as: {save_path}")

    plt.close()


def parse_args(raw_args: list[str]) -> tuple[list[str], str, bool]:
    """
    コマンドライン引数を解析する。

    対応:
        python3 compare_cdf_plot.py scenario21 scenario43 scenario57
        python3 compare_cdf_plot.py scenario21 scenario43 scenario57 --condition_key nosystem
        python3 compare_cdf_plot.py scenario21 scenario43 scenario57 --condition-key nosystem
        python3 compare_cdf_plot.py scenario21 scenario43 scenario57 --no_legend
    """

    no_legend = "--no_legend" in raw_args
    args = [arg for arg in raw_args if arg != "--no_legend"]

    condition_key = DEFAULT_CONDITION_KEY

    for option_name in ("--condition_key", "--condition-key"):
        if option_name in args:
            idx = args.index(option_name)

            if idx + 1 >= len(args):
                raise ValueError(
                    f"{option_name} の後に条件キーを指定してください。 "
                    f"例: {option_name} 1.0"
                )

            condition_key = args[idx + 1]

            # option と値を scenario 引数から除外
            args = args[:idx] + args[idx + 2:]
            break

    scenario_args = args

    if not scenario_args:
        raise ValueError("scenarioID が指定されていません。")

    scenario_ids = [normalize_scenario_arg(arg) for arg in scenario_args]

    return scenario_ids, condition_key, no_legend


def main() -> None:
    if len(sys.argv) < 2:
        print(
            "Usage: python3 compare_cdf_plot.py "
            "<scenarioID1> <scenarioID2> ... "
            "[--condition_key 1.0|nosystem] [--no_legend]"
        )
        print(
            "Example: python3 compare_cdf_plot.py "
            "scenario21 scenario43 scenario57"
        )
        print(
            "Example: python3 compare_cdf_plot.py "
            "scenario21 scenario43 scenario57 --condition_key nosystem"
        )
        print(
            "Example: python3 compare_cdf_plot.py "
            "21 43 57 --no_legend"
        )
        sys.exit(1)

    raw_args = [arg.strip() for arg in sys.argv[1:]]

    scenario_ids, condition_key, no_legend = parse_args(raw_args)

    base_dir = Path(__file__).resolve().parent
    output_dir = base_dir / "compared"
    output_dir.mkdir(parents=True, exist_ok=True)

    scenario_to_values: dict[str, list[float]] = {}

    for scenario_id in scenario_ids:
        scenario_dir = base_dir / scenario_id
        json_path = scenario_dir / "output.json"

        values = load_arrival_time_list(
        json_path=json_path,
            condition_key=condition_key,
        )

        scenario_to_values[scenario_id] = values

        # arrival_time_list の最後の値を避難完了時刻として表示
        evacuation_completion_time = float(values[-1])

        print(
            f"[INFO] {scenario_id}: loaded {len(values)} arrival times "
            f"from condition_key='{condition_key}'"
        )

        print(
            f"[RESULT] {scenario_id}: "
            f"evacuation_completion_time = {evacuation_completion_time:.2f} s"
        )

    scenario_part = "_vs_".join(scenario_ids)
    safe_condition_key = condition_key.replace("/", "_")
    legend_part = "_no_legend" if no_legend else ""

    output_path = (
        output_dir
        / f"arrival_time_cdf_compare_{scenario_part}_condition_{safe_condition_key}{legend_part}.pdf"
    )

    plot_compare_arrival_time_cdfs(
        scenario_to_values=scenario_to_values,
        save_path=output_path,
        no_legend=no_legend,
    )

    print(f"[INFO] Arrival-time CDF comparison plot saved: {output_path}")


if __name__ == "__main__":
    main()