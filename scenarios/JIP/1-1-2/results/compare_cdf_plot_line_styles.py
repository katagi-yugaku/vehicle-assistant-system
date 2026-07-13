import json
import sys
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np


DEFAULT_CONDITION_KEY = "1.0"

# 実線で描画するシナリオ
SOLID_SCENARIO_NUMBERS = {1, 5, 21, 25}

# 点線で描画するシナリオ
DOTTED_SCENARIO_NUMBERS = {70, 71, 72, 73}


def normalize_scenario_arg(arg: str) -> str:
    """
    scenario引数を正規化する。

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


def extract_scenario_number(scenario_id: str) -> int | None:
    """
    "scenario21" から 21 を取得する。
    数値部分を取得できない場合はNoneを返す。
    """
    prefix = "scenario"

    if not scenario_id.startswith(prefix):
        return None

    number_part = scenario_id[len(prefix):]

    if not number_part.isdigit():
        return None

    return int(number_part)


def get_linestyle(scenario_id: str) -> str:
    """
    シナリオ番号に応じて線種を返す。

    scenario1, 5, 21, 25:
        実線 "-"

    scenario70, 71, 72, 73:
        点線 ":"

    それ以外:
        実線 "-"
    """
    scenario_number = extract_scenario_number(scenario_id)

    if scenario_number in DOTTED_SCENARIO_NUMBERS:
        return ":"

    return "-"


def load_arrival_time_list(
    json_path: Path,
    condition_key: str = DEFAULT_CONDITION_KEY,
) -> list[float]:
    """
    output.jsonからarrival_time_listを取得する。

    想定形式1:
    {
      "arrival_time": {
        "arrival_time_list": [...]
      }
    }

    想定形式2:
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
    """
    if not json_path.exists():
        raise FileNotFoundError(f"JSON file not found: {json_path}")

    with json_path.open("r", encoding="utf-8") as file:
        data = json.load(file)

    if not isinstance(data, dict):
        raise TypeError(f"Root JSON object must be a dict: {json_path}")

    # 形式1
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

        return [float(value) for value in values]

    # 形式2
    if condition_key in data:
        condition_data = data[condition_key]

        if not isinstance(condition_data, dict):
            raise TypeError(
                f"data['{condition_key}'] must be a dict in {json_path}"
            )

        arrival_time = condition_data.get("arrival_time")

        if not isinstance(arrival_time, dict):
            raise KeyError(
                f"'arrival_time' not found under condition_key="
                f"'{condition_key}' in {json_path}"
            )

        values = arrival_time.get("arrival_time_list")

        if values is None:
            raise KeyError(
                f"'arrival_time_list' not found under condition_key="
                f"'{condition_key}' in {json_path}"
            )

        if not isinstance(values, list):
            raise TypeError(
                f"arrival_time_list under condition_key='{condition_key}' "
                f"in {json_path} must be a list, "
                f"but got {type(values).__name__}"
            )

        print(f"[INFO] {json_path}: using condition key '{condition_key}'")
        return [float(value) for value in values]

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
    arrival_time_listを基にCDFを描画してPDF保存する。

    scenario1, scenario5, scenario21, scenario25:
        実線

    scenario70, scenario71, scenario72, scenario73:
        点線
    """
    plt.figure(figsize=(10, 6))

    plotted_count = 0

    for scenario_id, values in scenario_to_values.items():
        value_array = np.asarray(values, dtype=float)

        if value_array.size == 0:
            print(
                f"[WARN] {scenario_id}のarrival_time_listは空です。"
                "スキップします。"
            )
            continue

        sorted_values = np.sort(value_array)
        cdf = np.arange(1, sorted_values.size + 1) / sorted_values.size

        linestyle = get_linestyle(scenario_id)

        plt.plot(
            sorted_values,
            cdf,
            label=scenario_id,
            linestyle=linestyle,
            linewidth=2.5,
        )

        plotted_count += 1

    if plotted_count == 0:
        raise ValueError("プロット対象のarrival_time_listがありません。")

    min_time = 100
    max_time = 2500
    y_min = 0.0

    plt.xlim(min_time, max_time)
    plt.ylim(y_min, 1.0)

    plt.xticks(
        np.arange(min_time, max_time + 1, 200),
        fontsize=14,
        fontweight="semibold",
    )
    plt.yticks(
        np.arange(y_min, 1.01, 0.1),
        fontsize=14,
        fontweight="semibold",
    )

    plt.xlabel("Arrival time (s)", fontsize=14)
    plt.ylabel("CDF", fontsize=14)
    plt.grid(True, linestyle="--", linewidth=0.5, alpha=0.5)

    if not no_legend:
        plt.legend(fontsize=12)

    save_path.parent.mkdir(parents=True, exist_ok=True)
    plt.savefig(save_path, bbox_inches="tight")
    plt.close()

    print(f"Saved figure as: {save_path}")


def parse_args(raw_args: list[str]) -> tuple[list[str], str, bool]:
    """
    コマンドライン引数を解析する。

    例:
        python3 compare_cdf_plot_line_styles.py \
            1 5 21 25 70 71 72 73

        python3 compare_cdf_plot_line_styles.py \
            1 5 21 25 70 71 72 73 \
            --condition-key nosystem

        python3 compare_cdf_plot_line_styles.py \
            1 5 21 25 70 71 72 73 \
            --no-legend
    """
    no_legend = (
        "--no_legend" in raw_args
        or "--no-legend" in raw_args
    )

    args = [
        arg
        for arg in raw_args
        if arg not in {"--no_legend", "--no-legend"}
    ]

    condition_key = DEFAULT_CONDITION_KEY

    for option_name in ("--condition_key", "--condition-key"):
        if option_name not in args:
            continue

        option_index = args.index(option_name)

        if option_index + 1 >= len(args):
            raise ValueError(
                f"{option_name}の後に条件キーを指定してください。"
            )

        condition_key = args[option_index + 1]
        args = args[:option_index] + args[option_index + 2:]
        break

    if not args:
        raise ValueError("scenario IDが指定されていません。")

    scenario_ids = [
        normalize_scenario_arg(arg)
        for arg in args
    ]

    return scenario_ids, condition_key, no_legend


def main() -> None:
    if len(sys.argv) < 2:
        print(
            "Usage: python3 compare_cdf_plot_line_styles.py "
            "<scenarioID1> <scenarioID2> ... "
            "[--condition-key 1.0|nosystem] [--no-legend]"
        )
        print(
            "Example: python3 compare_cdf_plot_line_styles.py "
            "1 5 21 25 70 71 72 73"
        )
        sys.exit(1)

    raw_args = [arg.strip() for arg in sys.argv[1:]]

    try:
        scenario_ids, condition_key, no_legend = parse_args(raw_args)
    except ValueError as exc:
        print(f"[ERROR] {exc}", file=sys.stderr)
        sys.exit(2)

    base_dir = Path(__file__).resolve().parent
    output_dir = base_dir / "compared"

    scenario_to_values: dict[str, list[float]] = {}

    for scenario_id in scenario_ids:
        scenario_dir = base_dir / scenario_id
        json_path = scenario_dir / "output.json"

        try:
            values = load_arrival_time_list(
                json_path=json_path,
                condition_key=condition_key,
            )
        except (OSError, ValueError, TypeError, KeyError, json.JSONDecodeError) as exc:
            print(
                f"[ERROR] Failed to load {scenario_id}: {exc}",
                file=sys.stderr,
            )
            sys.exit(3)

        scenario_to_values[scenario_id] = values

        evacuation_completion_time = max(values) if values else float("nan")

        linestyle_name = (
            "dotted"
            if get_linestyle(scenario_id) == ":"
            else "solid"
        )

        print(
            f"[INFO] {scenario_id}: loaded {len(values)} arrival times, "
            f"condition_key='{condition_key}', "
            f"linestyle='{linestyle_name}'"
        )
        print(
            f"[RESULT] {scenario_id}: "
            f"evacuation_completion_time="
            f"{evacuation_completion_time:.2f} s"
        )

    scenario_part = "_vs_".join(scenario_ids)
    safe_condition_key = condition_key.replace("/", "_")
    legend_part = "_no_legend" if no_legend else ""

    output_path = (
        output_dir
        / (
            f"arrival_time_cdf_compare_{scenario_part}"
            f"_condition_{safe_condition_key}"
            f"{legend_part}.pdf"
        )
    )

    plot_compare_arrival_time_cdfs(
        scenario_to_values=scenario_to_values,
        save_path=output_path,
        no_legend=no_legend,
    )

    print(
        "[INFO] Arrival-time CDF comparison plot saved: "
        f"{output_path}"
    )


if __name__ == "__main__":
    main()
