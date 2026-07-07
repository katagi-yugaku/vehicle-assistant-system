import json
import sys
from pathlib import Path

import matplotlib.pyplot as plt


COLOR_MAP = {
    "nosystem": "#6F94E6",
    "0.1": "red",
    "0.5": "blue",
    "0.9": "green",
}

DEFAULT_CONDITION_ORDER = ["nosystem", "0.1", "0.5", "0.9"]


def normalize_scenario_arg(arg: str) -> str:
    """
    scenario1 / 1 のどちらでも受け取れるようにする。
    """
    arg = arg.strip()

    if arg.isdigit():
        return f"scenario{int(arg)}"

    if arg.startswith("scenario") and arg[len("scenario"):].isdigit():
        return f"scenario{int(arg[len('scenario'):])}"

    return arg


def print_usage() -> None:
    script_name = Path(sys.argv[0]).name
    print(
        "Usage:\n"
        f"  python {script_name} <scenario_name> <metric_name> [condition ...]\n\n"
        "Examples:\n"
        f"  python {script_name} scenario1 pedestrian_count\n"
        f"  python {script_name} scenario1 pedestrian_count 0.1 0.9\n"
        f"  python {script_name} scenario1 route_changed_vehicle_count 0.1 0.5 0.9",
        file=sys.stderr,
    )


def load_average_count_metrics(json_path: Path) -> dict:
    if not json_path.exists():
        raise FileNotFoundError(f"output.json が存在しません: {json_path}")

    with json_path.open("r", encoding="utf-8") as f:
        data = json.load(f)

    if "average_count_metrics" not in data:
        raise KeyError(f"'average_count_metrics' が存在しません: {json_path}")

    average_count_metrics = data["average_count_metrics"]

    if not isinstance(average_count_metrics, dict):
        raise TypeError("'average_count_metrics' は dict 形式である必要があります。")

    return average_count_metrics


def decide_conditions(
    average_count_metrics: dict,
    requested_conditions: list[str],
) -> list[str]:
    available_conditions = set(average_count_metrics.keys())

    if requested_conditions:
        missing_conditions = [
            condition
            for condition in requested_conditions
            if condition not in available_conditions
        ]

        if missing_conditions:
            raise KeyError(
                "指定した条件が average_count_metrics に存在しません: "
                + ", ".join(missing_conditions)
            )

        return requested_conditions

    return [
        condition
        for condition in DEFAULT_CONDITION_ORDER
        if condition in available_conditions
    ]


def collect_metric_values(
    average_count_metrics: dict,
    metric_name: str,
    conditions: list[str],
) -> list[float]:
    if not conditions:
        raise ValueError("描画対象の条件がありません。")

    values = []

    for condition in conditions:
        condition_metrics = average_count_metrics[condition]

        if metric_name not in condition_metrics:
            raise KeyError(
                f"指定した metric が存在しません: '{metric_name}' "
                f"(condition: {condition})"
            )

        values.append(float(condition_metrics[metric_name]))

    return values


def format_value(value: float) -> str:
    if abs(value - round(value)) < 1e-9:
        return str(int(round(value)))

    return f"{value:.2f}".rstrip("0").rstrip(".")


def make_output_path(
    scenario_name: str,
    metric_name: str,
    conditions: list[str],
    conditions_were_specified: bool,
) -> Path:
    output_dir = Path("slide_fig")
    output_dir.mkdir(parents=True, exist_ok=True)

    if conditions_were_specified:
        condition_part = "_".join(conditions)
        filename = f"{scenario_name}_{metric_name}_{condition_part}_bar.pdf"
    else:
        filename = f"{scenario_name}_{metric_name}_bar.pdf"

    return output_dir / filename


def plot_horizontal_bar(
    conditions: list[str],
    values: list[float],
    output_path: Path,
) -> None:
    bar_colors = [COLOR_MAP.get(condition, "gray") for condition in conditions]

    fig_height = max(1.4, 0.45 * len(conditions) + 0.3)
    fig_width = 5.0

    fig, ax = plt.subplots(figsize=(fig_width, fig_height))
    fig.patch.set_facecolor("none")
    ax.set_facecolor("none")

    y_positions = list(range(len(conditions)))

    ax.barh(
        y_positions,
        values,
        color=bar_colors,
        height=0.80,
        edgecolor="none",
        linewidth=1.0,
    )

    ax.invert_yaxis()

    max_value = max(values) if values else 0
    x_margin = max(max_value * 0.05, 1.0)
    ax.set_xlim(0, max_value + x_margin)

    # x軸・y軸の表示を消す
    ax.xaxis.set_visible(False)
    ax.set_yticks([])
    ax.set_yticklabels([])

    for spine in ax.spines.values():
        spine.set_visible(False)

    ax.tick_params(axis="both", length=0)

    # 表のように見せるための薄い区切り線
    for i in range(len(conditions) + 1):
        # color="#444444",
        ax.axhline(
            i - 0.5,
            color="none",
            linewidth=0.8,
            alpha=0.7,
            zorder=0,
        )

    ax.axvline(
        0,
        color="none",
        linewidth=1.0,
        alpha=0.8,
        zorder=0,
    )

    # ラベル・数値を表示しないため，余白を詰める
    plt.subplots_adjust(left=0.02, right=0.98, top=0.95, bottom=0.08)

    fig.savefig(
        output_path,
        format="pdf",
        facecolor=fig.get_facecolor(),
        bbox_inches="tight",
    )

    plt.close(fig)

def main() -> None:
    if len(sys.argv) < 3:
        print_usage()
        sys.exit(1)

    scenario_name = normalize_scenario_arg(sys.argv[1])
    metric_name = sys.argv[2]
    requested_conditions = sys.argv[3:]

    json_path = Path(scenario_name) / "output.json"

    try:
        average_count_metrics = load_average_count_metrics(json_path)

        conditions = decide_conditions(
            average_count_metrics=average_count_metrics,
            requested_conditions=requested_conditions,
        )

        values = collect_metric_values(
            average_count_metrics=average_count_metrics,
            metric_name=metric_name,
            conditions=conditions,
        )

        output_path = make_output_path(
            scenario_name=scenario_name,
            metric_name=metric_name,
            conditions=conditions,
            conditions_were_specified=bool(requested_conditions),
        )

        plot_horizontal_bar(
            conditions=conditions,
            values=values,
            output_path=output_path,
        )

        print(f"[INFO] Bar plot saved: {output_path}")

    except Exception as e:
        print(f"[ERROR] {e}", file=sys.stderr)
        sys.exit(1)


if __name__ == "__main__":
    main()