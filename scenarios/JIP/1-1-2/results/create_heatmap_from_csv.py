#!/usr/bin/env python3
from __future__ import annotations

import argparse
import csv
import math
from pathlib import Path


import matplotlib.pyplot as plt


# ==================================================
# Default style settings
# ==================================================
DEFAULT_INPUT_CSV = "heatmap_evacuation_completion_time.csv"

DEFAULT_TITLE = "Evacuation Completion Time"
DEFAULT_XLABEL = "p_follow"
DEFAULT_YLABEL = "active_route_change_threshold_center"
DEFAULT_COLORBAR_LABEL = "Completion Time [s]"

DEFAULT_CMAP = "viridis"

FIG_WIDTH = 10.0
FIG_HEIGHT = 7.0

ANNOTATION_FONT_SIZE = 13
TITLE_FONT_SIZE = 13
AXIS_LABEL_FONT_SIZE = 13
TICK_LABEL_FONT_SIZE = 13
COLORBAR_LABEL_FONT_SIZE = 13

VALUE_FORMAT = ".1f"
DEFAULT_DPI = 300

X_TICK_ROTATION = 0
Y_TICK_ROTATION = 0

SHOW_GRID_LINES = False
GRID_LINE_COLOR = "white"
GRID_LINE_WIDTH = 0.8


def is_empty_row(row: list[str]) -> bool:
    """
    CSV の空行を判定する。

    例:
        []
        ["", "", ""]
        [" ", " ", " "]
    """
    return all(cell.strip() == "" for cell in row)


def parse_float_or_nan(value: str) -> float:
    """
    空セルは math.nan として扱う。
    数値に変換できない場合も math.nan とする。
    """
    value = value.strip()

    if value == "":
        return math.nan

    try:
        return float(value)
    except ValueError:
        return math.nan


def resolve_input_csv_path(csv_path: Path) -> Path:
    """
    入力 CSV のパスを解決する。

    まず現在の作業ディレクトリ基準で探す。
    見つからない場合は、このスクリプトと同一階層から探す。
    """
    if csv_path.exists():
        return csv_path.resolve()

    script_dir = Path(__file__).resolve().parent
    script_relative_path = script_dir / csv_path

    if script_relative_path.exists():
        return script_relative_path.resolve()

    raise FileNotFoundError(f"CSV file not found: {csv_path}")


def build_default_output_path(csv_path: Path) -> Path:
    """
    入力 CSV に対応するデフォルト出力 PDF パスを作る。

    例:
        heatmap_evacuation_completion_time.csv
        -> heatmap_evacuation_completion_time_custom.pdf
    """
    return csv_path.with_name(f"{csv_path.stem}_custom.pdf")


def resolve_output_pdf_path(csv_path: Path, output: Path | None) -> Path:
    """
    出力 PDF のパスを解決する。

    --output が省略された場合:
        入力 CSV と同じ階層に *_custom.pdf を保存する。

    --output が相対パスの場合:
        入力 CSV と同じ階層からの相対パスとして扱う。
    """
    if output is None:
        return build_default_output_path(csv_path)

    if output.is_absolute():
        return output

    return csv_path.parent / output


def load_heatmap_csv(csv_path: Path) -> tuple[list[str], list[str], list[list[float]]]:
    """
    CSV を読み込み、x 軸ラベル、y 軸ラベル、値の2次元リストを返す。

    CSV の仕様:
        1行目:
            1列目 = y軸名
            2列目以降 = x軸ラベル

        2行目以降:
            1列目 = y軸ラベル
            2列目以降 = heatmap の値

    末尾の空行は無視する。
    空セルは math.nan として扱う。
    """
    with csv_path.open("r", encoding="utf-8", newline="") as f:
        reader = csv.reader(f)
        rows = [row for row in reader if not is_empty_row(row)]

    if not rows:
        raise ValueError(f"CSV is empty: {csv_path}")

    header = rows[0]

    if len(header) < 2:
        raise ValueError(
            "CSV header must contain at least one y-axis column "
            "and one x-axis label."
        )

    x_labels = [cell.strip() for cell in header[1:]]

    y_labels: list[str] = []
    values: list[list[float]] = []

    expected_value_count = len(x_labels)

    for row_index, row in enumerate(rows[1:], start=2):
        if len(row) == 0 or is_empty_row(row):
            continue

        y_label = row[0].strip()

        if y_label == "":
            continue

        value_cells = row[1:]

        if len(value_cells) < expected_value_count:
            value_cells = value_cells + [""] * (expected_value_count - len(value_cells))

        if len(value_cells) > expected_value_count:
            value_cells = value_cells[:expected_value_count]

        row_values = [parse_float_or_nan(cell) for cell in value_cells]

        y_labels.append(y_label)
        values.append(row_values)

    if not y_labels:
        raise ValueError(f"No data rows found in CSV: {csv_path}")

    return x_labels, y_labels, values


def create_heatmap(
    x_labels: list[str],
    y_labels: list[str],
    values: list[list[float]],
    output_pdf_path: Path,
    title: str,
    xlabel: str,
    ylabel: str,
    colorbar_label: str,
    cmap: str,
    vmin: float | None,
    vmax: float | None,
    fig_width: float,
    fig_height: float,
    annot_format: str,
    save_png: bool,
    dpi: int,
) -> None:
    """
    matplotlib を使って heatmap を作成し、
    PDF と必要に応じて PNG を保存する。
    """
    if not x_labels:
        raise ValueError("x_labels is empty")

    if not y_labels:
        raise ValueError("y_labels is empty")

    if not values:
        raise ValueError("values is empty")

    fig, ax = plt.subplots(figsize=(fig_width, fig_height))

    im = ax.imshow(
    values,
    aspect="auto",
    cmap="Reds_r",
    vmin=vmin,
    vmax=vmax,
    )

    # 横向き colorbar
    colorbar = fig.colorbar(
        im,
        ax=ax,
        orientation="vertical",
        pad=0.05,
    )
    # colorbar.ax.invert_yaxis()
    # colorbar.set_label(colorbar_label, fontsize=COLORBAR_LABEL_FONT_SIZE)
    colorbar.ax.tick_params(labelsize=TICK_LABEL_FONT_SIZE)

    # ax.set_title(title, fontsize=TITLE_FONT_SIZE)
    # ax.set_xlabel(xlabel, fontsize=AXIS_LABEL_FONT_SIZE)
    # ax.set_ylabel(ylabel, fontsize=AXIS_LABEL_FONT_SIZE)

    ax.set_xticks(range(len(x_labels)))
    ax.set_yticks(range(len(y_labels)))

    ax.set_xticklabels(
        x_labels,
        fontsize=TICK_LABEL_FONT_SIZE,
        rotation=X_TICK_ROTATION,
    )
    ax.set_yticklabels(
        y_labels,
        fontsize=TICK_LABEL_FONT_SIZE,
        rotation=Y_TICK_ROTATION,
    )

    if SHOW_GRID_LINES:
        ax.set_xticks(
            [x - 0.5 for x in range(1, len(x_labels))],
            minor=True,
        )
        ax.set_yticks(
            [y - 0.5 for y in range(1, len(y_labels))],
            minor=True,
        )
        ax.grid(
            which="minor",
            color=GRID_LINE_COLOR,
            linestyle="-",
            linewidth=GRID_LINE_WIDTH,
        )
        ax.tick_params(which="minor", bottom=False, left=False)

    for row_index, row_values in enumerate(values):
        for col_index, value in enumerate(row_values):
            if math.isnan(value):
                continue

            ax.text(
                col_index,
                row_index,
                format(value, annot_format),
                ha="center",
                va="center",
                fontsize=ANNOTATION_FONT_SIZE,
                fontweight="bold",
            )

    output_pdf_path.parent.mkdir(parents=True, exist_ok=True)

    fig.tight_layout()
    fig.savefig(output_pdf_path, format="pdf", bbox_inches="tight")

    print(f"Saved PDF: {output_pdf_path}")

    if save_png:
        output_png_path = output_pdf_path.with_suffix(".png")
        fig.savefig(
            output_png_path,
            format="png",
            dpi=dpi,
            bbox_inches="tight",
        )
        print(f"Saved PNG: {output_png_path}")

    plt.close(fig)


def parse_args() -> argparse.Namespace:
    """
    コマンドライン引数を解析する。
    """
    parser = argparse.ArgumentParser(
        description="Create a heatmap PDF from a CSV file."
    )

    parser.add_argument(
        "csv_path",
        nargs="?",
        type=Path,
        default=Path(DEFAULT_INPUT_CSV),
        help=(
            "Input CSV file. "
            f"Default: {DEFAULT_INPUT_CSV}"
        ),
    )

    parser.add_argument(
        "--output",
        type=Path,
        default=None,
        help=(
            "Output PDF file. "
            "Default: <input_csv_stem>_custom.pdf"
        ),
    )

    parser.add_argument(
        "--title",
        type=str,
        default=DEFAULT_TITLE,
        help=f'Figure title. Default: "{DEFAULT_TITLE}"',
    )

    parser.add_argument(
        "--xlabel",
        type=str,
        default=DEFAULT_XLABEL,
        help=f'X-axis label. Default: "{DEFAULT_XLABEL}"',
    )

    parser.add_argument(
        "--ylabel",
        type=str,
        default=DEFAULT_YLABEL,
        help=f'Y-axis label. Default: "{DEFAULT_YLABEL}"',
    )

    parser.add_argument(
        "--colorbar-label",
        type=str,
        default=DEFAULT_COLORBAR_LABEL,
        help=f'Colorbar label. Default: "{DEFAULT_COLORBAR_LABEL}"',
    )

    parser.add_argument(
        "--cmap",
        type=str,
        default=DEFAULT_CMAP,
        help=f'Matplotlib colormap name. Default: "{DEFAULT_CMAP}"',
    )

    parser.add_argument(
        "--vmin",
        type=float,
        default=None,
        help="Minimum value for color scale. Default: CSV minimum.",
    )

    parser.add_argument(
        "--vmax",
        type=float,
        default=None,
        help="Maximum value for color scale. Default: CSV maximum.",
    )

    parser.add_argument(
        "--fig-width",
        type=float,
        default=FIG_WIDTH,
        help=f"Figure width. Default: {FIG_WIDTH}",
    )

    parser.add_argument(
        "--fig-height",
        type=float,
        default=FIG_HEIGHT,
        help=f"Figure height. Default: {FIG_HEIGHT}",
    )

    parser.add_argument(
        "--annot-format",
        type=str,
        default=VALUE_FORMAT,
        help=f'Annotation number format. Default: "{VALUE_FORMAT}"',
    )

    parser.add_argument(
        "--png",
        action="store_true",
        help="Also save PNG output.",
    )

    parser.add_argument(
        "--dpi",
        type=int,
        default=DEFAULT_DPI,
        help=f"PNG DPI. Default: {DEFAULT_DPI}",
    )

    return parser.parse_args()


def main() -> None:
    """
    メイン処理。
    """
    args = parse_args()

    input_csv_path = resolve_input_csv_path(args.csv_path)
    output_pdf_path = resolve_output_pdf_path(
        csv_path=input_csv_path,
        output=args.output,
    )

    x_labels, y_labels, values = load_heatmap_csv(input_csv_path)

    create_heatmap(
        x_labels=x_labels,
        y_labels=y_labels,
        values=values,
        output_pdf_path=output_pdf_path,
        title=args.title,
        xlabel=args.xlabel,
        ylabel=args.ylabel,
        colorbar_label=args.colorbar_label,
        cmap=args.cmap,
        vmin=args.vmin,
        vmax=args.vmax,
        fig_width=args.fig_width,
        fig_height=args.fig_height,
        annot_format=args.annot_format,
        save_png=args.png,
        dpi=args.dpi,
    )


if __name__ == "__main__":
    main()