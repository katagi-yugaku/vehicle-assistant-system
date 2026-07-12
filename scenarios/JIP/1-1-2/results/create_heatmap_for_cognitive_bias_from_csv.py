#!/usr/bin/env python3
from __future__ import annotations

import argparse
import csv
import math
import re
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

# 小さい値を濃い赤にしたい場合は "Reds_r"
# 大きい値を濃い赤にしたい場合は "Reds"
DEFAULT_CMAP = "Reds_r"

FIG_WIDTH = 10.0
FIG_HEIGHT = 7.0

ANNOTATION_FONT_SIZE = 18
TITLE_FONT_SIZE = 15
AXIS_LABEL_FONT_SIZE = 15
TICK_LABEL_FONT_SIZE = 15
COLORBAR_LABEL_FONT_SIZE = 15

VALUE_FORMAT = ".2f"
DEFAULT_DPI = 300

X_TICK_ROTATION = 0
Y_TICK_ROTATION = 0

SHOW_GRID_LINES = False
GRID_LINE_COLOR = "white"
GRID_LINE_WIDTH = 0.8

# 背景色の輝度がこの値未満の場合、文字を白色にする。
# 値を大きくすると白文字になるセルが増える。
ANNOTATION_LUMINANCE_THRESHOLD = 0.55


def is_empty_row(row: list[str]) -> bool:
    """
    CSV の空行を判定する。
    """
    return all(cell.strip() == "" for cell in row)


def parse_float_or_nan(value: str) -> float:
    """
    空セルは math.nan として扱う。

    数値だけのセル:
        "2050.24" -> 2050.24

    注釈付きセル:
        "80.6\\nN49 M51" -> 80.6
        "80.6
         N49 M51" -> 80.6

    数値で始まらないセル:
        "N49 M51" -> math.nan
    """
    value = value.strip()

    if value == "":
        return math.nan

    # 通常の数値セル
    try:
        return float(value)
    except ValueError:
        pass

    # セル内に実改行または "\n" 文字列がある場合でも、
    # 先頭行の数値を取得する。
    normalized_value = value.replace("\\n", "\n")
    first_line = normalized_value.splitlines()[0].strip()

    match = re.match(
        r"^[+-]?(?:\d+(?:\.\d*)?|\.\d+)(?:[eE][+-]?\d+)?",
        first_line,
    )

    if match is None:
        return math.nan

    try:
        return float(match.group(0))
    except ValueError:
        return math.nan


def normalize_annotation_text(value: str) -> str:
    """
    注釈CSVのセル文字列を matplotlib 表示用に整形する。

    "80.6\\nN49 M51" のように、バックスラッシュ+n で
    保存されている場合も実改行に変換する。
    """
    return value.strip().replace("\\n", "\n")


def has_finite_value(values: list[list[float]]) -> bool:
    """
    heatmap に描画可能な数値が1つでもあるか確認する。
    """
    for row in values:
        for value in row:
            if not math.isnan(value):
                return True

    return False


def get_annotation_text_color(
    image,
    value: float,
    luminance_threshold: float = ANNOTATION_LUMINANCE_THRESHOLD,
) -> str:
    """
    heatmap のセル背景色に応じて、注釈文字の色を返す。

    背景色が暗い場合:
        white

    背景色が明るい場合:
        black

    image.norm() で値をカラーマップの範囲に正規化し、
    image.cmap() で実際のセル背景色を取得する。
    """
    normalized_value = image.norm(value)

    red, green, blue, _ = image.cmap(normalized_value)

    # 人間の視覚特性を考慮した相対輝度
    luminance = (
        0.2126 * red
        + 0.7152 * green
        + 0.0722 * blue
    )

    if luminance < luminance_threshold:
        return "white"

    return "black"


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


def resolve_optional_csv_path(csv_path: Path | None) -> Path | None:
    """
    任意指定の CSV パスを解決する。
    """
    if csv_path is None:
        return None

    return resolve_input_csv_path(csv_path)


def build_default_output_path(csv_path: Path) -> Path:
    """
    入力 CSV に対応するデフォルト出力 PDF パスを作る。
    """
    return csv_path.with_name(f"{csv_path.stem}_custom.pdf")


def resolve_output_pdf_path(
    csv_path: Path,
    output: Path | None,
) -> Path:
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


def load_heatmap_csv(
    csv_path: Path,
) -> tuple[list[str], list[str], list[list[float]]]:
    """
    CSV を読み込み、x 軸ラベル、y 軸ラベル、
    値の2次元リストを返す。

    CSV の仕様:
        1行目:
            1列目 = y軸名
            2列目以降 = x軸ラベル

        2行目以降:
            1列目 = y軸ラベル
            2列目以降 = heatmap の値

    空セルは math.nan として扱う。
    """
    with csv_path.open(
        "r",
        encoding="utf-8",
        newline="",
    ) as file:
        reader = csv.reader(file)
        rows = [
            row
            for row in reader
            if not is_empty_row(row)
        ]

    if not rows:
        raise ValueError(f"CSV is empty: {csv_path}")

    header = rows[0]

    if len(header) < 2:
        raise ValueError(
            "CSV header must contain at least one y-axis column "
            "and one x-axis label."
        )

    x_labels = [
        cell.strip()
        for cell in header[1:]
    ]

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
            value_cells = value_cells + [""] * (
                expected_value_count - len(value_cells)
            )

        if len(value_cells) > expected_value_count:
            value_cells = value_cells[:expected_value_count]

        row_values = [
            parse_float_or_nan(cell)
            for cell in value_cells
        ]

        y_labels.append(y_label)
        values.append(row_values)

    if not y_labels:
        raise ValueError(
            f"No data rows found in CSV: {csv_path}"
        )

    return x_labels, y_labels, values


def load_annotation_csv(
    csv_path: Path,
    expected_x_labels: list[str],
    expected_y_labels: list[str],
) -> list[list[str]]:
    """
    注釈用 CSV を読み込む。

    想定する CSV 形式は heatmap CSV と同じ。

    例:
        active_route_change_threshold_center,0.10,0.15
        100.0,"111.8
        N50 M50","112.6
        N48 M52"

    セル内改行を含む場合、csv.writer で保存されていれば
    自動で読み込める。
    """
    with csv_path.open(
        "r",
        encoding="utf-8",
        newline="",
    ) as file:
        reader = csv.reader(file)
        rows = [
            row
            for row in reader
            if not is_empty_row(row)
        ]

    if not rows:
        raise ValueError(
            f"Annotation CSV is empty: {csv_path}"
        )

    header = rows[0]

    if len(header) < 2:
        raise ValueError(
            "Annotation CSV header must contain at least one "
            "y-axis column and one x-axis label."
        )

    x_labels = [
        cell.strip()
        for cell in header[1:]
    ]

    if x_labels != expected_x_labels:
        raise ValueError(
            "Annotation CSV x labels do not match "
            "input CSV x labels.\n"
            f"input      = {expected_x_labels}\n"
            f"annotation = {x_labels}"
        )

    annotation_y_labels: list[str] = []
    annotations: list[list[str]] = []

    expected_value_count = len(expected_x_labels)

    for row_index, row in enumerate(rows[1:], start=2):
        if len(row) == 0 or is_empty_row(row):
            continue

        y_label = row[0].strip()

        if y_label == "":
            continue

        value_cells = row[1:]

        if len(value_cells) < expected_value_count:
            value_cells = value_cells + [""] * (
                expected_value_count - len(value_cells)
            )

        if len(value_cells) > expected_value_count:
            value_cells = value_cells[:expected_value_count]

        row_annotations = [
            normalize_annotation_text(cell)
            for cell in value_cells
        ]

        annotation_y_labels.append(y_label)
        annotations.append(row_annotations)

    if annotation_y_labels != expected_y_labels:
        raise ValueError(
            "Annotation CSV y labels do not match "
            "input CSV y labels.\n"
            f"input      = {expected_y_labels}\n"
            f"annotation = {annotation_y_labels}"
        )

    return annotations


def validate_heatmap_dimensions(
    x_labels: list[str],
    y_labels: list[str],
    values: list[list[float]],
) -> None:
    """
    heatmap の行列サイズがラベル数と一致するか確認する。
    """
    if len(values) != len(y_labels):
        raise ValueError(
            "values row count does not match y label count: "
            f"values={len(values)}, "
            f"y_labels={len(y_labels)}"
        )

    expected_column_count = len(x_labels)

    for row_index, row_values in enumerate(values):
        if len(row_values) != expected_column_count:
            raise ValueError(
                "values column count does not match "
                f"x label count at row {row_index}: "
                f"values={len(row_values)}, "
                f"x_labels={expected_column_count}"
            )


def validate_annotations(
    annotations: list[list[str]],
    values: list[list[float]],
) -> None:
    """
    注釈行列のサイズが heatmap の値行列と一致するか確認する。
    """
    if len(annotations) != len(values):
        raise ValueError(
            "annotations row count does not match "
            "values row count: "
            f"annotations={len(annotations)}, "
            f"values={len(values)}"
        )

    for row_index, (
        annotation_row,
        value_row,
    ) in enumerate(zip(annotations, values)):
        if len(annotation_row) != len(value_row):
            raise ValueError(
                "annotations column count does not match "
                "values column count "
                f"at row {row_index}: "
                f"annotations={len(annotation_row)}, "
                f"values={len(value_row)}"
            )


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
    annotations: list[list[str]] | None = None,
    show_title: bool = True,
    show_axis_labels: bool = True,
) -> None:
    """
    matplotlib を使って heatmap を作成し、
    PDF と必要に応じて PNG を保存する。

    annotations が指定された場合:
        heatmap の色は values を使う。
        セル上の文字は annotations を使う。

    annotations が指定されない場合:
        セル上の文字は values を annot_format で表示する。

    セル背景色が暗い場合は白文字、
    明るい場合は黒文字で表示する。
    """
    if not x_labels:
        raise ValueError("x_labels is empty")

    if not y_labels:
        raise ValueError("y_labels is empty")

    if not values:
        raise ValueError("values is empty")

    validate_heatmap_dimensions(
        x_labels=x_labels,
        y_labels=y_labels,
        values=values,
    )

    if not has_finite_value(values):
        raise ValueError(
            "No numeric values found in the input CSV. "
            "heatmap の値がすべて NaN として読まれています。"
            "注釈CSVをメインCSVとして渡していないか"
            "確認してください。"
        )

    if annotations is not None:
        validate_annotations(
            annotations=annotations,
            values=values,
        )

    fig, ax = plt.subplots(
        figsize=(fig_width, fig_height)
    )

    im = ax.imshow(
        values,
        aspect="auto",
        cmap="Reds",
        vmin=vmin,
        vmax=vmax,
    )

    colorbar = fig.colorbar(
        im,
        ax=ax,
        orientation="vertical",
        pad=0.05,
    )
    # colorbar.ax.invert_yaxis()

    colorbar.ax.tick_params(
        labelsize=TICK_LABEL_FONT_SIZE
    )

    # if show_title:
    #     ax.set_title(
    #         title,
    #         fontsize=TITLE_FONT_SIZE,
    #     )

    # if show_axis_labels:
    #     ax.set_xlabel(
    #         xlabel,
    #         fontsize=AXIS_LABEL_FONT_SIZE,
    #     )
    #     ax.set_ylabel(
    #         ylabel,
    #         fontsize=AXIS_LABEL_FONT_SIZE,
    #     )

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
            [
                x - 0.5
                for x in range(1, len(x_labels))
            ],
            minor=True,
        )

        ax.set_yticks(
            [
                y - 0.5
                for y in range(1, len(y_labels))
            ],
            minor=True,
        )

        ax.grid(
            which="minor",
            color=GRID_LINE_COLOR,
            linestyle="-",
            linewidth=GRID_LINE_WIDTH,
        )

        ax.tick_params(
            which="minor",
            bottom=False,
            left=False,
        )

    for row_index, row_values in enumerate(values):
        for col_index, value in enumerate(row_values):
            if math.isnan(value):
                continue

            if annotations is not None:
                annotation_text = annotations[
                    row_index
                ][col_index]
            else:
                annotation_text = format(
                    value,
                    annot_format,
                )

            if annotation_text == "":
                continue

            text_color = get_annotation_text_color(
                image=im,
                value=value,
            )

            ax.text(
                col_index,
                row_index,
                annotation_text,
                ha="center",
                va="center",
                fontsize=ANNOTATION_FONT_SIZE,
                fontweight="bold",
                color=text_color,
            )

    output_pdf_path.parent.mkdir(
        parents=True,
        exist_ok=True,
    )

    fig.tight_layout()

    fig.savefig(
        output_pdf_path,
        format="pdf",
        bbox_inches="tight",
    )

    print(f"Saved PDF: {output_pdf_path}")

    if save_png:
        output_png_path = output_pdf_path.with_suffix(
            ".png"
        )

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
        description=(
            "Create a heatmap PDF from a CSV file."
        )
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
        "--annotation-csv",
        type=Path,
        default=None,
        help=(
            "Optional annotation CSV file. "
            "If specified, cell texts are read "
            "from this CSV."
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
        help=(
            f'Figure title. Default: "{DEFAULT_TITLE}"'
        ),
    )

    parser.add_argument(
        "--xlabel",
        type=str,
        default=DEFAULT_XLABEL,
        help=(
            f'X-axis label. Default: "{DEFAULT_XLABEL}"'
        ),
    )

    parser.add_argument(
        "--ylabel",
        type=str,
        default=DEFAULT_YLABEL,
        help=(
            f'Y-axis label. Default: "{DEFAULT_YLABEL}"'
        ),
    )

    parser.add_argument(
        "--colorbar-label",
        type=str,
        default=DEFAULT_COLORBAR_LABEL,
        help=(
            "Colorbar label. "
            f'Default: "{DEFAULT_COLORBAR_LABEL}"'
        ),
    )

    parser.add_argument(
        "--cmap",
        type=str,
        default=DEFAULT_CMAP,
        help=(
            "Matplotlib colormap name. "
            f'Default: "{DEFAULT_CMAP}"'
        ),
    )

    parser.add_argument(
        "--vmin",
        type=float,
        default=None,
        help=(
            "Minimum value for color scale. "
            "Default: CSV minimum."
        ),
    )

    parser.add_argument(
        "--vmax",
        type=float,
        default=None,
        help=(
            "Maximum value for color scale. "
            "Default: CSV maximum."
        ),
    )

    parser.add_argument(
        "--fig-width",
        type=float,
        default=FIG_WIDTH,
        help=(
            f"Figure width. Default: {FIG_WIDTH}"
        ),
    )

    parser.add_argument(
        "--fig-height",
        type=float,
        default=FIG_HEIGHT,
        help=(
            f"Figure height. Default: {FIG_HEIGHT}"
        ),
    )

    parser.add_argument(
        "--annot-format",
        type=str,
        default=VALUE_FORMAT,
        help=(
            "Annotation number format. "
            f'Default: "{VALUE_FORMAT}"'
        ),
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

    parser.add_argument(
        "--no-title",
        action="store_true",
        help="Hide figure title.",
    )

    parser.add_argument(
        "--no-axis-labels",
        action="store_true",
        help="Hide x/y axis labels.",
    )

    return parser.parse_args()


def main() -> None:
    """
    メイン処理。
    """
    args = parse_args()

    input_csv_path = resolve_input_csv_path(
        args.csv_path
    )

    annotation_csv_path = resolve_optional_csv_path(
        args.annotation_csv
    )

    output_pdf_path = resolve_output_pdf_path(
        csv_path=input_csv_path,
        output=args.output,
    )

    x_labels, y_labels, values = load_heatmap_csv(
        input_csv_path
    )

    annotations: list[list[str]] | None = None

    if annotation_csv_path is not None:
        annotations = load_annotation_csv(
            csv_path=annotation_csv_path,
            expected_x_labels=x_labels,
            expected_y_labels=y_labels,
        )

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
        annotations=annotations,
        show_title=not args.no_title,
        show_axis_labels=not args.no_axis_labels,
    )


if __name__ == "__main__":
    main()
# #!/usr/bin/env python3
# from __future__ import annotations

# import argparse
# import csv
# import math
# import re
# from pathlib import Path

# import matplotlib.pyplot as plt


# # ==================================================
# # Default style settings
# # ==================================================
# DEFAULT_INPUT_CSV = "heatmap_evacuation_completion_time.csv"

# DEFAULT_TITLE = "Evacuation Completion Time"
# DEFAULT_XLABEL = "p_follow"
# DEFAULT_YLABEL = "active_route_change_threshold_center"
# DEFAULT_COLORBAR_LABEL = "Completion Time [s]"

# # 小さい値を濃い赤にしたい場合は "Reds_r"
# # 大きい値を濃い赤にしたい場合は "Reds"
# DEFAULT_CMAP = "Reds_r"

# FIG_WIDTH = 10.0
# FIG_HEIGHT = 7.0

# ANNOTATION_FONT_SIZE = 13
# TITLE_FONT_SIZE = 13
# AXIS_LABEL_FONT_SIZE = 13
# TICK_LABEL_FONT_SIZE = 13
# COLORBAR_LABEL_FONT_SIZE = 13

# VALUE_FORMAT = ".2f"
# DEFAULT_DPI = 300

# X_TICK_ROTATION = 0
# Y_TICK_ROTATION = 0

# SHOW_GRID_LINES = False
# GRID_LINE_COLOR = "white"
# GRID_LINE_WIDTH = 0.8


# def is_empty_row(row: list[str]) -> bool:
#     """
#     CSV の空行を判定する。
#     """
#     return all(cell.strip() == "" for cell in row)


# def parse_float_or_nan(value: str) -> float:
#     """
#     空セルは math.nan として扱う。

#     数値だけのセル:
#         "2050.24" -> 2050.24

#     注釈付きセル:
#         "80.6\\nN49 M51" -> 80.6
#         "80.6
#          N49 M51" -> 80.6

#     数値で始まらないセル:
#         "N49 M51" -> math.nan
#     """
#     value = value.strip()

#     if value == "":
#         return math.nan

#     # 通常の数値セル
#     try:
#         return float(value)
#     except ValueError:
#         pass

#     # セル内に実改行または "\n" 文字列がある場合でも、先頭行の数値を拾う
#     normalized_value = value.replace("\\n", "\n")
#     first_line = normalized_value.splitlines()[0].strip()

#     match = re.match(
#         r"^[+-]?(?:\d+(?:\.\d*)?|\.\d+)(?:[eE][+-]?\d+)?",
#         first_line,
#     )

#     if match is None:
#         return math.nan

#     try:
#         return float(match.group(0))
#     except ValueError:
#         return math.nan


# def normalize_annotation_text(value: str) -> str:
#     """
#     注釈CSVのセル文字列を matplotlib 表示用に整形する。

#     "80.6\\nN49 M51" のように、バックスラッシュ+n で保存されている場合も
#     実改行に変換する。
#     """
#     return value.strip().replace("\\n", "\n")


# def has_finite_value(values: list[list[float]]) -> bool:
#     """
#     heatmap に描画可能な数値が1つでもあるか確認する。
#     """
#     for row in values:
#         for value in row:
#             if not math.isnan(value):
#                 return True

#     return False


# def resolve_input_csv_path(csv_path: Path) -> Path:
#     """
#     入力 CSV のパスを解決する。

#     まず現在の作業ディレクトリ基準で探す。
#     見つからない場合は、このスクリプトと同一階層から探す。
#     """
#     if csv_path.exists():
#         return csv_path.resolve()

#     script_dir = Path(__file__).resolve().parent
#     script_relative_path = script_dir / csv_path

#     if script_relative_path.exists():
#         return script_relative_path.resolve()

#     raise FileNotFoundError(f"CSV file not found: {csv_path}")


# def resolve_optional_csv_path(csv_path: Path | None) -> Path | None:
#     """
#     任意指定の CSV パスを解決する。
#     """
#     if csv_path is None:
#         return None

#     return resolve_input_csv_path(csv_path)


# def build_default_output_path(csv_path: Path) -> Path:
#     """
#     入力 CSV に対応するデフォルト出力 PDF パスを作る。
#     """
#     return csv_path.with_name(f"{csv_path.stem}_custom.pdf")


# def resolve_output_pdf_path(csv_path: Path, output: Path | None) -> Path:
#     """
#     出力 PDF のパスを解決する。

#     --output が省略された場合:
#         入力 CSV と同じ階層に *_custom.pdf を保存する。

#     --output が相対パスの場合:
#         入力 CSV と同じ階層からの相対パスとして扱う。
#     """
#     if output is None:
#         return build_default_output_path(csv_path)

#     if output.is_absolute():
#         return output

#     return csv_path.parent / output


# def load_heatmap_csv(csv_path: Path) -> tuple[list[str], list[str], list[list[float]]]:
#     """
#     CSV を読み込み、x 軸ラベル、y 軸ラベル、値の2次元リストを返す。

#     CSV の仕様:
#         1行目:
#             1列目 = y軸名
#             2列目以降 = x軸ラベル

#         2行目以降:
#             1列目 = y軸ラベル
#             2列目以降 = heatmap の値

#     空セルは math.nan として扱う。
#     """
#     with csv_path.open("r", encoding="utf-8", newline="") as f:
#         reader = csv.reader(f)
#         rows = [row for row in reader if not is_empty_row(row)]

#     if not rows:
#         raise ValueError(f"CSV is empty: {csv_path}")

#     header = rows[0]

#     if len(header) < 2:
#         raise ValueError(
#             "CSV header must contain at least one y-axis column "
#             "and one x-axis label."
#         )

#     x_labels = [cell.strip() for cell in header[1:]]

#     y_labels: list[str] = []
#     values: list[list[float]] = []

#     expected_value_count = len(x_labels)

#     for row_index, row in enumerate(rows[1:], start=2):
#         if len(row) == 0 or is_empty_row(row):
#             continue

#         y_label = row[0].strip()

#         if y_label == "":
#             continue

#         value_cells = row[1:]

#         if len(value_cells) < expected_value_count:
#             value_cells = value_cells + [""] * (expected_value_count - len(value_cells))

#         if len(value_cells) > expected_value_count:
#             value_cells = value_cells[:expected_value_count]

#         row_values = [parse_float_or_nan(cell) for cell in value_cells]

#         y_labels.append(y_label)
#         values.append(row_values)

#     if not y_labels:
#         raise ValueError(f"No data rows found in CSV: {csv_path}")

#     return x_labels, y_labels, values


# def load_annotation_csv(
#     csv_path: Path,
#     expected_x_labels: list[str],
#     expected_y_labels: list[str],
# ) -> list[list[str]]:
#     """
#     注釈用 CSV を読み込む。

#     想定する CSV 形式は heatmap CSV と同じ。

#     例:
#         active_route_change_threshold_center,0.10,0.15
#         100.0,"111.8
#         N50 M50","112.6
#         N48 M52"

#     セル内改行を含む場合、csv.writer で保存されていれば自動で読める。
#     """
#     with csv_path.open("r", encoding="utf-8", newline="") as f:
#         reader = csv.reader(f)
#         rows = [row for row in reader if not is_empty_row(row)]

#     if not rows:
#         raise ValueError(f"Annotation CSV is empty: {csv_path}")

#     header = rows[0]

#     if len(header) < 2:
#         raise ValueError(
#             "Annotation CSV header must contain at least one y-axis column "
#             "and one x-axis label."
#         )

#     x_labels = [cell.strip() for cell in header[1:]]

#     if x_labels != expected_x_labels:
#         raise ValueError(
#             "Annotation CSV x labels do not match input CSV x labels.\n"
#             f"input      = {expected_x_labels}\n"
#             f"annotation = {x_labels}"
#         )

#     annotation_y_labels: list[str] = []
#     annotations: list[list[str]] = []

#     expected_value_count = len(expected_x_labels)

#     for row_index, row in enumerate(rows[1:], start=2):
#         if len(row) == 0 or is_empty_row(row):
#             continue

#         y_label = row[0].strip()

#         if y_label == "":
#             continue

#         value_cells = row[1:]

#         if len(value_cells) < expected_value_count:
#             value_cells = value_cells + [""] * (expected_value_count - len(value_cells))

#         if len(value_cells) > expected_value_count:
#             value_cells = value_cells[:expected_value_count]

#         row_annotations = [normalize_annotation_text(cell) for cell in value_cells]

#         annotation_y_labels.append(y_label)
#         annotations.append(row_annotations)

#     if annotation_y_labels != expected_y_labels:
#         raise ValueError(
#             "Annotation CSV y labels do not match input CSV y labels.\n"
#             f"input      = {expected_y_labels}\n"
#             f"annotation = {annotation_y_labels}"
#         )

#     return annotations


# def create_heatmap(
#     x_labels: list[str],
#     y_labels: list[str],
#     values: list[list[float]],
#     output_pdf_path: Path,
#     title: str,
#     xlabel: str,
#     ylabel: str,
#     colorbar_label: str,
#     cmap: str,
#     vmin: float | None,
#     vmax: float | None,
#     fig_width: float,
#     fig_height: float,
#     annot_format: str,
#     save_png: bool,
#     dpi: int,
#     annotations: list[list[str]] | None = None,
#     show_title: bool = True,
#     show_axis_labels: bool = True,
# ) -> None:
#     """
#     matplotlib を使って heatmap を作成し、
#     PDF と必要に応じて PNG を保存する。

#     annotations が指定された場合:
#         heatmap の色は values を使う。
#         セル上の文字は annotations を使う。

#     annotations が指定されない場合:
#         セル上の文字は values を annot_format で表示する。
#     """
#     if not x_labels:
#         raise ValueError("x_labels is empty")

#     if not y_labels:
#         raise ValueError("y_labels is empty")

#     if not values:
#         raise ValueError("values is empty")

#     if not has_finite_value(values):
#         raise ValueError(
#             "No numeric values found in the input CSV. "
#             "heatmap の値がすべて NaN として読まれています。"
#             "注釈CSVをメインCSVとして渡していないか確認してください。"
#         )

#     if annotations is not None:
#         if len(annotations) != len(values):
#             raise ValueError(
#                 "annotations row count does not match values row count: "
#                 f"annotations={len(annotations)}, values={len(values)}"
#             )

#         for row_index, (annotation_row, value_row) in enumerate(
#             zip(annotations, values)
#         ):
#             if len(annotation_row) != len(value_row):
#                 raise ValueError(
#                     "annotations column count does not match values column count "
#                     f"at row {row_index}: "
#                     f"annotations={len(annotation_row)}, values={len(value_row)}"
#                 )

#     fig, ax = plt.subplots(figsize=(fig_width, fig_height))

#     im = ax.imshow(
#         values,
#         aspect="auto",
#         cmap="Reds",
#         vmin=vmin,
#         vmax=vmax,
#     )

#     im = ax.imshow(
#         values,
#         aspect="auto",
#         cmap=cmap,
#         vmin=vmin,
#         vmax=vmax,
#     )

#     colorbar = fig.colorbar(
#         im,
#         ax=ax,
#         orientation="vertical",
#         pad=0.05,
#     )
#     # colorbar.set_label(colorbar_label, fontsize=COLORBAR_LABEL_FONT_SIZE)
#     # colorbar.ax.invert_yaxis()
#     colorbar.ax.tick_params(labelsize=TICK_LABEL_FONT_SIZE)

#     # if show_title:
#     #     ax.set_title(title, fontsize=TITLE_FONT_SIZE)

#     # if show_axis_labels:
#     #     ax.set_xlabel(xlabel, fontsize=AXIS_LABEL_FONT_SIZE)
#     #     ax.set_ylabel(ylabel, fontsize=AXIS_LABEL_FONT_SIZE)

#     ax.set_xticks(range(len(x_labels)))
#     ax.set_yticks(range(len(y_labels)))

#     ax.set_xticklabels(
#         x_labels,
#         fontsize=TICK_LABEL_FONT_SIZE,
#         rotation=X_TICK_ROTATION,
#     )
#     ax.set_yticklabels(
#         y_labels,
#         fontsize=TICK_LABEL_FONT_SIZE,
#         rotation=Y_TICK_ROTATION,
#     )

#     if SHOW_GRID_LINES:
#         ax.set_xticks(
#             [x - 0.5 for x in range(1, len(x_labels))],
#             minor=True,
#         )
#         ax.set_yticks(
#             [y - 0.5 for y in range(1, len(y_labels))],
#             minor=True,
#         )
#         ax.grid(
#             which="minor",
#             color=GRID_LINE_COLOR,
#             linestyle="-",
#             linewidth=GRID_LINE_WIDTH,
#         )
#         ax.tick_params(which="minor", bottom=False, left=False)

#     for row_index, row_values in enumerate(values):
#         for col_index, value in enumerate(row_values):
#             if math.isnan(value):
#                 continue

#             if annotations is not None:
#                 annotation_text = annotations[row_index][col_index]
#             else:
#                 annotation_text = format(value, annot_format)

#             if annotation_text == "":
#                 continue

#             ax.text(
#                 col_index,
#                 row_index,
#                 annotation_text,
#                 ha="center",
#                 va="center",
#                 fontsize=ANNOTATION_FONT_SIZE,
#                 fontweight="bold",
#             )

#     output_pdf_path.parent.mkdir(parents=True, exist_ok=True)

#     fig.tight_layout()
#     fig.savefig(output_pdf_path, format="pdf", bbox_inches="tight")

#     print(f"Saved PDF: {output_pdf_path}")

#     if save_png:
#         output_png_path = output_pdf_path.with_suffix(".png")
#         fig.savefig(
#             output_png_path,
#             format="png",
#             dpi=dpi,
#             bbox_inches="tight",
#         )
#         print(f"Saved PNG: {output_png_path}")

#     plt.close(fig)


# def get_annotation_text_color(
#     image,
#     value: float,
#     luminance_threshold: float = 0.55,
# ) -> str:
#     """
#     heatmap のセル背景色に応じて、注釈文字の色を返す。

#     背景色が暗い場合:
#         "white"

#     背景色が明るい場合:
#         "black"
#     """
#     normalized_value = image.norm(value)
#     red, green, blue, _ = image.cmap(normalized_value)

#     # 人間の視覚特性を考慮した相対輝度
#     luminance = (
#         0.2126 * red
#         + 0.7152 * green
#         + 0.0722 * blue
#     )

#     if luminance < luminance_threshold:
#         return "white"

#     return "black"


# def parse_args() -> argparse.Namespace:
#     """
#     コマンドライン引数を解析する。
#     """
#     parser = argparse.ArgumentParser(
#         description="Create a heatmap PDF from a CSV file."
#     )

#     parser.add_argument(
#         "csv_path",
#         nargs="?",
#         type=Path,
#         default=Path(DEFAULT_INPUT_CSV),
#         help=(
#             "Input CSV file. "
#             f"Default: {DEFAULT_INPUT_CSV}"
#         ),
#     )

#     parser.add_argument(
#         "--annotation-csv",
#         type=Path,
#         default=None,
#         help=(
#             "Optional annotation CSV file. "
#             "If specified, cell texts are read from this CSV."
#         ),
#     )

#     parser.add_argument(
#         "--output",
#         type=Path,
#         default=None,
#         help=(
#             "Output PDF file. "
#             "Default: <input_csv_stem>_custom.pdf"
#         ),
#     )

#     parser.add_argument(
#         "--title",
#         type=str,
#         default=DEFAULT_TITLE,
#         help=f'Figure title. Default: "{DEFAULT_TITLE}"',
#     )

#     parser.add_argument(
#         "--xlabel",
#         type=str,
#         default=DEFAULT_XLABEL,
#         help=f'X-axis label. Default: "{DEFAULT_XLABEL}"',
#     )

#     parser.add_argument(
#         "--ylabel",
#         type=str,
#         default=DEFAULT_YLABEL,
#         help=f'Y-axis label. Default: "{DEFAULT_YLABEL}"',
#     )

#     parser.add_argument(
#         "--colorbar-label",
#         type=str,
#         default=DEFAULT_COLORBAR_LABEL,
#         help=f'Colorbar label. Default: "{DEFAULT_COLORBAR_LABEL}"',
#     )

#     parser.add_argument(
#         "--cmap",
#         type=str,
#         default=DEFAULT_CMAP,
#         help=f'Matplotlib colormap name. Default: "{DEFAULT_CMAP}"',
#     )

#     parser.add_argument(
#         "--vmin",
#         type=float,
#         default=None,
#         help="Minimum value for color scale. Default: CSV minimum.",
#     )

#     parser.add_argument(
#         "--vmax",
#         type=float,
#         default=None,
#         help="Maximum value for color scale. Default: CSV maximum.",
#     )

#     parser.add_argument(
#         "--fig-width",
#         type=float,
#         default=FIG_WIDTH,
#         help=f"Figure width. Default: {FIG_WIDTH}",
#     )

#     parser.add_argument(
#         "--fig-height",
#         type=float,
#         default=FIG_HEIGHT,
#         help=f"Figure height. Default: {FIG_HEIGHT}",
#     )

#     parser.add_argument(
#         "--annot-format",
#         type=str,
#         default=VALUE_FORMAT,
#         help=f'Annotation number format. Default: "{VALUE_FORMAT}"',
#     )

#     parser.add_argument(
#         "--png",
#         action="store_true",
#         help="Also save PNG output.",
#     )

#     parser.add_argument(
#         "--dpi",
#         type=int,
#         default=DEFAULT_DPI,
#         help=f"PNG DPI. Default: {DEFAULT_DPI}",
#     )

#     parser.add_argument(
#         "--no-title",
#         action="store_true",
#         help="Hide figure title.",
#     )

#     parser.add_argument(
#         "--no-axis-labels",
#         action="store_true",
#         help="Hide x/y axis labels.",
#     )

#     return parser.parse_args()


# def main() -> None:
#     """
#     メイン処理。
#     """
#     args = parse_args()

#     input_csv_path = resolve_input_csv_path(args.csv_path)
#     annotation_csv_path = resolve_optional_csv_path(args.annotation_csv)

#     output_pdf_path = resolve_output_pdf_path(
#         csv_path=input_csv_path,
#         output=args.output,
#     )

#     x_labels, y_labels, values = load_heatmap_csv(input_csv_path)

#     annotations = None

#     if annotation_csv_path is not None:
#         annotations = load_annotation_csv(
#             csv_path=annotation_csv_path,
#             expected_x_labels=x_labels,
#             expected_y_labels=y_labels,
#         )

#     create_heatmap(
#         x_labels=x_labels,
#         y_labels=y_labels,
#         values=values,
#         output_pdf_path=output_pdf_path,
#         title=args.title,
#         xlabel=args.xlabel,
#         ylabel=args.ylabel,
#         colorbar_label=args.colorbar_label,
#         cmap=args.cmap,
#         vmin=args.vmin,
#         vmax=args.vmax,
#         fig_width=args.fig_width,
#         fig_height=args.fig_height,
#         annot_format=args.annot_format,
#         save_png=args.png,
#         dpi=args.dpi,
#         annotations=annotations,
#         show_title=not args.no_title,
#         show_axis_labels=not args.no_axis_labels,
#     )


# if __name__ == "__main__":
#     main()