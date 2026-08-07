from __future__ import annotations

import csv
import itertools
import sys
from copy import deepcopy
from pathlib import Path
from typing import Any


SCRIPT_DIR = Path(__file__).resolve().parent

BASE_CONFIG_PATH = SCRIPT_DIR / "config_scenario_base.toml"
OUTPUT_DIR = SCRIPT_DIR

SCENARIO_ID_START = 1
CONDITIONS_CSV_FILENAME = "conditions_grid_6x6.csv"


# 6 × 6 = 36条件
parameter_grid = {
    "p_follow": [
        0.10,
        0.15,
        0.20,
        0.25,
        0.30,
    ],
    "active_route_change_threshold_center": [
        150.0,
        200.0,
        250.0,
        300.0,
        350.0,
    ],
}


def load_toml(path: Path) -> dict[str, Any]:
    """
    TOMLファイルを読み込む。
    """
    try:
        import tomllib  # Python 3.11+
    except ModuleNotFoundError:
        try:
            import tomli as tomllib  # Python 3.10
        except ModuleNotFoundError:
            print(
                "ERROR: Python 3.10でTOMLを読み込むには "
                "tomliが必要です。\n\n"
                "    pip install tomli tomli-w\n",
                file=sys.stderr,
            )
            raise

    with path.open("rb") as file:
        return tomllib.load(file)


def write_toml_without_overwrite(
    data: dict[str, Any],
    path: Path,
) -> None:
    """
    既存ファイルを上書きせずにTOMLを書き込む。
    """
    try:
        import tomli_w
    except ModuleNotFoundError:
        print(
            "ERROR: TOMLを書き出すにはtomli-wが必要です。\n\n"
            "    pip install tomli-w\n",
            file=sys.stderr,
        )
        raise

    with path.open("xb") as file:
        tomli_w.dump(data, file)


def validate_parameter_grid(
    base_config: dict[str, Any],
    grid: dict[str, list[Any]],
) -> None:
    """
    parameter_gridとベースconfigを検証する。
    """
    if not grid:
        raise ValueError(
            "parameter_gridが空です。"
        )

    if "scenario_id" not in base_config:
        raise KeyError(
            "ベースTOMLに'scenario_id'が存在しません。"
        )

    for parameter_name, values in grid.items():
        if parameter_name == "scenario_id":
            raise ValueError(
                "'scenario_id'は自動設定するため、"
                "parameter_gridには指定できません。"
            )

        if parameter_name not in base_config:
            raise KeyError(
                f"parameter_gridのキー'{parameter_name}'が"
                "ベースTOMLに存在しません。"
            )

        if not isinstance(values, list):
            raise TypeError(
                f"parameter_grid['{parameter_name}']は"
                "listで指定してください。"
            )

        if not values:
            raise ValueError(
                f"parameter_grid['{parameter_name}']が空です。"
            )

        normalized_values = [
            round(value, 10) if isinstance(value, float) else value
            for value in values
        ]

        if len(normalized_values) != len(set(normalized_values)):
            raise ValueError(
                f"parameter_grid['{parameter_name}']に"
                f"重複値があります: {values}"
            )


def build_combinations(
    grid: dict[str, list[Any]],
) -> tuple[list[str], list[dict[str, Any]]]:
    """
    parameter_gridから全組み合わせを作成する。
    """
    parameter_names = list(grid.keys())
    value_lists = [
        grid[parameter_name]
        for parameter_name in parameter_names
    ]

    conditions = [
        dict(zip(parameter_names, values))
        for values in itertools.product(*value_lists)
    ]

    return parameter_names, conditions


def make_config_filename(scenario_id: int) -> str:
    return f"config_scenario_{scenario_id}.toml"


def precheck_output_files(
    *,
    base_config_path: Path,
    output_dir: Path,
    scenario_ids: list[int],
    conditions_csv_path: Path,
) -> None:
    """
    生成予定ファイルが既に存在しないことを確認する。

    一部だけ生成された状態を避けるため、
    実際の生成前にすべて確認する。
    """
    base_config_resolved = base_config_path.resolve()

    existing_paths: list[Path] = []

    for scenario_id in scenario_ids:
        config_path = (
            output_dir
            / make_config_filename(scenario_id)
        )

        if config_path.resolve() == base_config_resolved:
            raise FileExistsError(
                f"出力先がベースconfigと同じです: "
                f"{config_path}"
            )

        if config_path.exists():
            existing_paths.append(config_path)

    if conditions_csv_path.exists():
        existing_paths.append(conditions_csv_path)

    if existing_paths:
        existing_text = "\n".join(
            f"  - {path}"
            for path in existing_paths
        )

        raise FileExistsError(
            "以下の出力ファイルが既に存在します。\n"
            "上書き防止のため処理を停止しました。\n"
            f"{existing_text}"
        )


def generate_configs(
    *,
    base_config_path: Path,
    output_dir: Path,
    grid: dict[str, list[Any]],
) -> None:
    """
    grid内の全組み合わせについてTOMLを生成する。
    """
    if not base_config_path.exists():
        raise FileNotFoundError(
            f"ベース設定ファイルが見つかりません: "
            f"{base_config_path}"
        )

    base_config = load_toml(base_config_path)

    validate_parameter_grid(
        base_config=base_config,
        grid=grid,
    )

    output_dir.mkdir(
        parents=True,
        exist_ok=True,
    )

    parameter_names, conditions = build_combinations(
        grid
    )

    total_conditions = len(conditions)

    scenario_ids = [
        SCENARIO_ID_START + index
        for index in range(total_conditions)
    ]

    conditions_csv_path = (
        output_dir
        / CONDITIONS_CSV_FILENAME
    )

    precheck_output_files(
        base_config_path=base_config_path,
        output_dir=output_dir,
        scenario_ids=scenario_ids,
        conditions_csv_path=conditions_csv_path,
    )

    generated_rows: list[dict[str, Any]] = []

    for condition_index, condition in enumerate(
        conditions
    ):
        scenario_id = (
            SCENARIO_ID_START
            + condition_index
        )
        condition_id = condition_index + 1

        config = deepcopy(base_config)

        for parameter_name, value in condition.items():
            config[parameter_name] = value

        config["scenario_id"] = scenario_id

        config_filename = make_config_filename(
            scenario_id
        )
        config_path = (
            output_dir
            / config_filename
        )

        write_toml_without_overwrite(
            config,
            config_path,
        )

        row: dict[str, Any] = {
            "condition_id": condition_id,
            "scenario_id": scenario_id,
            "config_file": config_filename,
        }
        row.update(condition)
        generated_rows.append(row)

        print(
            "[GENERATED]",
            f"scenario_id={scenario_id}",
            f"p_follow={condition['p_follow']:.2f}",
            "active_route_change_threshold_center="
            f"{condition['active_route_change_threshold_center']:.1f}",
            f"path={config_path}",
        )

    csv_columns = [
        "condition_id",
        "scenario_id",
        "config_file",
        *parameter_names,
    ]

    with conditions_csv_path.open(
        "x",
        newline="",
        encoding="utf-8",
    ) as file:
        writer = csv.DictWriter(
            file,
            fieldnames=csv_columns,
        )
        writer.writeheader()
        writer.writerows(generated_rows)

    scenario_id_end = (
        SCENARIO_ID_START
        + total_conditions
        - 1
    )

    print()
    print("========================================")
    print("6x6 grid config generation completed.")
    print(f"Base config      : {base_config_path}")
    print(f"Output directory : {output_dir}")
    print(f"Conditions CSV   : {conditions_csv_path}")
    print(f"Generated files  : {total_conditions}")
    print(
        "Scenario ID range: "
        f"{SCENARIO_ID_START} - {scenario_id_end}"
    )
    print("========================================")


if __name__ == "__main__":
    generate_configs(
        base_config_path=BASE_CONFIG_PATH,
        output_dir=OUTPUT_DIR,
        grid=parameter_grid,
    )