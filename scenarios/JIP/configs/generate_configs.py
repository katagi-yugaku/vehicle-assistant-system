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

SCENARIO_ID_START = 31
CONDITIONS_CSV_FILENAME = "conditions_additional.csv"


# 最終的に扱いたい全体 grid
parameter_grid = {
    "p_follow": [0.10, 0.15, 0.20, 0.25, 0.30, 0.35, 0.40],
    "active_route_change_threshold_center": [
        100.0,
        150.0,
        200.0,
        250.0,
        300.0,
        350.0,
        400.0,
    ],
}


# # すでに作成済み・実行済みの grid
# existing_parameter_grid = {
#     "p_follow": [0.10, 0.15, 0.20, 0.25, 0.30],
#     "active_route_change_threshold_center": [
#         200.0,
#         250.0,
#         300.0,
#         350.0,
#         400.0,
#         450.0,
#     ],
# }

# 追加でデータあり

def load_toml(path: Path) -> dict[str, Any]:
    try:
        import tomllib  # Python 3.11+
    except ModuleNotFoundError:
        try:
            import tomli as tomllib  # Python 3.10
        except ModuleNotFoundError:
            print(
                "ERROR: Python 3.10 で TOML を読み込むには tomli が必要です。\n"
                "以下を実行してください:\n\n"
                "    pip install tomli tomli-w\n",
                file=sys.stderr,
            )
            raise

    with path.open("rb") as f:
        return tomllib.load(f)


def write_toml_without_overwrite(data: dict[str, Any], path: Path) -> None:
    try:
        import tomli_w
    except ModuleNotFoundError:
        print(
            "ERROR: TOML を書き出すには tomli-w が必要です。\n"
            "以下を実行してください:\n\n"
            "    pip install tomli-w\n",
            file=sys.stderr,
        )
        raise

    # "xb" にすることで、万一ファイルが存在していた場合も上書きしない
    with path.open("xb") as f:
        tomli_w.dump(data, f)


def normalize_value_for_compare(value: Any) -> Any:
    """
    float の比較誤差を避けるため、比較用の値を正規化する。
    """
    if isinstance(value, float):
        return round(value, 10)
    return value


def make_condition_key(
    condition: dict[str, Any],
    param_names: list[str],
) -> tuple[Any, ...]:
    """
    組み合わせ比較用の key を作る。

    dict の順序や float 誤差の影響を避けるため、
    parameter_grid 側の param_names の順序に揃えて tuple 化する。
    """
    return tuple(
        normalize_value_for_compare(condition[name])
        for name in param_names
    )


def build_combinations(
    grid: dict[str, list[Any]],
    param_names: list[str],
) -> list[dict[str, Any]]:
    """
    指定された grid から全組み合わせを作る。
    """
    value_lists = [grid[name] for name in param_names]

    combinations: list[dict[str, Any]] = []
    for values in itertools.product(*value_lists):
        condition = dict(zip(param_names, values))
        combinations.append(condition)

    return combinations


def validate_parameter_grid(
    base_config: dict[str, Any],
    grid: dict[str, list[Any]],
    grid_name: str,
) -> None:
    if not grid:
        raise ValueError(f"{grid_name} が空です。少なくとも1つのパラメータを指定してください。")

    if "scenario_id" not in base_config:
        raise KeyError("ベースTOMLに 'scenario_id' が存在しません。")

    for key, values in grid.items():
        if key == "scenario_id":
            raise ValueError(
                f"'scenario_id' は自動で連番を振るため、{grid_name} には指定しないでください。"
            )

        if key not in base_config:
            raise KeyError(
                f"{grid_name} に指定されたキー '{key}' は "
                f"ベースTOMLに存在しません。"
            )

        if not isinstance(values, list):
            raise TypeError(f"{grid_name}['{key}'] は list で指定してください。")

        if len(values) == 0:
            raise ValueError(f"{grid_name}['{key}'] の値リストが空です。")


def validate_grid_keys(
    parameter_grid: dict[str, list[Any]],
    existing_parameter_grid: dict[str, list[Any]],
) -> None:
    parameter_keys = set(parameter_grid.keys())
    existing_keys = set(existing_parameter_grid.keys())

    if parameter_keys != existing_keys:
        only_in_parameter_grid = sorted(parameter_keys - existing_keys)
        only_in_existing_grid = sorted(existing_keys - parameter_keys)

        raise ValueError(
            "parameter_grid と existing_parameter_grid のキーが一致していません。\n"
            f"parameter_grid のみに存在: {only_in_parameter_grid}\n"
            f"existing_parameter_grid のみに存在: {only_in_existing_grid}"
        )


def extract_additional_conditions(
    parameter_grid: dict[str, list[Any]],
    existing_parameter_grid: dict[str, list[Any]],
) -> list[dict[str, Any]]:
    """
    全体 grid から既存 grid の組み合わせを除外し、
    追加で生成すべき条件だけを返す。
    """
    param_names = list(parameter_grid.keys())

    all_conditions = build_combinations(parameter_grid, param_names)
    existing_conditions = build_combinations(existing_parameter_grid, param_names)

    existing_condition_keys = {
        make_condition_key(condition, param_names)
        for condition in existing_conditions
    }

    additional_conditions = [
        condition
        for condition in all_conditions
        if make_condition_key(condition, param_names) not in existing_condition_keys
    ]

    return additional_conditions


def make_config_filename(scenario_id: int) -> str:
    return f"config_scenario_{scenario_id}.toml"


def precheck_output_files(
    base_config_path: Path,
    output_dir: Path,
    scenario_ids: list[int],
    conditions_csv_path: Path,
) -> None:
    """
    生成前に出力予定ファイルをすべて確認する。

    途中まで生成してからエラーになることを避けるため、
    TOML と CSV の存在チェックを先にまとめて実行する。
    """
    base_config_resolved = base_config_path.resolve()

    for scenario_id in scenario_ids:
        config_path = output_dir / make_config_filename(scenario_id)

        if config_path.resolve() == base_config_resolved:
            raise FileExistsError(
                f"出力ファイル {config_path} がベースファイルと同じです。\n"
                "config_scenario_base.toml を上書きしないため、処理を停止します。"
            )

        if config_path.exists():
            raise FileExistsError(
                f"出力先ファイルがすでに存在します: {config_path}\n"
                "既存ファイルを上書きしないため、処理を停止します。"
            )

    if conditions_csv_path.exists():
        raise FileExistsError(
            f"条件CSVがすでに存在します: {conditions_csv_path}\n"
            "既存CSVを上書きしないため、処理を停止します。"
        )


def generate_configs(
    base_config_path: Path,
    output_dir: Path,
    parameter_grid: dict[str, list[Any]],
    existing_parameter_grid: dict[str, list[Any]],
) -> None:
    if not base_config_path.exists():
        raise FileNotFoundError(
            f"ベース設定ファイルが見つかりません: {base_config_path}"
        )

    base_config = load_toml(base_config_path)

    validate_parameter_grid(
        base_config=base_config,
        grid=parameter_grid,
        grid_name="parameter_grid",
    )
    validate_parameter_grid(
        base_config=base_config,
        grid=existing_parameter_grid,
        grid_name="existing_parameter_grid",
    )
    validate_grid_keys(
        parameter_grid=parameter_grid,
        existing_parameter_grid=existing_parameter_grid,
    )

    output_dir.mkdir(parents=True, exist_ok=True)

    param_names = list(parameter_grid.keys())

    additional_conditions = extract_additional_conditions(
        parameter_grid=parameter_grid,
        existing_parameter_grid=existing_parameter_grid,
    )

    total_conditions = len(additional_conditions)

    scenario_ids = [
        SCENARIO_ID_START + condition_index
        for condition_index in range(total_conditions)
    ]

    conditions_csv_path = output_dir / CONDITIONS_CSV_FILENAME

    precheck_output_files(
        base_config_path=base_config_path,
        output_dir=output_dir,
        scenario_ids=scenario_ids,
        conditions_csv_path=conditions_csv_path,
    )

    generated_rows: list[dict[str, Any]] = []

    for condition_index, condition_dict in enumerate(additional_conditions):
        condition_id = condition_index + 1
        scenario_id = SCENARIO_ID_START + condition_index

        config = deepcopy(base_config)

        for key, value in condition_dict.items():
            config[key] = value

        config["scenario_id"] = scenario_id

        config_filename = make_config_filename(scenario_id)
        config_path = output_dir / config_filename

        write_toml_without_overwrite(config, config_path)

        row = {
            "condition_id": condition_id,
            "scenario_id": scenario_id,
            "config_file": config_filename,
        }
        row.update(condition_dict)
        generated_rows.append(row)

    csv_columns = ["condition_id", "scenario_id", "config_file"] + param_names

    # "x" にすることで、conditions_additional.csv も上書きしない
    with conditions_csv_path.open("x", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=csv_columns)
        writer.writeheader()
        writer.writerows(generated_rows)

    if total_conditions > 0:
        scenario_id_range = (
            f"{SCENARIO_ID_START} - {SCENARIO_ID_START + total_conditions - 1}"
        )
    else:
        scenario_id_range = "N/A"

    print("========================================")
    print("Additional config generation completed.")
    print(f"Base config      : {base_config_path}")
    print(f"Output directory : {output_dir}")
    print(f"Conditions CSV   : {conditions_csv_path}")
    print(f"Generated files  : {total_conditions}")
    print(f"Scenario ID range: {scenario_id_range}")
    print("========================================")


if __name__ == "__main__":
    generate_configs(
        base_config_path=BASE_CONFIG_PATH,
        output_dir=OUTPUT_DIR,
        parameter_grid=parameter_grid,
        existing_parameter_grid=existing_parameter_grid,
    )