from __future__ import annotations

import csv
import itertools
import sys
from copy import deepcopy
from pathlib import Path
from typing import Any


# ==================================================
# generate_config.py が置かれているディレクトリ
# ==================================================

SCRIPT_DIR = Path(__file__).resolve().parent

# 注意:
# 出力ファイル名を config_scenario_1.toml, config_scenario_2.toml ...
# にするため、ベースファイルは別名にする
BASE_CONFIG_PATH = SCRIPT_DIR / "config_scenario_base.toml"

# 生成ファイルは generate_config.py と同じ階層に作成する
OUTPUT_DIR = SCRIPT_DIR

# scenario_id の開始番号
SCENARIO_ID_START = 1


# ==================================================
# パラメータスイープ設定
# ==================================================

# parameter_grid = {
#     "p_follow": [0.0, 0.25, 0.5, 0.75],
#     "active_route_change_threshold_center": [150.0, 300.0, 450.0],
#     "active_route_change_threshold_spread": [50.0, 100.0],
# }

# 5条件の例
parameter_grid = {
    "choice_shortest_route_rate": [0.1, 0.3, 0.5, 0.7, 0.9],
}


# ==================================================
# TOML 読み込み・書き込み
# ==================================================

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


def write_toml(data: dict[str, Any], path: Path) -> None:
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

    with path.open("wb") as f:
        tomli_w.dump(data, f)


# ==================================================
# バリデーション
# ==================================================

def validate_parameter_grid(
    base_config: dict[str, Any],
    parameter_grid: dict[str, list[Any]],
) -> None:
    if not parameter_grid:
        raise ValueError("parameter_grid が空です。少なくとも1つのパラメータを指定してください。")

    if "scenario_id" not in base_config:
        raise KeyError("ベースTOMLに 'scenario_id' が存在しません。")

    for key, values in parameter_grid.items():
        if key == "scenario_id":
            raise ValueError(
                "'scenario_id' は自動で連番を振るため、parameter_grid には指定しないでください。"
            )

        if key not in base_config:
            raise KeyError(
                f"parameter_grid に指定されたキー '{key}' は "
                f"ベースTOMLに存在しません。"
            )

        if not isinstance(values, list):
            raise TypeError(f"parameter_grid['{key}'] は list で指定してください。")

        if len(values) == 0:
            raise ValueError(f"parameter_grid['{key}'] の値リストが空です。")


# ==================================================
# config 生成
# ==================================================

def generate_configs(
    base_config_path: Path,
    output_dir: Path,
    parameter_grid: dict[str, list[Any]],
) -> None:
    if not base_config_path.exists():
        raise FileNotFoundError(
            f"ベース設定ファイルが見つかりません: {base_config_path}"
        )

    base_config = load_toml(base_config_path)

    validate_parameter_grid(base_config, parameter_grid)

    output_dir.mkdir(parents=True, exist_ok=True)

    param_names = list(parameter_grid.keys())
    param_values_list = [parameter_grid[name] for name in param_names]

    combinations = list(itertools.product(*param_values_list))
    total_conditions = len(combinations)

    conditions_csv_path = output_dir / "conditions.csv"

    generated_rows: list[dict[str, Any]] = []

    for condition_index, values in enumerate(combinations):
        condition_id = condition_index + 1
        scenario_id = SCENARIO_ID_START + condition_index

        config = deepcopy(base_config)

        condition_dict = dict(zip(param_names, values))

        for key, value in condition_dict.items():
            config[key] = value

        # scenario_id を連番にする
        config["scenario_id"] = scenario_id

        # ==================================================
        # ファイル名規則:
        # config_scenario_1.toml
        # config_scenario_2.toml
        # config_scenario_3.toml
        # ...
        # ==================================================
        config_filename = f"config_scenario_{scenario_id}.toml"
        config_path = output_dir / config_filename

        # ベースファイルを誤って上書きしないための安全チェック
        if config_path.resolve() == base_config_path.resolve():
            raise FileExistsError(
                f"出力ファイル {config_path} がベースファイルと同じです。\n"
                "元ファイルを上書きしないため、ベースファイル名を "
                "'config_scenario_base.toml' などに変更してください。"
            )

        write_toml(config, config_path)

        row = {
            "condition_id": condition_id,
            "scenario_id": scenario_id,
            "config_file": config_filename,
        }
        row.update(condition_dict)
        generated_rows.append(row)

    csv_columns = ["condition_id", "scenario_id", "config_file"] + param_names

    with conditions_csv_path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=csv_columns)
        writer.writeheader()
        writer.writerows(generated_rows)

    print("========================================")
    print("Config generation completed.")
    print(f"Base config      : {base_config_path}")
    print(f"Output directory : {output_dir}")
    print(f"Conditions CSV   : {conditions_csv_path}")
    print(f"Generated files  : {total_conditions}")
    print(f"Scenario ID range: {SCENARIO_ID_START} - {SCENARIO_ID_START + total_conditions - 1}")
    print("========================================")


# ==================================================
# main
# ==================================================

if __name__ == "__main__":
    generate_configs(
        base_config_path=BASE_CONFIG_PATH,
        output_dir=OUTPUT_DIR,
        parameter_grid=parameter_grid,
    )