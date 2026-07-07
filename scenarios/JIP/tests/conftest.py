import ast
import random
import sys
from pathlib import Path

import numpy as np
import pytest

from scenarios.dicomo2026.tests.data.test_agent_ids import TEST_VEH_IDS
from scenarios.dicomo2026.tests.data.test_shelter_edges import (
    TEST_EDGE_ID_BY_SHELTER_ID,
)

# ============================================================
# Import path setup
# ============================================================

PROJECT_ROOT = Path(__file__).resolve().parents[3]

if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))


# ============================================================
# init_agent_list import
# ============================================================
# runner_simulator.py は TraCI / SUMO や実行処理を含むため，
# pytest の conftest.py では import しない。
# init_agent_list() だけを initialization から直接 import する。
# ============================================================

from evacsim.sim.initialization import init_agent_list


# ============================================================
# TOML loader
# ============================================================

def _load_simple_toml(path: Path) -> dict:
    config = {}

    for raw_line in path.read_text(encoding="utf-8").splitlines():
        line = raw_line.strip()

        if not line or line.startswith("#"):
            continue

        if "#" in line:
            line = line.split("#", 1)[0].strip()

        if "=" not in line:
            continue

        key, value = line.split("=", 1)
        key = key.strip()
        value = value.strip()

        try:
            config[key] = ast.literal_eval(value)
        except (ValueError, SyntaxError):
            config[key] = value.strip('"').strip("'")

    return config


def _load_toml(path: Path) -> dict:
    try:
        import tomllib

        with path.open("rb") as f:
            return tomllib.load(f)
    except ModuleNotFoundError:
        pass

    try:
        import tomli

        with path.open("rb") as f:
            return tomli.load(f)
    except ModuleNotFoundError:
        return _load_simple_toml(path)


@pytest.fixture(scope="session")
def test_config() -> dict:
    config_path = Path(__file__).resolve().parent / "data" / "config_scenario1_test.toml"
    return _load_toml(config_path)


@pytest.fixture(scope="session")
def test_veh_ids() -> list[str]:
    return TEST_VEH_IDS


@pytest.fixture(scope="session")
def test_edge_id_by_shelter_id() -> dict[str, list[str]]:
    return TEST_EDGE_ID_BY_SHELTER_ID


@pytest.fixture()
def agent_list(
    test_config: dict,
    test_veh_ids: list[str],
    test_edge_id_by_shelter_id: dict[str, list[str]],
):
    random.seed(0)
    np.random.seed(0)

    agents = init_agent_list(
        vehIDs=test_veh_ids,
        edgeID_by_shelterID=test_edge_id_by_shelter_id,
        ATTR_RATE=test_config["attr_rate"],

        ACTIVE_ROUTE_CHANGE_THRESHOLD_CENTER=test_config[
            "active_route_change_threshold_center"
        ],
        ACTIVE_ROUTE_CHANGE_THRESHOLD_SPREAD=test_config[
            "active_route_change_threshold_spread"
        ],
        CAUTIOUS_ROUTE_CHANGE_THRESHOLD_CENTER=test_config[
            "cautious_route_change_threshold_center"
        ],
        CAUTIOUS_ROUTE_CHANGE_THRESHOLD_SPREAD=test_config[
            "cautious_route_change_threshold_spread"
        ],

        NORMALCY_VALUE_ABOUT_ROUTE_CHANGE_CENTER=test_config[
            "normalcy_value_about_route_change_center"
        ],
        NORMALCY_VALUE_ABOUT_ROUTE_CHANGE_SPREAD=test_config[
            "normalcy_value_about_route_change_spread"
        ],

        ACTIVE_WRONG_WAY_DRIVING_THRESHOLD_CENTER=test_config[
            "active_wrong_way_driving_threshold_center"
        ],
        ACTIVE_WRONG_WAY_DRIVING_THRESHOLD_SPREAD=test_config[
            "active_wrong_way_driving_threshold_spread"
        ],
        CAUTIOUS_WRONG_WAY_DRIVING_THRESHOLD_CENTER=test_config[
            "cautious_wrong_way_driving_threshold_center"
        ],
        CAUTIOUS_WRONG_WAY_DRIVING_THRESHOLD_SPREAD=test_config[
            "cautious_wrong_way_driving_threshold_spread"
        ],

        MIN_MOTIVATION_START=test_config["min_motivation_start"],
        MIN_MOTIVATION_END=test_config["min_motivation_end"],

        ACTIVE_NORMALCY_VALUE_ABOUT_TSUNAMI_PRECURSOR_INFO_CENTER=test_config[
            "active_normalcy_value_about_tsunami_precursor_info_center"
        ],
        ACTIVE_NORMALCY_VALUE_ABOUT_TSUNAMI_PRECURSOR_INFO_SPREAD=test_config[
            "active_normalcy_value_about_tsunami_precursor_info_spread"
        ],
        CAUTIOUS_NORMALCY_VALUE_ABOUT_TSUNAMI_PRECURSOR_INFO_CENTER=test_config[
            "cautious_normalcy_value_about_tsunami_precursor_info_center"
        ],
        CAUTIOUS_NORMALCY_VALUE_ABOUT_TSUNAMI_PRECURSOR_INFO_SPREAD=test_config[
            "cautious_normalcy_value_about_tsunami_precursor_info_spread"
        ],

        ACTIVE_NORMALCY_VALUE_ABOUT_ROUTE_CONGESTION_INFO_CENTER=test_config[
            "active_normalcy_value_about_route_congestion_info_center"
        ],
        ACTIVE_NORMALCY_VALUE_ABOUT_ROUTE_CONGESTION_INFO_SPREAD=test_config[
            "active_normalcy_value_about_route_congestion_info_spread"
        ],
        CAUTIOUS_NORMALCY_VALUE_ABOUT_ROUTE_CONGESTION_INFO_CENTER=test_config[
            "cautious_normalcy_value_about_route_congestion_info_center"
        ],
        CAUTIOUS_NORMALCY_VALUE_ABOUT_ROUTE_CONGESTION_INFO_SPREAD=test_config[
            "cautious_normalcy_value_about_route_congestion_info_spread"
        ],

        ACTIVE_NORMALCY_VALUE_ABOUT_SHELTER_FULL_INFO_CENTER=test_config[
            "active_normalcy_value_about_shelter_full_info_center"
        ],
        ACTIVE_NORMALCY_VALUE_ABOUT_SHELTER_FULL_INFO_SPREAD=test_config[
            "active_normalcy_value_about_shelter_full_info_spread"
        ],
        CAUTIOUS_NORMALCY_VALUE_ABOUT_SHELTER_FULL_INFO_CENTER=test_config[
            "cautious_normalcy_value_about_shelter_full_info_center"
        ],
        CAUTIOUS_NORMALCY_VALUE_ABOUT_SHELTER_FULL_INFO_SPREAD=test_config[
            "cautious_normalcy_value_about_shelter_full_info_spread"
        ],

        ACTIVE_MAJORITY_INCREASE_VALUE_CENTER=test_config[
            "active_majority_increase_value_center"
        ],
        ACTIVE_MAJORITY_INCREASE_VALUE_SPREAD=test_config[
            "active_majority_increase_value_spread"
        ],
        CAUTIOUS_MAJORITY_INCREASE_VALUE_CENTER=test_config[
            "cautious_majority_increase_value_center"
        ],
        CAUTIOUS_MAJORITY_INCREASE_VALUE_SPREAD=test_config[
            "cautious_majority_increase_value_spread"
        ],

        ACTIVE_MAJORITY_DECREASE_VALUE_CENTER=test_config[
            "active_majority_decrease_value_center"
        ],
        ACTIVE_MAJORITY_DECREASE_VALUE_SPREAD=test_config[
            "active_majority_decrease_value_spread"
        ],
        CAUTIOUS_MAJORITY_DECREASE_VALUE_CENTER=test_config[
            "cautious_majority_decrease_value_center"
        ],
        CAUTIOUS_MAJORITY_DECREASE_VALUE_SPREAD=test_config[
            "cautious_majority_decrease_value_spread"
        ],

        ACTIVE_SHELTER_OCCUPANCY_RATE_THRESHOLD_START=test_config[
            "active_shelter_occupancy_rate_threshold_start"
        ],
        ACTIVE_SHELTER_OCCUPANCY_RATE_THRESHOLD_END=test_config[
            "active_shelter_occupancy_rate_threshold_end"
        ],
        CAUTIOUS_SHELTER_OCCUPANCY_RATE_THRESHOLD_START=test_config[
            "cautious_shelter_occupancy_rate_threshold_start"
        ],
        CAUTIOUS_SHELTER_OCCUPANCY_RATE_THRESHOLD_END=test_config[
            "cautious_shelter_occupancy_rate_threshold_end"
        ],
    )

    return agents

# @pytest.fixture()
# set_motivation_curve_dicts_to_agents(agent_list, rho, delta):