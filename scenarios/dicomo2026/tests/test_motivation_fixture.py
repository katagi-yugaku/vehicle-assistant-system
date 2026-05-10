import ast
import random
import sys
from pathlib import Path

import numpy as np

from scenarios.dicomo2026.tests.data.test_agent_ids import TEST_VEH_IDS
from scenarios.dicomo2026.tests.data.test_shelter_edges import (
    TEST_EDGE_ID_BY_SHELTER_ID,
)


# =========================
# Standard library
# =========================
import os
import random
import sys
from collections import Counter
from pathlib import Path
import optparse
import argparse
import datetime

# =========================
# Third-party libraries
# =========================
from evacsim.sim.sumo_env import ensure_sumo_tools_on_path

ensure_sumo_tools_on_path()

from sumolib import checkBinary  # noqa: E402
import traci  # noqa: E402
import numpy as np
from numpy import double


# =========================
# Local / intra-package
# =========================
from evacsim.agents.Agent import Agent
from evacsim.agents.Shelter import Shelter
from evacsim.agents.VehicleInfo import VehicleInfo

from evacsim.sim.traci_cache import (
    create_step_cache,
    get_vehicle_road_id_cached,
    get_vehicle_position_cached,
)

from evacsim.utils.lookup import (
    find_shelter_by_edgeID_connect_target_shelter,
    find_shelterID_by_edgeID_by_shelterID,
)

from evacsim.sim.arrival import (
    handle_arrival,
    handle_arrival_for_pedestrian,
    extract_vehicle_id,
)

from evacsim.maps.edge_utils import (
    is_pre_edgeID_near_shelter,
    get_vehicle_start_edges,
    get_vehicle_end_edges,
)

from evacsim.sim.congestion import (
    get_local_density,
    is_vehID_in_congested_edge,
)

from evacsim.sim.traffic_state import (
    apply_gap_density_speed_control,
)

from evacsim.sim.abandonment import (
    count_near_abandoned_vehicle_in_right_lane,
    vehicle_abandant_behavior_with_vehicle_remove,
)

from evacsim.core.decision import (
    is_again_driver_vehicle_abandant,
)

from evacsim.sim.neighbors import (
    get_around_vehIDs,
)

from evacsim.sim.communication import (
    v2v_communication,
    v2shelter_communication,
    v2v_communication_about_tsunami_info,
)

from evacsim.sim.routing import (
    is_route_time_difference_exceeding_threshold,
    find_alternative_shelter_choice,
    find_alternative_route_calculated_time,
)

from evacsim.core.motivation import (
    re_calculate_motivation_value,
    calculate_motivation_for_evacuation_action,
    generate_motivation_curve,
    generate_info_activation_dict,
)

from evacsim.metrics.evacuation_time import (
    calculate_avg_evac_time_by_route,
)

from evacsim.maps.map_loader import (
    init_shelter,
    init_custome_edge,
)

from evacsim.maps.route_map import (
    get_vehicle_end_list_by_start_edge_dict,
)

from evacsim.io.json_io import (
    import_connected_edges_from_json,
    import_start_end_edgeIDs_from_json,
)

from evacsim.utils.random_utils import (
    choose_edge_by_probability,
)

from evacsim.sim.vehicle_generation import (
    generate_simple_init_vehID,
    generate_new_veh_based_on_route_time,
)

from evacsim.sim.initialization import (
    init_vehicleInfo_list_base,
    init_agent_list,
    init_driver_behavior,
)


# ============================================================
# init_agent_list import
# ============================================================
# init_agent_list() の実装は変更しない。
# もし既存コード上の配置が異なる場合は，この import 行だけ変更する。
# ============================================================


import sys as _sys
if _sys.version_info >= (3, 11):
    import tomllib as _toml_loader
else:
    try:
        import tomli as _toml_loader
    except ImportError as e:
        raise ImportError("Python 3.10以下では `pip install tomli` が必要です。") from e


# ============================================================
# TOML loader
# ============================================================
# Python 3.11 以降では tomllib を使う。
# Python 3.10 でも動くように，tomli があれば tomli を使う。
# どちらも無い場合は，今回のような flat な key = value だけを読む
# 簡易 parser にフォールバックする。
# ============================================================

toml_path = "scenarios/dicomo2026/configs/config_scenario_1.toml"

def _req(cfg: dict, key: str, typ=float):
    if key not in cfg:
        raise KeyError(f"Config missing required key: '{key}'")
    return typ(cfg[key])



def load_toml(path: Path) -> dict:
    with path.open("rb") as f:
        return _toml_loader.load(f)
cfg = load_toml(Path(toml_path))

if __name__ == "__main__":
    cfg = load_toml(Path(toml_path))
    COMM_RANGE: float = _req(cfg, "comm_range", float)
    NUM_VEHICLES: int = 10 
    VEHICLE_INTERVAL: float = _req(cfg, "vehicle_interval", float)
    ACTIVE_ROUTE_CHANGE_THRESHOLD_CENTER: float = _req(cfg, "active_route_change_threshold_center", float)
    ACTIVE_ROUTE_CHANGE_THRESHOLD_SPREAD: float = _req(cfg, "active_route_change_threshold_spread", float)
    CAUTIOUS_ROUTE_CHANGE_THRESHOLD_CENTER: float = _req(cfg, "cautious_route_change_threshold_center", float)
    CAUTIOUS_ROUTE_CHANGE_THRESHOLD_SPREAD: float = _req(cfg, "cautious_route_change_threshold_spread", float)
    NORMALCY_VALUE_ABOUT_ROUTE_CHANGE_CENTER: float = _req(cfg, "normalcy_value_about_route_change_center", float)
    NORMALCY_VALUE_ABOUT_ROUTE_CHANGE_SPREAD: float = _req(cfg, "normalcy_value_about_route_change_spread", float)
    ACTIVE_WRONG_WAY_DRIVING_THRESHOLD_CENTER: float = _req(cfg, "active_wrong_way_driving_threshold_center", float)
    ACTIVE_WRONG_WAY_DRIVING_THRESHOLD_SPREAD: float = _req(cfg, "active_wrong_way_driving_threshold_spread", float)
    CAUTIOUS_WRONG_WAY_DRIVING_THRESHOLD_CENTER: float = _req(cfg, "cautious_wrong_way_driving_threshold_center", float)
    CAUTIOUS_WRONG_WAY_DRIVING_THRESHOLD_SPREAD: float = _req(cfg, "cautious_wrong_way_driving_threshold_spread", float)
    MIN_MOTIVATION_START: float = _req(cfg, "min_motivation_start", float)
    MIN_MOTIVATION_END: float = _req(cfg, "min_motivation_end", float)
    ACTIVE_NORMALCY_VALUE_ABOUT_TSUNAMI_PRECURSOR_INFO_CENTER: float = _req(cfg, "active_normalcy_value_about_tsunami_precursor_info_center", float)
    ACTIVE_NORMALCY_VALUE_ABOUT_TSUNAMI_PRECURSOR_INFO_SPREAD: float = _req(cfg, "active_normalcy_value_about_tsunami_precursor_info_spread", float)
    CAUTIOUS_NORMALCY_VALUE_ABOUT_TSUNAMI_PRECURSOR_INFO_CENTER: float = _req(cfg, "cautious_normalcy_value_about_tsunami_precursor_info_center", float)
    CAUTIOUS_NORMALCY_VALUE_ABOUT_TSUNAMI_PRECURSOR_INFO_SPREAD: float = _req(cfg, "cautious_normalcy_value_about_tsunami_precursor_info_spread", float)
    ACTIVE_NORMALCY_VALUE_ABOUT_ROUTE_CONGESTION_INFO_CENTER: float = _req(cfg, "active_normalcy_value_about_route_congestion_info_center", float)
    ACTIVE_NORMALCY_VALUE_ABOUT_ROUTE_CONGESTION_INFO_SPREAD: float = _req(cfg, "active_normalcy_value_about_route_congestion_info_spread", float)
    CAUTIOUS_NORMALCY_VALUE_ABOUT_ROUTE_CONGESTION_INFO_CENTER: float = _req(cfg, "cautious_normalcy_value_about_route_congestion_info_center", float)
    CAUTIOUS_NORMALCY_VALUE_ABOUT_ROUTE_CONGESTION_INFO_SPREAD: float = _req(cfg, "cautious_normalcy_value_about_route_congestion_info_spread", float)
    ACTIVE_NORMALCY_VALUE_ABOUT_SHELTER_FULL_INFO_CENTER: float = _req(cfg, "active_normalcy_value_about_shelter_full_info_center", float)
    ACTIVE_NORMALCY_VALUE_ABOUT_SHELTER_FULL_INFO_SPREAD: float = _req(cfg, "active_normalcy_value_about_shelter_full_info_spread", float)
    CAUTIOUS_NORMALCY_VALUE_ABOUT_SHELTER_FULL_INFO_CENTER: float = _req(cfg, "cautious_normalcy_value_about_shelter_full_info_center", float)
    CAUTIOUS_NORMALCY_VALUE_ABOUT_SHELTER_FULL_INFO_SPREAD: float = _req(cfg, "cautious_normalcy_value_about_shelter_full_info_spread", float)
    ACTIVE_MAJORITY_INCREASE_VALUE_CENTER: float = _req(cfg, "active_majority_increase_value_center", float)
    ACTIVE_MAJORITY_INCREASE_VALUE_SPREAD: float = _req(cfg, "active_majority_increase_value_spread", float)
    CAUTIOUS_MAJORITY_INCREASE_VALUE_CENTER: float = _req(cfg, "cautious_majority_increase_value_center", float)
    CAUTIOUS_MAJORITY_INCREASE_VALUE_SPREAD: float = _req(cfg, "cautious_majority_increase_value_spread", float)
    ACTIVE_MAJORITY_DECREASE_VALUE_CENTER: float = _req(cfg, "active_majority_decrease_value_center", float)
    ACTIVE_MAJORITY_DECREASE_VALUE_SPREAD: float = _req(cfg, "active_majority_decrease_value_spread", float)
    CAUTIOUS_MAJORITY_DECREASE_VALUE_CENTER: float = _req(cfg, "cautious_majority_decrease_value_center", float)
    CAUTIOUS_MAJORITY_DECREASE_VALUE_SPREAD: float = _req(cfg, "cautious_majority_decrease_value_spread", float)
    ACTIVE_SHELTER_OCCUPANCY_RATE_THRESHOLD_START: float = _req(cfg, "active_shelter_occupancy_rate_threshold_start", float)
    ACTIVE_SHELTER_OCCUPANCY_RATE_THRESHOLD_END: float = _req(cfg, "active_shelter_occupancy_rate_threshold_end", float)
    CAUTIOUS_SHELTER_OCCUPANCY_RATE_THRESHOLD_START: float = _req(cfg, "cautious_shelter_occupancy_rate_threshold_start", float)
    CAUTIOUS_SHELTER_OCCUPANCY_RATE_THRESHOLD_END: float = _req(cfg, "cautious_shelter_occupancy_rate_threshold_end", float)
    SHELTER_CAPACITY_THRESHOLD: float = _req(cfg, "shelter_capacity_threshold", float)
    TSUNAMI_SIGN_START_TIME: float = _req(cfg, "tsunami_sign_start_time", float)
    TSUNAMI_SIGN_END_TIME: float = _req(cfg, "tsunami_sign_end_time", float)
    DRIVER_VISIBILITY_DISTANCE: float = _req(cfg, "driver_visibility_distance", float)
    ALPHA: float = _req(cfg, "alpha", float)

    vehID_list = TEST_VEH_IDS
    edgeID_by_shelterID = TEST_EDGE_ID_BY_SHELTER_ID
    early_rate = 0.5
    agent_list: list[Agent] = init_agent_list(
        vehIDs=vehID_list,
        edgeID_by_shelterID=edgeID_by_shelterID,
        ATTR_RATE=early_rate,

        ACTIVE_ROUTE_CHANGE_THRESHOLD_CENTER=ACTIVE_ROUTE_CHANGE_THRESHOLD_CENTER,
        ACTIVE_ROUTE_CHANGE_THRESHOLD_SPREAD=ACTIVE_ROUTE_CHANGE_THRESHOLD_SPREAD,
        CAUTIOUS_ROUTE_CHANGE_THRESHOLD_CENTER=CAUTIOUS_ROUTE_CHANGE_THRESHOLD_CENTER,
        CAUTIOUS_ROUTE_CHANGE_THRESHOLD_SPREAD=CAUTIOUS_ROUTE_CHANGE_THRESHOLD_SPREAD,

        NORMALCY_VALUE_ABOUT_ROUTE_CHANGE_CENTER=NORMALCY_VALUE_ABOUT_ROUTE_CHANGE_CENTER,
        NORMALCY_VALUE_ABOUT_ROUTE_CHANGE_SPREAD=NORMALCY_VALUE_ABOUT_ROUTE_CHANGE_SPREAD,

        ACTIVE_WRONG_WAY_DRIVING_THRESHOLD_CENTER=ACTIVE_WRONG_WAY_DRIVING_THRESHOLD_CENTER,
        ACTIVE_WRONG_WAY_DRIVING_THRESHOLD_SPREAD=ACTIVE_WRONG_WAY_DRIVING_THRESHOLD_SPREAD,
        CAUTIOUS_WRONG_WAY_DRIVING_THRESHOLD_CENTER=CAUTIOUS_WRONG_WAY_DRIVING_THRESHOLD_CENTER,
        CAUTIOUS_WRONG_WAY_DRIVING_THRESHOLD_SPREAD=CAUTIOUS_WRONG_WAY_DRIVING_THRESHOLD_SPREAD,

        MIN_MOTIVATION_START=MIN_MOTIVATION_START,
        MIN_MOTIVATION_END=MIN_MOTIVATION_END,

        ACTIVE_NORMALCY_VALUE_ABOUT_TSUNAMI_PRECURSOR_INFO_CENTER=ACTIVE_NORMALCY_VALUE_ABOUT_TSUNAMI_PRECURSOR_INFO_CENTER,
        ACTIVE_NORMALCY_VALUE_ABOUT_TSUNAMI_PRECURSOR_INFO_SPREAD=ACTIVE_NORMALCY_VALUE_ABOUT_TSUNAMI_PRECURSOR_INFO_SPREAD,
        CAUTIOUS_NORMALCY_VALUE_ABOUT_TSUNAMI_PRECURSOR_INFO_CENTER=CAUTIOUS_NORMALCY_VALUE_ABOUT_TSUNAMI_PRECURSOR_INFO_CENTER,
        CAUTIOUS_NORMALCY_VALUE_ABOUT_TSUNAMI_PRECURSOR_INFO_SPREAD=CAUTIOUS_NORMALCY_VALUE_ABOUT_TSUNAMI_PRECURSOR_INFO_SPREAD,

        ACTIVE_NORMALCY_VALUE_ABOUT_ROUTE_CONGESTION_INFO_CENTER=ACTIVE_NORMALCY_VALUE_ABOUT_ROUTE_CONGESTION_INFO_CENTER,
        ACTIVE_NORMALCY_VALUE_ABOUT_ROUTE_CONGESTION_INFO_SPREAD=ACTIVE_NORMALCY_VALUE_ABOUT_ROUTE_CONGESTION_INFO_SPREAD,
        CAUTIOUS_NORMALCY_VALUE_ABOUT_ROUTE_CONGESTION_INFO_CENTER=CAUTIOUS_NORMALCY_VALUE_ABOUT_ROUTE_CONGESTION_INFO_CENTER,
        CAUTIOUS_NORMALCY_VALUE_ABOUT_ROUTE_CONGESTION_INFO_SPREAD=CAUTIOUS_NORMALCY_VALUE_ABOUT_ROUTE_CONGESTION_INFO_SPREAD,

        ACTIVE_NORMALCY_VALUE_ABOUT_SHELTER_FULL_INFO_CENTER=ACTIVE_NORMALCY_VALUE_ABOUT_SHELTER_FULL_INFO_CENTER,
        ACTIVE_NORMALCY_VALUE_ABOUT_SHELTER_FULL_INFO_SPREAD=ACTIVE_NORMALCY_VALUE_ABOUT_SHELTER_FULL_INFO_SPREAD,
        CAUTIOUS_NORMALCY_VALUE_ABOUT_SHELTER_FULL_INFO_CENTER=CAUTIOUS_NORMALCY_VALUE_ABOUT_SHELTER_FULL_INFO_CENTER,
        CAUTIOUS_NORMALCY_VALUE_ABOUT_SHELTER_FULL_INFO_SPREAD=CAUTIOUS_NORMALCY_VALUE_ABOUT_SHELTER_FULL_INFO_SPREAD,

        ACTIVE_MAJORITY_INCREASE_VALUE_CENTER=ACTIVE_MAJORITY_INCREASE_VALUE_CENTER,
        ACTIVE_MAJORITY_INCREASE_VALUE_SPREAD=ACTIVE_MAJORITY_INCREASE_VALUE_SPREAD,
        CAUTIOUS_MAJORITY_INCREASE_VALUE_CENTER=CAUTIOUS_MAJORITY_INCREASE_VALUE_CENTER,
        CAUTIOUS_MAJORITY_INCREASE_VALUE_SPREAD=CAUTIOUS_MAJORITY_INCREASE_VALUE_SPREAD,

        ACTIVE_MAJORITY_DECREASE_VALUE_CENTER=ACTIVE_MAJORITY_DECREASE_VALUE_CENTER,
        ACTIVE_MAJORITY_DECREASE_VALUE_SPREAD=ACTIVE_MAJORITY_DECREASE_VALUE_SPREAD,
        CAUTIOUS_MAJORITY_DECREASE_VALUE_CENTER=CAUTIOUS_MAJORITY_DECREASE_VALUE_CENTER,
        CAUTIOUS_MAJORITY_DECREASE_VALUE_SPREAD=CAUTIOUS_MAJORITY_DECREASE_VALUE_SPREAD,

        ACTIVE_SHELTER_OCCUPANCY_RATE_THRESHOLD_START=ACTIVE_SHELTER_OCCUPANCY_RATE_THRESHOLD_START,
        ACTIVE_SHELTER_OCCUPANCY_RATE_THRESHOLD_END=ACTIVE_SHELTER_OCCUPANCY_RATE_THRESHOLD_END,
        CAUTIOUS_SHELTER_OCCUPANCY_RATE_THRESHOLD_START=CAUTIOUS_SHELTER_OCCUPANCY_RATE_THRESHOLD_START,
        CAUTIOUS_SHELTER_OCCUPANCY_RATE_THRESHOLD_END=CAUTIOUS_SHELTER_OCCUPANCY_RATE_THRESHOLD_END,
    )
    # 時間経過に伴うモチベーションの増加を設定する
    INFOS = ("tsu", "jam", "full")
    rho: dict[str, float] = {
        "tsu": 0.08,
        "jam": 0.02,
        "full": 0.05,
    }

    # 情報取得後の反応遅れ [s]
    delta: dict[str, float] = {
        "tsu": 20.0,
        "jam": 60.0,
        "full": 30.0,
    }
    base_motivation_value_for_elapsed_time_dict = generate_motivation_curve(max_time=450,step=1)
    tusnami_normalcy_values_by_info = generate_info_activation_dict(rho=rho["tsu"], delta=delta["tsu"], max_time=450, step=1)
    jam_normalcy_values_by_info = generate_info_activation_dict(rho=rho["jam"], delta=delta["jam"], max_time=450, step=1)
    full_normalcy_values_by_info = generate_info_activation_dict(rho=rho["full"], delta=delta["full"], max_time=450, step=1)
    for agent in agent_list:
        agent.set_base_motivation_value_by_elapsed_time_dict(base_motivation_value_for_elapsed_time_dict)
        for info in INFOS:
            if info=="tsu":
                agent.set_tsunami_precursor_normalcy_value_by_elapsed_time_dict(tusnami_normalcy_values_by_info)
            elif info=="jam":
                agent.set_route_congestion_normalcy_value_by_elapsed_time_dict(jam_normalcy_values_by_info)
            elif info=="full":
                agent.set_shelter_full_normalcy_value_by_elapsed_time_dict(full_normalcy_values_by_info)
            else:
                print(f"[WARN] unknown info type: {info}")

