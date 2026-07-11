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
import sys as _sys
if _sys.version_info >= (3, 11):
    import tomllib as _toml_loader
else:
    try:
        import tomli as _toml_loader
    except ImportError as e:
        raise ImportError("Python 3.10以下では `pip install tomli` が必要です。") from e

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
    find_agent_by_vehID
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
    get_opposite_edgeID_by_edgeID,
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
    vehicle_abandant_behavior
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
    merge_route_info_within_shelters,
)

from evacsim.metrics.evacuation_time import (
    create_arrival_time_list,
    calculate_avg_evac_time_by_route,
    merge_arrival_vehs_of_shelter,
)

from evacsim.sim.routing import (
    is_route_time_difference_exceeding_threshold,
    find_alternative_shelter_choice,
    find_alternative_route_calculated_time,
    get_route_time_difference_exceeding_threshold,
    find_uturn_shortest_route_to_current_shelter_group,
)

from evacsim.core.motivation import (
    re_calculate_motivation_value,
    calculate_motivation_for_evacuation_action,
    generate_motivation_curve,
    generate_info_activation_dict,
    set_motivation_curve_dicts_to_agents
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
    choose_route_edges_by_probability
)

from evacsim.sim.vehicle_generation import (
    generate_simple_init_vehID,
    generate_new_veh_based_on_route_time,
    generate_init_vehID_with_route_edges,
    parse_route_id,
    validate_route_choice_rates,
    allocate_vehicle_counts_by_route,
    build_assigned_routeID_list,
    generate_simple_init_vehID_based_one_routefile
)

from evacsim.sim.initialization import (
    init_vehicleInfo_list_base,
    init_agent_list,
    init_driver_behavior,
)

from evacsim.io.route_file import (
    convert_routefile_to_routes_by_id,
)

from evacsim.sim.neighbors import (
    count_rc_around_vehicles,
)

# =========================
# Runtime config / seeds
# =========================
random.seed()
random.seed(os.getpid() + int(datetime.datetime.now().timestamp()))
np.random.seed(os.getpid() + int(datetime.datetime.now().timestamp()))

# runner_base.py のファイル位置から data ディレクトリを解決
HERE = Path(__file__).resolve()
MAP_ONE_DIR = HERE.parent.parent          # .../scenarios/its102/map_one
DATA_DIR = MAP_ONE_DIR / "data"
SUMO_CFG = DATA_DIR / "ishinomaki_one_four_one.sumocfg"

# =========================
# 1) シミュレーション基本設定・時間評価
# =========================
END_SIMULATION_TIME = 3500
THRESHOLD_SPEED = 2.00 # 7.2km/h
STOPPING_TIME_IN_SHELTER = 10000000
SLOW_DURATION = 15.0
SLOW_SPEED = 3.0

# =========================
# Route-change decision model
# =========================
P_FOLLOW = 0.25
SHELTER_FULL_INFO_THRESHOLD = 0.98
ROUTE_CHANGE_COLOR = (60, 180, 120)
DEBUG_ROUTE_CHANGE_COUNT = True
DEBUG_ROUTE_CHANGE_DECISION_SKIP = False

# =========================
# 2) 車両生成・発進関連
# =========================
VEHICLE_NUM = 0
DEPART_TIME: double = 0.0
ROUTE_NUM = 0


# =========================
# 8) 心理モデル：初期タイプ別しきい（早期/遅延）
# =========================
EARLY_AGENT_THRESHOLD_LIST = [60, 90, 100, 130] # 早期決断者の閾値
LATE_AGENT_THRESHOLD_LIST = [180, 220, 300, 350] # 遅延決断者の閾値

# =========================
# 10) 実行時カウンタ（毎試行でリセットされる動的値）
# =========================
ROUTE_CHANGED_VEHICLE_COUNT = 0
NORMALCY_BIAS_ROUTE_CHANGE_COUNT = 0
MAJORITY_BIAS_ROUTE_CHANGE_COUNT = 0
LANE_CHANGED_VEHICLE_COUNT = 0
WRONG_WAY_SUCCESS_COUNT = 0
OBTAIN_INFO_LANE_CHANGE_COUNT = 0
ELAPSED_TIME_LANE_CHANGE_COUNT = 0
POSITIVE_MAJORITY_BIAS_COUNT = 0
PEDESTRIAN_COUNT = 0
NEW_VEHICLE_COUNT = 0
ROUTE_CHANGE_COUNT_AFTER_SHELTER_CAPACITY_FULL = 0
VEHICLE_ABANDONMENT_SUCCESS_COUNT = 0


# リストの初期化
custome_edge_list: list = []
shelter_list: list[Shelter] = []
vehInfo_list = []
vehID_list = []
arrival_time_by_vehID_dict = {}
arrival_time_list = []
elapsed_time_list = []
vehicle_abandant_time_by_pedestrianID_dict = {}
walking_distance_by_pedestrianID_dict = {}
pedstrianID_list = []
agent_by_vehID_dict = {}
vehInfo_by_vehID_dict = {}
route_edges_by_routeID_dict = {}
route_change_time_by_vehID_dict = {}




def run():
    while traci.simulation.getTime() < END_SIMULATION_TIME:
        traci.simulationStep()
        control_vehicles()
    traci.close()
    sys.stdout.flush()


def _build_remaining_route_edges(
    current_edgeID: str,
    route_edges_for_sumo: tuple[str, ...] | list[str],
) -> list[str]:
    """
    route_edges_for_sumo から、現在 edge 以降の残り経路を作る。

    TraCI の vehicle.setRoute() は、現在車両がいる edge を含む
    残り経路を渡す必要がある。
    """
    route_edges = list(route_edges_for_sumo)

    if not route_edges:
        return []

    if current_edgeID not in route_edges:
        return []

    current_edge_index = route_edges.index(current_edgeID)
    return route_edges[current_edge_index:]


def _apply_route_edges_for_sumo(
    vehID: str,
    current_edgeID: str,
    route_edges_for_sumo: tuple[str, ...] | list[str],
    current_time: float,
) -> bool:
    """
    candidate 評価で得た route_edges_for_sumo を使って、
    現在車両の残り経路を直接差し替える。

    成功したら True、失敗したら False を返す。
    失敗時もシミュレーション全体は止めない。
    """
    remaining_route_edges = _build_remaining_route_edges(
        current_edgeID=current_edgeID,
        route_edges_for_sumo=route_edges_for_sumo,
    )

    if not remaining_route_edges:
        print(
            f"Skip route change: current_edgeID={current_edgeID} is not in "
            f"route_edges_for_sumo={route_edges_for_sumo}. "
            f"vehID={vehID}, time={current_time}"
        )
        return False

    if remaining_route_edges[0] != current_edgeID:
        print(
            f"Skip route change: remaining route does not start with current edge. "
            f"vehID={vehID}, current_edgeID={current_edgeID}, "
            f"remaining_route_edges={remaining_route_edges}, time={current_time}"
        )
        return False

    try:
        traci.vehicle.setRoute(vehID, remaining_route_edges)
    except traci.exceptions.TraCIException as e:
        print(
            f"Route replacement failed by setRoute. "
            f"vehID={vehID}, current_edgeID={current_edgeID}, "
            f"remaining_route_edges={remaining_route_edges}, "
            f"time={current_time}, error={e}"
        )
        return False

    return True


def _try_set_vehicle_color(
    vehID: str,
    color: tuple[int, int, int],
    current_time: float,
) -> None:
    """
    車両がすでに削除済みの場合でも simulation を止めずに色変更を試みる。
    """
    try:
        traci.vehicle.setColor(vehID, color)
    except traci.exceptions.TraCIException as e:
        if DEBUG_ROUTE_CHANGE_COUNT:
            print(
                f"Skip setColor: vehID={vehID}, color={color}, "
                f"time={current_time}, error={e}"
            )


def _record_route_change_once(
    vehID: str,
    agent: Agent,
    current_time: float,
    reason: str,
) -> bool:
    """
    経路変更成功後に一度だけ状態更新とカウントを行う。

    Args:
        vehID:
            経路変更した車両ID。
        agent:
            対象車両の Agent。
        current_time:
            現在時刻。
        reason:
            "normalcy", "majority", "shelter_full" のいずれか。

    Returns:
        初回の経路変更として記録した場合 True。
        すでに記録済みの場合 False。
    """
    allowed_reasons = {"normalcy", "majority", "shelter_full"}
    if reason not in allowed_reasons:
        raise ValueError(
            f"Invalid route change reason: {reason}. "
            f"Allowed reasons are {sorted(allowed_reasons)}"
        )

    global ROUTE_CHANGED_VEHICLE_COUNT
    global NORMALCY_BIAS_ROUTE_CHANGE_COUNT
    global MAJORITY_BIAS_ROUTE_CHANGE_COUNT
    global ROUTE_CHANGE_COUNT_AFTER_SHELTER_CAPACITY_FULL

    if agent.get_evacuation_route_changed_flg() or vehID in route_change_time_by_vehID_dict:
        if DEBUG_ROUTE_CHANGE_COUNT:
            print(
                f"Skip duplicate route change count: vehID={vehID}, "
                f"reason={reason}, time={current_time}"
            )
        return False

    agent.set_evacuation_route_changed_flg(True)
    agent.set_agent_action_name("rc")
    route_change_time_by_vehID_dict[vehID] = current_time

    ROUTE_CHANGED_VEHICLE_COUNT += 1
    if reason == "normalcy":
        NORMALCY_BIAS_ROUTE_CHANGE_COUNT += 1
    elif reason == "majority":
        MAJORITY_BIAS_ROUTE_CHANGE_COUNT += 1
    elif reason == "shelter_full":
        ROUTE_CHANGE_COUNT_AFTER_SHELTER_CAPACITY_FULL += 1

    if DEBUG_ROUTE_CHANGE_COUNT:
        print(
            f"Route change recorded: vehID={vehID}, reason={reason}, "
            f"time={current_time}, total={ROUTE_CHANGED_VEHICLE_COUNT}, "
            f"normalcy={NORMALCY_BIAS_ROUTE_CHANGE_COUNT}, "
            f"majority={MAJORITY_BIAS_ROUTE_CHANGE_COUNT}, "
            f"shelter_full={ROUTE_CHANGE_COUNT_AFTER_SHELTER_CAPACITY_FULL}"
        )

    return True


def _mark_generated_route_change_agents(
    new_vehIDs: set[str],
    current_time: float,
) -> None:
    """
    generate_new_veh_based_on_route_time() で生成された新車両を、
    「経路変更後の車両」として扱う。

    ここではカウントしない。
    カウントは元車両IDに対する _record_route_change_once() のみで行う。
    これにより、新車両が次 step 以降に再度 route-change 判定へ入ることを防ぐ。
    """
    for new_vehID in new_vehIDs:
        new_agent = agent_by_vehID_dict.get(new_vehID)
        if new_agent is None:
            continue
        new_agent.set_evacuation_route_changed_flg(True)
        new_agent.set_agent_action_name("rc")
        if DEBUG_ROUTE_CHANGE_COUNT:
            print(
                f"Mark generated route-change vehicle: vehID={new_vehID}, "
                f"time={current_time}"
            )


def _generate_new_vehicle_for_route_change(
    *,
    target_vehID: str,
    current_new_vehicle_count: int,
    vehInfo_by_target_vehID: VehicleInfo,
    agent_by_target_vehID: Agent,
    from_edgeID: str,
    new_shelterID: str,
    to_edgeID: str,
    current_time: float,
) -> tuple[int, bool]:
    """
    generate_new_veh_based_on_route_time() を呼び、
    NEW_VEHICLE_COUNT が増えた場合のみ成功として扱う。
    """
    if not from_edgeID or not new_shelterID or not to_edgeID:
        if DEBUG_ROUTE_CHANGE_COUNT:
            print(
                f"Skip new vehicle route change: empty candidate. "
                f"vehID={target_vehID}, from_edgeID={from_edgeID}, "
                f"new_shelterID={new_shelterID}, to_edgeID={to_edgeID}, "
                f"time={current_time}"
            )
        return current_new_vehicle_count, False

    agent_ids_before = set(agent_by_vehID_dict.keys())

    try:
        updated_new_vehicle_count = generate_new_veh_based_on_route_time(
            target_vehID=target_vehID,
            NEW_VEHICLE_COUNT=current_new_vehicle_count,
            agent_list=agent_list,
            vehInfo_list=vehInfo_list,
            vehInfo_by_target_vehID=vehInfo_by_target_vehID,
            agent_by_target_vehID=agent_by_target_vehID,
            from_edgeID=from_edgeID,
            new_shelterID=new_shelterID,
            to_edgeID=to_edgeID,
            color_mode="",
            agent_by_vehID_dict=agent_by_vehID_dict,
            vehInfo_by_vehID_dict=vehInfo_by_vehID_dict,
        )
    except Exception as e:
        print(
            f"Route replacement failed by generate_new_veh_based_on_route_time. "
            f"vehID={target_vehID}, from_edgeID={from_edgeID}, "
            f"new_shelterID={new_shelterID}, to_edgeID={to_edgeID}, "
            f"time={current_time}, error={e}"
        )
        return current_new_vehicle_count, False

    success = updated_new_vehicle_count > current_new_vehicle_count
    if not success:
        if DEBUG_ROUTE_CHANGE_COUNT:
            print(
                f"Skip route change count: new vehicle was not generated. "
                f"vehID={target_vehID}, before={current_new_vehicle_count}, "
                f"after={updated_new_vehicle_count}, time={current_time}"
            )
        return updated_new_vehicle_count, False

    new_agent_ids = set(agent_by_vehID_dict.keys()) - agent_ids_before
    _mark_generated_route_change_agents(
        new_vehIDs=new_agent_ids,
        current_time=current_time,
    )
    return updated_new_vehicle_count, True


def _print_route_change_count_consistency() -> None:
    """
    経路変更カウントと理由別カウント、記録辞書の整合性を確認する。
    """
    route_change_reason_total = (
        NORMALCY_BIAS_ROUTE_CHANGE_COUNT
        + MAJORITY_BIAS_ROUTE_CHANGE_COUNT
        + ROUTE_CHANGE_COUNT_AFTER_SHELTER_CAPACITY_FULL
    )
    route_change_time_count = len(route_change_time_by_vehID_dict)

    if (
        route_change_reason_total != ROUTE_CHANGED_VEHICLE_COUNT
        or route_change_time_count != ROUTE_CHANGED_VEHICLE_COUNT
    ):
        print(
            "WARNING route change count mismatch: "
            f"route_changed={ROUTE_CHANGED_VEHICLE_COUNT}, "
            f"reason_total={route_change_reason_total}, "
            f"route_change_time_count={route_change_time_count}, "
            f"normalcy={NORMALCY_BIAS_ROUTE_CHANGE_COUNT}, "
            f"majority={MAJORITY_BIAS_ROUTE_CHANGE_COUNT}, "
            f"shelter_full={ROUTE_CHANGE_COUNT_AFTER_SHELTER_CAPACITY_FULL}"
        )
    else:
        print(
            "OK route change count consistency: "
            f"route_changed={ROUTE_CHANGED_VEHICLE_COUNT}, "
            f"normalcy={NORMALCY_BIAS_ROUTE_CHANGE_COUNT}, "
            f"majority={MAJORITY_BIAS_ROUTE_CHANGE_COUNT}, "
            f"shelter_full={ROUTE_CHANGE_COUNT_AFTER_SHELTER_CAPACITY_FULL}"
        )


def _is_route_change_prohibited_edge(current_edgeID: str) -> bool:
    """
    目的地直前・目的地 edge では経路変更を行わない。
    """
    return current_edgeID in ["E13", "E150", "E15", "E16", "E12", "E5"]


def _get_route_change_neighbor_counts(
    *,
    agent: Agent,
    vehInfo: VehicleInfo,
    current_edgeID: str,
    distance_threshold: float,
) -> tuple[int, int]:
    """
    count_rc_around_vehicles() の戻り値を安全に (rc_count, uturn_count) に正規化する。

    新実装では (rc_count, uturn_count) を返す想定。
    ただし、古い実装や例外時に int だけ返った場合でも runner 側で落ちないようにする。
    """
    result = count_rc_around_vehicles(
        agent=agent,
        vehInfo=vehInfo,
        agent_list=agent_list,
        candidate_action="rc",
        agent_by_vehID_dict=agent_by_vehID_dict,
        custome_edge_list=custome_edge_list,
        current_edgeID=current_edgeID,
        distance_threshold=distance_threshold,
    )

    if isinstance(result, tuple):
        if len(result) != 2:
            raise ValueError(
                f"count_rc_around_vehicles() must return 2 values, but got {result}"
            )
        return int(result[0]), int(result[1])

    if DEBUG_ROUTE_CHANGE_COUNT:
        print(
            "WARNING count_rc_around_vehicles() returned a non-tuple value. "
            f"Treat it as rc_count only: result={result}"
        )
    return int(result), 0


def control_vehicles():
    vehIDs = traci.vehicle.getIDList()
    pedestrianIDs = traci.person.getIDList()
    current_time = traci.simulation.getTime()
    is_step_5 = (current_time % 5 == 0)
    is_step_10 = (current_time % 10 == 0)
    is_step_50 = (current_time % 50 == 0)
    step_cache = create_step_cache(current_time=current_time)
    route_avg_dirty = False

    global PEDESTRIAN_COUNT
    global NEW_VEHICLE_COUNT
    global VEHICLE_ABANDONMENT_SUCCESS_COUNT
    global WRONG_WAY_SUCCESS_COUNT

    # VEHINFO: 車両に対する処理を行う
    for current_vehID in vehIDs:
        vehInfo_by_current_vehID: VehicleInfo = vehInfo_by_vehID_dict.get(current_vehID)
        agent_by_current_vehID: Agent = agent_by_vehID_dict.get(current_vehID)

        if vehInfo_by_current_vehID is None or agent_by_current_vehID is None:
            continue

        shelter_for_current_vehID: Shelter = find_shelter_by_edgeID_connect_target_shelter(
            vehInfo_by_current_vehID.get_edgeID_connect_target_shelter(),
            shelter_list,
        )
        current_edgeID: str = get_vehicle_road_id_cached(current_vehID, step_cache=step_cache)
        current_position = None
        if not agent_by_current_vehID.get_created_time_flg():
            agent_by_current_vehID.set_created_time(current_time)
            agent_by_current_vehID.set_created_time_flg(True)

        # VEHINFO: 到着処理を行う
        if current_edgeID in ["E13", "E16"] and not vehInfo_by_current_vehID.get_arrival_flag():
            handle_arrival(
                current_vehID=current_vehID,
                vehInfo_by_current_vehID=vehInfo_by_current_vehID,
                agent_by_current_vehID=agent_by_current_vehID,
                shelter_for_current_vehID=shelter_for_current_vehID,
                shelter_list=shelter_list,
                arrival_time_list=arrival_time_list,
                arrival_time_by_vehID_dict=arrival_time_by_vehID_dict,
                elapsed_time_list=elapsed_time_list,
            )
            # traci.vehicle.remove(current_vehID)
            continue

        # VEHINFO: 避難所近くのエッジにいる場合、密度に応じて速度制御を行う
        if not vehInfo_by_current_vehID.get_decline_edge_arrival_flag():
            traci.vehicle.setLaneChangeMode(current_vehID, 1024)
            pre_edgeID_near_shelter_flag = is_pre_edgeID_near_shelter(
                current_edgeID=current_edgeID,
                edgeID_near_shelter=vehInfo_by_current_vehID.get_edgeID_connect_target_shelter(),
                custome_edge_list=custome_edge_list,
            )
            if pre_edgeID_near_shelter_flag and not vehInfo_by_current_vehID.get_decline_edge_arrival_flag():
                local_density = get_local_density(vehID=current_vehID, radius=50.0)
                apply_gap_density_speed_control(
                    vehID=current_vehID,
                    local_density=local_density,
                    v_free=6.0,
                    v_min=3.0,
                    gap_min=7.0,
                    tau=1.8,
                    alpha=0.5,
                    slow_time=1.0,
                )
            # else:
            #     traci.vehicle.slowDown(current_vehID, 5.0, 1.0)

        # V2V: 車両間通信による情報共有
        if vehInfo_by_current_vehID.get_vehicle_comm_enabled_flag() and not vehInfo_by_current_vehID.get_arrival_flag():
            if is_step_10:
                if current_position is None:
                    current_position = get_vehicle_position_cached(current_vehID, step_cache=step_cache)
                around_vehIDs: list = get_around_vehIDs(
                    target_vehID=current_vehID,
                    custome_edge_list=custome_edge_list,
                    step_cache=step_cache,
                )
                v2v_communication(
                    target_vehID=current_vehID,
                    target_vehInfo=vehInfo_by_current_vehID,
                    around_vehIDs=around_vehIDs,
                    agent_list=agent_list,
                    vehInfo_list=vehInfo_list,
                    COMMUNICATION_RANGE=COMM_RANGE,
                    target_position=current_position,
                    agent_by_vehID_dict=agent_by_vehID_dict,
                    vehInfo_by_vehID_dict=vehInfo_by_vehID_dict,
                    step_cache=step_cache,
                )
                v2shelter_communication(
                    target_vehID=current_vehID,
                    shelterID=vehInfo_by_current_vehID.get_target_shelter(),
                    vehInfo_list=vehInfo_list,
                    shelter_list=shelter_list,
                    COMMUNICATION_RANGE=COMM_RANGE,
                    target_vehInfo=vehInfo_by_current_vehID,
                    target_position=current_position,
                    vehInfo_by_vehID_dict=vehInfo_by_vehID_dict,
                    step_cache=step_cache,
                )

        # VEHINFO: 到着していない車両に対して処理を行う
        if not vehInfo_by_current_vehID.get_arrival_flag():
            # V2V: 車両間通信が可能な場合、情報を取得する
            if vehInfo_by_current_vehID.get_vehicle_comm_enabled_flag():
                try:
                    # VEHINFO: 満杯情報を取得したか否かを確認する
                    if (
                        not agent_by_current_vehID.get_shelter_full_info_obtained_flg()
                        and vehInfo_by_current_vehID.has_shelter_full_info(
                            shelterID=agent_by_current_vehID.get_target_shelter(),
                            threshold=SHELTER_FULL_INFO_THRESHOLD,
                        )
                        and vehInfo_by_current_vehID.get_vehicle_comm_enabled_flag()
                    ):
                        agent_by_current_vehID.set_shelter_full_info_obtained_time(current_time)
                        agent_by_current_vehID.set_shelter_full_info_obtained_flg(True)

                except Exception as e:
                    vehInfo_by_current_vehID.print_all_info()
                    print(
                        f"Error during information obtaining check for vehicle "
                        f"{current_vehID} at time {traci.simulation.getTime()}: {e}"
                    )

            if is_vehID_in_congested_edge(
                vehID=current_vehID,
                threshold_speed=THRESHOLD_SPEED,
            ):
                already_route_changed = (
                    agent_by_current_vehID.get_evacuation_route_changed_flg()
                    or current_vehID in route_change_time_by_vehID_dict
                )

                # 経路変更済み車両は緑色を維持し、未変更車両のみ渋滞色にする。
                if not already_route_changed:
                    _try_set_vehicle_color(current_vehID, (255, 0, 0), current_time)

                if not agent_by_current_vehID.get_encounted_congestion_flg():
                    agent_by_current_vehID.set_encounted_congestion_time(current_time)
                    agent_by_current_vehID.set_encounted_congestion_flg(True)

                # すでに経路変更済みなら、到着処理・V2V通信・満杯情報取得は済ませたうえで、
                # 経路変更判定ブロックだけをスキップする。
                if already_route_changed:
                    if DEBUG_ROUTE_CHANGE_DECISION_SKIP:
                        print(
                            f"Skip route change decision for already changed vehicle: "
                            f"vehID={current_vehID}, time={current_time}"
                        )
                    continue

                # ============================================================
                # 1. 避難所満杯情報による目的地変更
                # ============================================================
                if has_multiple_shelters(edgeID_by_shelterID) and agent_by_current_vehID.get_shelter_full_info_obtained_flg():
                    from_edgeID, shelterID, to_edge_list = find_alternative_shelter_choice(
                        current_target_shelterID=agent_by_current_vehID.get_target_shelter(),
                        vehID=current_vehID,
                        current_edgeID=current_edgeID,
                        vehInfo=vehInfo_by_current_vehID,
                        shelter_list=shelter_list,
                        agent=agent_by_current_vehID,
                    )
                    print(
                        f"Vehicle {current_vehID} is considering changing route due to "
                        f"shelter full information. From edge: {from_edgeID}, "
                        f"New shelter: {shelterID}, To edges: {to_edge_list}"
                    )
                    if from_edgeID and shelterID and to_edge_list:
                        NEW_VEHICLE_COUNT, route_changed = _generate_new_vehicle_for_route_change(
                            target_vehID=current_vehID,
                            current_new_vehicle_count=NEW_VEHICLE_COUNT,
                            vehInfo_by_target_vehID=vehInfo_by_current_vehID,
                            agent_by_target_vehID=agent_by_current_vehID,
                            from_edgeID=from_edgeID,
                            new_shelterID=shelterID,
                            to_edgeID=to_edge_list[0],
                            current_time=current_time,
                        )
                        if route_changed:
                            # _try_set_vehicle_color(current_vehID, ROUTE_CHANGE_COLOR, current_time)
                            _record_route_change_once(
                                vehID=current_vehID,
                                agent=agent_by_current_vehID,
                                current_time=current_time,
                                reason="shelter_full",
                            )
                    continue

                # ============================================================
                # 2. 正常性バイアスによる経路変更
                # ============================================================
                if is_step_10:
                    from_edgeID = ""
                    shelterID = ""
                    to_edge_list = []
                    time_gain = 0.0
                    congestion_flg = False
                    route_edges_for_sumo = tuple()

                    if not _is_route_change_prohibited_edge(current_edgeID):
                        (
                            from_edgeID,
                            shelterID,
                            to_edge_list,
                            time_gain,
                            congestion_flg,
                            route_edges_for_sumo,
                        ) = get_route_time_difference_exceeding_threshold(
                            current_edgeID=current_edgeID,
                            agent_by_target_vehID=agent_by_current_vehID,
                            shelter=shelter_for_current_vehID,
                            vehInfo_by_target_vehID=vehInfo_by_current_vehID,
                            shelter_list=shelter_list,
                            custome_edge_list=custome_edge_list,
                        )
                    agent_by_current_vehID.set_route_congestion_info_obtained_time(current_time)
                    agent_by_current_vehID.set_route_congestion_info_obtained_flg(congestion_flg)

                    if congestion_flg:
                        if current_edgeID in ["E1", "E20"]:
                            routeID = find_alternative_route_calculated_time(
                                current_edgeID=current_edgeID,
                                vehInfo=vehInfo_by_current_vehID,
                                agent=agent_by_current_vehID,
                                shelter_list=shelter_list,
                                custome_edge_list=custome_edge_list,
                            )
                            if routeID is None:
                                continue
                            try:
                                traci.vehicle.setRouteID(current_vehID, routeID)
                            except traci.exceptions.TraCIException as e:
                                print(
                                    f"Route replacement failed by setRouteID. "
                                    f"vehID={current_vehID}, current_edgeID={current_edgeID}, "
                                    f"routeID={routeID}, time={current_time}, error={e}"
                                )
                                continue
                            _try_set_vehicle_color(current_vehID, ROUTE_CHANGE_COLOR, current_time)
                            _record_route_change_once(
                                vehID=current_vehID,
                                agent=agent_by_current_vehID,
                                current_time=current_time,
                                reason="normalcy",
                            )
                            continue

                        if _is_route_change_prohibited_edge(current_edgeID):
                            continue

                        if not to_edge_list:
                            if DEBUG_ROUTE_CHANGE_COUNT:
                                print(
                                    f"normalcy bias: empty to_edge_list. "
                                    f"vehID={current_vehID}, current_edgeID={current_edgeID}, "
                                    f"from_edgeID={from_edgeID}, shelterID={shelterID}, "
                                    f"time_gain={time_gain:.3f}, time={current_time}"
                                )
                            continue

                        if from_edgeID == current_edgeID:
                            route_changed = _apply_route_edges_for_sumo(
                                vehID=current_vehID,
                                current_edgeID=current_edgeID,
                                route_edges_for_sumo=route_edges_for_sumo,
                                current_time=current_time,
                            )
                            if route_changed:
                                _try_set_vehicle_color(current_vehID, ROUTE_CHANGE_COLOR, current_time)
                                _record_route_change_once(
                                    vehID=current_vehID,
                                    agent=agent_by_current_vehID,
                                    current_time=current_time,
                                    reason="normalcy",
                                )
                            continue

                        if from_edgeID and shelterID and to_edge_list:
                            NEW_VEHICLE_COUNT, route_changed = _generate_new_vehicle_for_route_change(
                                target_vehID=current_vehID,
                                current_new_vehicle_count=NEW_VEHICLE_COUNT,
                                vehInfo_by_target_vehID=vehInfo_by_current_vehID,
                                agent_by_target_vehID=agent_by_current_vehID,
                                from_edgeID=from_edgeID,
                                new_shelterID=shelterID,
                                to_edgeID=to_edge_list[0],
                                current_time=current_time,
                            )
                            if route_changed:
                                # _try_set_vehicle_color(current_vehID, ROUTE_CHANGE_COLOR, current_time)
                                _record_route_change_once(
                                    vehID=current_vehID,
                                    agent=agent_by_current_vehID,
                                    current_time=current_time,
                                    reason="normalcy",
                                )
                        continue

                    # ============================================================
                    # 3. 同調バイアスによる経路変更
                    #    正常性バイアスで変更しなかった場合のみ
                    # ============================================================
                    rc_around_count, uturn_change_count = _get_route_change_neighbor_counts(
                        agent=agent_by_current_vehID,
                        vehInfo=vehInfo_by_current_vehID,
                        current_edgeID=current_edgeID,
                        distance_threshold=50.0,
                    )

                    # 周囲に追従対象がいない場合は何もしない
                    if rc_around_count <= 0 and uturn_change_count <= 0:
                        continue

                    # 確率的に追従しない場合は何もしない
                    if random.random() >= P_FOLLOW:
                        continue

                    # 目的地直前・目的地 edge では経路変更しない
                    if _is_route_change_prohibited_edge(current_edgeID):
                        continue

                    # ------------------------------------------------------------
                    # 3-1. edge ID に依存せず、まず routeID による経路変更を試す
                    # ------------------------------------------------------------
                    route_changed_by_route_id = False

                    routeID = find_alternative_route_calculated_time(
                        current_edgeID=current_edgeID,
                        vehInfo=vehInfo_by_current_vehID,
                        agent=agent_by_current_vehID,
                        shelter_list=shelter_list,
                        custome_edge_list=custome_edge_list,
                    )

                    if routeID is not None:
                        try:
                            traci.vehicle.setRouteID(current_vehID, routeID)
                            route_changed_by_route_id = True
                        except traci.exceptions.TraCIException as e:
                            print(
                                f"Route replacement failed by setRouteID. "
                                f"vehID={current_vehID}, current_edgeID={current_edgeID}, "
                                f"routeID={routeID}, time={current_time}, error={e}"
                            )

                    if route_changed_by_route_id:
                        _try_set_vehicle_color(current_vehID, ROUTE_CHANGE_COLOR, current_time)
                        _record_route_change_once(
                            vehID=current_vehID,
                            agent=agent_by_current_vehID,
                            current_time=current_time,
                            reason="majority",
                        )
                        continue

                    # ------------------------------------------------------------
                    # 3-2. routeID で変更できなかった場合のみ U-turn 系に fallback する
                    # ------------------------------------------------------------
                    if uturn_change_count <= 0:
                        continue

                    majority_from_edgeID, majority_shelterID, majority_to_edge_list = (
                        find_uturn_shortest_route_to_current_shelter_group(
                            current_edgeID=current_edgeID,
                            vehID=current_vehID,
                            vehInfo=vehInfo_by_current_vehID,
                            agent=agent_by_current_vehID,
                            shelter_list=shelter_list,
                        )
                    )

                    if not (majority_from_edgeID and majority_shelterID and majority_to_edge_list):
                        continue

                    NEW_VEHICLE_COUNT, route_changed = _generate_new_vehicle_for_route_change(
                        target_vehID=current_vehID,
                        current_new_vehicle_count=NEW_VEHICLE_COUNT,
                        vehInfo_by_target_vehID=vehInfo_by_current_vehID,
                        agent_by_target_vehID=agent_by_current_vehID,
                        from_edgeID=majority_from_edgeID,
                        new_shelterID=majority_shelterID,
                        to_edgeID=majority_to_edge_list[0],
                        current_time=current_time,
                    )

                    if route_changed:
                        _record_route_change_once(
                            vehID=current_vehID,
                            agent=agent_by_current_vehID,
                            current_time=current_time,
                            reason="majority",
                        )

                    continue


    # SHELTER: 避難地の混雑率を計算する
    calculate_avg_evac_time_by_route(shelter_list=shelter_list)
    # shelterごとで情報を共有する
    merge_route_info_within_shelters(shelter_list[0], shelter_list[1])
    # shelterごとに到着した車両の数を共有する
    merge_arrival_vehs_of_shelter(shelter_list=shelter_list)
    for shelter in shelter_list:
        shelter.update_congestion_rate()
        # if is_step_50:
        #     print(f"{shelter.get_shelterID()}: {shelter.get_avg_evac_time_by_route()}")

def get_base_shelter_id(shelterID: str) -> str:
    """
    ShelterA_1 -> ShelterA
    ShelterA_2 -> ShelterA
    ShelterB_1 -> ShelterB
    ShelterA   -> ShelterA
    """
    parts = shelterID.rsplit("_", 1)

    if len(parts) == 2 and parts[1].isdigit():
        return parts[0]

    return shelterID


def has_multiple_shelters(edgeID_by_shelterID: dict[str, str]) -> bool:
    """
    避難地が2種類以上ある場合 True を返す。
    ShelterA_1 と ShelterA_2 は同じ ShelterA とみなす。
    """
    base_shelter_ids = set()

    for shelterID in edgeID_by_shelterID.keys():
        base_shelter_ids.add(get_base_shelter_id(shelterID))

    return len(base_shelter_ids) >= 2

def extract_category(vehID):
    if "ShelterA_1" in vehID:
        return "A1"
    elif "ShelterA_2" in vehID:
        return "A2"
    elif "ShelterB_1" in vehID:
        return "B1"
    elif "ShelterB_2" in vehID:
        return "B2"
    else:
        return "UNKNOWN"

def get_options():
    optParser = optparse.OptionParser()
    optParser.add_option(
                            "--nogui", action="store_true",
                            default=False, 
                            help="run the commandline version of sumo"
                            )
    options = optParser.parse_args()[0]
    return options


def load_toml(path: Path) -> dict:
    with path.open("rb") as f:
        return _toml_loader.load(f)

def _req(cfg: dict, key: str, typ=float):
    if key not in cfg:
        raise KeyError(f"Config missing required key: '{key}'")
    return typ(cfg[key])

def parse_runner_args(argv=None):
    parser = argparse.ArgumentParser(
        description="runner_simulator using TOML config",
        add_help=False  # get_options() と競合しないように独自help無効化
    )
    parser.add_argument("--config", required=True, help="Path to TOML config file")
    parser.add_argument("--nogui", action="store_true", help="Run SUMO without GUI")
    parser.add_argument("--early-rate", type=float, help="Override early_rate value from TOML")

    # argvを明示的に渡す（Noneならsys.argv[1:]）
    args, remaining = parser.parse_known_args(argv)
    return args, remaining

if __name__ == "__main__":
    # python3 -m scenarios.dicomo2026.map_one.simulation.runner_simulator --nogui scenarios/its105/configs/config_scenario_1.toml 0.5 1.0
    toml_path = sys.argv[2]
    early_rate:float= float(sys.argv[3]) 
    v2v_capable_vehicle_rate:float=float(sys.argv[4])

    options = get_options()
    if options.nogui:
        sumoBinary = checkBinary('sumo')
    else:
        sumoBinary = checkBinary('sumo-gui')
    cfg = load_toml(Path(toml_path))
    COMM_RANGE: float = _req(cfg, "comm_range", float)
    NUM_VEHICLES: int = _req(cfg, "num_vehicles", int)
    VEHICLE_INTERVAL: float = _req(cfg, "vehicle_interval", float)
    CHICE_SHORTEST_ROUTE_RATE: float = _req(cfg, "choice_shortest_route_rate", float)
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
    ACTIVE_VEHICLE_ABANDANTMENT_THRESHOLD_CENTER: float = _req(cfg, "active_vehicle_abandantment_threshold_center", float)
    ACTIVE_VEHICLE_ABANDANTMENT_TTHRESHOLD_SPREAD: float = _req(cfg, "active_vehicle_abandantment_threshold_spread", float)
    CAUTIOUS_VEHICLE_ABANDANTMENT_TTHRESHOLD_CENTER: float = _req(cfg, "cautious_vehicle_abandantment_threshold_center", float)
    CAUTIOUS_VEHICLE_ABANDANTMENT_TTHRESHOLD_SPREAD: float = _req(cfg, "cautious_vehicle_abandantment_threshold_spread", float)

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
    P_FOLLOW: float = _req(cfg, "p_follow", float)

    traci.start(
                [sumoBinary,
                    "-c", str(SUMO_CFG),
                    "--tripinfo-output", "tripinfo.xml",
                    "--tls.all-off", "true" ,
                    "--time-to-teleport", "1000" 
                ],
                # traceFile="traci_log.txt",
                # traceGetters=False,

                )

    # 避難地の情報をもとに、Shelter一覧を生成
    shelter_capacity_by_ID:dict = {"ShelterA_1": 200, "ShelterA_2": 200}
    edgeID_by_shelterID:dict = {"ShelterA_1": 'E16', "ShelterA_2": 'E13'}
    shelter_choice_prob_list = [CHICE_SHORTEST_ROUTE_RATE, 1-CHICE_SHORTEST_ROUTE_RATE]
    for shelterID, near_edgeID in edgeID_by_shelterID.items():
        shelter_list:list[Shelter] = init_shelter(
                                                    shelterID=shelterID, 
                                                    shelter_capacity_by_ID=shelter_capacity_by_ID, 
                                                    near_edgeID=near_edgeID, 
                                                    shelter_list=shelter_list
                                                    )
        
    custome_edge_list:list = init_custome_edge()
    vehicle_start_edges:list = get_vehicle_start_edges(custome_edge_list=custome_edge_list)
    vehicle_end_edges:list = get_vehicle_end_edges(custome_edge_list=custome_edge_list)
    # 車両の開始エッジと終了エッジの組み合わせを辞書にする
    get_vehicle_end_list_by_start_edge_dict(vehicle_start_edges=vehicle_start_edges, vehicle_end_edges=vehicle_end_edges)
    # 全経路で総当たりをし、通行可能経路を取得しておく。
    import_connected_edges_from_json(file_path=str(DATA_DIR / "all_edgeIDs.json"))
    nearest_end_edgeID_by_start_edgeID_dict:dict = import_start_end_edgeIDs_from_json(file_path=str(DATA_DIR / "start_end_edgeIDs_ishinomaki_two_shelter.json"))
    all_route_edgeID_list_by_routeID:dict = convert_routefile_to_routes_by_id(file_path=str(DATA_DIR / "ishinomaki_one_four_one.rou.xml"))
    routeID_list = list(all_route_edgeID_list_by_routeID.keys())

    for start_end_edges, edges_list in all_route_edgeID_list_by_routeID.items():
        max_route_num = 0
        start_edgeIDs, end_edgeID, routeIndex = start_end_edges.split("_")
        max_route_num = max(max_route_num, int(routeIndex))
    route_prob_list = [0.3, 0.7]
    # route_prob_list = utilities.generate_route_prob_list(max_route_num)
    each_vehnum_to_shelter = int(NUM_VEHICLES / len(nearest_end_edgeID_by_start_edgeID_dict))

    mapping = nearest_end_edgeID_by_start_edgeID_dict

    route_choice_rate_by_routeID = {
        "E0_E16_0": 0.80,  # nearest-shelter shortest route
        "E0_E13_0": 0.10,  # route to Shelter E13 via the short-side corridor
        "E0_E13_1": 0.05,  # route to Shelter E13 via the detour corridor
        "E0_E16_1": 0.05,  # route to Shelter E16 via the detour corridor
    }

    # 再現性が不要なら None にする
    ROUTE_ASSIGNMENT_SEED = 42

    available_routeIDs = list(traci.route.getIDList())

    validate_route_choice_rates(
        route_choice_rate_by_routeID=route_choice_rate_by_routeID,
        available_routeIDs=available_routeIDs,
    )

    route_vehicle_count_by_routeID = allocate_vehicle_counts_by_route(
        total_vehicle_count=NUM_VEHICLES,
        route_choice_rate_by_routeID=route_choice_rate_by_routeID,
    )

    assigned_routeID_list = build_assigned_routeID_list(
        route_vehicle_count_by_routeID=route_vehicle_count_by_routeID,
        seed=ROUTE_ASSIGNMENT_SEED,
    )

    # route ID ごとの割り当て結果を表示
    route_assignment_counter = Counter(assigned_routeID_list)
    route_assignment_total = sum(route_assignment_counter.values())

    print("Route assignment counts:")
    for routeID in route_choice_rate_by_routeID.keys():
        count = route_assignment_counter.get(routeID, 0)
        rate = count / route_assignment_total if route_assignment_total > 0 else 0.0
        print(f"  {routeID}: {count} ({rate:.2%})")
    print(f"  Total: {route_assignment_total}")

    if route_assignment_total != NUM_VEHICLES:
        raise ValueError(
            f"Assigned vehicle count mismatch: "
            f"route_assignment_total={route_assignment_total}, NUM_VEHICLES={NUM_VEHICLES}"
        )

    vehID_list = []

    start_interval = 4.0
    end_interval = 3.0

    vehicle_intervals = np.linspace(
        start_interval,
        end_interval,
        num=int(NUM_VEHICLES)
    )

    DEPART_TIME = 0.0

    for vehicle_index, assigned_routeID in enumerate(assigned_routeID_list):
        from_edgeID, to_edgeID, route_index = parse_route_id(assigned_routeID)

        target_shelterID = find_shelterID_by_edgeID_by_shelterID(
            edgeID=to_edgeID,
            edgeID_by_shelterID=edgeID_by_shelterID
        )

        if target_shelterID is None:
            raise ValueError(
                f"No shelter found for to_edgeID={to_edgeID}. "
                f"assigned_routeID={assigned_routeID}, "
                f"edgeID_by_shelterID={edgeID_by_shelterID}"
            )

        VEHICLE_NUM, ROUTE_NUM, vehID_list_by_shelter, DEPART_TIME = \
            generate_simple_init_vehID_based_one_routefile(
                from_edgeID=from_edgeID,
                to_edgeID=to_edgeID,
                shelterID=target_shelterID,
                generate_interval=float(vehicle_intervals[vehicle_index]),
                generate_route_count=ROUTE_NUM,
                generate_veh_count=VEHICLE_NUM,
                depart_time=DEPART_TIME,

                # 追加
                routeID=assigned_routeID,
                route_index=route_index,
            )

        vehID_list.extend(vehID_list_by_shelter)


    # ============================================================
    # 既存のカテゴリ集計は維持
    # ============================================================

    counter = Counter(extract_category(v) for v in vehID_list)

    total = sum(counter.values())
    print("出現数:", dict(counter))
    print("割合:")
    print(f"  Total: {total}")

    for cat in ["A1", "A2", "B1", "B2"]:
        count = counter.get(cat, 0)
        rate = count / total if total > 0 else 0.0
        print(f"  {cat}: {count} ({rate:.2%})")


    # ============================================================
    # 車両情報の初期化
    # ============================================================

    vehInfo_list: list[VehicleInfo] = init_vehicleInfo_list_base(
        vehIDs=vehID_list,
        shelter_list=shelter_list,
        v2v_capable_vehicle_rate=v2v_capable_vehicle_rate,
    )

    # Agentの初期化
    # 乗り捨てに関する初期化
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

        ACTIVE_VEHICLE_ABANDANTMENT_THRESHOLD_CENTER=ACTIVE_VEHICLE_ABANDANTMENT_THRESHOLD_CENTER,
        ACTIVE_VEHICLE_ABANDANTMENT_TTHRESHOLD_SPREAD=ACTIVE_VEHICLE_ABANDANTMENT_TTHRESHOLD_SPREAD,
        CAUTIOUS_VEHICLE_ABANDANTMENT_TTHRESHOLD_CENTER=CAUTIOUS_VEHICLE_ABANDANTMENT_TTHRESHOLD_CENTER,
        CAUTIOUS_VEHICLE_ABANDANTMENT_TTHRESHOLD_SPREAD=CAUTIOUS_VEHICLE_ABANDANTMENT_TTHRESHOLD_SPREAD,

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

    vehInfo_by_vehID_dict = {vehInfo.get_vehID(): vehInfo for vehInfo in vehInfo_list}
    agent_by_vehID_dict = {agent.get_vehID(): agent for agent in agent_list}

    run()

    _print_route_change_count_consistency()

    print(f"mean elapsed_time: {np.mean(elapsed_time_list)}")
    if len(arrival_time_by_vehID_dict) == NUM_VEHICLES:
        print("OK all vehs arrived ")
    else:
        print(f"NG all vehs not arrived {len(arrival_time_by_vehID_dict)}")
    check_vehicle_abandonment_count = 0

    avg_congestion_duration = 0.0
    total_congestion_duration = 0.0
    for agent in agent_list:
        agent_encounted_congestion_time = agent.get_encounted_congestion_time()
        agent_arrival_time = agent.get_arrival_time()
        total_congestion_duration += max(0.0, agent_arrival_time - agent_encounted_congestion_time) if agent_encounted_congestion_time is not None and agent_arrival_time is not None else 0.0 
        
        if agent.get_vehicle_abandoned_flg():
            check_vehicle_abandonment_count += 1
    avg_congestion_duration = total_congestion_duration / len(agent_list) if len(agent_list) > 0 else 0.0
    if avg_congestion_duration > 0:
        print(f"平均渋滞継続時間: {avg_congestion_duration:.2f}秒")
    if check_vehicle_abandonment_count == PEDESTRIAN_COUNT:
        print("OK all vehicle abandonment were detected")
    else:
        print(f"NG not all vehicle abandonment were detected {check_vehicle_abandonment_count} / {PEDESTRIAN_COUNT}")
    
    print("===== Simlation Result Summary =====")
    print(f"avg_congestion_duration: {avg_congestion_duration:.2f} ")
    print(f"arrival_time_by_vehID_dict:{arrival_time_by_vehID_dict}")
    print(f"vehicle_abandant_time_by_pedestrianID_dict:{vehicle_abandant_time_by_pedestrianID_dict}")
    print(f"walking_distance_by_pedestrianID_dict:{walking_distance_by_pedestrianID_dict}")
    print(f"route_change_time_by_vehID_dict:{route_change_time_by_vehID_dict}")
    print(f"pedestrian_count:{PEDESTRIAN_COUNT}")
    print(f"route_changed_vehicle_count:{ROUTE_CHANGED_VEHICLE_COUNT}")
    print(f"rate_vehicle_abandonment:{VEHICLE_ABANDONMENT_SUCCESS_COUNT / VEHICLE_NUM * 100:.2f}%")
    print(f"wrong_way_driving_count:{WRONG_WAY_SUCCESS_COUNT}")
    print(f"vehicle_abandonment_count:{VEHICLE_ABANDONMENT_SUCCESS_COUNT}")
    print(f"normalcy_bias_route_change_count:{NORMALCY_BIAS_ROUTE_CHANGE_COUNT}")
    print(f"majority_bias_route_change_count:{MAJORITY_BIAS_ROUTE_CHANGE_COUNT}")
    print(f"lane_changed_vehicle_count:{LANE_CHANGED_VEHICLE_COUNT}")
    print(f"info_obtained_lanechange_count:{OBTAIN_INFO_LANE_CHANGE_COUNT}")
    print(f"elapsed_time_lanechange_count:{ELAPSED_TIME_LANE_CHANGE_COUNT}")
    print(f"majority_bias_lanechange_count:{POSITIVE_MAJORITY_BIAS_COUNT}")
