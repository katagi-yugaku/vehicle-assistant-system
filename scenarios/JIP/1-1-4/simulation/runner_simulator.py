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
    global ROUTE_CHANGED_VEHICLE_COUNT
    global NEW_VEHICLE_COUNT
    global ROUTE_CHANGE_COUNT_AFTER_SHELTER_CAPACITY_FULL
    global VEHICLE_ABANDONMENT_SUCCESS_COUNT
    global WRONG_WAY_SUCCESS_COUNT
    global NORMALCY_BIAS_ROUTE_CHANGE_COUNT
    global MAJORITY_BIAS_ROUTE_CHANGE_COUNT

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
        if current_edgeID in["E13", "E16"] and not vehInfo_by_current_vehID.get_arrival_flag():
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
        if vehInfo_by_current_vehID.get_vehicle_comm_enabled_flag() and not vehInfo_by_current_vehID.get_arrival_flag() :
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
                # VEHINFO: 津波接近に関する情報を取得したか否かを確認する
                try:
                    # VEHINFO: 満杯情報を取得したか否かを確認する
                    if (
                        not agent_by_current_vehID.get_shelter_full_info_obtained_flg()
                        and vehInfo_by_current_vehID.has_shelter_full_info(shelterID=agent_by_current_vehID.get_target_shelter(), threshold=SHELTER_FULL_INFO_THRESHOLD)
                        and vehInfo_by_current_vehID.get_vehicle_comm_enabled_flag()
                        ):
                        # print(f"Vehicle {current_vehID} obtained shelter full information for shelter {agent_by_current_vehID.get_target_shelter()} at time {current_time}")
                        agent_by_current_vehID.set_shelter_full_info_obtained_time(current_time)
                        agent_by_current_vehID.set_shelter_full_info_obtained_flg(True)

                except Exception as e:
                    vehInfo_by_current_vehID.print_all_info()
                    print(f"Error during information obtaining check for vehicle {current_vehID} at time {traci.simulation.getTime()}: {e}")

            if is_vehID_in_congested_edge(
                vehID=current_vehID,
                threshold_speed=THRESHOLD_SPEED,
            ):
                traci.vehicle.setColor(current_vehID, (255, 0, 0))

                if not agent_by_current_vehID.get_encounted_congestion_flg():
                    agent_by_current_vehID.set_encounted_congestion_time(current_time)
                    agent_by_current_vehID.set_encounted_congestion_flg(True)

                # すでに経路変更済みなら処理しない
                # if agent_by_current_vehID.get_evacuation_route_changed_flg():
                #     continue

                # try:
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
                    print(f"Vehicle {current_vehID} is considering changing route due to shelter full information. From edge: {from_edgeID}, New shelter: {shelterID}, To edges: {to_edge_list}")
                    if from_edgeID != "" and shelterID != "" and to_edge_list != "":
                        NEW_VEHICLE_COUNT = generate_new_veh_based_on_route_time(
                            target_vehID=current_vehID,
                            NEW_VEHICLE_COUNT=NEW_VEHICLE_COUNT,
                            agent_list=agent_list,
                            vehInfo_list=vehInfo_list,
                            vehInfo_by_target_vehID=vehInfo_by_current_vehID,
                            agent_by_target_vehID=agent_by_current_vehID,
                            from_edgeID=from_edgeID,
                            new_shelterID=shelterID,
                            to_edgeID=to_edge_list[0],
                            color_mode="",
                            agent_by_vehID_dict=agent_by_vehID_dict,
                            vehInfo_by_vehID_dict=vehInfo_by_vehID_dict,
                        )
                        agent_by_current_vehID.set_evacuation_route_changed_flg(True)
                        agent_by_current_vehID.set_agent_action_name("rc")
                        route_change_time_by_vehID_dict[current_vehID] = current_time
                        ROUTE_CHANGED_VEHICLE_COUNT += 1
                        ROUTE_CHANGE_COUNT_AFTER_SHELTER_CAPACITY_FULL += 1
                
                else:
                    # ============================================================
                    # 2. 正常性バイアスによる経路変更
                    # ============================================================
                    if is_step_10 :
                        # ここでに題がある
                        from_edgeID, shelterID, to_edge_list, congestion_flg = (
                            get_route_time_difference_exceeding_threshold(
                                current_edgeID=current_edgeID,
                                agent_by_target_vehID=agent_by_current_vehID,
                                shelter=shelter_for_current_vehID,
                                vehInfo_by_target_vehID=vehInfo_by_current_vehID,
                                shelter_list=shelter_list,
                                custome_edge_list=custome_edge_list,
                            )
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
                                if routeID is not None:
                                    traci.vehicle.setRouteID(current_vehID, routeID)
                                    traci.vehicle.setColor(current_vehID, (60, 180, 120))
                                    agent_by_current_vehID.set_evacuation_route_changed_flg(True)
                                    agent_by_current_vehID.set_agent_action_name("rc")
                                    ROUTE_CHANGED_VEHICLE_COUNT += 1
                                    NORMALCY_BIAS_ROUTE_CHANGE_COUNT += 1
                                    route_change_time_by_vehID_dict[current_vehID] = current_time
                            elif current_edgeID in ["E13","E14", "E15", "E16", "E12", "E5"]: 
                                continue
                                # print(f"Vehicle {current_vehID} is on edge {current_edgeID} and cannot change route due to normalcy bias.")
                            else:
                                if len(to_edge_list[0]) == 0:
                                        print(f"nor malcy biasto_edge_list[0]: {to_edge_list[0]}")
                                        sys.exit()
                                print(f"{current_vehID} is {current_edgeID} changed to {to_edge_list[0]} due to normalcy bias at time {current_time}")
                                if from_edgeID != "" and shelterID != "" and to_edge_list != "":
                                    NEW_VEHICLE_COUNT = generate_new_veh_based_on_route_time(
                                        target_vehID=current_vehID,
                                        NEW_VEHICLE_COUNT=NEW_VEHICLE_COUNT,
                                        agent_list=agent_list,
                                        vehInfo_list=vehInfo_list,
                                        vehInfo_by_target_vehID=vehInfo_by_current_vehID,
                                        agent_by_target_vehID=agent_by_current_vehID,
                                        from_edgeID=from_edgeID,
                                        new_shelterID=shelterID,
                                        to_edgeID=to_edge_list[0],
                                        color_mode="",
                                        agent_by_vehID_dict=agent_by_vehID_dict,
                                        vehInfo_by_vehID_dict=vehInfo_by_vehID_dict,
                                    )
                                    agent_by_current_vehID.set_evacuation_route_changed_flg(True)
                                    agent_by_current_vehID.set_agent_action_name("rc")
                                    ROUTE_CHANGED_VEHICLE_COUNT += 1
                                    NORMALCY_BIAS_ROUTE_CHANGE_COUNT += 1
                                    route_change_time_by_vehID_dict[current_vehID] = current_time
                                
                        # ============================================================
                        # 3. 同調バイアスによる経路変更
                        #    正常性バイアスで変更しなかった場合のみ
                        # ============================================================
                        else:
                            rc_around_count = count_rc_around_vehicles(
                                agent=agent_by_current_vehID,
                                vehInfo=vehInfo_by_current_vehID,
                                agent_list=agent_list,
                                candidate_action="rc",
                                agent_by_vehID_dict=agent_by_vehID_dict,
                                custome_edge_list=custome_edge_list,
                                current_edgeID=current_edgeID,
                                distance_threshold=50.0,
                            )
                            if rc_around_count > 0 and random.random() < P_FOLLOW:
                                if current_edgeID in ["E1", "E20"]:
                                    routeID = find_alternative_route_calculated_time(
                                        current_edgeID=current_edgeID,
                                        vehInfo=vehInfo_by_current_vehID,
                                        agent=agent_by_current_vehID,
                                        shelter_list=shelter_list,
                                        custome_edge_list=custome_edge_list,
                                    )
                                    if routeID is not None:
                                        traci.vehicle.setRouteID(current_vehID, routeID)
                                        traci.vehicle.setColor(current_vehID, (60, 180, 120))
                                        agent_by_current_vehID.set_evacuation_route_changed_flg(True)
                                        agent_by_current_vehID.set_agent_action_name("rc")
                                        ROUTE_CHANGED_VEHICLE_COUNT += 1
                                        MAJORITY_BIAS_ROUTE_CHANGE_COUNT += 1
                                        route_change_time_by_vehID_dict[current_vehID] = current_time
                                elif current_edgeID in ["E13","E14", "E15", "E16", "E12", "E5"]: 
                                    continue
                                    # print(f"Vehicle {current_vehID} is on edge {current_edgeID} and cannot change route due to normalcy bias.")
                                else:
                                    print("check10")
                                    from_edgeID, shelterID, to_edge_list = (
                                                                            find_uturn_shortest_route_to_current_shelter_group(
                                                                                current_edgeID=current_edgeID,
                                                                                vehID=current_vehID,
                                                                                vehInfo=vehInfo_by_current_vehID,
                                                                                agent=agent_by_current_vehID,
                                                                                shelter_list=shelter_list,
                                                                            )
                                                                        )
                                    print("check11")
                                    if from_edgeID != "" and shelterID != "" and to_edge_list != "":
                                        NEW_VEHICLE_COUNT = generate_new_veh_based_on_route_time(
                                            target_vehID=current_vehID,
                                            NEW_VEHICLE_COUNT=NEW_VEHICLE_COUNT,
                                            agent_list=agent_list,
                                            vehInfo_list=vehInfo_list,
                                            vehInfo_by_target_vehID=vehInfo_by_current_vehID,
                                            agent_by_target_vehID=agent_by_current_vehID,
                                            from_edgeID=from_edgeID,
                                            new_shelterID=shelterID,
                                            to_edgeID=to_edge_list[0],
                                            color_mode="",
                                            agent_by_vehID_dict=agent_by_vehID_dict,
                                            vehInfo_by_vehID_dict=vehInfo_by_vehID_dict,
                                        )
                                        agent_by_current_vehID.set_evacuation_route_changed_flg(True)
                                        agent_by_current_vehID.set_agent_action_name("rc")
                                        ROUTE_CHANGED_VEHICLE_COUNT += 1
                                        MAJORITY_BIAS_ROUTE_CHANGE_COUNT += 1
                                        route_change_time_by_vehID_dict[current_vehID] = current_time
                # ecept Exception as e:
                #    print(
                #         f"Error during route change for vehicle {current_vehID} "
                #         f"at time {traci.simulation.getTime()}: {e}"
                #     )
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
    shelter_choice_prob_list = [CHICE_SHORTEST_ROUTE_RATE, 0.1]
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
    # 車両情報の初期化は既存処理を維持
    # ============================================================

    vehInfo_list: list[VehicleInfo] = init_vehicleInfo_list_base(
        vehIDs=vehID_list,
        shelter_list=shelter_list,
        v2v_capable_vehicle_rate=v2v_capable_vehicle_rate
    )

    # カテゴリのカウント
    counter = Counter(extract_category(v) for v in vehID_list)

    # 合計と割合を表示
    total = sum(counter.values())
    print("出現数:", dict(counter))
    print("割合:")
    print(f"  Total: {total}")
    for cat in ["A1", "A2", "B1", "B2"]:
        count = counter.get(cat, 0)
        print(f"  {cat}: {count} ({count / total:.2%})")
    # 車両情報の初期化
    # ここで調整がいる
    vehInfo_list:list[VehicleInfo] = init_vehicleInfo_list_base(vehIDs=vehID_list, shelter_list=shelter_list, v2v_capable_vehicle_rate=v2v_capable_vehicle_rate) 

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


