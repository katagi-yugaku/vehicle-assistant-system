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
SUMO_CFG = DATA_DIR / "ishinomaki_two_shelter.sumocfg"

# =========================
# 1) シミュレーション基本設定・時間評価
# =========================
END_SIMULATION_TIME = 4500
THRESHOLD_SPEED = 2.00 # 7.2km/h
STOPPING_TIME_IN_SHELTER = 10000000
SLOW_DURATION = 15.0
SLOW_SPEED = 3.0

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




def run():
    while traci.simulation.getTime() < END_SIMULATION_TIME:
        traci.simulationStep()
        control_vehicles()
    traci.close()
    sys.stdout.flush()


def reset_motivation_after_action(agent: Agent, current_time: float):
    """
    行動決意後にモチベーションをリセットする。

    目的:
      - 基礎モチベーション B(t) の経過時間を 0 に戻す
      - 情報取得後の活性化時間も 0 に戻す
      - 行動直後に別の行動が連続して発生することを抑制する

    注意:
      - 情報取得フラグ自体は消さない
      - 情報を忘れるのではなく，
        その情報による心理的上昇を現在時刻から再計算する
    """
    agent.set_encounted_congestion_time(current_time)

    if agent.get_shelter_full_info_obtained_flg():
        agent.set_shelter_full_info_obtained_time(current_time)

    if agent.get_tsunami_info_obtained_flg():
        agent.set_tsunami_info_obtained_time(current_time)

    if agent.get_route_congestion_info_obtained_flg():
        agent.set_route_congestion_info_obtained_time(current_time)


def control_vehicles():
    vehIDs = traci.vehicle.getIDList()
    pedestrianIDs = traci.person.getIDList()
    current_time = traci.simulation.getTime()
    is_step_5 = (current_time % 5 == 0)
    is_step_10 = (current_time % 10 == 0)
    step_cache = create_step_cache(current_time=current_time)
    route_avg_dirty = False

    global PEDESTRIAN_COUNT
    global ROUTE_CHANGED_VEHICLE_COUNT
    global NEW_VEHICLE_COUNT
    global ROUTE_CHANGE_COUNT_AFTER_SHELTER_CAPACITY_FULL
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

        # # VEHINFO: 到着処理
        # if (
        #     traci.vehicle.isStoppedParking(current_vehID)
        #     and not vehInfo_by_current_vehID.get_parked_flag()
        # ):
        #     handle_arrival(
        #         current_vehID=current_vehID,
        #         vehInfo_by_current_vehID=vehInfo_by_current_vehID,
        #         agent_by_current_vehID=agent_by_current_vehID,
        #         shelter_for_current_vehID=shelter_for_current_vehID,
        #         shelter_list=shelter_list,
        #         arrival_time_list=arrival_time_list,
        #         arrival_time_by_vehID_dict=arrival_time_by_vehID_dict,
        #         elapsed_time_list=elapsed_time_list,
        #     )
        # 到着処理を行う
        if current_edgeID in["E2", "E3"] and not vehInfo_by_current_vehID.get_arrival_flag():
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
            traci.vehicle.remove(current_vehID)
            continue

        # 逆走車両とUターン経路変更車両が向かい合っているかを確認する
        if agent_by_current_vehID.get_wrong_way_driving_flg():
            last_encounter_time = (agent_by_current_vehID.get_wrong_way_driving_encounted_other_vehicle_time())
            # 直近15秒以内に対向車両を検知していたら，減速を継続する
            if (
                last_encounter_time >= 0.0
                and current_time - last_encounter_time <= SLOW_DURATION
            ):
                traci.vehicle.setSpeed(current_vehID, SLOW_SPEED)
                traci.vehicle.setColor(current_vehID, (255, 255, 0))
            else:
                # 15秒以上経過している場合は，通常速度制御に戻す
                traci.vehicle.setSpeed(current_vehID, -1)
                # 新しく対向車両を検知する
                opposite_edgeID = get_opposite_edgeID_by_edgeID(current_edgeID)
                opposite_vehIDs = traci.edge.getLastStepVehicleIDs(opposite_edgeID)
                current_lane_index = int(traci.vehicle.getLaneID(current_vehID).rsplit("_", 1)[1])
                for opposite_vehID in opposite_vehIDs:
                    opposite_lane_index = int(traci.vehicle.getLaneID(opposite_vehID).rsplit("_", 1)[1])
                    if opposite_lane_index == 1 and current_lane_index == 2:
                        # 対向車両を検知した時刻を記録
                        agent_by_current_vehID.set_wrong_way_driving_encounted_other_vehicle_time(current_time)
                        # 検知した瞬間から減速開始
                        traci.vehicle.setSpeed(current_vehID, SLOW_SPEED)
                        traci.vehicle.setColor(current_vehID, (255, 255, 0))
                        # print(
                        #     f"Vehicle {current_vehID} is slowing down due to facing "
                        #     f"opposite vehicle on edge {current_edgeID} at time {current_time}"
                        # )
                        break


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
                    v_free=5.0,
                    v_min=2.0,
                    gap_min=7.0,
                    tau=1.8,
                    alpha=0.5,
                    slow_time=1.0,
                )
            # else:
            #     traci.vehicle.slowDown(current_vehID, 5.0, 1.0)

        # V2V: 車両間通信による情報共有
        if vehInfo_by_current_vehID.get_vehicle_comm_enabled_flag():
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
                v2v_communication_about_tsunami_info(
                    target_vehID=current_vehID,
                    target_vehInfo=vehInfo_by_current_vehID,
                    around_vehIDs=around_vehIDs,
                    vehInfo_list=vehInfo_list,
                    COMMUNICATION_RANGE=COMM_RANGE,
                    target_position=current_position,
                    vehInfo_by_vehID_dict=vehInfo_by_vehID_dict,
                    step_cache=step_cache,
                )

        # VEHINFO: 到着していない車両に対して処理を行う
        if not vehInfo_by_current_vehID.get_arrival_flag():

            if vehInfo_by_current_vehID.get_vehicle_comm_enabled_flag():
                # VEHINFO: 津波接近に関する情報を取得したか否かを確認する
                try:
                    if (
                        not agent_by_current_vehID.get_tsunami_info_obtained_flg()
                        and vehInfo_by_current_vehID.has_tsunami_precursor_info()
                        and vehInfo_by_current_vehID.get_vehicle_comm_enabled_flag()
                    ):
                        agent_by_current_vehID.set_tsunami_info_obtained_time(current_time)
                        agent_by_current_vehID.set_tsunami_info_obtained_flg(True)
                    # VEHINFO: 満杯情報を取得したか否かを確認する
                    if (
                        not agent_by_current_vehID.get_shelter_full_info_obtained_flg()
                        and vehInfo_by_current_vehID.has_shelter_full_info(shelterID=agent_by_current_vehID.get_target_shelter(), threshold=0.99)
                        and vehInfo_by_current_vehID.get_vehicle_comm_enabled_flag()
                        ):
                        agent_by_current_vehID.set_shelter_full_info_obtained_time(current_time)
                        agent_by_current_vehID.set_shelter_full_info_obtained_flg(True)
                    
                    # VEHINFO: 経路の混雑情報を取得したか否かを確認する
                    if (
                        not agent_by_current_vehID.get_route_congestion_info_obtained_flg()
                        and is_route_time_difference_exceeding_threshold(current_edgeID=current_edgeID, agent_by_target_vehID=agent_by_current_vehID, 
                                                                                shelter=shelter_for_current_vehID, vehInfo_by_target_vehID=vehInfo_by_current_vehID, 
                                                                                shelter_list=shelter_list, custome_edge_list=custome_edge_list)
                        and vehInfo_by_current_vehID.get_vehicle_comm_enabled_flag()
                        ):
                        agent_by_current_vehID.set_route_congestion_info_obtained_time(current_time)
                        agent_by_current_vehID.set_route_congestion_info_obtained_flg(True)
                except Exception as e:
                    vehInfo_by_current_vehID.print_all_info()
                    print(f"Error during information obtaining check for vehicle {current_vehID} at time {traci.simulation.getTime()}: {e}")


            if  is_step_10 and is_vehID_in_congested_edge(vehID=current_vehID, threshold_speed=THRESHOLD_SPEED):
                traci.vehicle.setColor(current_vehID, (255, 0, 0)) # 赤色に変更して、渋滞していることを視覚的に示す
                if not agent_by_current_vehID.get_encounted_congestion_flg():
                    agent_by_current_vehID.set_encounted_congestion_time(current_time)
                    agent_by_current_vehID.set_encounted_congestion_flg(True)

                # MOTIVATION: モチベーションの計算
                # AGENT: 経路変更の意思決定
                rc_current_value = calculate_motivation_for_evacuation_action(
                        agent=agent_by_current_vehID, 
                        agent_list=agent_list, 
                        vehInfo=vehInfo_by_current_vehID, 
                        current_time=current_time, 
                        action="rc", 
                        debug=False, 
                        agent_by_vehID_dict=agent_by_vehID_dict,
                        custome_edge_list=custome_edge_list)
                if rc_current_value > agent_by_current_vehID.get_route_change_threshold() and not agent_by_current_vehID.get_evacuation_route_changed_flg():
                    # 実行可能性を検証するために、別レーンにいる車両とのこの距離を測定して、判定する
                    # 隣のレーンが埋まっていたら、経路変更は実施不能と判断して、フラグを立てない
                    # 経路変更
        
                    try:
                        route_changed = False
                        if current_edgeID in ["E106","E105","E104","E130","E129","E128"]:
                            # 避難地周辺の道路上ではUターンによって経路変更を行う
                            # Uターンを試みる
                            from_edgeID, shelterID, to_edge_list = find_alternative_shelter_choice(
                                                                                                    current_target_shelterID=agent_by_current_vehID.get_target_shelter(),
                                                                                                    vehID=current_vehID,
                                                                                                    current_edgeID=current_edgeID,
                                                                                                    vehInfo=vehInfo_by_current_vehID,
                                                                                                    shelter_list=shelter_list,
                                                                                                    agent=agent_by_current_vehID,
                                                                                                    )
                            if from_edgeID != "" and shelterID != "" and to_edge_list != "":
                                # print(f"from_edgeID: {from_edgeID} shelterID: {shelterID} to_edgeID: {to_edgeID}")
                                NEW_VEHICLE_COUNT = generate_new_veh_based_on_route_time(
                                                                                    target_vehID=current_vehID, 
                                                                                    NEW_VEHICLE_COUNT= NEW_VEHICLE_COUNT, 
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
                                # print("Route Uturn change for vehicle {} at time {}: new shelterID {}, new routeID {}".format(current_vehID, traci.simulation.getTime(), shelterID, to_edge_list[0]))
                                route_changed = True
                                ROUTE_CHANGED_VEHICLE_COUNT += 1
                        else:
                            # そうではない場合は、通常の経路変更を行う
                            routeID = find_alternative_route_calculated_time(
                                                                current_edgeID=current_edgeID,
                                                                vehInfo=vehInfo_by_current_vehID,
                                                                agent=agent_by_current_vehID,
                                                                shelter_list=shelter_list,
                                                                custome_edge_list=custome_edge_list
                                                                )
                            if routeID is not None and not agent_by_current_vehID.get_evacuation_route_changed_flg():
                                # print("Route change for vehicle {} at time {}: new routeID {}".format(current_vehID, traci.simulation.getTime(), routeID))
                                traci.vehicle.setRouteID(current_vehID, routeID)
                                traci.vehicle.setColor(current_vehID,(60, 180, 120))
                                agent_by_current_vehID.set_evacuation_route_changed_flg(True)
                                ROUTE_CHANGED_VEHICLE_COUNT += 1
                                route_changed = True
                        if route_changed:
                            agent_by_current_vehID.set_evacuation_route_changed_flg(True)
                            agent_by_current_vehID.set_agent_action_name("rc")
                            reset_motivation_after_action(
                                agent=agent_by_current_vehID,
                                current_time=current_time,
                            )
                            
                            continue  # 重要：この車両のww/va判定へ進まない
                        
                    except Exception as e:
                        print(f"Error during route change for vehicle {current_vehID} at time {traci.simulation.getTime()}: {e}")
                        continue

                # AGENT: 逆走行為の意思決定
                ww_current_value = calculate_motivation_for_evacuation_action(agent=agent_by_current_vehID, agent_list=agent_list, vehInfo=vehInfo_by_current_vehID, current_time=current_time, action="ww", debug=False, agent_by_vehID_dict=agent_by_vehID_dict, custome_edge_list=custome_edge_list)
                if ww_current_value > agent_by_current_vehID.get_wrong_way_driving_threshold() and not agent_by_current_vehID.get_wrong_way_driving_flg():
                    # 実行可能性を検証するために、別レーンにいる車両とのこの距離を測定して、判定する
                    ww_changed = False
                    try:
                        if current_edgeID not in ["E2", "E3", "E107", "E130", "-E130", "-E129", "-E128", "-E104", "-E105", "-E106"]:
                            # print("逆走行為を行います current_vehID: {} current_edgeID: {}".format(current_vehID, current_edgeID))
                            # current_road_id = traci.vehicle.getRoadID(current_vehID)
                            traci.vehicle.changeLane(
                                vehID=current_vehID,
                                laneIndex=2,
                                duration=1000.0,
                            )
                            # current_shelter = agent_by_current_vehID.get_target_shelter()
                            # next_shelter = current_shelter[:-2] + "_1"
                            # agent_by_current_vehID.set_target_shelter(next_shelter)
                            traci.vehicle.setLaneChangeMode(current_vehID, 512)
                            agent_by_current_vehID.set_wrong_way_driving_flg(True)
                            agent_by_current_vehID.set_agent_action_name("ww")
                            ww_changed = True

                    except Exception as e:
                        print(f"Error during wrong way driving for vehicle {current_vehID} at time {traci.simulation.getTime()}: {e}")
                        continue
                    
                    if ww_changed:
                        agent_by_current_vehID.set_wrong_way_driving_flg(True)
                        agent_by_current_vehID.set_agent_action_name("ww")
                        reset_motivation_after_action(
                            agent=agent_by_current_vehID,
                            current_time=current_time,
                        )
                        WRONG_WAY_SUCCESS_COUNT += 1
                        continue
                # else:
                #     if current_vehID == "init_ShelterB_1_250":
                #         print("逆走行為を行いません current_vehID: {} current_edgeID: {} ww_current_value: {} threshold: {} current_time: {}".format(current_vehID, current_edgeID, ww_current_value, agent_by_current_vehID.get_wrong_way_driving_threshold(), current_time))
                # AGENT: 車両乗り捨て行為の意思決定
                va_current_value = calculate_motivation_for_evacuation_action(agent=agent_by_current_vehID, agent_list=agent_list, vehInfo=vehInfo_by_current_vehID, current_time=current_time, action="va", debug=False, agent_by_vehID_dict=agent_by_vehID_dict, custome_edge_list=custome_edge_list)
                if va_current_value > agent_by_current_vehID.get_vehicle_abandoned_threshold() and not agent_by_current_vehID.get_vehicle_abandoned_flg():
                    try:
                        if current_edgeID not in ["-E130", "-E129", "-E128", "-E104", "-E105", "-E106"]:
                            # 車両乗り捨ては実行可能なはずなので、すぐにフラグを立てる
                            PEDESTRIAN_COUNT, pedestrianID, walking_distance = vehicle_abandant_behavior(
                                current_vehID=current_vehID,
                                current_edgeID=current_edgeID,
                                agent_by_current_vehID=agent_by_current_vehID,
                                vehInfo_by_target_vehID=vehInfo_by_current_vehID,
                                PEDESTRIAN_COUNT=PEDESTRIAN_COUNT,
                                STOPPING_TIME_IN_SHELTER=STOPPING_TIME_IN_SHELTER,
                                shelter=shelter_for_current_vehID,
                            )
                            agent_by_current_vehID.set_vehicle_abandoned_flg(vehicle_abandoned_flg=True)
                            traci.vehicle.remove(current_vehID)

                            # 車両乗り捨て後は、経路変更も逆走は実施不能なため、両方のフラグを立てる
                            agent_by_current_vehID.set_vehicle_abandoned_flg(True)
                            agent_by_current_vehID.set_wrong_way_driving_flg(True)
                            agent_by_current_vehID.set_evacuation_route_changed_flg(True)  
                            agent_by_current_vehID.set_agent_action_name("va")
                            VEHICLE_ABANDONMENT_SUCCESS_COUNT += 1
                    except Exception as e:
                        print(f"Error during vehicle abandonment for vehicle {current_vehID} at time {traci.simulation.getTime()}: {e}")
                        continue
                
    # PEDESTRIAN: 乗り捨て後の歩行者に対する処理を行う
    for pedestrianID in pedestrianIDs:
        vehID_by_pedestrianID: str = extract_vehicle_id(pedestrianID)
        agent_by_pedestrianID: Agent = agent_by_vehID_dict.get(vehID_by_pedestrianID)
        vehInfo_by_pedestrianID: VehicleInfo = vehInfo_by_vehID_dict.get(vehID_by_pedestrianID)
        if agent_by_pedestrianID is None or vehInfo_by_pedestrianID is None:
            continue
        shelter_by_pedestrianID: Shelter = find_shelter_by_edgeID_connect_target_shelter(
            vehInfo_by_pedestrianID.get_edgeID_connect_target_shelter(),
            shelter_list,
        )
        if not agent_by_pedestrianID.get_arrival_shelter_flg():
            if traci.person.getRoadID(pedestrianID) in ["E2", "E3"]:
                handle_arrival_for_pedestrian(
                    pedestrianID=pedestrianID,
                    vehInfo_by_current_vehID=vehInfo_by_pedestrianID,
                    agent_by_current_vehID=agent_by_pedestrianID,
                    shelter_for_current_vehID=shelter_by_pedestrianID,
                    shelter_list=shelter_list,
                    arrival_time_list=arrival_time_list,
                    arrival_time_by_vehID_dict=arrival_time_by_vehID_dict,
                    elapsed_time_list=elapsed_time_list,
                )
                route_avg_dirty = True

    # SHELTER: 経路ごとの平均避難時間を計算する
    if route_avg_dirty:
        calculate_avg_evac_time_by_route(shelter_list=shelter_list)
    # SHELTER: 避難地の混雑率を計算する
    for shelter in shelter_list:
        shelter.update_congestion_rate()
        # print(f"shelterID: {shelter.get_shelterID()}, congestion_rate: {shelter.get_congestion_rate()}, arrival_vehIDs: {shelter.get_arrival_vehID_list()}")


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

import sys as _sys
if _sys.version_info >= (3, 11):
    import tomllib as _toml_loader
else:
    try:
        import tomli as _toml_loader
    except ImportError as e:
        raise ImportError("Python 3.10以下では `pip install tomli` が必要です。") from e

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

    traci.start(
                [sumoBinary,
                    "-c", str(SUMO_CFG),
                    "--tripinfo-output", "tripinfo.xml",
                    "--tls.all-off", "false" ,
                    "--time-to-teleport", "1000" 
                ],
                traceFile="traci_log.txt",
                traceGetters=False,

                )
    
    # 避難地の情報をもとに、Shelter一覧を生成
    shelter_capacity_by_ID:dict = {"ShelterA_1": 100, "ShelterB_1": 500}
    edgeID_by_shelterID:dict = {"ShelterA_1": 'E2', "ShelterB_1": "E3"}
    shelter_choice_prob_list = [1.0]
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

    probabilities_by_start_edge = {
        "-E1": [1.0, 0.0],
        "E1": [0.0, 1.0],
    }
    vehicle_count_by_start_edge = {
        "-E1": 200,  # 例: 10台
        "E1": 200   # 例: 200台
    }
    start_interval = 5.0
    end_interval = 4.0
    for start_edgeID, end_edgeID_list in nearest_end_edgeID_by_start_edgeID_dict.items():
        # この開始エッジに割り当てられた車両数を取得
        assigned_vehicle_num = vehicle_count_by_start_edge.get(start_edgeID, 0)
        if assigned_vehicle_num == 0:
            continue # このエッジからは生成しないので次のループへ
        vehicle_intervals = np.linspace(start_interval, end_interval, num=int(assigned_vehicle_num))
        probabilities = probabilities_by_start_edge.get(
            start_edgeID,
            [1.0 / len(end_edgeID_list)] * len(end_edgeID_list)
        )
        if len(probabilities) != len(end_edgeID_list):
            raise ValueError(
                f"Probability length mismatch for {start_edgeID}: "
                f"len(probabilities)={len(probabilities)} vs len(end_edgeID_list)={len(end_edgeID_list)}."
            )
        # 出発時刻リセット
        DEPART_TIME = 0.0
        # 車両生成ループ
        for vehicle_index in range(int(assigned_vehicle_num)):
            selected_end_edgeID = choose_edge_by_probability(edgeID_list=end_edgeID_list, probabilities=probabilities)
            target_shelterID = find_shelterID_by_edgeID_by_shelterID(edgeID=selected_end_edgeID, edgeID_by_shelterID=edgeID_by_shelterID)

            VEHICLE_NUM, ROUTE_NUM, vehID_list_by_shelter, DEPART_TIME = generate_simple_init_vehID(
                                                                                                                from_edgeID=start_edgeID,
                                                                                                                to_edgeID=selected_end_edgeID,
                                                                                                                shelterID=target_shelterID,
                                                                                                                generate_interval=vehicle_intervals[vehicle_index],
                                                                                                                generate_route_count=ROUTE_NUM,
                                                                                                                generate_veh_count=VEHICLE_NUM,
                                                                                                                depart_time=DEPART_TIME
                                                                                                            )
            vehID_list.extend(vehID_list_by_shelter)
    # sys.exit()
    # カテゴリのカウント
    counter = Counter(extract_category(v) for v in vehID_list)

    # 合計と割合を表示
    total = sum(counter.values())
    print("出現数:", dict(counter))
    print("割合:")
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
    set_motivation_curve_dicts_to_agents(agent_list, rho=rho, delta=delta)

    vehInfo_by_vehID_dict = {vehInfo.get_vehID(): vehInfo for vehInfo in vehInfo_list}
    agent_by_vehID_dict = {agent.get_vehID(): agent for agent in agent_list}
    route_edges_by_routeID_dict = {
        routeID: tuple(traci.route.getEdges(routeID))
        for routeID in traci.route.getIDList()
    }
    # ドライバーの行動の初期化
    init_driver_behavior(vehIDs = vehID_list, lane_change_mode=1)
    for vehID in traci.vehicle.getIDList():
        traci.vehicle.setMaxSpeed(vehID, 7.0)

    run()

    # for vehID, vehInfo in vehInfo_by_vehID_dict.items():
    #     print(f"vehID: {vehID}, {vehInfo.get_avg_evac_time_by_route_by_recive_time()}")
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
    print(f"pedestrian_count:{PEDESTRIAN_COUNT}")
    print(f"route_changed_vehicle_count:{ROUTE_CHANGED_VEHICLE_COUNT}")
    print(f"wrong_way_driving_count:{WRONG_WAY_SUCCESS_COUNT}")
    print(f"vehicle_abandonment_count:{VEHICLE_ABANDONMENT_SUCCESS_COUNT}")
    print(f"normalcy_bias_route_change_count:{NORMALCY_BIAS_ROUTE_CHANGE_COUNT}")
    print(f"majority_bias_route_change_count:{MAJORITY_BIAS_ROUTE_CHANGE_COUNT}")
    print(f"lane_changed_vehicle_count:{LANE_CHANGED_VEHICLE_COUNT}")
    print(f"info_obtained_lanechange_count:{OBTAIN_INFO_LANE_CHANGE_COUNT}")
    print(f"elapsed_time_lanechange_count:{ELAPSED_TIME_LANE_CHANGE_COUNT}")
    print(f"majority_bias_lanechange_count:{POSITIVE_MAJORITY_BIAS_COUNT}")


