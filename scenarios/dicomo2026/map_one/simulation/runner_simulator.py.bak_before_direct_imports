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
# =========================
# Third-party libraries
# =========================
# SUMO tools need SUMO_HOME on sys.path *before* importing sumolib/traci
if "SUMO_HOME" in os.environ:
    sys.path.append(os.path.join(os.environ["SUMO_HOME"], "tools"))

from sumolib import checkBinary  # noqa: E402
import traci  # noqa: E402
import numpy as np
from numpy import double
from dataclasses import dataclass, field

# =========================
# Local / intra-package
# =========================
from evacsim import utilities
from evacsim.agents.Agent import Agent
from evacsim.agents.Shelter import Shelter
from evacsim.agents.VehicleInfo import VehicleInfo
import datetime

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
END_SIMULATION_TIME = 3000
THRESHOLD_SPEED = 2.00 # 7.2km/h
STOPPING_TIME_IN_SHELTER = 10000000

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
OBTAIN_INFO_LANE_CHANGE_COUNT = 0
ELAPSED_TIME_LANE_CHANGE_COUNT = 0
POSITIVE_MAJORITY_BIAS_COUNT = 0
PEDESTRIAN_COUNT = 0
NEW_VEHICLE_COUNT = 0
ROUTE_CHANGE_COUNT_AFTER_SHELTER_CAPACITY_FULL = 0

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

def control_vehicles():
    vehIDs = traci.vehicle.getIDList()
    pedestrianIDs = traci.person.getIDList()
    current_time = traci.simulation.getTime()
    is_step_5 = (current_time % 5 == 0)
    is_step_10 = (current_time % 10 == 0)
    step_cache = utilities.create_step_cache(current_time=current_time)
    route_avg_dirty = False

    global PEDESTRIAN_COUNT
    global ROUTE_CHANGED_VEHICLE_COUNT
    global NEW_VEHICLE_COUNT
    global ROUTE_CHANGE_COUNT_AFTER_SHELTER_CAPACITY_FULL

    for current_vehID in vehIDs:
        vehInfo_by_current_vehID: VehicleInfo = vehInfo_by_vehID_dict.get(current_vehID)
        agent_by_current_vehID: Agent = agent_by_vehID_dict.get(current_vehID)

        if vehInfo_by_current_vehID is None or agent_by_current_vehID is None:
            continue

        shelter_for_current_vehID: Shelter = utilities.find_shelter_by_edgeID_connect_target_shelter(
            vehInfo_by_current_vehID.get_edgeID_connect_target_shelter(),
            shelter_list,
        )
        current_edgeID: str = utilities.get_vehicle_road_id_cached(current_vehID, step_cache=step_cache)
        current_position = None
        elapsed_time = traci.simulation.getTime() - agent_by_current_vehID.get_created_time() if agent_by_current_vehID.get_created_time_flg() else 0.0

        # VEHINFO: 到着処理
        if (
            traci.vehicle.isStoppedParking(current_vehID)
            and not vehInfo_by_current_vehID.get_parked_flag()
        ):
            utilities.handle_arrival(
                current_vehID=current_vehID,
                vehInfo_by_current_vehID=vehInfo_by_current_vehID,
                agent_by_current_vehID=agent_by_current_vehID,
                shelter_for_current_vehID=shelter_for_current_vehID,
                shelter_list=shelter_list,
                arrival_time_list=arrival_time_list,
                arrival_time_by_vehID_dict=arrival_time_by_vehID_dict,
                elapsed_time_list=elapsed_time_list,
            )
            route_avg_dirty = True

        # VEHINFO: 避難所近くのエッジにいる場合、密度に応じて速度制御を行う
        if not vehInfo_by_current_vehID.get_decline_edge_arrival_flag():
            pre_edgeID_near_shelter_flag = utilities.is_pre_edgeID_near_shelter(
                current_edgeID=current_edgeID,
                edgeID_near_shelter=vehInfo_by_current_vehID.get_edgeID_connect_target_shelter(),
                custome_edge_list=custome_edge_list,
            )
            if pre_edgeID_near_shelter_flag and not vehInfo_by_current_vehID.get_decline_edge_arrival_flag():
                local_density = utilities.get_local_density(vehID=current_vehID, radius=50.0)
                utilities.apply_gap_density_speed_control(
                    vehID=current_vehID,
                    local_density=local_density,
                    v_free=4.0,
                    v_min=2.0,
                    gap_min=7.0,
                    tau=1.8,
                    alpha=0.5,
                    slow_time=1.0,
                )
            else:
                traci.vehicle.slowDown(current_vehID, 5.0, 1.0)

        # VEHINFO: 到着していない車両に対して処理を行う
        if not vehInfo_by_current_vehID.get_arrival_flag():
            # 乗り捨てが現在のpositionで成功しなかった場合、次の場所で乗り捨てを試みる
            if current_edgeID == agent_by_current_vehID.get_reserved_vehicle_abandonment_edgeID() and agent_by_current_vehID.get_avoiding_abandoned_vehicle_flg():
                print("予約していた乗り捨てエッジに到達 current_vehID: {}, current_edgeID: {}, time: {}".format(current_vehID, current_edgeID, current_time))
                if current_position is None:
                    current_position = utilities.get_vehicle_position_cached(current_vehID, step_cache=step_cache)
                neighbor_vehicle_abandant_count: int = utilities.count_near_abandoned_vehicle_in_right_lane(
                    vehID=current_vehID,
                    agent_list=agent_list,
                    pedestrianID_list=pedestrianIDs,
                    veh_position=current_position,
                    step_cache=step_cache,
                )
                if utilities.is_again_driver_vehicle_abandant(
                    agent_by_target_vehID=agent_by_current_vehID,
                    vehInfo_by_target_vehID=vehInfo_by_current_vehID,
                    current_time=current_time,
                    neighbor_vehicle_abandant_nums=neighbor_vehicle_abandant_count,
                ):
                    print("乗り捨てに成功 current_vehID: {}, current_edgeID: {}, time: {}".format(current_vehID, current_edgeID, current_time))
                    agent_by_current_vehID.set_avoiding_abandoned_vehicle_flg(False)

            # # 津波接近に関する情報を取得したか否かを確認する
            # if (
            #     not agent_by_current_vehID.get_tsunami_info_obtained_flg()
            #     and list(vehInfo_by_current_vehID.get_tsunami_precursor_info().values())[0][0]
            #     and vehInfo_by_current_vehID.get_vehicle_comm_enabled_flag()
            # ):
            #     tsunami_info = vehInfo_by_current_vehID.get_tsunami_precursor_info()
            #     min_time = min(info[1] for info in tsunami_info.values())
            #     if min_time > agent_by_current_vehID.get_created_time():
            #         min_time = agent_by_current_vehID.get_created_time()
            #     agent_by_current_vehID.set_tsunami_info_obtained_time(min_time)
            #     agent_by_current_vehID.set_tsunami_info_obtained_flg(True)

            # # 満杯情報を取得したか否かを確認する
            # if (
            #     not agent_by_current_vehID.get_shelter_full_info_obtained_flg()
            #     and shelter_for_current_vehID.get_congestion_rate() > 0.99
            #     and vehInfo_by_current_vehID.get_vehicle_comm_enabled_flag()
            # ):
            #     agent_by_current_vehID.set_shelter_full_info_obtained_flg(True)
            
            # # 経路の混雑情報を取得したか否かを確認する TODO ここを完成させる
            # if (
            #     not agent_by_current_vehID.get_route_congestion_info_obtained_flg()
            #     and utilities.is_route_time_difference_exceeding_threshold(current_edgeID=current_edgeID, agent_by_target_vehID=agent_by_current_vehID, 
            #                                                             shelter=shelter_for_current_vehID, vehInfo_by_target_vehID=vehInfo_by_current_vehID, 
            #                                                             shelter_list=shelter_list, custome_edge_list=custome_edge_list)
            #     and vehInfo_by_current_vehID.get_vehicle_comm_enabled_flag()
            # ):
            #     agent_by_current_vehID.set_route_congestion_info_obtained_flg(True)

            if not agent_by_current_vehID.get_created_time_flg():
                agent_by_current_vehID.set_created_time(current_time)
                agent_by_current_vehID.set_created_time_flg(True)

            # 車両間通信による情報共有
            if vehInfo_by_current_vehID.get_vehicle_comm_enabled_flag():
                if is_step_10:
                    if current_position is None:
                        current_position = utilities.get_vehicle_position_cached(current_vehID, step_cache=step_cache)

                    around_vehIDs: list = utilities.get_around_vehIDs(
                        target_vehID=current_vehID,
                        custome_edge_list=custome_edge_list,
                        step_cache=step_cache,
                    )
                    utilities.v2v_communication(
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

                    utilities.v2shelter_communication(
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

                    utilities.v2v_communication_about_tsunami_info(
                        target_vehID=current_vehID,
                        target_vehInfo=vehInfo_by_current_vehID,
                        around_vehIDs=around_vehIDs,
                        vehInfo_list=vehInfo_list,
                        COMMUNICATION_RANGE=COMM_RANGE,
                        target_position=current_position,
                        vehInfo_by_vehID_dict=vehInfo_by_vehID_dict,
                        step_cache=step_cache,
                    )
            
            # 渋滞に巻き込まれているか否かを確認する
            if utilities.is_vehID_in_congested_edge(vehID=current_vehID, threshold_speed=THRESHOLD_SPEED) and is_step_5:
                traci.vehicle.setColor(current_vehID, (255, 0, 0)) # 赤色に変更
                # 行動に対する基本的なモチベーションを増加させる
                # utilities.re_calculate_motivation_value(info_activation_dict=agent_by_current_vehID.get_base_motivation_value_by_elapsed_time_dict(), elapsed_time=elapsed_time)
                # 車両間通信が可能な場合、情報を取得するごとにモチベーションの値を更新する（正常性バイアスの領域）
                if vehInfo_by_current_vehID.get_vehicle_comm_enabled_flag():
                    # *津波接近に関する情報を取得したか否かを確認する
                    if (
                        not agent_by_current_vehID.get_tsunami_info_obtained_flg()
                        and list(vehInfo_by_current_vehID.get_tsunami_precursor_info().values())[0][0]
                        and vehInfo_by_current_vehID.get_vehicle_comm_enabled_flag()
                        ):
                        tsunami_info = vehInfo_by_current_vehID.get_tsunami_precursor_info()
                        min_time = min(info[1] for info in tsunami_info.values())
                        if min_time > agent_by_current_vehID.get_created_time():
                            min_time = agent_by_current_vehID.get_created_time()
                        agent_by_current_vehID.set_tsunami_info_obtained_time(min_time)
                        utilities.re_calculate_motivation_value(info_activation_dict=agent_by_current_vehID.get_tsunami_precursor_normalcy_value_by_elapsed_time_dict(), elapsed_time=current_time - agent_by_current_vehID.get_created_time())
                        agent_by_current_vehID.set_tsunami_info_obtained_flg(True)

                    # *満杯情報を取得したか否かを確認する
                    if (
                        not agent_by_current_vehID.get_shelter_full_info_obtained_flg()
                        and shelter_for_current_vehID.get_congestion_rate() > 0.99
                        and vehInfo_by_current_vehID.get_vehicle_comm_enabled_flag()
                        ):
                        utilities.re_calculate_motivation_value(info_activation_dict=agent_by_current_vehID.get_route_congestion_normalcy_value_by_elapsed_time_dict(), elapsed_time=current_time - agent_by_current_vehID.get_created_time())
                        agent_by_current_vehID.set_shelter_full_info_obtained_flg(True)
                    
                    # *経路の混雑情報を取得したか否かを確認する
                    if (
                        not agent_by_current_vehID.get_route_congestion_info_obtained_flg()
                        and utilities.is_route_time_difference_exceeding_threshold(current_edgeID=current_edgeID, agent_by_target_vehID=agent_by_current_vehID, 
                                                                                shelter=shelter_for_current_vehID, vehInfo_by_target_vehID=vehInfo_by_current_vehID, 
                                                                                shelter_list=shelter_list, custome_edge_list=custome_edge_list)
                        and vehInfo_by_current_vehID.get_vehicle_comm_enabled_flag()
                        ):
                        agent_by_current_vehID.set_route_congestion_info_obtained_flg(True)
                        utilities.re_calculate_motivation_value(info_activation_dict=agent_by_current_vehID.get_shelter_full_normalcy_value_by_elapsed_time_dict(), elapsed_time=current_time - agent_by_current_vehID.get_created_time())

                    if agent_by_current_vehID.get_route_congestion_info_obtained_flg() or agent_by_current_vehID.get_shelter_full_info_obtained_flg() or agent_by_current_vehID.get_tsunami_info_obtained_flg():
                        traci.vehicle.setColor(current_vehID, (255, 165, 0)) # オレンジ色に変更
                        # MOTIVATION: 運転者が車両を乗り捨てるか否かを決める
                        current_motivation = utilities.calculate_motivation_for_evacuation_action(
                                                                                                        agent=agent_by_current_vehID,
                                                                                                        current_time=current_time,
                                                                                                        action="va",
                                                                                                        debug=False,
                                                                                                    )
                        # AGENT: 渋滞脱出行動をとる 
                        # MOTIVATION: 車両乗り捨てに対するもモチベーションが閾値を超えているか否かを確認する
                        if current_motivation >= agent_by_current_vehID.get_vehicle_abandoned_threshold():
                            print("車両乗り捨てを行います current_vehID: {}, current_edgeID: {}, time: {}, motivation: {}, thresshold: {}".format(current_vehID, current_edgeID, current_time, current_motivation, agent_by_current_vehID.get_vehicle_abandoned_threshold()))
                            
                            # AGENT: 車両を乗り捨てる
                            # TRACI: 車両をremove pedestrianを追加する
                            agent_by_current_vehID.set_vehicle_abandoned_flg(vehicle_abandoned_flg=True)
                            PEDESTRIAN_COUNT, pedestrianID, walking_distance = utilities.vehicle_abandant_behavior_with_vehicle_remove(
                                    current_vehID=current_vehID,
                                    current_edgeID=current_edgeID,
                                    agent_by_current_vehID=agent_by_current_vehID,
                                    vehInfo_by_target_vehID=vehInfo_by_current_vehID,
                                    PEDESTRIAN_COUNT=PEDESTRIAN_COUNT,
                                    STOPPING_TIME_IN_SHELTER=STOPPING_TIME_IN_SHELTER,
                                    shelter=shelter_for_current_vehID,
                                )
                            traci.vehicle.remove(current_vehID)
                            if pedestrianID is None:
                                print(f"[WARN] pedestrian was not created for {current_vehID}; skip abandonment registration")
                                continue
                            agent_by_current_vehID.set_vehicle_abandoned_flg(True)
                            # print(f"test: {agent_by_current_vehID.get_vehicle_abandoned_time()} current_time: {current_time}")
                            vehicle_abandant_time_by_pedestrianID_dict[pedestrianID] = current_time
                            walking_distance_by_pedestrianID_dict[pedestrianID] = walking_distance
                            agent_by_current_vehID.set_vehicle_abandoned_time(current_time)
                            pedstrianID_list.append(pedestrianID)
                            print("予約なしで車両乗り捨てを行いました")
                        
                    # MOTIVATION: agentの渋滞継続時間を更新する　次の計算用に時計を進める
                        agent_by_current_vehID.update_congestion_duration(5.0)
                        agent_by_current_vehID.updated_encounted_congestion_time(5.0)
                        agent_by_current_vehID.update_shelter_full_info_obtained_time(5.0)

                    # *行動1：経路変更を行うか否かを決定する
                    # *行動2: 逆走行為を行う
                    # *行動3: 車両を乗り捨てる

            if current_edgeID == "E100":
                if traci.simulation.getTime() > TSUNAMI_SIGN_START_TIME and traci.simulation.getTime() < TSUNAMI_SIGN_END_TIME:
                    vehInfo_by_current_vehID.update_tsunami_precursor_info(vehID=current_vehID, tsunami_precursor_flag=True, current_time=traci.simulation.getTime())
            # # 避難地"E2"が満杯になったことを知った場合
            # if shelter_for_current_vehID.get_congestion_rate() > 0.99 and not agent_by_current_vehID.get_shelter_full_flg():
            #     # 避難地"E2"日和台方面で、満杯情報を取得した場合、車両乗り捨てあるいは経路変更を試みる
            #     if current_edgeID in ["E106", "E15", "E104"]:
            #         # print("shelter {} is congested. current_vehID: {}, current_edgeID: {}, time: {}".format(shelter_for_current_vehID.get_shelterID(), current_vehID, current_edgeID, current_time))
            #         agent_by_current_vehID.set_shelter_full_flg(True)
            #         if random.random() < 0.50: # 50%の確率で経路変更を試みる
            #             agent_by_current_vehID.set_shelter_full_flg(True)
            #             from_edgeID, shelterID, to_edge_list = utilities.find_alternative_shelter_choice(
            #                                                                 current_target_shelterID=agent_by_current_vehID.get_target_shelter(),
            #                                                                 vehID=current_vehID,
            #                                                                 current_edgeID=current_edgeID,
            #                                                                 vehInfo=vehInfo_by_current_vehID,
            #                                                                 shelter_list=shelter_list,
            #                                                                 agent=agent_by_current_vehID,
            #                                                                 )
            #             if from_edgeID != "" and shelterID != "" and len(to_edge_list) > 0:
            #                 NEW_VEHICLE_COUNT = utilities.generate_new_veh_based_on_route_time(
            #                                                                     target_vehID=current_vehID, 
            #                                                                     NEW_VEHICLE_COUNT= NEW_VEHICLE_COUNT, 
            #                                                                     agent_list=agent_list, 
            #                                                                     vehInfo_list=vehInfo_list,
            #                                                                     vehInfo_by_target_vehID=vehInfo_by_current_vehID, 
            #                                                                     agent_by_target_vehID=agent_by_current_vehID, 
            #                                                                     from_edgeID=from_edgeID,
            #                                                                     new_shelterID=shelterID,
            #                                                                     to_edgeID=to_edge_list[0],
            #                                                                     color_mode=0
            #                                                                     )
            #                 ROUTE_CHANGE_COUNT_AFTER_SHELTER_CAPACITY_FULL+= 1
            #                 route_info_with_receive_time = vehInfo_by_current_vehID.get_avg_evac_time_by_route_by_recive_time()
            #                 receive_time = list(route_info_with_receive_time.keys())[0]
            #                 routes_dict = route_info_with_receive_time[receive_time]

            #                 print(f"receive_time {receive_time}")
            #                 print(f"routes_dict {routes_dict}")

            #                 continue
            #         else: # 経路変更を試みない場合、乗り捨てを試みる
            #             agent_by_current_vehID.set_vehicle_abandoned_flg(vehicle_abandoned_flg=True)
            #             PEDESTRIAN_COUNT, pedestrianID, walking_distance = utilities.vehicle_abandant_behavior_with_vehicle_remove(
            #                     current_vehID=current_vehID,
            #                     current_edgeID=current_edgeID,
            #                     agent_by_current_vehID=agent_by_current_vehID,
            #                     vehInfo_by_target_vehID=vehInfo_by_current_vehID,
            #                     PEDESTRIAN_COUNT=PEDESTRIAN_COUNT,
            #                     STOPPING_TIME_IN_SHELTER=STOPPING_TIME_IN_SHELTER,
            #                     shelter=shelter_for_current_vehID,
            #                 )
            #             traci.vehicle.remove(current_vehID)
            #             if pedestrianID is None:
            #                 print(f"[WARN] pedestrian was not created for {current_vehID}; skip abandonment registration")
            #                 continue
            #             agent_by_current_vehID.set_vehicle_abandoned_flg(True)
            #             # print(f"test: {agent_by_current_vehID.get_vehicle_abandoned_time()} current_time: {current_time}")
            #             vehicle_abandant_time_by_pedestrianID_dict[pedestrianID] = current_time
            #             walking_distance_by_pedestrianID_dict[pedestrianID] = walking_distance
            #             agent_by_current_vehID.set_vehicle_abandoned_time(current_time)
            #             pedstrianID_list.append(pedestrianID)
            #             print("予約なしで車両乗り捨てを行いました")
                
            # 交差点手前で目的避難地の満杯情報あるいは混雑情報を取得した場合、経路変更を試みる
            # if current_edgeID in ["E100","E101", "E102", "E103", "E120", "E121", "E122", "E123", "E124", "E125", "E126", "E127"]:
                # print("route change")

            # 逆走行為を実施するかの意思決定を行う


    for pedestrianID in vehicle_abandant_time_by_pedestrianID_dict:
        vehID_by_pedestrianID: str = utilities.extract_vehicle_id(pedestrianID)
        agent_by_pedestrianID: Agent = agent_by_vehID_dict.get(vehID_by_pedestrianID)
        vehInfo_by_pedestrianID: VehicleInfo = vehInfo_by_vehID_dict.get(vehID_by_pedestrianID)
        if agent_by_pedestrianID is None or vehInfo_by_pedestrianID is None:
            continue
        shelter_by_pedestrianID: Shelter = utilities.find_shelter_by_edgeID_connect_target_shelter(
            vehInfo_by_pedestrianID.get_edgeID_connect_target_shelter(),
            shelter_list,
        )
        if not agent_by_pedestrianID.get_arrival_shelter_flg():
            if traci.person.getRoadID(pedestrianID) in ["E2", "E3"]:
                utilities.handle_arrival_for_pedestrian(
                    pedestrianID=pedestrianID,
                    current_vehID=current_vehID,
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
        utilities.calculate_avg_evac_time_by_route(shelter_list=shelter_list)
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
    # python3 -m scenarios.its105.map_one.simulation.runner_simulator --nogui scenarios/its105/configs/config_scenario_1.toml 0.5 1.0
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
    ACTIVE_ROUTE_CHANGE_MEAN: float = _req(cfg, "active_route_change_mean", float)
    ACTIVE_ROUTE_CHANGE_VAR: float = _req(cfg, "active_route_change_var", float)
    CAUTIOUS_ROUTE_CHANGE_MEAN: float = _req(cfg, "cautious_route_change_mean", float)
    CAUTIOUS_ROUTE_CHANGE_VAR: float = _req(cfg, "cautious_route_change_var", float)
    SHELTER_CAPACITY_THRESHOLD: float = _req(cfg, "shelter_capacity_threshold", float)
    TSUNAMI_SIGN_START_TIME: float = _req(cfg, "tsunami_sign_start_time", float)
    TSUNAMI_SIGN_END_TIME: float = _req(cfg, "tsunami_sign_end_time", float)
    MOTIVATION_THRESHOLD_START: float = _req(cfg, "motivation_threshold_start", float)
    MOTIVATION_THRESHOLD_END: float = _req(cfg, "motivation_threshold_end", float)
    MIN_MOTIVATION_START: float = _req(cfg, "min_motivation_start", float)
    MIN_MOTIVATION_END: float = _req(cfg, "min_motivation_end", float)
    POSITIVE_LANECHANGE_START: float = _req(cfg, "positive_lanechange_start", float)
    POSITIVE_LANECHANGE_END: float = _req(cfg, "positive_lanechange_end", float)
    NEGATIVE_LANECHANGE_START: float = _req(cfg, "negative_lanechange_start", float)
    NEGATIVE_LANECHANGE_END: float = _req(cfg, "negative_lanechange_end", float)
    DRIVER_VISIBILITY_DISTANCE: float = _req(cfg, "driver_visibility_distance", float)
    POSITIVE_MAJORITY_BIAS: float = _req(cfg, "positive_majority_bias", float)
    NEGATIVE_MAJORITY_BIAS: float = _req(cfg, "negative_majority_bias", float)
    ACTIVE_SHELTER_OCCUPANCY_RATE_THRESHOLD_START:float = _req(cfg, "active_shelter_occupancy_rate_threshold_start", float)
    ACTIVE_SHELTER_OCCUPANCY_RATE_THRESHOLD_END:float = _req(cfg, "active_shelter_occupancy_rate_threshold_end", float)
    CAUTIOUS_SHELTER_OCCUPANCY_RATE_THRESHOLD_START:float = _req(cfg, "cautious_shelter_occupancy_rate_threshold_start", float)
    CAUTIOUS_SHELTER_OCCUPANCY_RATE_THRESHOLD_END:float = _req(cfg, "cautious_shelter_occupancy_rate_threshold_end", float)
    ACTIVE_NORMALCY_VALUE_ABOUT_VEHICLE_ABANDONMENT_MEAN: float =  _req(cfg, "active_normalcy_value_about_vehicle_abandonment_mean", float)
    ACTIVE_NORMALCY_VALUE_ABOUT_VEHICLE_ABANDONMENT_VAR: float  = _req(cfg, "active_normalcy_value_about_vehicle_abandonment_var", float)
    ACTIVE_MAJORITY_VALUE_ABOUT_VEHICLE_ABANDONMENT_MEAN: float = _req(cfg, "active_majority_value_about_vehicle_abandonment_mean", float)
    ACTIVE_MAJORITY_VALUE_ABOUT_VEHICLE_ABANDONMENT_VAR: float  =  _req(cfg, "active_majority_value_about_vehicle_abandonment_var", float)
    ACTIVE_VEHICLE_ABANDONED_THRESHOLD_MEAN: float = _req(cfg, "active_vehicle_abandoned_threshold_mean", float)
    ACTIVE_VEHICLE_ABANDONED_THRESHOLD_VAR: float  = _req(cfg, "active_vehicle_abandoned_threshold_var", float)
    CAUTIOUS_NORMALCY_VALUE_ABOUT_VEHICLE_ABANDONMENT_MEAN: float = _req(cfg, "cautious_normalcy_value_about_vehicle_abandonment_mean", float)
    CAUTIOUS_NORMALCY_VALUE_ABOUT_VEHICLE_ABANDONMENT_VAR: float  = _req(cfg, "cautious_normalcy_value_about_vehicle_abandonment_var", float)
    CAUTIOUS_MAJORITY_VALUE_ABOUT_VEHICLE_ABANDONMENT_MEAN: float = _req(cfg, "cautious_majority_value_about_vehicle_abandonment_mean", float)
    CAUTIOUS_MAJORITY_VALUE_ABOUT_VEHICLE_ABANDONMENT_VAR: float  = _req(cfg, "cautious_majority_value_about_vehicle_abandonment_var", float)
    CAUTIOUS_VEHICLE_ABANDONED_THRESHOLD_MEAN: float = _req(cfg, "cautious_vehicle_abandoned_threshold_mean", float)
    CAUTIOUS_VEHICLE_ABANDONED_THRESHOLD_VAR: float  = _req(cfg, "cautious_vehicle_abandoned_threshold_var", float)
    ALPHA: float = _req(cfg, "alpha", float)

    traci.start(
                [sumoBinary,
                    "-c", str(SUMO_CFG),
                    "--tripinfo-output", "tripinfo.xml",
                    "--tls.all-off", "true" ,
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
        shelter_list:list[Shelter] = utilities.init_shelter(
                                                    shelterID=shelterID, 
                                                    shelter_capacity_by_ID=shelter_capacity_by_ID, 
                                                    near_edgeID=near_edgeID, 
                                                    shelter_list=shelter_list
                                                    )
    custome_edge_list:list = utilities.init_custome_edge()
    vehicle_start_edges:list = utilities.get_vehicle_start_edges(custome_edge_list=custome_edge_list)
    vehicle_end_edges:list = utilities.get_vehicle_end_edges(custome_edge_list=custome_edge_list)
    # 車両の開始エッジと終了エッジの組み合わせを辞書にする
    utilities.get_vehicle_end_list_by_start_edge_dict(vehicle_start_edges=vehicle_start_edges, vehicle_end_edges=vehicle_end_edges)
    # 全経路で総当たりをし、通行可能経路を取得しておく。
    utilities.import_connected_edges_from_json(file_path=str(DATA_DIR / "all_edgeIDs.json"))
    nearest_end_edgeID_by_start_edgeID_dict:dict = utilities.import_start_end_edgeIDs_from_json(file_path=str(DATA_DIR / "start_end_edgeIDs_ishinomaki_two_shelter.json"))

    probabilities_by_start_edge = {
        "-E1": [1.0, 0.0],
        "E1": [0.0, 1.0],
    }
    vehicle_count_by_start_edge = {
        "-E1": 200,  # 例: 10台
        "E1": 300   # 例: 200台
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
            selected_end_edgeID = utilities.choose_edge_by_probability(edgeID_list=end_edgeID_list, probabilities=probabilities)
            target_shelterID = utilities.find_shelterID_by_edgeID_by_shelterID(edgeID=selected_end_edgeID, edgeID_by_shelterID=edgeID_by_shelterID)

            VEHICLE_NUM, ROUTE_NUM, vehID_list_by_shelter, DEPART_TIME = utilities.generate_simple_init_vehID(
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
    vehInfo_list:list[VehicleInfo] = utilities.init_vehicleInfo_list_base(vehIDs=vehID_list, shelter_list=shelter_list, v2v_capable_vehicle_rate=v2v_capable_vehicle_rate) 

    # Agentの初期化
    # 乗り捨てに関する初期化
    agent_list:list[Agent] = utilities.init_agent_list(
                                                vehIDs=vehID_list, 
                                                edgeID_by_shelterID=edgeID_by_shelterID, 
                                                EARLY_AGENT_THRESHOLD_LIST=EARLY_AGENT_THRESHOLD_LIST, 
                                                LATE_AGENT_THRESHOLD_LIST=LATE_AGENT_THRESHOLD_LIST, 
                                                ATTR_RATE=early_rate,
                                                MOTIVATION_THRESHOLD_START=MOTIVATION_THRESHOLD_START,
                                                MOTIVATION_THRESHOLD_END=MOTIVATION_THRESHOLD_END,
                                                MIN_MOTIVATION_START=MIN_MOTIVATION_START,
                                                MIN_MOTIVATION_END=MIN_MOTIVATION_END,
                                                ACTIVE_ROUTE_CHANGE_MEAN=ACTIVE_ROUTE_CHANGE_MEAN,
                                                ACTIVE_ROUTE_CHANGE_VAR=ACTIVE_ROUTE_CHANGE_VAR,
                                                CAUTIOUS_ROUTE_CHANGE_MEAN=CAUTIOUS_ROUTE_CHANGE_MEAN,
                                                CAUTIOUS_ROUTE_CHANGE_VAR=CAUTIOUS_ROUTE_CHANGE_VAR,
                                                POSITIVE_LANECHANGE_START=POSITIVE_LANECHANGE_START,
                                                POSITIVE_LANECHANGE_END=POSITIVE_LANECHANGE_END,
                                                NEGATIVE_LANECHANGE_START=NEGATIVE_LANECHANGE_START,
                                                NEGATIVE_LANECHANGE_END=NEGATIVE_LANECHANGE_END,
                                                POSITIVE_MAJORITY_BIAS=POSITIVE_MAJORITY_BIAS,
                                                NEGATIVE_MAJORITY_BIAS=NEGATIVE_MAJORITY_BIAS,
                                                ACTIVE_SHELTER_OCCUPANCY_RATE_THRESHOLD_START=ACTIVE_SHELTER_OCCUPANCY_RATE_THRESHOLD_START,
                                                ACTIVE_SHELTER_OCCUPANCY_RATE_THRESHOLD_END=ACTIVE_SHELTER_OCCUPANCY_RATE_THRESHOLD_END,
                                                CAUTIOUS_SHELTER_OCCUPANCY_RATE_THRESHOLD_START=CAUTIOUS_SHELTER_OCCUPANCY_RATE_THRESHOLD_START,
                                                CAUTIOUS_SHELTER_OCCUPANCY_RATE_THRESHOLD_END=CAUTIOUS_SHELTER_OCCUPANCY_RATE_THRESHOLD_END,
                                                ACTIVE_NORMALCY_VALUE_ABOUT_VEHICLE_ABANDONMENT_MEAN = ACTIVE_NORMALCY_VALUE_ABOUT_VEHICLE_ABANDONMENT_MEAN,
                                                ACTIVE_NORMALCY_VALUE_ABOUT_VEHICLE_ABANDONMENT_VAR = ACTIVE_NORMALCY_VALUE_ABOUT_VEHICLE_ABANDONMENT_VAR,
                                                ACTIVE_MAJORITY_VALUE_ABOUT_VEHICLE_ABANDONMENT_MEAN = ACTIVE_MAJORITY_VALUE_ABOUT_VEHICLE_ABANDONMENT_MEAN,
                                                ACTIVE_MAJORITY_VALUE_ABOUT_VEHICLE_ABANDONMENT_VAR = ACTIVE_MAJORITY_VALUE_ABOUT_VEHICLE_ABANDONMENT_VAR,
                                                ACTIVE_VEHICLE_ABANDONED_THRESHOLD_MEAN = ACTIVE_VEHICLE_ABANDONED_THRESHOLD_MEAN,
                                                ACTIVE_VEHICLE_ABANDONED_THRESHOLD_VAR = ACTIVE_VEHICLE_ABANDONED_THRESHOLD_VAR,
                                                CAUTIOUS_NORMALCY_VALUE_ABOUT_VEHICLE_ABANDONMENT_MEAN = CAUTIOUS_NORMALCY_VALUE_ABOUT_VEHICLE_ABANDONMENT_MEAN,
                                                CAUTIOUS_NORMALCY_VALUE_ABOUT_VEHICLE_ABANDONMENT_VAR = CAUTIOUS_NORMALCY_VALUE_ABOUT_VEHICLE_ABANDONMENT_VAR,
                                                CAUTIOUS_MAJORITY_VALUE_ABOUT_VEHICLE_ABANDONMENT_MEAN = CAUTIOUS_MAJORITY_VALUE_ABOUT_VEHICLE_ABANDONMENT_MEAN,
                                                CAUTIOUS_MAJORITY_VALUE_ABOUT_VEHICLE_ABANDONMENT_VAR = CAUTIOUS_MAJORITY_VALUE_ABOUT_VEHICLE_ABANDONMENT_VAR,
                                                CAUTIOUS_VEHICLE_ABANDONED_THRESHOLD_MEAN = CAUTIOUS_VEHICLE_ABANDONED_THRESHOLD_MEAN,
                                                CAUTIOUS_VEHICLE_ABANDONED_THRESHOLD_VAR = CAUTIOUS_VEHICLE_ABANDONED_THRESHOLD_VAR,
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
    base_motivation_value_for_elapsed_time_dict = utilities.generate_motivation_curve(max_time=450,step=1)
    tusnami_normalcy_values_by_info = utilities.generate_info_activation_dict(rho=rho["tsu"], delta=delta["tsu"], max_time=450, step=1)
    jam_normalcy_values_by_info = utilities.generate_info_activation_dict(rho=rho["jam"], delta=delta["jam"], max_time=450, step=1)
    full_normalcy_values_by_info = utilities.generate_info_activation_dict(rho=rho["full"], delta=delta["full"], max_time=450, step=1)
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

    # for agent in agent_list:
    #     print(f"agentID:{agent.get_vehID()} {agent.get_lane_change_xy_dict()}")
    #     print(f"agent {agent.get_vehID()} motivation curve: {agent.get_base_motivation_value_by_elapsed_time_dict()}")
    #     print(f"agent {agent.get_vehID()} tsunami precursor normalcy curve: {agent.get_tsunami_precursor_normalcy_value_by_elapsed_time_dict()}")
    #     print(f"agent {agent.get_vehID()} route congestion normalcy curve: {agent.get_route_congestion_normalcy_value_by_elapsed_time_dict()}")
    #     print(f"agent {agent.get_vehID()} shelter full normalcy curve: {agent.get_shelter_full_normalcy_value_by_elapsed_time_dict()}")
    #     sys.exit()
    vehInfo_by_vehID_dict = {vehInfo.get_vehID(): vehInfo for vehInfo in vehInfo_list}
    agent_by_vehID_dict = {agent.get_vehID(): agent for agent in agent_list}
    route_edges_by_routeID_dict = {
        routeID: tuple(traci.route.getEdges(routeID))
        for routeID in traci.route.getIDList()
    }
    # ドライバーの行動の初期化
    utilities.init_driver_behavior(vehIDs = vehID_list, lane_change_mode=1)
    for vehID in traci.vehicle.getIDList():
        traci.vehicle.setMaxSpeed(vehID, 7.0)
    run()
    for vehID, vehInfo in vehInfo_by_vehID_dict.items():
        print(f"vehID: {vehID}, {vehInfo.get_avg_evac_time_by_route_by_recive_time()}")
    print(f"mean elapsed_time: {np.mean(elapsed_time_list)}")
    if len(arrival_time_by_vehID_dict) == NUM_VEHICLES:
        print("OK all vehs arrived ")
    else:
        print(f"NG all vehs not arrived {len(arrival_time_by_vehID_dict)}")
    check_vehicle_abandonment_count = 0
    for agent in agent_list:
        if agent.get_vehicle_abandoned_flg():
            check_vehicle_abandonment_count += 1
    if check_vehicle_abandonment_count == PEDESTRIAN_COUNT:
        print("OK all vehicle abandonment were detected")
    else:
        print(f"NG not all vehicle abandonment were detected {check_vehicle_abandonment_count} / {PEDESTRIAN_COUNT}")
    
    print("===== Simlation Result Summary =====")
    print(f"arrival_time_by_vehID_dict:{arrival_time_by_vehID_dict}")
    print(f"vehicle_abandant_time_by_pedestrianID_dict:{vehicle_abandant_time_by_pedestrianID_dict}")
    print(f"walking_distance_by_pedestrianID_dict:{walking_distance_by_pedestrianID_dict}")
    print(f"pedestrian_count:{PEDESTRIAN_COUNT}")
    print(f"route_changed_vehicle_count:{ROUTE_CHANGED_VEHICLE_COUNT}")
    print(f"normalcy_bias_route_change_count:{NORMALCY_BIAS_ROUTE_CHANGE_COUNT}")
    print(f"majority_bias_route_change_count:{MAJORITY_BIAS_ROUTE_CHANGE_COUNT}")
    print(f"lane_changed_vehicle_count:{LANE_CHANGED_VEHICLE_COUNT}")
    print(f"info_obtained_lanechange_count:{OBTAIN_INFO_LANE_CHANGE_COUNT}")
    print(f"elapsed_time_lanechange_count:{ELAPSED_TIME_LANE_CHANGE_COUNT}")
    print(f"majority_bias_lanechange_count:{POSITIVE_MAJORITY_BIAS_COUNT}")


