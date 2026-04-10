# =========================
# Standard library
# =========================
import copy
import os
import random
import sys
from collections import Counter, defaultdict
from math import sqrt
from pathlib import Path
import optparse
import sys
import argparse
# sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '../../../..')))
# =========================
# Third-party libraries
# =========================
# SUMO tools need SUMO_HOME on sys.path *before* importing sumolib/traci
if "SUMO_HOME" in os.environ:
    sys.path.append(os.path.join(os.environ["SUMO_HOME"], "tools"))

from sumolib import checkBinary  # noqa: E402
import traci  # noqa: E402
import matplotlib.pyplot as plt
import numpy as np
from numpy import double

# =========================
# Local / intra-package
# =========================
from evacsim import utilities
from evacsim.agents.Agent import Agent
from evacsim.agents.CustomeEdge import CustomeEdge, ConnectedEdges  # ファイル名の綴りに注意
from evacsim.agents.Shelter import Shelter
from evacsim.agents.VehicleInfo import VehicleInfo
import datetime

# =========================
# Runtime config / seeds
# =========================
# random.seed(318)  # 乱数シードを固定（再現性）
random.seed()
random.seed(os.getpid() + int(datetime.datetime.now().timestamp()))
np.random.seed(os.getpid() + int(datetime.datetime.now().timestamp()))

# runner_base.py のファイル位置から data ディレクトリを解決
HERE = Path(__file__).resolve()
MAP_ONE_DIR = HERE.parent.parent          # .../scenarios/its102/map_one
DATA_DIR = MAP_ONE_DIR / "data"
SUMO_CFG = DATA_DIR / "ishinomaki_toymap.sumocfg"

# =========================
# 1) シミュレーション基本設定・時間評価
# =========================
END_SIMULATION_TIME = 3000
DECISION_EVALUATION_INTERVAL = 10.0
THRESHOLD_SPEED = 2.77 # 10km/h
STOPPING_TIME_IN_SHELTER = 10000000
SPEED_ARRANGE = 1
CONGESTION_RATE = 0.3
SHOW_DEBUG_COUNT = 0

# =========================
# 2) 車両生成・発進関連
# =========================
VEHICLE_NUM = 0
DEPART_TIME: double = 0.0
ROUTE_NUM = 0

# ======================================================================================================================================================
# 3) 通信・知覚範囲,  4) ルート変更分布パラメータ・収容率, 5) 津波予兆（取得タイミング）, 6) 心理モデル：動機付け関連パラメータ, 7) 心理モデル：同調行動関連パラメータ 
# ======================================================================================================================================================
# DRIVER_VISIBILITY_DISTANCE: float = 30.0
# ACTIVE_ROUTE_CHANGE_MEAN = 60.0
# ACTIVE_ROUTE_CHANGE_VAR = 30.0
# CAUTIOUS_ROUTE_CHANGE_MEAN = -60.0
# CAUTIOUS_ROUTE_CHANGE_VAR = 30.0
# SHELTER_CAPACITY_THRESHOLD = 0.5
# TSUNAMI_SIGN_START_TIME = 300
# TSUNAMI_SIGN_END_TIME = 400
# TSUNAMI_PRECURSOR_INFO_OBTAIN_TIME = 0
# MOTIVATION_THRESHOLD_START = 800.0
# MOTIVATION_THRESHOLD_END = 1200.0
# MIN_MOTIVATION_START = 500.0
# MIN_MOTIVATION_END = 700.0
# POSITIVE_LANECHANGE_START = 900.0
# POSITIVE_LANECHANGE_END = 1100.0
# NEGATIVE_LANECHANGE_START = 500.0
# NEGATIVE_LANECHANGE_END = 700.0
# POSITIVE_MAJORITY_BIAS = 100.0
# NEGATIVE_MAJORITY_BIAS = 100.0

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
NORMALCY_BIAS_COUNT = 0
POSITIVE_MAJORITY_BIAS_COUNT = 0
NEGATIVE_MAJORITY_BIAS_COUNT = 0
NEW_VEHICLE_COUNT = 0
PEDESTRIAN_COUNT = 0

# リストの初期化
custome_edge_list: list = []
shelter_list = []
vehInfo_list = []
vehID_list = []
veh_written_list = []
connected_edges_list = []
shelterA_arrival_time_list = []
shelterB_arrival_time_list = []
arrival_time_by_vehID_dict = {}
arrival_time_list = []
elapsed_time_list = []
vehicle_abandant_time_by_pedestrianID_dict = {}
walking_distance_by_pedestrianID_dict = {}
pedstrianID_list = []

# dictの初期化
current_route_dict = {}

import argparse  # ← 追加



def run():
    while traci.simulation.getTime() < END_SIMULATION_TIME:
        traci.simulationStep()
        control_vehicles()
    # if traci.simulatioo
    traci.close()
    sys.stdout.flush()


def control_vehicles():
    vehIDs = traci.vehicle.getIDList()
    pedestrianIDs = traci.person.getIDList()
    current_time = traci.simulation.getTime()
    global NEW_VEHICLE_COUNT
    global LANE_CHANGED_VEHICLE_COUNT
    global POSITIVE_MAJORITY_BIAS_COUNT
    global NEGATIVE_MAJORITY_BIAS_COUNT
    global NORMALCY_BIAS_COUNT
    global OBTAIN_INFO_LANE_CHANGE_COUNT
    global ELAPSED_TIME_LANE_CHANGE_COUNT
    global PEDESTRIAN_COUNT
    global ROUTE_CHANGED_VEHICLE_COUNT
    # 現在の存在するrouteを確認する
    for routeID in traci.route.getIDList():
        # 現在のrouteを取得
        current_route_dict[routeID] = traci.route.getEdges(routeID)

    for current_vehID in vehIDs:
        vehInfo_by_current_vehID: VehicleInfo = utilities.find_vehInfo_by_vehID(current_vehID, vehInfo_list)
        shelter_for_current_vehID: Shelter = utilities.find_shelter_by_edgeID_connect_target_shelter(vehInfo_by_current_vehID.get_edgeID_connect_target_shelter(), shelter_list)
        agent_by_current_vehID: Agent = utilities.find_agent_by_vehID(current_vehID, agent_list)
        current_edgeID: str = traci.vehicle.getRoadID(current_vehID)

        if current_edgeID == "E43" or current_edgeID == "E22":
            traci.vehicle.changeLane(current_vehID, 1, 5)
            traci.vehicle.setLaneChangeMode(current_vehID, 1)

        if current_edgeID == agent_by_current_vehID.get_reserved_vehicle_abandonment_edgeID() and agent_by_current_vehID.get_avoiding_abandoned_vehicle_flg():
            print("予約していた乗り捨てエッジに到達 current_vehID: {}, current_edgeID: {}, time: {}".format(current_vehID, current_edgeID, current_time))
            neighbor_vehicle_abandant_count: int = utilities.count_near_abandoned_vehicle_in_right_lane(vehID=current_vehID, agent_list=agent_list, pedestrianID_list=pedestrianIDs)
            # 乗り捨てられるかテスト
            if utilities.is_again_driver_vehicle_abandant(agent_by_target_vehID=agent_by_current_vehID, 
                                                                        vehInfo_by_target_vehID=vehInfo_by_current_vehID, 
                                                                        current_time=current_time, 
                                                                        neighbor_vehicle_abandant_nums=neighbor_vehicle_abandant_count):
                print("乗り捨てに成功 current_vehID: {}, current_edgeID: {}, time: {}".format(current_vehID, current_edgeID, current_time))
                agent_by_current_vehID.set_avoiding_abandoned_vehicle_flg(False)
    
        if (not agent_by_current_vehID.get_tsunami_info_obtaiend_flg() 
            and list(vehInfo_by_current_vehID.get_tsunami_precursor_info().values())[0][0] 
            and vehInfo_by_current_vehID.get_vehicle_comm_enabled_flag()
            ):
            tsunami_info = vehInfo_by_current_vehID.get_tsunami_precursor_info()
            min_time = min(info[1] for info in tsunami_info.values())
            # print(f"Vehicle {current_vehID} obtained tsunami precursor
            # min info at time {min_time}")
            if min_time > agent_by_current_vehID.get_created_time():
                min_time = agent_by_current_vehID.get_created_time()
            agent_by_current_vehID.set_tsunami_info_obtaiend_time(min_time)
            agent_by_current_vehID.set_tsunami_info_obtaiend_flg(True)


        if not agent_by_current_vehID.get_created_time_flg():
            agent_by_current_vehID.set_created_time(traci.simulation.getTime())
            agent_by_current_vehID.set_created_time_flg(True)

        # 到着処理 # 到着によってparked_flagがTrue 到着車両に関しての処理
        if (traci.vehicle.isStoppedParking(current_vehID) 
            and not vehInfo_by_current_vehID.get_parked_flag()):
            utilities.handle_arrival(
                            current_vehID=current_vehID,
                            vehInfo_by_current_vehID=vehInfo_by_current_vehID,
                            agent_by_current_vehID=agent_by_current_vehID,
                            shelter_for_current_vehID=shelter_for_current_vehID,
                            shelter_list=shelter_list,
                            arrival_time_list=arrival_time_list,
                            arrival_time_by_vehID_dict=arrival_time_by_vehID_dict,
                            elapsed_time_list=elapsed_time_list
                            )
                            

        if not vehInfo_by_current_vehID.get_decline_edge_arrival_flag():  # 減速処理を行う
            #避難地に接続する(直前の)道路にいる場合、減速する
            pre_edgeID_near_shelter_flag = \
                    utilities.is_pre_edgeID_near_shelter(
                                                            current_edgeID=current_edgeID, 
                                                            edgeID_near_shelter=vehInfo_by_current_vehID.get_edgeID_connect_target_shelter(),
                                                            custome_edge_list=custome_edge_list
                                                            )
            if pre_edgeID_near_shelter_flag and not vehInfo_by_current_vehID.get_decline_edge_arrival_flag():
                # print(f"current_edgeID: {current_edgeID}, edgeID_near_shelter: {vehInfo_by_current_vehID.get_edgeID_connect_target_shelter()}, pre_edgeID_near_shelter_flag: {utilities.is_pre_edgeID_near_shelter(current_edgeID=current_edgeID, edgeID_near_shelter=vehInfo_by_current_vehID.get_edgeID_connect_target_shelter(), custome_edge_list=custome_edge_list)}, arrival_flag: {vehInfo_by_current_vehID.get_arrival_flag()}")
                # 避難地直前のエッジに入った車両だけ減速制御
                local_density = utilities.get_local_density(vehID=current_vehID, radius=50.0)
                utilities.apply_gap_density_speed_control(
                                                            vehID=current_vehID,
                                                            local_density=local_density,
                                                            v_free=4.0,     # 自由流速度
                                                            v_min=2.0,      # 最低速度
                                                            gap_min=7.0,    # 強い減速を始めるギャップ
                                                            tau=1.8,        # ギャップ→速度変換の傾き
                                                            alpha=0.5,      # 平滑化
                                                            slow_time=1.0   # 速度変更時間
                                                            )
            else:
                # それ以外は自由流走行
                traci.vehicle.slowDown(current_vehID, 5.0, 1.0)

        if not vehInfo_by_current_vehID.get_arrival_flag(): # 未到着の車両に対して処理を実行
            # 通信可能範囲内にいる車両と通信を行う　通信可能範囲は100m設定になる
            if vehInfo_by_current_vehID.get_vehicle_comm_enabled_flag() and current_time < VEHICLE_NUM*VEHICLE_INTERVAL+100:
                if traci.simulation.getTime() % 10 == 0:
                    around_vehIDs: list = utilities.get_around_vehIDs(target_vehID=current_vehID, custome_edge_list=custome_edge_list)
                    utilities.v2v_communication(
                                                target_vehID=current_vehID, 
                                                target_vehInfo=vehInfo_by_current_vehID, 
                                                around_vehIDs=around_vehIDs,
                                                agent_list=agent_list,
                                                vehInfo_list=vehInfo_list,
                                                COMMUNICATION_RANGE=COMM_RANGE
                                                )

                    utilities.v2shelter_communication(
                                                        target_vehID=current_vehID, 
                                                        shelterID=vehInfo_by_current_vehID.get_target_shelter(),
                                                        vehInfo_list=vehInfo_list,
                                                        shelter_list=shelter_list,
                                                        COMMUNICATION_RANGE=COMM_RANGE
                                                        )
                    
                    utilities.v2v_communication_about_tsunami_info(
                                                                    target_vehID=current_vehID, 
                                                                    target_vehInfo=vehInfo_by_current_vehID, 
                                                                    around_vehIDs=around_vehIDs, 
                                                                    vehInfo_list=vehInfo_list, 
                                                                    COMMUNICATION_RANGE=COMM_RANGE
                                                                    )
            # 経路変更するのか検討する
            if current_edgeID in ["E0", "E1", "E20"] and not agent_by_current_vehID.get_evacuation_route_changed_flg():
                # 避難所要時間をどのように定義するのかを考える
                if traci.simulation.getTime() % 5 == 0:
                    routeID = utilities.find_alternative_route_calculated_time(
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

            if current_edgeID in ["E14", "E15", "E38", "E39", "E40", "E41", "E42", "E7"]:
                # not agent_by_current_vehID.get_evacuation_route_changed_flg()
                leader_info = traci.vehicle.getLeader(vehID=current_vehID, dist=10.0)
                current_lane_index = traci.vehicle.getLaneIndex(current_vehID)

                # -----------------------------------
                # lane1走行中：前方の放置車両を検知したら lane2 へ回避
                # -----------------------------------
                if current_lane_index == 1 and leader_info is not None:
                    leader_vehID = leader_info[0]
                    leader_agent: Agent = utilities.find_agent_by_vehID(leader_vehID, agent_list)

                    if leader_agent is not None and leader_agent.get_vehicle_abandoned_flg():
                        right_leaders = traci.vehicle.getRightLeaders(current_vehID)
                        right_followers = traci.vehicle.getRightFollowers(current_vehID)
                        # print(f"right leaders of {current_vehID}: {right_leaders}, right followers of {current_vehID}: {right_followers}")

                        right_lane_has_abandoned_vehicle = False
                        right_lane_is_safe = True

                        if right_leaders:
                            right_leader_vehID = right_leaders[0][0]
                            right_leader_agent: Agent = utilities.find_agent_by_vehID(right_leader_vehID, agent_list)
                            if right_leader_agent is not None and right_leader_agent.get_vehicle_abandoned_flg():
                                right_lane_has_abandoned_vehicle = True

                        # 後方車両が近すぎる場合は危険とみなす
                        if right_followers:
                            # print(f"Vehicle {current_vehID} right followers: {right_followers}")
                            right_follower_vehID = right_followers[0][0]
                            right_follower_dist = right_followers[0][1]

                            # dist の意味は環境依存で確認推奨
                            if right_follower_dist > 10:
                                right_lane_is_safe = False

                        if (not right_lane_has_abandoned_vehicle) and right_lane_is_safe:
                            # print(f"leader_agent.get_vehicle_abandoned_flg():{leader_agent.get_vehicle_abandoned_flg()}")
                            # print(f"lane 1->2Time {traci.simulation.getTime()}: Vehicle {current_vehID} is changing lane to avoid abandoned vehicle {leader_vehID}")
                            traci.vehicle.changeLane(current_vehID, 2, 10)
                            traci.vehicle.slowDown(current_vehID, 2.0, 1.0)
                            traci.vehicle.setLaneChangeMode(current_vehID, 1621)

                            # 回避対象の放置車両を記録
                            agent_by_current_vehID.set_avoiding_abandoned_vehicle_flg(True)
                            agent_by_current_vehID.set_target_abandoned_vehID(leader_vehID)

                # 車両乗り捨て行動 TODO 経路を確認する
                if (
                    not agent_by_current_vehID.get_vehicle_abandoned_flg()
                    and (
                        utilities.is_vehID_in_congested_edge(
                            vehID=current_vehID,
                            threshold_speed=THRESHOLD_SPEED
                        )
                        or utilities.is_vehID_blocked_by_abandoned_vehicle(
                            vehID=current_vehID,
                            agent_list=agent_list
                        )
                        )
                    ):                    # 渋滞に巻き込まれたら、渋滞継続時間測定のためにフラグをTrueにし、計測時間を作る
                    traci.vehicle.setColor(current_vehID,(225, 90, 40))
                    if not agent_by_current_vehID.get_encounted_congestion_flg():
                        agent_by_current_vehID.set_congestion_duration(traci.simulation.getTime())
                        agent_by_current_vehID.set_encounted_congestion_flg(True)
                    
                    if traci.simulation.getTime() % 5 == 0:
                        # 運転者の車両乗り捨て行動の実装　渋滞継続時間が一定時間を超えたら、車両を乗り捨てる
                        if not (current_edgeID == "E15"):
                            if not agent_by_current_vehID.get_vehicle_abandoned_flg() and not agent_by_current_vehID.get_avoiding_abandoned_vehicle_flg():
                                neighbor_vehicle_abandant_count: int = utilities.count_near_abandoned_vehicle_in_right_lane(vehID=current_vehID, agent_list=agent_list, pedestrianID_list=pedestrianIDs)
                                if (utilities.is_driver_vehicle_abandant(agent_by_target_vehID=agent_by_current_vehID, 
                                                                        vehInfo_by_target_vehID=vehInfo_by_current_vehID, 
                                                                        current_time=current_time, 
                                                                        neighbor_vehicle_abandant_nums=neighbor_vehicle_abandant_count,
                                                                        alpha=ALPHA)
                                    or agent_by_current_vehID.get_congestion_duration() - current_time > 600): # 600秒を超えたら強制的に乗り捨てる
                                    traci.vehicle.setColor(current_vehID,(0, 103,192))
                                    agent_by_current_vehID.set_vehicle_abandoned_flg(True)
                                    if utilities.is_vehicle_abandant_this_position(
                                                                            current_vehID=current_vehID, 
                                                                            current_edgeID=current_edgeID,
                                                                            agent_by_current_vehID=agent_by_current_vehID, 
                                                                            vehInfo_by_target_vehID=vehInfo_by_current_vehID, 
                                                                            PEDESTRIAN_COUNT=PEDESTRIAN_COUNT, 
                                                                            STOPPING_TIME_IN_SHELTER=STOPPING_TIME_IN_SHELTER,
                                                                            shelter=shelter_for_current_vehID
                                                                            ):
                                        PEDESTRIAN_COUNT, pedestrianID, walking_distance= utilities.vehicle_abandant_behavior(
                                                                                                            current_vehID=current_vehID, 
                                                                                                            current_edgeID=current_edgeID,
                                                                                                            agent_by_current_vehID=agent_by_current_vehID, 
                                                                                                            vehInfo_by_target_vehID=vehInfo_by_current_vehID, 
                                                                                                            PEDESTRIAN_COUNT=PEDESTRIAN_COUNT, 
                                                                                                            STOPPING_TIME_IN_SHELTER=STOPPING_TIME_IN_SHELTER,
                                                                                                            shelter=shelter_for_current_vehID
                                                                                                            )
                                        
                                        # print(f"Vehicle {current_vehID} has been abandoned at edge {current_edgeID} and changed to pedestrian {pedestrianID}")
                                        if pedestrianID is None:
                                            print(f"[WARN] pedestrian was not created for {current_vehID}; skip abandonment registration")
                                            continue
                                        agent_by_current_vehID.set_vehicle_abandoned_flg(True)
                                        vehicle_abandant_time_by_pedestrianID_dict[pedestrianID] = current_time
                                        walking_distance_by_pedestrianID_dict[pedestrianID] = walking_distance
                                        pedstrianID_list.append(pedestrianID)
                                        print("予約なしで車両乗り捨てを行いました")
                                    else:
                                        print("車両乗り捨てを予約しました current_vehID: {}, current_edgeID: {}, time: {}".format(current_vehID, current_edgeID, current_time))
                                        PEDESTRIAN_COUNT, pedestrianID, walking_distance= utilities.vehicle_abandant_behavior(
                                                                                                                current_vehID=current_vehID, 
                                                                                                                current_edgeID=current_edgeID,
                                                                                                                agent_by_current_vehID=agent_by_current_vehID, 
                                                                                                                vehInfo_by_target_vehID=vehInfo_by_current_vehID, 
                                                                                                                PEDESTRIAN_COUNT=PEDESTRIAN_COUNT, 
                                                                                                                STOPPING_TIME_IN_SHELTER=STOPPING_TIME_IN_SHELTER,
                                                                                                                shelter=shelter_for_current_vehID
                                                                                                                )
                                        agent_by_current_vehID.set_vehicle_abandoned_flg(True)
                                        vehicle_abandant_time_by_pedestrianID_dict[pedestrianID] = agent_by_current_vehID.get_vehicle_abandoned_time()
                                        walking_distance_by_pedestrianID_dict[pedestrianID] = walking_distance
                                        pedstrianID_list.append(pedestrianID)
                                        edgeIDs_of_target_vehID = traci.route.getEdges(traci.vehicle.getRouteID(current_vehID))
                                        next_edge_of_current_edgeID = utilities.get_next_edge(edgeIDs=edgeIDs_of_target_vehID, current_edgeID=current_edgeID)
                                        print(f"current_vehID: {current_vehID}, cur_edge: {current_edgeID}, 予約した乗り捨てエッジnext_edge_of_current_edgeID: {next_edge_of_current_edgeID}")
                                        agent_by_current_vehID.set_reserved_vehicle_abandonment_edgeID(next_edge_of_current_edgeID)
                                        agent_by_current_vehID.set_avoiding_abandoned_vehicle_flg(True)
                                        traci.vehicle.highlight(current_vehID, (255, 0, 0))                           

            if current_edgeID == "E0":
                if traci.simulation.getTime() > TSUNAMI_SIGN_START_TIME and traci.simulation.getTime() < TSUNAMI_SIGN_END_TIME:
                    vehInfo_by_current_vehID.update_tsunami_precursor_info(vehID=current_vehID, tsunami_precursor_flag=True, current_time=traci.simulation.getTime())

    for pedestrianID, vehicle_abandant_time in vehicle_abandant_time_by_pedestrianID_dict.items():
        vehID_by_pedestrianID: str = utilities.extract_vehicle_id(pedestrianID)
        agent_by_pedestrianID: Agent = utilities.find_agent_by_vehID(vehID_by_pedestrianID, agent_list=agent_list)
        vehInfo_by_pedestrianID: VehicleInfo = utilities.find_vehInfo_by_vehID(vehID_by_pedestrianID, vehInfo_list)
        shelter_by_pedestrianID: Shelter = utilities.find_shelter_by_edgeID_connect_target_shelter(vehInfo_by_pedestrianID.get_edgeID_connect_target_shelter(), shelter_list)
        if not agent_by_pedestrianID.get_arrival_shelter_flg():
            if traci.person.getRoadID(pedestrianID) == "E16"  :
                # pedstrianID_list.remove(pedestrianID)
                utilities.handle_arrival_for_pedestrian(
                                                        pedestrianID=pedestrianID,
                                                        current_vehID=current_vehID,
                                                        vehInfo_by_current_vehID=vehInfo_by_pedestrianID,
                                                        agent_by_current_vehID=agent_by_pedestrianID,
                                                        shelter_for_current_vehID=shelter_by_pedestrianID,
                                                        shelter_list=shelter_list,
                                                        arrival_time_list=arrival_time_list,
                                                        arrival_time_by_vehID_dict=arrival_time_by_vehID_dict,
                                                        elapsed_time_list=elapsed_time_list
                                                        )
    
    utilities.calculate_avg_evac_time_by_route(shelter_list=shelter_list)
    # # shelterごとで情報を共有する
    # utilities.merge_route_info_within_shelters(shelter_list[0], shelter_list[1])
    # # shelterごとに到着した車両の数を共有する
    # utilities.merge_arrival_vehs_of_shelter(shelter_list=shelter_list)

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
    options, args = optParser.parse_args()
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
    # print(f"Loaded config: {cfg}")

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
    shelter_capacity_by_ID:dict = {"ShelterA_1": 150}
    edgeID_by_shelterID:dict = {"ShelterA_1": 'E16'}
    tmp_prob_list = [1.0]
    for shelterID, near_edgeID in edgeID_by_shelterID.items():
        shelter_list:list = utilities.init_shelter(
                                                    shelterID=shelterID, 
                                                    shelter_capacity_by_ID=shelter_capacity_by_ID, 
                                                    near_edgeID=near_edgeID, 
                                                    shelter_list=shelter_list
                                                    )
    
    custome_edge_list:list = utilities.init_custome_edge()
    vehicle_start_edges:list = utilities.get_vehicle_start_edges(custome_edge_list=custome_edge_list)
    vehicle_end_edges:list = utilities.get_vehicle_end_edges(custome_edge_list=custome_edge_list)
    # 車両の開始エッジと終了エッジの組み合わせを辞書にする
    vehicle_end_list_by_start_edge_dict:dict = utilities.get_vehicle_end_list_by_start_edge_dict(vehicle_start_edges=vehicle_start_edges, vehicle_end_edges=vehicle_end_edges)
    # 全経路で総当たりをし、通行可能経路を取得しておく。
    connected_edges_list:list = utilities.import_connected_edges_from_json(file_path=str(DATA_DIR / "all_edgeIDs.json"))
    nearest_end_edgeID_by_start_edgeID_dict:dict = utilities.import_start_end_edgeIDs_from_json(file_path=str(DATA_DIR / "start_end_edgeIDs_ishinomaki_toymap.json"))
    all_route_edgeID_list_by_routeID:dict = utilities.convert_routefile_to_routes_by_id(file_path=str(DATA_DIR / "ishinomaki_toymap.rou.xml"))
    # print(f"all_route_edgeID_list_by_routeID: {all_route_edgeID_list_by_routeID}")
    routeID_list = list(all_route_edgeID_list_by_routeID.keys())
    # print(f"routeID_list: {routeID_list}")

    for start_end_edges, edges_list in all_route_edgeID_list_by_routeID.items():
        max_route_num = 0
        start_edgeIDs, end_edgeID, routeIndex = start_end_edges.split("_")
        max_route_num = max(max_route_num, int(routeIndex))
    route_prob_list = [0.3, 0.7]
    # route_prob_list = utilities.generate_route_prob_list(max_route_num)
    each_vehnum_to_shelter = int(NUM_VEHICLES / len(nearest_end_edgeID_by_start_edgeID_dict))

    # each_vehnum_to_shelter = 5
    for vehicle_start_edgeID, end_edgeIDs in nearest_end_edgeID_by_start_edgeID_dict.items():
        DEPART_TIME=0
        routeIndex:list = []
        for index in range(each_vehnum_to_shelter):
            end_edgeID:str = utilities.choose_edge_by_probability(edgeID_list=end_edgeIDs, probabilities=tmp_prob_list)
            # print(f"end_edgeID: {end_edgeID}, edgeID_by_shelterID: {edgeID_by_shelterID}")
            shelterID_by_end_edgeID:str = utilities.find_shelterID_by_edgeID_by_shelterID(edgeID=end_edgeID, edgeID_by_shelterID=edgeID_by_shelterID)

            for start_end_edges, edges_list in all_route_edgeID_list_by_routeID.items():
                start_edgeIDs, end_edgeID, routeIndex = start_end_edges.split("_")
                if routeIndex not in routeIndex:
                    routeIndex.append(routeIndex)
            route_edges:list = utilities.choose_route_edges_by_probability(route_edges_list_by_start_end_index=all_route_edgeID_list_by_routeID, routeID_list=routeID_list,probabilities=route_prob_list)
            VEHICLE_NUM, ROUTE_NUM, vehID_list_by_shelter, DEPART_TIME = \
                utilities.generate_init_vehID_with_route_edges(
                                                        from_edgeID=vehicle_start_edgeID, 
                                                        to_edgeID=end_edgeID, 
                                                        shelterID=shelterID_by_end_edgeID, 
                                                        generate_interval=VEHICLE_INTERVAL,
                                                        generate_route_count=ROUTE_NUM, 
                                                        generate_veh_count=VEHICLE_NUM, 
                                                        depart_time=DEPART_TIME,
                                                        route_edges=route_edges
                                                        )
            vehID_list.extend(vehID_list_by_shelter)

    # カテゴリのカウント
    categories = [extract_category(v) for v in vehID_list]
    counter = Counter(categories)

    # 合計と割合を表示
    total = sum(counter.values())
    print("出現数:", dict(counter))
    print("割合:")
    for cat in ["A1", "A2", "B1", "B2"]:
        count = counter.get(cat, 0)
        print(f"  {cat}: {count} ({count / total:.2%})")
    # 車両情報の初期化
    # ここで調整がいる
    vehInfo_list:list = utilities.init_vehicleInfo_list_base(vehIDs=vehID_list, shelter_list=shelter_list, v2v_capable_vehicle_rate=v2v_capable_vehicle_rate) 

    # Agentの初期化
    # 乗り捨てに関する初期化
    agent_list:list = utilities.init_agent_list(
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

    # ドライバーの行動の初期化
    utilities.init_driver_behavior(vehIDs = vehID_list, lane_change_mode=1)
    for vehID in traci.vehicle.getIDList():
        traci.vehicle.setMaxSpeed(vehID, 7.0)
    run()
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




