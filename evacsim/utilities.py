# =========================
# Standard library
# =========================
import copy
import json
import math
import os
import random
import sys
from math import sqrt
from typing import List

import optparse  # keep as-is

# =========================
# SUMO (SUMO_HOME must be on sys.path before importing sumolib/traci)
# =========================
if "SUMO_HOME" in os.environ:
    sys.path.append(os.path.join(os.environ["SUMO_HOME"], "tools"))

from sumolib import checkBinary  # noqa: E402
import traci  # noqa: E402

# =========================
# Third-party libraries
# =========================
import matplotlib.pyplot as plt
import numpy as np
from numpy import double

# =========================
# Local / intra-package
# =========================
from .agents.Agent import Agent
from .agents.CustomeEdge import CustomeEdge, ConnectedEdges
from .agents.Shelter import Shelter
from .agents.VehicleInfo import VehicleInfo

# =========================
# Runtime config
# =========================
# random.seed(316)  # 乱数シードを314に設定（元のコメントを維持）　確率系にも影響を与えるのでかなり注意する
random.seed()

VEHICLE_SHELTER_DURATION_TIME = 10000

# CustomEdgeリストから特定のedgeIDを持つCustomEdgeを取得
def get_custom_edge_by_edgeID(edgeID:str, custome_edge_list:list):
    for custom_edge in custome_edge_list:
        if custom_edge.get_current_edgeID() == edgeID:
            return custom_edge

# CustomeEdgeのincom/outgo のなかからjuntionを除いたedgeIDリストを取得
def get_edgeIDs_without_junction(customeEdge:CustomeEdge):
    inco_edgeIDs =[
        inco_edgeID for inco_edgeID in customeEdge.obtain_neighbour_incom_edgeIDs_with_junc_by_start_junc()
        if not inco_edgeID.startswith(":")
    ]
    outgo_edgeIDs =[
        outgo_edgeID for outgo_edgeID in customeEdge.obtain_neighbour_outcom_edgeIDs_with_junc_by_end_junc()
        if not outgo_edgeID.startswith(":")
    ]
    return inco_edgeIDs, outgo_edgeIDs

# 車両IDを元に、周辺edgeIDを取得
def get_around_edgeIDs(target_vehID:str, custome_edge_list:list):
    # 車両がいるedgeIDを取得
    current_edgeID = traci.vehicle.getRoadID(target_vehID)
    # 車両がいるedgeIDをもとに、周辺のedgeIDを取得する
    current_edge:CustomeEdge = get_custom_edge_by_edgeID(current_edgeID, custome_edge_list)
    around_edgeIDs_with_junction = remove_junction_from_edgeID(current_edge.around_edgeIDs())

    return around_edgeIDs_with_junction

# 車両IDを元に、周辺edgeIDを取得し、周辺車両IDを取得
def get_around_vehIDs(target_vehID:str, custome_edge_list:list):
    around_edgeIDs_for_target_vehID = get_around_edgeIDs(target_vehID, custome_edge_list)
    around_vehIDs = []
    for around_edgeID in around_edgeIDs_for_target_vehID:
        tmp_vehIDs = traci.edge.getLastStepVehicleIDs(around_edgeID)
        around_vehIDs.extend(tmp_vehIDs)
    return around_vehIDs

def get_vehicle_start_edges(custome_edge_list: list[CustomeEdge]) -> list[CustomeEdge]:
    """開始エッジ(CustomEdge)だけを抽出して返す"""
    vehicle_start_edges: list[CustomeEdge] = []
    for custome_edge in custome_edge_list:
        if custome_edge.is_current_edgeID_start_edge():
            vehicle_start_edges.append(custome_edge)
    return vehicle_start_edges

def get_vehicle_end_edges(custome_edge_list: list[CustomeEdge]) -> list[CustomeEdge]:
    """終了エッジ(CustomEdge)だけを抽出して返す"""
    vehicle_end_edges:list = []
    for custome_edge in custome_edge_list:
        if custome_edge.is_current_edgeID_end_edge():
            vehicle_end_edges.append(custome_edge)
    return vehicle_end_edges

# def get_vehicle_end_list_by_start_edge_dict(vehicle_start_edges:list, vehicle_end_edges:list):
def get_vehicle_end_list_by_start_edge_dict(vehicle_start_edges: list[CustomeEdge], vehicle_end_edges: list[CustomeEdge]) -> dict[str, list[str]]:
    vehicle_end_list_by_start_edge_dict:dict = {}
    for start_edge in vehicle_start_edges:
        tmp_list = []
        for end_edge in vehicle_end_edges:
            if len(list(traci.simulation.findRoute(start_edge.get_current_edgeID(), end_edge.get_current_edgeID()).edges)) > 0:
                tmp_list.append(end_edge.get_current_edgeID())
        vehicle_end_list_by_start_edge_dict[start_edge.get_current_edgeID()] = tmp_list
    return vehicle_end_list_by_start_edge_dict

def get_nearest_end_edgeID_by_start_edgeID(vehicle_end_list_by_start_edge_dict:dict):
    print(f'vehicle_end_list_by_start_edge_dict: {vehicle_end_list_by_start_edge_dict}')
    nearest_end_edgeID_by_start_edgeID_dict:dict = {}
    for vehicle_start_edgeID, vehicle_end_edgeID_list in vehicle_end_list_by_start_edge_dict.items():
            start_edge_shape = traci.lane.getShape("{}_0".format(vehicle_start_edgeID))
            start_edge_center = ((start_edge_shape[0][0] + start_edge_shape[1][0])/2, (start_edge_shape[0][1] + start_edge_shape[1][1])/2)
            disatnce_by_start_edge_ID:dict = {}
            for vehicle_end_edgeID in vehicle_end_edgeID_list:
                end_edge_shape = traci.lane.getShape("{}_0".format(vehicle_end_edgeID))
                end_edge_center = ((end_edge_shape[0][0] + end_edge_shape[1][0])/2, (end_edge_shape[0][1] + end_edge_shape[1][1])/2)
                distance = sqrt((start_edge_center[0] - end_edge_center[0])**2 + (start_edge_center[1] - end_edge_center[1])**2)
                disatnce_by_start_edge_ID[vehicle_end_edgeID] = distance
            sorted_disatnce_by_start_edgeID = sorted(disatnce_by_start_edge_ID.items(), key=lambda x:x[1])
            nearest_end_edgeID_by_start_edgeID_dict[vehicle_start_edgeID] = [edgeID for edgeID, _ in sorted_disatnce_by_start_edgeID]
    return nearest_end_edgeID_by_start_edgeID_dict

def get_opposie_edgeID_by_edgeID(edgeID:str):
    if edgeID.startswith("-"):
        opposite_edgeID:str = edgeID.lstrip('-')
    else:
        opposite_edgeID:str = "-" + edgeID
    # 反対車線が存在するか確認
    if opposite_edgeID in traci.edge.getIDList():
        return opposite_edgeID
    else:
        # print(f'反対車線が存在しません: {edgeID}')
        return edgeID

def get_next_edge(edgeIDs: tuple, current_edgeID: str) -> str | None:
    """
    指定したエッジ(current_edge)の次のエッジを返す。
    
    :param edgeIDs: エッジの順序付きタプル
    :param current_edge: 現在のエッジID
    :return: 次のエッジID、または存在しない場合は None
    """
    if current_edgeID in edgeIDs:
        idx = edgeIDs.index(current_edgeID)
        if idx + 1 < len(edgeIDs):
            return edgeIDs[idx + 1]
    return None

def get_prev_edge(edgeIDs: tuple, current_edgeID: str):
    """
    指定したエッジ(current_edgeID)の一つ前のエッジを返す。

    :param edgeIDs: エッジの順序付きタプル
    :param current_edgeID: 現在のエッジID
    :return: 一つ前のエッジID、または存在しない場合は None
    """
    if current_edgeID in edgeIDs:
        idx = edgeIDs.index(current_edgeID)
        if idx > 0:
            return edgeIDs[idx - 1]
    return None

def generate_new_veh_based_on_route_time(target_vehID:str, NEW_VEHICLE_COUNT:int, agent_list:list, vehInfo_list:list, \
                    vehInfo_by_target_vehID:VehicleInfo, agent_by_target_vehID:Agent, 
                    from_edgeID:str, new_shelterID:str, to_edgeID:str):
    if from_edgeID == "" and new_shelterID == "" and to_edgeID == "":
        # print(f"避難経路が存在しません")
        return NEW_VEHICLE_COUNT
    # 現在のエッジIDを取得
    current_edgeID = traci.vehicle.getRoadID(target_vehID)
    opposite_edgeID = get_opposie_edgeID_by_edgeID(current_edgeID)
    if current_edgeID == opposite_edgeID:
        return NEW_VEHICLE_COUNT

    # 候補地が全て満杯の場合はreturn
    if is_candidate_shelter_full(agent_by_target_vehID, vehInfo_by_target_vehID):
        # print(f"候補地が全て満杯です")
        return NEW_VEHICLE_COUNT

    # 経由地点を取得する
    via_edgeIDs_with_intial_end_edge:list = list(traci.simulation.findRoute(from_edgeID, to_edgeID).edges)
    if via_edgeIDs_with_intial_end_edge is None or len(via_edgeIDs_with_intial_end_edge) == 0:
        via_edgeIDs_with_intial_end_edge:list = list(traci.simulation.findRoute(opposite_edgeID, to_edgeID).edges)
        if via_edgeIDs_with_intial_end_edge is None or len(via_edgeIDs_with_intial_end_edge) == 0:
            return NEW_VEHICLE_COUNT
    # target_vehIDから数字をとる
    vehID_num = target_vehID.split("_")[3]
    # 新しい車両IDを生成と出発時間を設定
    new_veh_ID:str = "{}_{}_{}_{}".format("newveh", new_shelterID, vehID_num, NEW_VEHICLE_COUNT)
    deparet_time:double = traci.simulation.getTime()
    # 現在のagentを無効にする
    agent_by_target_vehID.set_shelter_changed_flg(True)
    # 候補地を更新
    # TODO　満杯情報を受け取った後は、当該避難地を候補地から削除
    updated_candidate_shelter = agent_by_target_vehID.get_candidate_shelter()
    # 元のvehIDを削除 新しい車両IDを追加
    # Agentを新規に作成する
    agent:Agent = Agent(vehID=new_veh_ID, 
                        target_shelter=new_shelterID, 
                        tunning_threshold=copy.deepcopy(agent_by_target_vehID.get_tunning_threshold()), 
                        route_change_threshold=copy.deepcopy(agent_by_target_vehID.get_route_change_threshold())
                        )
    agent.set_near_edgeID_by_target_shelter(copy.deepcopy(agent_by_target_vehID.get_near_edgeID_by_target_shelter()))
    agent.set_candidate_edge_by_shelterID(updated_candidate_shelter)
    agent.init_set_candidate_near_shelter(shelter_edge_by_IDs=updated_candidate_shelter)
    
    agent_list.append(agent)

    # 車両情報を継承する
    new_vehInfo_by_target_vehID:VehicleInfo = VehicleInfo(vehID=new_veh_ID, 
                                                      target_shelter=new_shelterID, 
                                                      edgeID_connect_target_shelter=to_edgeID, 
                                                      create_time=deparet_time)
    new_vehInfo_by_target_vehID.set_shelter_congestion_info(copy.deepcopy(vehInfo_by_target_vehID.get_shelter_congestion_info()))
    new_vehInfo_by_target_vehID.set_avg_evac_time_by_route_by_recive_time(copy.deepcopy(vehInfo_by_target_vehID.get_avg_evac_time_by_route_by_recive_time()))
    vehInfo_list.append(new_vehInfo_by_target_vehID)
    # !# agentが変更したので、フラグを立てる
    vehInfo_by_target_vehID.set_agent_changed_flag(True)

    # edge上の始点からどこにいるのかを取得する
    edge_position = traci.vehicle.getLanePosition(target_vehID)
    current_laneID = traci.vehicle.getLaneID(target_vehID)
    if traci.lane.getLength(current_laneID) > 100:
        depart_position = 200 - edge_position
    if traci.lane.getLength(current_laneID) <= 100:
        depart_position = 100 - edge_position
    traci.vehicle.remove(target_vehID)
    new_route_ID:str = "{}_{}_{}".format("newroute", new_shelterID, NEW_VEHICLE_COUNT)
    traci.route.add(routeID=new_route_ID, edges=via_edgeIDs_with_intial_end_edge)
    traci.vehicle.add(vehID=new_veh_ID, routeID=new_route_ID, depart=deparet_time, departPos=depart_position)
    traci.vehicle.setParkingAreaStop(vehID=new_veh_ID, stopID=new_shelterID, duration=100000)
    # traci.vehicle.setColor(new_veh_ID, (128, 249, 255, 255))
    NEW_VEHICLE_COUNT += 1
    # print(f"normacy is changed evacuation") 
    return NEW_VEHICLE_COUNT

def generate_new_veh(
    target_vehID: str,
    NEW_VEHICLE_COUNT: int,
    agent_list: list,
    vehInfo_list: list,
    vehInfo_by_target_vehID: VehicleInfo,
    agent_by_target_vehID: Agent,
    shelter_list: list,
    connected_edges_list: list,
    LATE_AGENT_THRESHOLD_LIST: list,
    lane_change_mode: int
    ):
    """
    既存車両 target_vehID を基点に、候補避難地のうち新たな避難地へ向かう
    新規車両を生成して置き換える。

    返り値:
        NEW_VEHICLE_COUNT (int): インクリメント後の生成カウンタ
    """
    # === 現在位置と対向情報の取得 ===
    current_edgeID = traci.vehicle.getRoadID(target_vehID)
    opposite_edgeID = get_opposie_edgeID_by_edgeID(current_edgeID)  # 既存関数そのまま

    # 同一路線判定（対向が同じなら生成不要）
    if current_edgeID == opposite_edgeID:
        # print(f"現在地と反対車線が同じです: {current_edgeID}")
        return NEW_VEHICLE_COUNT

    # 候補地満杯チェック（全候補が満杯なら生成しない）
    if is_candidate_shelter_full(agent_by_target_vehID, vehInfo_by_target_vehID):
        # print("候補地が全て満杯です")
        return NEW_VEHICLE_COUNT

    # === 新しい避難地・経路端点を決定 ===
    from_edgeID, new_shelterID, to_edgeID = get_new_shelterID_and_near_edgeID_by_vehID_based_on_distance(
        current_edgeID=current_edgeID,
        opposite_edgeID=opposite_edgeID,
        agent_by_target_vehID=agent_by_target_vehID,
        connected_edges_list=connected_edges_list
    )

    # 避難経路が存在しない場合は生成しない
    if from_edgeID == "" and new_shelterID == "" and to_edgeID == "":
        return NEW_VEHICLE_COUNT

    # === 経路探索（from → to） ===
    via_edgeIDs_with_intial_end_edge: list = list(
        traci.simulation.findRoute(from_edgeID, to_edgeID).edges
    )

    # === 新車両ID・時刻などの準備 ===
    vehID_num = target_vehID.split("_")[3]  # 既存仕様：ID末尾のナンバを継承
    new_veh_ID: str = "{}_{}_{}_{}".format("newveh", new_shelterID, vehID_num, NEW_VEHICLE_COUNT)
    deparet_time: double = traci.simulation.getTime()  # 既存の変数名を維持（typo含む）

    # === 既存Agentの状態更新 & 新Agent生成 ===
    agent_by_target_vehID.set_shelter_changed_flg(True)  # 現在のagentを無効化

    # TODO: 満杯情報を受け取った後は、当該避難地を候補から削除するロジックを入れる
    updated_candidate_shelter = agent_by_target_vehID.get_candidate_shelter()

    agent: Agent = Agent(
                            vehID=new_veh_ID,
                            target_shelter=new_shelterID,
                            tunning_threshold=copy.deepcopy(agent_by_target_vehID.get_tunning_threshold()),
                            route_change_threshold=copy.deepcopy(agent_by_target_vehID.get_route_change_threshold()),
                            lane_change_init_threshold=copy.deepcopy(agent_by_target_vehID.get_lane_change_decision_threshold()),
                            normalcy_motivation_increase=copy.deepcopy(agent_by_target_vehID.get_motivation_increase_from_info_receive()),
                            motivation_decrease_due_to_inactive_neighbors=copy.deepcopy(agent_by_target_vehID.get_motivation_decrease_due_to_inactive_neighbors()),
                            motivation_increase_due_to_following_neighbors=copy.deepcopy(agent_by_target_vehID.get_motivation_increase_due_to_following_neighbors()),
                            lane_minimum_motivation_value=copy.deepcopy(agent_by_target_vehID.get_minimum_motivation_value())
                            )
    agent.set_near_edgeID_by_target_shelter(copy.deepcopy(agent_by_target_vehID.get_near_edgeID_by_target_shelter()))
    agent.set_candidate_edge_by_shelterID(updated_candidate_shelter)
    agent.init_set_candidate_near_shelter(shelter_edge_by_IDs=updated_candidate_shelter)

    agent.set_x_elapsed_time_for_lane_change_list(copy.deepcopy(agent_by_target_vehID.get_x_elapsed_time_for_lane_change_list()))
    agent.set_y_motivation_value_for_lane_change_list(copy.deepcopy(agent_by_target_vehID.get_y_motivation_value_for_lane_change_list()))
    agent.set_lane_change_xy_dict(copy.deepcopy(agent_by_target_vehID.get_lane_change_xy_dict()))

    agent.set_motivation_decrease_due_to_inactive_neighbors(copy.deepcopy(agent_by_target_vehID.get_motivation_decrease_due_to_inactive_neighbors()))
    agent.set_motivation_increase_due_to_following_neighbors(copy.deepcopy(agent_by_target_vehID.get_motivation_increase_due_to_following_neighbors()))
    agent.set_calculated_motivation_value(copy.deepcopy(agent_by_target_vehID.get_calculated_motivation_value()))
    agent_list.append(agent)

    # === VehicleInfo を継承して新規作成 ===
    new_vehInfo_by_target_vehID: VehicleInfo = VehicleInfo(
        vehID=new_veh_ID,
        target_shelter=new_shelterID,
        edgeID_connect_target_shelter=to_edgeID,
        create_time=deparet_time
    )
    new_vehInfo_by_target_vehID.set_shelter_congestion_info(
        copy.deepcopy(vehInfo_by_target_vehID.get_shelter_congestion_info())
    )
    new_vehInfo_by_target_vehID.set_avg_evac_time_by_route_by_recive_time(
        copy.deepcopy(vehInfo_by_target_vehID.get_avg_evac_time_by_route_by_recive_time())
    )
    new_vehInfo_by_target_vehID.set_tsunami_precursor_info(
        copy.deepcopy(vehInfo_by_target_vehID.get_tsunami_precursor_info())
    )

    vehInfo_list.append(new_vehInfo_by_target_vehID)

    # === 置換のための出発位置計算 ===
    edge_position = traci.vehicle.getLanePosition(target_vehID)
    current_laneID = traci.vehicle.getLaneID(target_vehID)
    current_lane_len = traci.lane.getLength(current_laneID)

    if current_lane_len > 100:
        depart_position = 200 - edge_position
    else:
        depart_position = 100 - edge_position

    # === 旧車両を削除し、新車両を追加 ===
    traci.vehicle.remove(target_vehID)

    new_route_ID: str = "{}_{}_{}".format("newroute", new_shelterID, NEW_VEHICLE_COUNT)
    traci.route.add(routeID=new_route_ID, edges=via_edgeIDs_with_intial_end_edge)
    traci.vehicle.add(
        vehID=new_veh_ID,
        routeID=new_route_ID,
        depart=deparet_time,
        departPos=depart_position
    )
    traci.vehicle.setParkingAreaStop(
        vehID=new_veh_ID, stopID=new_shelterID, duration=100000
    )

    init_driver_behavior(vehIDs=[new_veh_ID], lane_change_mode=lane_change_mode)

    # === カウンタ更新 ===
    NEW_VEHICLE_COUNT += 1
    return NEW_VEHICLE_COUNT

def get_new_shelterID_and_near_edgeID_by_vehID(current_edgeID, opposite_edgeID, agent_by_target_vehID: Agent, vehInfo_by_target_vehID: VehicleInfo, shelter_list: list):
    from_edgeID = ""; new_shelterID = ""; to_edgeID = ""
    shelter_for_vehInfo: Shelter = \
        find_shelter_by_edgeID_connect_target_shelter(
                                                        edgeID=agent_by_target_vehID.get_near_edgeID_by_target_shelter(),
                                                        shelter_list=shelter_list
                                                        )

    # 現在地から避難地までの距離・時間計算
    remaining_distance = distance_each_vehIDs(
                                                one_veh_pos=shelter_for_vehInfo.get_position(),
                                                other_veh_pos=traci.vehicle.getPosition(agent_by_target_vehID.get_vehID())
                                                )

    veh_speed = traci.vehicle.getSpeed(agent_by_target_vehID.get_vehID()) or 1  # 0除算防止
    remaining_time = remaining_distance / veh_speed
    route_info_with_receive_time = vehInfo_by_target_vehID.get_avg_evac_time_by_route_by_recive_time()
    current_route_edgeIDs = tuple(traci.vehicle.getRoute(agent_by_target_vehID.get_vehID()))
    routes_dict = list(route_info_with_receive_time.values())[0]
    # 現在ルート以外で最も avg_time が短いルートを探す
    min_avg_time = float('inf')
    best_route = None
    for route, info in routes_dict.items():
        if route == current_route_edgeIDs :
            continue
        if info['avg_time'] < min_avg_time:
            min_avg_time = info['avg_time']
            if route[-1] != current_route_edgeIDs[-1]:
                best_route = route
    # print(f"best_route: {best_route}, current_route_edgeIDs: {current_route_edgeIDs}")
    if current_route_edgeIDs not in routes_dict:
        # print(f"current_route_edgeIDs: {current_route_edgeIDs} is not in routes_dict")
        return from_edgeID, new_shelterID, to_edgeID
    if best_route is None:
        # print(f"current_route_edgeIDs: {current_route_edgeIDs} is not in routes_dict")
        return from_edgeID, new_shelterID, to_edgeID

    # current_avg_time = routes_dict[current_route_edgeIDs]['avg_time']
    reverse_route_time =  \
        distance_each_vehIDs(one_veh_pos=traci.vehicle.getPosition(agent_by_target_vehID.get_vehID()), 
                            other_veh_pos=(100.0, 0.0)) / 8.0  # 交差点1まで戻る時間
    if remaining_time + agent_by_target_vehID.get_route_change_threshold() >  min_avg_time + reverse_route_time :
        from_edgeID = opposite_edgeID
        to_edgeID = best_route[-1]
        new_shelter: Shelter = find_shelter_by_edgeID_connect_target_shelter(edgeID=to_edgeID, shelter_list=shelter_list)
        if new_shelter is None:
            return from_edgeID, new_shelterID, to_edgeID
        new_shelterID = new_shelter.get_shelterID()
        return from_edgeID, new_shelterID, to_edgeID
    return from_edgeID, new_shelterID, to_edgeID

def get_new_shelterID_and_near_edgeID_by_vehID_based_on_distance(current_edgeID, opposite_edgeID, agent_by_target_vehID: Agent, connected_edges_list: list):
    new_shelterID = ""; new_edgeID_near_shelter = ""; from_edgeID = ""
    min_distance = 10000000

    # 2つの候補edgeから探す（現在地と逆方向）
    for edgeID in [current_edgeID, opposite_edgeID]:
        for shelterID, near_edgeID in agent_by_target_vehID.get_candidate_shelter().items():
            # 現在のターゲット避難所とは異なり、かつ逆方向にはない候補だけ
            if shelterID != agent_by_target_vehID.get_target_shelter() and is_near_shelterID_on_opposite_edges(edgeID, near_edgeID):
                try:
                    if is_route_exist(edgeID, near_edgeID, connected_edges_list):
                        distance = calculate_distance_between_edgeIDs(edgeID, near_edgeID)
                        if distance < min_distance:
                            min_distance = distance
                            new_shelterID = shelterID
                            new_edgeID_near_shelter = near_edgeID
                            from_edgeID = edgeID
                except Exception as e:
                    print(f"[経路探索エラー] {edgeID} → {near_edgeID}, 理由: {e}")
            
        # print(f"避難地が同じです: {shelterID} == {agent_by_target_vehID.get_target_shelter()}")
                # traci.vehicle.setColor(agent_by_target_vehID.get_vehID(), (255, 0, 0, 255))

    # junction上だった場合、接続されるedgeからも再探索する
    if current_edgeID.startswith(":J"):
        junctionID = current_edgeID.split(':')[-1].split('_')[0]
        next_edgeIDs = traci.junction.getOutgoingEdges(junctionID) + traci.junction.getIncomingEdges(junctionID)

        for next_edgeID in next_edgeIDs:
            if not next_edgeID.startswith(":J"):
                for shelterID, near_edgeID in agent_by_target_vehID.get_candidate_shelter().items():
                    if shelterID != agent_by_target_vehID.get_target_shelter() and not is_near_shelterID_on_opposite_edges(next_edgeID, near_edgeID):
                        try:
                            if is_route_exist(next_edgeID, near_edgeID, connected_edges_list):
                                distance = calculate_distance_between_edgeIDs(next_edgeID, near_edgeID)

                                if distance < min_distance:
                                    min_distance = distance
                                    new_shelterID = shelterID
                                    new_edgeID_near_shelter = near_edgeID
                                    from_edgeID = next_edgeID
                        except Exception as e:
                            print(f"[Junction経由の経路探索エラー] {next_edgeID} → {near_edgeID}, 理由: {e}")

    return from_edgeID, new_shelterID, new_edgeID_near_shelter

def generate_initial_vehIDs_for_row_xml(start_edge:str, end_edge:str, via_edges:str, \
                        depart_time:double, interval:double, veh_count:int, \
                        vehs_written_list:list, shelter:Shelter):
    for num in range(veh_count):
        veh_line:str = \
        f'<trip id="nt_{shelter.get_shelterID()}_{num}" depart="{depart_time}" from="{start_edge}" to="{end_edge}" via="{via_edges}"> <stop parkingArea="{shelter.get_shelterID()}" duration="{VEHICLE_SHELTER_DURATION_TIME}"/></trip>\n'
        tmp_tuple:tuple = (depart_time, veh_line)
        vehs_written_list.append(tmp_tuple)
        veh_count += 1; depart_time += interval
    return vehs_written_list

def generate_init_vehID(from_edgeID:str, to_edgeID:str, shelterID:str, VEHICLE_FOR_SHELTER:int, \
                        start_time:double, generate_interval:double, generate_route_count:int, \
                        generate_veh_count:int):
    '''
    Parameters:
        from_edgeID: 出発エッジID to_edgeID: 到着エッジID shelterID: 避難地ID VEHICLE_FOR_SHELTER: 避難地に向かう車両数
        start_time: 避難地に向かう車両における先頭車両の出発時間
        generate_interval: 車両の生成間隔 generate_route_count: 生成したルートの数 generate_veh_count: 生成した車両の数
    Returns:
    generate_veh_count: 生成した車両の数 generate_route_count: 生成したルートの数 vehID_list: 生成した車両IDのリスト
    '''
    vehID_list = []
    for vehicle_count in range(VEHICLE_FOR_SHELTER):
        # 新しい車両IDを生成と出発時間を設定
        new_veh_ID:str = "{}_{}_{}".format("init", shelterID, generate_veh_count)
        vehID_list.append(new_veh_ID)
        deparet_time:double = start_time + generate_interval*vehicle_count
        # 経由地点を取得する
        via_edgeIDs_with_intial_end_edge:list = list(traci.simulation.findRoute(from_edgeID, to_edgeID).edges)
        # ルートを設定する
        new_route_ID:str = "{}_{}_{}".format("initroute", shelterID, generate_route_count)
        traci.route.add(routeID=new_route_ID, edges=via_edgeIDs_with_intial_end_edge)
        #　新しい車両を生成
        traci.vehicle.add(vehID=new_veh_ID, routeID=new_route_ID, depart=deparet_time)
        # 新規の避難地を設定
        # print(f"新しい車両を生成しました: {new_veh_ID} {shelterID} {to_edgeID}")
        traci.vehicle.setParkingAreaStop(vehID=new_veh_ID, stopID=shelterID, duration=100000)
        generate_veh_count += 1;generate_route_count += 1
    return generate_veh_count, generate_route_count, vehID_list

def generate_simple_init_vehID(from_edgeID:str, to_edgeID:str, shelterID:str, \
                        generate_interval:double, generate_route_count:int, \
                        generate_veh_count:int, depart_time:double):
    '''
    Parameters:
        from_edgeID: 出発エッジID to_edgeID: 到着エッジID shelterID: 避難地ID VEHICLE_FOR_SHELTER: 避難地に向かう車両数
        start_time: 避難地に向かう車両における先頭車両の出発時間
        generate_interval: 車両の生成間隔 generate_route_count: 生成したルートの数 generate_veh_count: 生成した車両の数
    Returns:
    generate_veh_count: 生成した車両の数 generate_route_count: 生成したルートの数 vehID_list: 生成した車両IDのリスト
    '''
    vehID_list = []
    # 新しい車両IDを生成と出発時間を設定
    new_veh_ID:str = "{}_{}_{}".format("init", shelterID, generate_veh_count)
    vehID_list.append(new_veh_ID)
    # 経由地点を取得する
    via_edgeIDs_with_intial_end_edge:list = list(traci.simulation.findRoute(from_edgeID, to_edgeID).edges)
    # ルートを設定する
    new_route_ID:str = "{}_{}_{}".format("initroute", shelterID, generate_route_count)
    traci.route.add(routeID=new_route_ID, edges=via_edgeIDs_with_intial_end_edge)
    #　新しい車両を生成
    traci.vehicle.add(vehID=new_veh_ID, routeID=new_route_ID, depart=depart_time)
    depart_time = depart_time + generate_interval
    # 新規の避難地を設定
    traci.vehicle.setParkingAreaStop(vehID=new_veh_ID, stopID=shelterID, duration=100000)
    generate_veh_count += 1;generate_route_count += 1
    return generate_veh_count, generate_route_count, vehID_list, depart_time

def create_arrival_time_list(vehInfo_list:list,):
    arrival_time_list = []
    for vehInfo in vehInfo_list:
        arrival_time_list.append(vehInfo.get_arrival_time())
    return sorted(arrival_time_list)

def write_initial_vehIDs_for_row_xml(file_path:str, veh_written_list:list):
    # depart_timeをキーにしてソートし、リストに変換
    sorted_by_key = sorted(veh_written_list, key=lambda x: x[0])
    veh_written_list = [value for key, value in sorted_by_key]
    print("書き込みを開始します")
    with open(file_path, "r") as file:
                current_content = file.readlines()
    for park_written in veh_written_list:
        try:
            with open(file_path, "r") as file:
                current_content = file.readlines()
            lines_num = len(current_content)
            current_content.insert(lines_num - 1, park_written)
            with open(file_path, "w") as file:
                file.writelines(current_content)
        except Exception as e:
            print(f"ファイルの書き込み中にエラーが発生しました: {e}")

def remove_junction_from_edgeID(edgeIDs:list):
    return [edgeID for edgeID in edgeIDs if not edgeID.startswith(":")]

def clean_vehIDs_for_row_xml(file_path:str, START_WRITTEN_LINE:int, END_WRITTEN_LINE:int):
    # ファイルを読み込んで、必要な行だけ保持
    with open(file_path, 'r') as file:
        lines = file.readlines()
    # 範囲外の行を保持
    lines_to_keep = \
        [line for i, line in enumerate(lines, start=1) if not (START_WRITTEN_LINE <= i <= END_WRITTEN_LINE)]
    # ファイルを上書き
    with open(file_path, 'w') as file:
        file.writelines(lines_to_keep)

def init_custom_edge() -> List["CustomeEdge"]:
    custome_edge_list: List[CustomeEdge] = []
    # 全てのedgeIDを取得する
    edgeIDs: list[str] = traci.edge.getIDList()
    # edgeIDをもとにCustomEdgeを生成
    for edgeID in edgeIDs:
        current_edgeIDs = [custom_edge.get_current_edgeID() for custom_edge in custome_edge_list]
        if edgeID not in current_edgeIDs:
            custom_edge: CustomeEdge = CustomeEdge(edgeID)
            custom_edge.setting_init_opposite_edgeID(edgeIDs=edgeIDs)
            custom_edge.setting_init_start_end_junctions()
            custome_edge_list.append(custom_edge)
    return custome_edge_list

def init_shelter(shelterID:str, shelter_capacity_by_ID:dict, near_edgeID:str, shelter_list:list) -> list:
    shelter:Shelter = Shelter(shelterID=shelterID, capacity=shelter_capacity_by_ID[shelterID], near_edgeID=near_edgeID)
    shelter_list.append(shelter)
    return shelter_list

def init_vehicleInfo_list(vehIDs: list, shelter_list: list):
    vehInfo_list = []
    for vehID in vehIDs:
        part_vehID = vehID.split("_")[1] + "_" + vehID.split("_")[2]
        target_shelter = next((shelter for shelter in shelter_list if shelter.get_shelterID() == part_vehID), None)
        if target_shelter is None:
            continue  # 対応する避難所がない場合はスキップ
        vehicleInfo = VehicleInfo(
                            vehID=vehID,
                            target_shelter=target_shelter.get_shelterID(),
                            edgeID_connect_target_shelter=target_shelter.get_near_edgeID(),
                            create_time=traci.simulation.getTime()
                            )
        [vehicleInfo.init_set_congestion_level_by_shelter(shelter.get_shelterID(), 0, traci.simulation.getTime()) for shelter in shelter_list]
        [vehicleInfo.init_set_avg_evac_time_by_route_by_recive_time()]
        [vehicleInfo.init_set_tsunami_precursor_info()]
        vehInfo_list.append(vehicleInfo)
    return vehInfo_list

def init_agent_list(vehIDs:list, 
                    edgeID_by_shelterID:dict, 
                    EARLY_AGENT_THRESHOLD_LIST:list, 
                    LATE_AGENT_THRESHOLD_LIST:list, 
                    ATTR_RATE:double,
                    MOTIVATION_THRESHOLD_START:float,
                    MOTIVATION_THRESHOLD_END:float,
                    MIN_MOTIVATION_START:float,
                    MIN_MOTIVATION_END:float,
                    ACTIVE_ROUTE_CHANGE_MEAN:float,
                    ACTIVE_ROUTE_CHANGE_VAR:float,
                    CAUTIOUS_ROUTE_CHANGE_MEAN:float,
                    CAUTIOUS_ROUTE_CHANGE_VAR:float,
                    POSITIVE_LANECHANGE_START:float,
                    POSITIVE_LANECHANGE_END:float,
                    NEGATIVE_LANECHANGE_START:float,
                    NEGATIVE_LANECHANGE_END:float,
                    POSITIVE_MAJORITY_BIAS:float, 
                    NEGATIVE_MAJORITY_BIAS:float):
    agent_list = []
    for vehID in vehIDs:
        # せっかちな人はこっち
        if random_true(ATTR_RATE):
            agent:Agent = Agent(
                            vehID=vehID, 
                            target_shelter=vehID.split("_")[1] + "_" + vehID.split("_")[2], 
                            tunning_threshold=random.randint(EARLY_AGENT_THRESHOLD_LIST[0], EARLY_AGENT_THRESHOLD_LIST[1]), 
                            route_change_threshold=max(0, random.gauss(mu=ACTIVE_ROUTE_CHANGE_MEAN, sigma=ACTIVE_ROUTE_CHANGE_VAR)),
                            lane_change_init_threshold=random.uniform(MOTIVATION_THRESHOLD_START, MOTIVATION_THRESHOLD_END),
                            normalcy_motivation_increase=random.uniform(POSITIVE_LANECHANGE_START, POSITIVE_LANECHANGE_END),
                            motivation_decrease_due_to_inactive_neighbors=NEGATIVE_MAJORITY_BIAS,
                            motivation_increase_due_to_following_neighbors=POSITIVE_MAJORITY_BIAS,
                            lane_minimum_motivation_value=random.uniform(MIN_MOTIVATION_START, MIN_MOTIVATION_END)
                            )
        else:
            agent:Agent = Agent(
                            vehID=vehID, 
                            target_shelter=vehID.split("_")[1] + "_" + vehID.split("_")[2], 
                            tunning_threshold=random.randint(LATE_AGENT_THRESHOLD_LIST[0], LATE_AGENT_THRESHOLD_LIST[1]), 
                            route_change_threshold=random.gauss(mu=CAUTIOUS_ROUTE_CHANGE_MEAN, sigma=CAUTIOUS_ROUTE_CHANGE_VAR),
                            lane_change_init_threshold=random.uniform(MOTIVATION_THRESHOLD_START, MOTIVATION_THRESHOLD_END),
                            normalcy_motivation_increase=random.uniform(NEGATIVE_LANECHANGE_START, NEGATIVE_LANECHANGE_END),
                            motivation_decrease_due_to_inactive_neighbors=NEGATIVE_MAJORITY_BIAS,
                            motivation_increase_due_to_following_neighbors=POSITIVE_MAJORITY_BIAS,
                            lane_minimum_motivation_value=random.uniform(MIN_MOTIVATION_START, MIN_MOTIVATION_END)
                            )

        agent.set_near_edgeID_by_target_shelter(edgeID_by_shelterID[vehID.split("_")[1] + "_" + vehID.split("_")[2]])
        agent.init_set_candidate_near_shelter(shelter_edge_by_IDs=edgeID_by_shelterID)
        agent.set_candidate_edge_by_shelterID(edgeID_by_shelterID)
        # ここは必須
        x_values = np.arange(0, 450, 1)
        y_values = [float(two_stage_sigmoid(x)) for x in x_values]
        agent.set_x_elapsed_time_for_lane_change_list(x_values.tolist())
        agent.set_y_motivation_value_for_lane_change_list(y_values)
        # ここまで必須
        lane_change_xy_dict = dict(zip(x_values.tolist(), y_values))
        agent.set_lane_change_xy_dict(lane_change_xy_dict)
        agent_list.append(agent)
    return agent_list

def init_driver_behavior(vehIDs:list, lane_change_mode:int):
    for vehID in vehIDs:
        traci.vehicle.setLaneChangeMode(vehID, lane_change_mode)

def init_connected_edges_list(custome_edge_list:list):
    connected_edges_list = []
    edgeIDs = [custome_edge.get_current_edgeID() for custome_edge in custome_edge_list]
    for one_edge in edgeIDs:
        for other_edge in edgeIDs:
            # 同一edgeはスキップ
            if not one_edge == other_edge and is_near_shelterID_on_opposite_edges(one_edge, other_edge) and not one_edge.startswith(":J") and not other_edge.startswith(":J"):
                via_edges = list(traci.simulation.findRoute(one_edge, other_edge).edges)
                if(len(via_edges) > 0):
                    connected_edges_list.append((one_edge, other_edge, via_edges))
    return connected_edges_list  

def find_agent_by_vehID(vehID:str, agent_list:list):
    for agent in agent_list:
        if agent.get_vehID() == vehID:
            return agent

def find_customedge_by_edgeID(edgeID:str, custome_edge_list: List[CustomeEdge]):
    for custome_edge in custome_edge_list:
        if custome_edge.get_current_edgeID() == edgeID:
            return custome_edge

def find_shelter_by_edgeID_connect_target_shelter(edgeID:str, shelter_list:list):
    for shelter in shelter_list:
        if shelter.get_near_edgeID() == edgeID:
            return shelter

# ここは確認する
def find_better_route(current_route_edgeIDs, route_info_with_receive_time:dict, agent:Agent):
    another_routes_dict = list(route_info_with_receive_time.values())[0]
    if not current_route_edgeIDs in another_routes_dict:
        return None

    current_avg_time = another_routes_dict[current_route_edgeIDs]['avg_time']
    better_route = None

    for route, data in another_routes_dict.items():
        if route == current_route_edgeIDs:
            continue  # 現在のルートは比較対象にしない
        avg_time_for_another_route = data['avg_time']

        # 0より大きな値があるかを確認
        if avg_time_for_another_route > 0.0:
            better_route = route

    return better_route

def find_shelterID_by_edgeID_by_shelterID(edgeID:str, edgeID_by_shelterID:dict):
    for shelterID, edgeID_near_shelterID in edgeID_by_shelterID.items():
        if edgeID_near_shelterID == edgeID:
            return shelterID
    #TODO 例外処理を追加
    return None

def find_vehInfo_by_vehID(vehID:str, vehInfo_list:list):
    for vehInfo in vehInfo_list:
        if vehInfo.get_vehID() == vehID:
            return vehInfo

def find_alternative_route_better(current_edgeID: str, vehInfo:VehicleInfo, agent:Agent, shelter_list:list, custome_edge_list:List[CustomeEdge], MIDDLE_EDGE_ID_LIST:list, NEAR_EDGE_MIDDLE_EDGE_LIST:list):
    from_edgeID = ""; to_edgeID=""; shelterID = ""
    shelter_for_vehInfo: Shelter = \
        find_shelter_by_edgeID_connect_target_shelter(
                                                        edgeID=agent.get_near_edgeID_by_target_shelter(),
                                                        shelter_list=shelter_list
                                                        )

    route_info_with_receive_time = vehInfo.get_avg_evac_time_by_route_by_recive_time()
    current_route_edgeIDs = tuple(traci.vehicle.getRoute(agent.get_vehID()))
    routes_dict = list(route_info_with_receive_time.values())[0]

    # 現在地から避難地までの距離・時間計算
    remaining_distance = distance_each_vehIDs(
                                                one_veh_pos=shelter_for_vehInfo.get_position(),
                                                other_veh_pos=traci.vehicle.getPosition(agent.get_vehID())
                                                )
    if remaining_distance < 50:
        return from_edgeID, to_edgeID, shelterID
    veh_speed = traci.vehicle.getSpeed(agent.get_vehID()) or 1  # 0除算防止
    remaining_time: float = remaining_distance / veh_speed

    # 現在ルート以外で最も avg_time が短いルートを探す
    min_avg_time = float('inf')
    best_route = None
    for route, info in routes_dict.items():
        if route == current_route_edgeIDs :
            continue
        if info['avg_time'] < min_avg_time:
            min_avg_time = info['avg_time']
            if route[-1] != current_route_edgeIDs[-1]:
                best_route = route
    if current_route_edgeIDs not in routes_dict:
        return from_edgeID, to_edgeID, shelterID
    if best_route is None:
        return from_edgeID, to_edgeID, shelterID
    # 最短路が交差点1以前を含む場合
    if not '-E2' in best_route :
        if remaining_time + agent.get_route_change_threshold() > min_avg_time :
            from_edgeID = get_opposie_edgeID_by_edgeID(current_edgeID); to_edgeID = best_route[-1]
            new_shelter: Shelter = find_shelter_by_edgeID_connect_target_shelter(
                                                            edgeID=to_edgeID,
                                                            shelter_list=shelter_list
                                                            )
            shelterID = new_shelter.get_shelterID()
            return from_edgeID, to_edgeID, shelterID

    if is_has_middle_edge(current_route_edgeIDs, MIDDLE_EDGE_ID_LIST):
        nearest_junctionID = ""
        for route_edgeID in current_route_edgeIDs:
            aroud_junctions_of_edgeID = []
            aroud_junctions_of_edgeID.append(traci.edge.getFromJunction(route_edgeID)); aroud_junctions_of_edgeID.append(traci.edge.getToJunction(route_edgeID))
            for middle_edgeID in MIDDLE_EDGE_ID_LIST:
                junctions_of_middle_edgeID = []
                junctions_of_middle_edgeID.append(traci.edge.getFromJunction(middle_edgeID)); junctions_of_middle_edgeID.append(traci.edge.getToJunction(middle_edgeID))
                if any(j in aroud_junctions_of_edgeID for j in junctions_of_middle_edgeID):
                    common = set(junctions_of_middle_edgeID) & set(aroud_junctions_of_edgeID)
                    nearest_junctionID = list(common)[0]  # または全部欲しければ list(common)
            if nearest_junctionID != "":
                return from_edgeID, to_edgeID, shelterID
        nearest_junctionID_positon = traci.junction.getPosition(nearest_junctionID)
        current_positon = traci.vehicle.getPosition(agent.get_vehID())
        distance_from_current_positon_to_nearest_junc = distance_each_vehIDs(one_veh_pos=current_positon, other_veh_pos=nearest_junctionID_positon)
        time_to_nearest_junc = distance_from_current_positon_to_nearest_junc / 8.0

        time_from_start_junc_to_nearest_junct = routes_dict[current_route_edgeIDs]["avg_time"] - remaining_time - time_to_nearest_junc
        candidate_route_time:float = min_avg_time - time_from_start_junc_to_nearest_junct + time_to_nearest_junc
        if remaining_time + agent.get_route_change_threshold()  >  candidate_route_time:
            # print(f"remaining_time: {remaining_time}, candidate_route_time: {candidate_route_time}, agent.get_route_change_threshold(){agent.get_route_change_threshold()}")
            from_edgeID = get_opposie_edgeID_by_edgeID(current_edgeID); to_edgeID = best_route[-1]
            new_shelter: Shelter = find_shelter_by_edgeID_connect_target_shelter(
                                                            edgeID=to_edgeID,
                                                            shelter_list=shelter_list
                                                            )
            shelterID = new_shelter.get_shelterID()
            return from_edgeID, to_edgeID, shelterID
    else:
        # 候補となる経路に中間経路がない場合
        reverse_route_time =  \
            distance_each_vehIDs(
                                    one_veh_pos=traci.vehicle.getPosition(agent.get_vehID()), 
                                    other_veh_pos=(100.0, 0.0)
                                    ) / 8.0  # 交差点1まで戻る時間
        candidate_route_time: float = min_avg_time + reverse_route_time
        if remaining_time + agent.get_route_change_threshold() > candidate_route_time :
            # print(f"remaining_time: {remaining_time}, candidate_route_time: {candidate_route_time}, agent.get_route_change_threshold(){agent.get_route_change_threshold()}")
            from_edgeID = get_opposie_edgeID_by_edgeID(current_edgeID); to_edgeID = best_route[-1]
            new_shelter: Shelter = find_shelter_by_edgeID_connect_target_shelter(
                                                            edgeID=to_edgeID,
                                                            shelter_list=shelter_list
                                                            )
            shelterID = new_shelter.get_shelterID()
            return from_edgeID, to_edgeID, shelterID
    return from_edgeID, to_edgeID, shelterID

def is_has_middle_edge(current_route_edgeIDs, middle_edge_id_list):
    return any(edge in current_route_edgeIDs for edge in middle_edge_id_list)

def is_near_shelterID_on_opposite_edges(current_ID, near_edgeID):
    # -を除いたIDが同じ場合は False（反対車線だが同じ場所なので対象外）
    if current_ID.lstrip("-") == near_edgeID.lstrip("-"):
        return False
    
    # currentとnearの片方が負、片方が正なら True（反対側）
    if current_ID.startswith("-") and not near_edgeID.startswith("-"):
        return True
    elif not current_ID.startswith("-") and near_edgeID.startswith("-"):
        return True
    else:
        return False

def is_pre_edgeID_near_shelter(current_edgeID:str, edgeID_near_shelter:str, custome_edge_list:list):
    edge_near_shelter:CustomeEdge = find_customedge_by_edgeID(edgeID_near_shelter, custome_edge_list)
    try:
        pre_edge_tuple:tuple = edge_near_shelter.obtain_neighbour_outgo_edgeIDs_with_junc_by_end_junc()
    except Exception as e:
        pre_edge_tuple = ()
    for pre_edge in pre_edge_tuple:
        if current_edgeID == pre_edge:
            return True
        
def is_route_exist(current_edgeID: str, near_edgeID: str, connected_edges_list: list):
    for connected_edges in connected_edges_list:
        if connected_edges.get_start_edgeID() == current_edgeID and connected_edges.get_end_edgeID() == near_edgeID:
            return True
    # ループを全部回っても見つからなかった場合
    return False

def is_candidate_shelter_full(agent:Agent, vehInfo:VehicleInfo):
    for shelterID, near_edgeID in agent.get_candidate_shelter().items():
        # 目的の避難地以外の避難地が混雑している場合
        if not agent.get_target_shelter() == shelterID:
            if vehInfo.get_congestion_level_by_shelter(shelterID) > 0.99:
                # traci.vehicle.setColor(agent.get_vehID(), (255, 255, 80, 255))
                # print(f'避難地{shelterID}の混雑率は、{vehInfo.get_congestion_level_by_shelter(shelterID)}')
                return True
    return False

def is_candidate_shelter(agent_by_target_vehID:Agent, target_vehInfo:VehicleInfo):
    for shelterID, nearedgeID in agent_by_target_vehID.get_candidate_edge_by_shelterID().items():
        # 目的の避難地以外で空きのある避難地を探す
        if not shelterID == target_vehInfo.get_target_shelter():
            if target_vehInfo.get_congestion_level_by_shelter(shelterID)  > 0.99:
                return True
    return False

def is_vehIDs_changed_evaciation(target_vehID:str, vehInfo_list:list):
    # 反対車線にいる車両のうち、避難地を変更した車両がいるかを判定
    target_vehInfo: VehicleInfo = find_vehInfo_by_vehID(target_vehID, vehInfo_list)
    if target_vehInfo.get_agent_changed_flag():
        return False
    try:
        current_edge_ID = traci.vehicle.getRoadID(target_vehID)
    except traci.TraCIException:
        print(f'Error in run: {e}')
        return False
    try:
        # 交差点Junction は除く
        if not current_edge_ID.startswith(':J'):
            if current_edge_ID.startswith("-"):
                opposite_edgeID = current_edge_ID.lstrip('-')
            else:
                opposite_edgeID = "-" + current_edge_ID

            # ここで存在チェック
            if opposite_edgeID not in traci.edge.getIDList():
                return False

            opposite_vehIDs = traci.edge.getLastStepVehicleIDs(opposite_edgeID)
            if len(opposite_vehIDs) > 0:
                for opposite_vehID in opposite_vehIDs:
                    if opposite_vehID != target_vehID:
                        if distance_each_vehIDs(traci.vehicle.getPosition(target_vehID), traci.vehicle.getPosition(opposite_vehID)) < 10:
                            return True
            return False
    except Exception as e:
        return False

def is_vehIDs_another_lane(target_vehID:str, vehInfo_list:list, DRIVER_VISIBILITY_DISTANCE:int):
    # 反対車線にいる車両のうち、避難地を変更した車両がいるかを判定
    target_vehInfo: VehicleInfo = find_vehInfo_by_vehID(target_vehID, vehInfo_list)
    if target_vehInfo.get_agent_changed_flag():
        return False
    try:
        current_lane_ID = traci.vehicle.getLaneID(target_vehID)
    except traci.TraCIException:
        print(f'Error in run: {e}')
        return False
    parts = current_lane_ID.split('_')
    # partsが2つ（エッジIDとインデックス）であることを確認
    try:
        if len(parts) == 2:
            edge_ID = parts[0]
            lane_index = int(parts[1]) # 文字列 '0' を 整数 0 に変換
            another_lane_index = lane_index + 1
            another_lane_ID = f"{edge_ID}_{another_lane_index}"
            another_lane_vehIDs = traci.lane.getLastStepVehicleIDs(another_lane_ID)

            for another_lane_vehIDs in another_lane_vehIDs:
                if another_lane_vehIDs != target_vehID:
                    if distance_each_vehIDs(traci.vehicle.getPosition(target_vehID), traci.vehicle.getPosition(another_lane_vehIDs)) < float(INSIGHT_RANGE):
                        return True

            return False
    except Exception as e:
        return False

def is_vehIDs_changed_evaciation_with_random_true(target_vehID:str):
    # 反対車線にいる車両のうち、避難地を変更した車両がいるかを判定
    try:
        current_edge_ID = traci.vehicle.getRoadID(target_vehID)
    except traci.TraCIException:
        return False
    try:
        # 交差点Junction は除く
        if not current_edge_ID.startswith(':J'):
            if current_edge_ID.startswith("-"):
                opposite_edgeID:str = current_edge_ID.lstrip('-')
            else:
                opposite_edgeID:str = "-" + current_edge_ID
            opposite_vehIDs = traci.edge.getLastStepVehicleIDs(opposite_edgeID)
            if opposite_edgeID is not None and len(opposite_vehIDs) > 0:
                for opposite_vehID in opposite_vehIDs:
                    if distance_each_vehIDs(traci.vehicle.getPosition(target_vehID), traci.vehicle.getPosition(opposite_vehID)) < 5:
                        # print(f'避難地を変更した車両がいる: {target_vehID} {opposite_vehID}')
                        if random_true(0.1):
                            return True
            else:
                return False
        else:
            return False
    except Exception as e:
        # print(f'Error in run: {e}')
        return False

def calculate_wait_time(vehicle_count, tau=1.2, k=0.04):
    return tau * math.exp(k * vehicle_count)

def calculate_speed(wait_time, max_speed=7, v_min=0.28):
    return max(max_speed / (1 + wait_time), v_min)

def calculate_cdf(arrival_time_dict):
    """
    arrival_time_dict: {比率: [到着時間リスト]}
    例: {0.1: [3, 5, 2, 8], 0.2: [4, 1, 6, 7]}

    戻り値: {比率: (到着時間リスト, CDF値リスト)}
    """
    cdf_dict = {}

    for ratio, arrival_times in arrival_time_dict.items():
        # print(f'test rate{str(round(ratio, 1)).replace(".", "")} = {arrival_times}')
        # sorted_times = np.sort(arrival_times)  # 到着時間をソート
        n = len(arrival_times)
        cdf_values = np.arange(1, n + 1) / n  # CDF 計算（1/N, 2/N, ..., N/N）
        
        cdf_dict[ratio] = (arrival_times, cdf_values)  # 結果を辞書に格納

    return cdf_dict

def calculate_distance_between_edgeIDs(current_edgeID:str, target_edgeID:str):
    try:
        start_edge_shape = traci.lane.getShape("{}_0".format(current_edgeID))
        start_edge_center = ((start_edge_shape[0][0] + start_edge_shape[1][0])/2, (start_edge_shape[0][1] + start_edge_shape[1][1])/2)
        end_edge_shape = traci.lane.getShape("{}_0".format(target_edgeID))
        end_edge_center = ((end_edge_shape[0][0] + end_edge_shape[1][0])/2, (end_edge_shape[0][1] + end_edge_shape[1][1])/2)
        distance = sqrt((start_edge_center[0] - end_edge_center[0])**2 + (start_edge_center[1] - end_edge_center[1])**2)
        return distance
    except Exception as e:
        return 100000

def calculate_avg_evac_time_by_route(shelter_list:List[Shelter]):
    """
    shelter_list 内の各避難所について、各ルートの平均避難時間と通過車両数を計算して
    各避難所に設定します。

    Args:
    - shelter_list (list): 避難所のリスト

    Returns:
    - None
    """
    #　TODO sheleter同士で近いなら情報を共有するように変更する
    for shelter in shelter_list:
        avg_evac_time_by_route = {}  # routeごとの [合計時間, 回数, 台数] を管理する

        # 避難時間とルートごとのデータを取得
        evac_time_with_route_by_vehID = shelter.get_evacuation_time_from_junction_multidict()

        # if shelter.get_shelterID() == "ShelterA_2" and evac_time_with_route_by_vehID is not None:
        #     print(f"避難時間とルートごとのデータ: {evac_time_with_route_by_vehID}")
        for vehID, route_with_evac_time in evac_time_with_route_by_vehID.items():
            route_by_vehID = route_with_evac_time[0]
            evac_time = route_with_evac_time[1]

            if route_by_vehID in avg_evac_time_by_route:
                avg_evac_time_by_route[route_by_vehID][0] += evac_time  # 合計時間を足す
                avg_evac_time_by_route[route_by_vehID][1] += 1          # 回数を増やす
                avg_evac_time_by_route[route_by_vehID][2] += 1          # 台数を増やす
            else:
                avg_evac_time_by_route[route_by_vehID] = [evac_time, 1, 1]  # [合計時間, 回数, 台数] 初期化

        # ここで平均を計算
        final_avg_evac_time_by_route = {}
        for route, (total_time, count, vehicles) in avg_evac_time_by_route.items():
            avg_time = total_time / count  # 平均避難時間
            final_avg_evac_time_by_route[route] = {'avg_time': avg_time, 'vehicles': vehicles}

        # ルートごとの平均避難時間と台数をセット
        # if shelter.get_shelterID() == "ShelterA_2":
        #     print(f"避難時間とルートごとのデータ: {final_avg_evac_time_by_route}")
        shelter.set_avg_evac_time_by_route(final_avg_evac_time_by_route)

def export_start_end_edgeIDs_to_json(start_end_edgeIDs:dict, file_path:str):
    with open(file_path, 'w') as f:
        json.dump(start_end_edgeIDs, f, indent=4)
    print(f"JSONファイルをエクスポートしました: {file_path}")

def import_start_end_edgeIDs_from_json(file_path:str):
    with open(file_path, 'r') as f:
        start_end_edgeIDs = json.load(f)
    print(f"JSONファイルをインポートしました: {file_path}")
    return start_end_edgeIDs

def export_connected_edges_to_json(connected_edges_list:list, file_path:str):
    # 辞書形式に変換（JSONで扱いやすくする）
    connected_edges_json = [
    {
        "start": start,
        "end": end,
        "path": path
    }
    for start, end, path in connected_edges_list
    ]   
    # JSONファイルとして保存
    with open(file_path, "w", encoding="utf-8") as f:
        json.dump(connected_edges_json, f, ensure_ascii=False, indent=4)
    print("JSONファイルに出力しました")

def import_connected_edges_from_json(file_path:str):
    connected_edges_list = []
    with open(file_path, "r", encoding="utf-8") as f:
        data = json.load(f)
        for item in data:
            one_edge = item["start"]
            other_edge = item["end"]
            via_edges = item["path"]
            connected_edges = ConnectedEdges(start_edgeID=one_edge, end_edgeID=other_edge, via_edgeIDs=via_edges)
            connected_edges_list.append(connected_edges)
        return connected_edges_list

def choose_edge_by_probability(edgeID_list:list, probabilities:list) -> str:
    """
    edge_list: エッジのリスト（例: ['E26', 'E28']）
    probabilities: 各エッジに対応する確率（例: [0.7, 0.3]）
    
    戻り値: 選ばれたエッジ（文字列）
    """
    # ガード節
    if len(edgeID_list) != len(probabilities):
        raise ValueError("edge_listとprobabilitiesの長さが一致しません。")

    if not abs(sum(probabilities) - 1.0) < 1e-6:
        raise ValueError("probabilitiesの合計は1.0である必要があります。")

    return random.choices(edgeID_list, weights=probabilities, k=1)[0]


# --- 周囲密度の計算（既存ロジックを維持） ---
def get_local_density(vehID: str, radius: float = 100.0) -> float:
    """
    指定車両の周囲 radius[m] 内にいる車両数を数え、最大10台で正規化（10台で密度1.0）
    """
    ego_pos = traci.vehicle.getPosition(vehID)
    cur_edge = traci.vehicle.getRoadID(vehID)

    nearby_vehIDs = list(traci.edge.getLastStepVehicleIDs(cur_edge))
    prev_edge = get_prev_edge(edgeIDs=nearby_vehIDs, current_edgeID=cur_edge)
    next_edge = get_next_edge(edgeIDs=nearby_vehIDs, current_edgeID=cur_edge)

    if prev_edge is not None:
        nearby_vehIDs += traci.edge.getLastStepVehicleIDs(prev_edge)
    if next_edge is not None:
        nearby_vehIDs += traci.edge.getLastStepVehicleIDs(next_edge)

    nearby_vehIDs = list(set(nearby_vehIDs))
    count = 0

    for other_vehID in nearby_vehIDs:
        if other_vehID == vehID:
            continue
        other_pos = traci.vehicle.getPosition(other_vehID)
        dx = ego_pos[0] - other_pos[0]
        dy = ego_pos[1] - other_pos[1]
        distance = (dx * dx + dy * dy) ** 0.5
        if distance <= radius:
            count += 1

    return min(count / 10.0, 1.0)


# --- 密度 → 速度係数 (0.4〜1.0) ---
def density_to_coeff(local_density: float) -> float:
    """
    局所密度に応じて速度係数を返す（0.4〜1.0）。
    密度0.3まではほぼ影響なし、それ以降は滑らかに低下。
    """
    k = 12.0          # 勾配
    inflection = 0.35 # 折れ点（約3台）
    slow_factor = 1 / (1 + math.exp(k * (local_density - inflection)))  # 0〜1
    coeff = 0.4 + 0.6 * slow_factor
    return max(0.4, min(1.0, coeff))


# --- ギャップ → 基本速度 ---
def speed_from_gap(gap: float, v_free: float, v_min: float, gap_min: float, tau: float) -> float:
    """
    gap<=gap_min では v_min、gapが大きいほど (gap-gap_min)/tau に比例して速度上昇。
    """
    if gap is None:
        return v_free
    if gap <= gap_min:
        return v_min
    v = (gap - gap_min) / max(tau, 1e-3)
    return max(v_min, min(v_free, v))


# --- 平滑化処理（ジャーク抑制） ---
_prev_speed_cmd = {}

def smooth_speed_command(vehID: str, v_des: float, alpha: float = 0.5) -> float:
    """
    前回速度との指数移動平均をとることで急加減速を防ぐ。
    alphaを下げるほど平滑（0.3〜0.5が推奨）。
    """
    v_prev = _prev_speed_cmd.get(vehID, v_des)
    v_cmd = (1 - alpha) * v_prev + alpha * v_des
    _prev_speed_cmd[vehID] = v_cmd
    return v_cmd


# --- メイン：ギャップ×密度の合成速度を適用 ---
def apply_gap_density_speed_control(
        vehID: str,
        local_density: float,
        v_free: float,
        v_min: float,
        gap_min: float,
        tau: float,
        alpha: float,
        slow_time: float
    ):
    """
    1. 前方車までのギャップから目標速度 v_gap を算出
    2. 局所密度から係数を掛けて最終速度 v_des を決定
    3. 平滑化して jerk を抑え、slowDown で適用
    """
    lead_info = traci.vehicle.getLeader(vehID)
    gap = lead_info[1] if lead_info is not None else None

    v_gap = speed_from_gap(gap, v_free=v_free, v_min=v_min, gap_min=gap_min, tau=tau)
    coeff = density_to_coeff(local_density)
    v_des = v_gap * coeff
    v_cmd = smooth_speed_command(vehID, v_des, alpha=alpha)

    traci.vehicle.slowDown(vehID, v_cmd, max(0.1, slow_time))

# レーン変更の意思決定箇所
def lane_change_by_vehID(vehID: str, agent: Agent, vehInfo: VehicleInfo):
    leader_info = traci.vehicle.getLeader(vehID)
    speed = traci.vehicle.getSpeed(vehID)

    # 条件：
    # 前方に車両が存在し（leader_infoがNoneでない）
    # 距離が近い（例: 20m未満）
    # 自車速度が遅い（例: 5.0 m/s 未満）
    if (leader_info is not None and leader_info[1] < 20.0) and (speed < 7.0):
        # print(f"vehID:{vehID}, spped:{traci.vehicle.getSpeed(vehID)}, leader_info:{leader_info}")
        try:
            # レーン変更（右 or 左 laneIndex は環境に合わせて調整）
            traci.vehicle.changeLane(vehID=vehID, laneIndex=1, duration=1000)
        except Exception as e:
            print(f"Error changing lane for vehicle {vehID}: {e}")
            return False  # 例外時はFalseを返す

        # レーン変更後の設定
        init_driver_behavior(vehIDs=[vehID], lane_change_mode=512)
        traci.vehicle.setParkingAreaStop(vehID=vehID, stopID="ShelterA_2", duration=100000)
        traci.vehicle.setSpeed(vehID, 9.0)
        vehInfo.set_target_shelter("ShelterA_2")

        # フラグと記録更新
        agent.set_evacuation_route_changed_flg(True)
        elapsed = traci.simulation.getTime() - agent.get_created_time()
        agent.set_lane_change_time(elapsed)

        # print(f"[LaneChange] vehID={vehID} | 距離={leader_info[1]:.1f}m | 速度={speed:.1f}m/s | 時刻={elapsed:.1f}s")

        return True  # 成功時はTrueを返す

    # 条件を満たさなかった場合（前が空いている or 速度が速い）
    return False

def merge_route_info_within_shelters(one_shelter:Shelter, another_shelter:Shelter):
    one_shelter_has_route_info = one_shelter.get_avg_evac_time_by_route()
    another_shelter_has_route_info = another_shelter.get_avg_evac_time_by_route()
    merged_info = {}

    # shelterAの情報をまずコピー
    for route, data in one_shelter_has_route_info.items():
        merged_info[route] = data.copy()

    # shelterBの情報をマージ
    for route, data in another_shelter_has_route_info.items():
        if route in merged_info:
            # 同じルートがある場合は、重み付き平均
            total_vehicles = merged_info[route]['vehicles'] + data['vehicles']
            if total_vehicles == 0:
                avg_time = 0
            else:
                avg_time = (
                    merged_info[route]['avg_time'] * merged_info[route]['vehicles'] +
                    data['avg_time'] * data['vehicles']
                ) / total_vehicles

            merged_info[route]['avg_time'] = avg_time
            merged_info[route]['vehicles'] = total_vehicles
        else:
            # 違うルートならそのまま追加
            merged_info[route] = data.copy()

    one_shelter.set_avg_evac_time_by_route(merged_info)
    another_shelter.set_avg_evac_time_by_route(merged_info)

def distance_each_vehIDs(one_veh_pos: double, other_veh_pos: double) -> double:
    distance = sqrt((one_veh_pos[0] - other_veh_pos[0])**2 + (one_veh_pos[1] - other_veh_pos[1])**2)
    return distance

def random_true(probablity: double) -> bool:
    return random.random() < probablity

def plot_cdf(cdf_dict, fulltime_by_ratio:dict):
    """
    CDF をプロットする
    """
    plt.figure(figsize=(8, 6))

    for ratio, (times, cdf_values) in cdf_dict.items():
        print(f'rate{str(round(ratio, 1)).replace(".", "")} = {times}')
        print(f'rate{str(round(ratio, 1)).replace(".", "")}fulltime = {fulltime_by_ratio[ratio]}')
        plt.plot(times, cdf_values, marker='.', linestyle='-', label=f"Ratio {ratio}")
        # plt.vlines(fulltime_by_ratio[ratio], 0.0 ,1.0,  linestyles='dotted')

    plt.xlabel("Arrival Time")
    plt.ylabel("CDF")
    plt.title("Cumulative Distribution Function (CDF) of Arrival Times")
    plt.savefig('/Users/kashiisamutakeshi/vehicle-assistant-system/sumo/its102/result_graph.pdf')
    plt.legend()
    plt.grid()
    plt.show()

def merge_arrival_vehs_of_shelter(shelter_list: List[Shelter]):
    """
    shelterIDの先頭タイプ（例: "A_1" → "A"）が一致する避難所の
    到着車両数を集計して、total_arrival_vehIDsにセットする。
    """
    for shelter in shelter_list:
        shelter.init_total_arrival_vehIDs()  # 初期化
        shelter_type = shelter.get_shelterID().split("_")[0]

        total = 0
        for other_shelter in shelter_list:
            other_type = other_shelter.get_shelterID().split("_")[0]
            if shelter_type == other_type and shelter is not other_shelter:
                total += len(shelter.get_arrival_vehID_list()) + len(other_shelter.get_arrival_vehID_list())

        shelter.set_total_arrival_vehIDs(total)

def is_vehID_in_congested_edge(vehID:str, THRESHOLD_SPEED):
    # 車両が通過するエッジが混雑しているか判定
    current_edgeID = traci.vehicle.getRoadID(vehID)
    edgeIDs_of_target_vehID = traci.route.getEdges(traci.vehicle.getRouteID(vehID))
    next_edge_of_current_edgeID = get_next_edge(edgeIDs=edgeIDs_of_target_vehID, current_edgeID=current_edgeID)
    prev_edge_of_current_edgeID = get_prev_edge(edgeIDs=edgeIDs_of_target_vehID, current_edgeID=current_edgeID)
    if next_edge_of_current_edgeID is None :
        next_edge_of_current_edgeID_num = 0
    else:
        next_edge_of_current_edgeID_num = len(traci.edge.getLastStepVehicleIDs(next_edge_of_current_edgeID))
    if prev_edge_of_current_edgeID is None:
        prev_edge_of_current_edgeID_num = 0
    else:
        prev_edge_of_current_edgeID_num = len(traci.edge.getLastStepVehicleIDs(prev_edge_of_current_edgeID))
    current_edgeID_vehs_flag = len(traci.edge.getLastStepVehicleIDs(current_edgeID)) \
                                + next_edge_of_current_edgeID_num \
                                + prev_edge_of_current_edgeID_num > 5
    try:
        # 平均車速だとあかんな平均車両数で判定する 車線上の平均速度を取得
        # average_speed = traci.edge.getLastStepMeanSpeed(current_edgeID)
        my_speed = traci.vehicle.getSpeed(vehID)
        # 渋滞判定
        if my_speed < THRESHOLD_SPEED:
            if current_edgeID_vehs_flag :
                return True
        else:
            return False
    except Exception as e:
        print(f"Error in is_lane_congested: {e}")
        return False # エラー時は渋滞していないと見なす

def v2shelter_communication(target_vehID:str, shelterID:str, vehInfo_list:list, shelter_list:list, COMMUNICATION_RANGE:double):
    # 車両が向かう避難所の混雑情報を取得する
    target_vehInfo:VehicleInfo = find_vehInfo_by_vehID(target_vehID, vehInfo_list)
    shelter_for_target_vehID:Shelter = \
        find_shelter_by_edgeID_connect_target_shelter(
                                                        target_vehInfo.get_edgeID_connect_target_shelter(), 
                                                        shelter_list
                                                        )

    if distance_each_vehIDs(traci.vehicle.getPosition(target_vehID), shelter_for_target_vehID.get_position()) < COMMUNICATION_RANGE:
        current_time = traci.simulation.getTime()
        # 避難地の混雑度を更新する
        current_congestion_rate_by_shelterID = shelter_for_target_vehID.get_congestion_rate()
        target_vehInfo.update_shelter_congestion_info(
                                                        shelterID=shelterID, 
                                                        congestion=current_congestion_rate_by_shelterID, 
                                                        time_stamp=current_time
                                                        )
        # print(f"after {target_vehID} {shelterID} {target_vehInfo.get_congestion_level_by_shelter(shelterID)}")

        # 避難経路の情報を更新
        avg_evac_time_by_route = shelter_for_target_vehID.get_avg_evac_time_by_route()
        target_vehInfo.v2shelter_update_avg_evac_time_by_route(avg_evac_time_by_route)
        target_vehInfo.v2v_avg_evac_time_by_route_by_recive_time(current_time=current_time)

def v2v_communication(target_vehID:str, target_vehInfo:VehicleInfo, around_vehIDs:list, agent_list:list, vehInfo_list:list, COMMUNICATION_RANGE:double):
    target_agent:Agent = find_agent_by_vehID(target_vehID, agent_list)
    for around_vehID in around_vehIDs:
        # 通信可能範囲内にある車両との距離を計算
        if distance_each_vehIDs(traci.vehicle.getPosition(target_vehID), traci.vehicle.getPosition(around_vehID)) < COMMUNICATION_RANGE:
            around_vehInfo:VehicleInfo = find_vehInfo_by_vehID(around_vehID, vehInfo_list)
            # 候補となるshelterを全て交換する
            # 混雑情報の交換
            for candidate_shelter, near_edgeID in target_agent.get_candidate_edge_by_shelterID().items():
                # お互いが持つ最新の混雑情報を交換する
                target_of_congestion_info_time = target_vehInfo.get_latest_time_stamp_of_shelter(candidate_shelter)
                around_of_congestion_info_time = around_vehInfo.get_latest_time_stamp_of_shelter(candidate_shelter)
                # 時間が異なる場合、最新の情報を更新
                if target_of_congestion_info_time !=  around_of_congestion_info_time:
                    info_source = target_vehInfo if target_of_congestion_info_time > around_of_congestion_info_time else around_vehInfo
                    info_target = around_vehInfo if target_of_congestion_info_time > around_of_congestion_info_time else target_vehInfo
                    info_target.set_shelter_congestion_info(info_source.get_shelter_congestion_info())
            # 避難経路の情報交換
            target_of_route_info_tuple, = target_vehInfo.get_avg_evac_time_by_route_by_recive_time().items(); target_of_route_info_time = target_of_route_info_tuple[0]
            around_of_route_info_tuple, = around_vehInfo.get_avg_evac_time_by_route_by_recive_time().items(); around_of_route_info_time = around_of_route_info_tuple[0]
            if (target_of_route_info_tuple[1] or around_of_route_info_tuple[1] ) and target_of_route_info_time != around_of_route_info_time:
                info_source = target_vehInfo if target_of_route_info_time > around_of_route_info_time else around_vehInfo
                info_target = around_vehInfo if target_of_route_info_time > around_of_route_info_time else target_vehInfo
                info_target.set_avg_evac_time_by_route_by_recive_time(info_source.get_avg_evac_time_by_route_by_recive_time())

def v2v_communication_about_tsunami_info(target_vehID:str, target_vehInfo:VehicleInfo, around_vehIDs:list, vehInfo_list:list, COMMUNICATION_RANGE:float):
    for around_vehID in around_vehIDs:
        # 通信可能範囲内にある車両との距離を計算
        if distance_each_vehIDs(traci.vehicle.getPosition(target_vehID), traci.vehicle.getPosition(around_vehID)) < COMMUNICATION_RANGE and target_vehID != around_vehID:
            around_vehInfo:VehicleInfo = find_vehInfo_by_vehID(around_vehID, vehInfo_list)

            target_of_tsunami_info_tuple = target_vehInfo.get_tsunami_precursor_info()
            target_of_tsunami_info_time_with_flag = list(target_of_tsunami_info_tuple.values())[0]
            around_of_tsunami_info_tuple = around_vehInfo.get_tsunami_precursor_info()
            around_of_tsunami_info_time_with_flag = list(around_of_tsunami_info_tuple.values())[0]
            target_flag = target_of_tsunami_info_time_with_flag[0]
            around_flag = around_of_tsunami_info_time_with_flag[0]

            # 片方の車両だけが有効な情報を持つ場合（情報の同期）
            if target_flag and not around_flag:
                around_vehInfo.set_tsunami_precursor_info(target_vehInfo.get_tsunami_precursor_info())
            elif not target_flag and around_flag:
                target_vehInfo.set_tsunami_precursor_info(around_vehInfo.get_tsunami_precursor_info())

            # 両方の車両が有効な情報を持つ場合（一番古い情報に統一）
            elif target_flag and around_flag:
                target_time = target_of_tsunami_info_time_with_flag[1]
                around_time = around_of_tsunami_info_time_with_flag[1]
                
                # target の持つ情報の方が古い場合
                if target_time < around_time:
                    # target の情報を around にコピーして、古い情報に統一する
                    around_vehInfo.set_tsunami_precursor_info(target_vehInfo.get_tsunami_precursor_info())
                
                # around の持つ情報の方が古い場合
                elif around_time < target_time:
                    # around の情報を target にコピーして、古い情報に統一する
                    target_vehInfo.set_tsunami_precursor_info(around_vehInfo.get_tsunami_precursor_info())

def convert_to_cdf(data:dict, plot=True):
    """
    辞書形式のデータをCDF形式に変換し、オプションでグラフを表示する。

    Parameters:
        data (dict): {key: list of time values} の形式の辞書
        plot (bool): TrueにするとCDFをグラフ表示

    Returns:
        dict: 各keyに対して [(time, cdf_value), ...] のリストを持つ辞書
    """
    cdf_result = {}

    for key, times in data.items():
        sorted_times = sorted(times)
        n = len(sorted_times)
        cdf = [i / n for i in range(1, n + 1)]
        cdf_result[key] = list(zip(sorted_times, cdf))

        if plot:
            plt.plot(sorted_times, cdf, label=f"Key = {key}")

    if plot:
        plt.title("CDF for Each Key")
        plt.xlabel("Time")
        plt.ylabel("Cumulative Probability")
        plt.legend()
        plt.grid(True)
        plt.tight_layout()
        plt.show()

    return cdf_result

def plot_dot(agent: Agent):
    """
    xリストとyリストを渡すと、散布図を描画し、
    agent.get_lane_change_threshold_list() の値を水平線として描画する。

    Parameters
    ----------
    x_list : list or array
        x軸の値
    y_list : list or array
        y軸の値
    agent : object
        get_lane_change_threshold_list() メソッドを持つオブジェクト
    """
    fig, ax = plt.subplots(figsize=(6, 5))
    # --- 散布図 ---
    ax.plot(
            agent.get_x_elapsed_time_for_lane_change_list(),
            agent.get_y_motivation_value_for_lane_change_list(),
            linewidth=2,
            label="Motivation over time"
            )  
    print(f"agent.get_lane_change_time(){agent.get_lane_change_time()}, agent.get_calculated_motivation_value(){agent.get_calculated_motivation_value()}")
    # ax.scatter(agent.get_lane_change_time(), agent.get_calculated_motivation_value(), color='orange', s=30, label="Data points")
    y_thr = agent.get_lane_change_decision_threshold()
    x_reach_min_motivation = agent.get_reach_lane_minimum_motivation_time()
    ax.axhline(y=y_thr, color="red", linestyle="--", linewidth=1.5, label=f"Threshold = {y_thr:.2f}")
    ax.axvline(x=x_reach_min_motivation, color="red", linestyle="--", linewidth=1.5, label=f"reach min time = {y_thr:.2f}")
    try:
        y_cur = agent.get_calculated_motivation_value()
        ax.axhline(y=y_cur, color="green", linestyle="--", linewidth=1.5, label=f"Current Value = {y_cur:.2f}")
    except:
        pass

    # --- グラフ設定 ---
    ax.legend(loc="best")
    ax.set_xlabel("X")
    ax.set_ylabel("Y")
    ax.grid(True, linestyle="--", alpha=0.5)
    plt.tight_layout()
    plt.show()

#TODO 消去かしょ
def update_agent_lane_change_motivation(agent:Agent):
    if len(agent.get_x_lane_change()) == 0:
        created_time = traci.simulation.getTime()
        agent.set_created_time(created_time)
    current_simulation_elapsed_time = traci.simulation.getTime() - agent.get_created_time()
    next_lane_change_motivation = two_stage_sigmoid(value=current_simulation_elapsed_time)
    agent.append_x_lane_change(current_simulation_elapsed_time)
    agent.append_y_lane_change(next_lane_change_motivation)

def two_stage_sigmoid(value: float):
    """
    二段階上昇型シグモイド（ストレス上昇モデル向け）
    - 第1段階: x=150 を中心にゆるやかに上昇（予兆・軽い緊張）
    - 第2段階: x=350 を中心に急激に上昇（危険察知による急上昇）
    """
    import numpy as np

    k1 = 0.015   # 緩やかな傾き
    x01 = 200    # 軽い上昇の中心
    
    k2 = 0.045   # 急な立ち上がり
    x02 = 330    # 急上昇の中心
    
    ymin = 0
    ymax = 1000

    s1 = 1 / (1 + np.exp(-k1 * (value - x01)))
    s2 = 1 / (1 + np.exp(-k2 * (value - x02)))
    
    y_norm = 0.5 * (s1 +  s2)  # 緩やか + 急上昇を合成
    return ymin + (ymax - ymin) * y_norm

