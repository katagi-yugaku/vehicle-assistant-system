# =========================
# Simulation initialization helpers
# =========================
#
# 注意:
# このモジュールは TraCI に依存する。
# 車両の初期設定、VehicleInfo 初期化、Agent 初期化などを扱う。
#
# まずは低リスクな init_driver_behavior と init_vehicleInfo_list_base を切り出す。
# init_agent_list は乱数・パラメータ数が多いため、次ステップで慎重に移行する。

from __future__ import annotations

import random

import numpy as np

from typing import TYPE_CHECKING

from evacsim.sim.sumo_env import ensure_sumo_tools_on_path

ensure_sumo_tools_on_path()

import traci

from evacsim.agents.Agent import Agent
from evacsim.agents.VehicleInfo import VehicleInfo
from evacsim.utils.random_utils import random_true

if TYPE_CHECKING:
    from evacsim.agents.Shelter import Shelter


def init_driver_behavior(
    vehIDs: list,
    lane_change_mode: int,
):
    """
    車両の laneChangeMode を設定する。

    既存 utilities.py の init_driver_behavior の挙動を維持する。
    """
    for vehID in vehIDs:
        traci.vehicle.setLaneChangeMode(vehID, lane_change_mode)


def init_vehicleInfo_list_base(
    vehIDs: list,
    shelter_list: list,
    v2v_capable_vehicle_rate: float,
):
    """
    VehicleInfo の初期リストを作成する。

    処理:
      - vehID から対象 shelterID を復元する
      - 対応する Shelter を shelter_list から探す
      - VehicleInfo を作成する
      - shelter ごとの混雑情報を初期化する
      - 経路別平均避難時間情報を初期化する
      - 津波前兆情報を初期化する
      - v2v_capable_vehicle_rate に基づき通信可否フラグを設定する

    既存 utilities.py の init_vehicleInfo_list_base の挙動を維持する。
    """
    vehInfo_list = []

    for vehID in vehIDs:
        part_vehID = vehID.split("_")[1] + "_" + vehID.split("_")[2]

        target_shelter = next(
            (
                shelter
                for shelter in shelter_list
                if shelter.get_shelterID() == part_vehID
            ),
            None,
        )

        if target_shelter is None:
            continue

        vehicleInfo = VehicleInfo(
            vehID=vehID,
            target_shelter=target_shelter.get_shelterID(),
            edgeID_connect_target_shelter=target_shelter.get_near_edgeID(),
            create_time=traci.simulation.getTime(),
        )

        [
            vehicleInfo.init_set_congestion_level_by_shelter(
                shelter.get_shelterID(),
                0,
                traci.simulation.getTime(),
            )
            for shelter in shelter_list
        ]

        [vehicleInfo.init_set_avg_evac_time_by_route_by_recive_time()]
        [vehicleInfo.init_set_tsunami_precursor_info()]

        if random_true(v2v_capable_vehicle_rate):
            [vehicleInfo.set_vehicle_comm_enabled_flag(True)]
        else:
            [vehicleInfo.set_vehicle_comm_enabled_flag(False)]

        vehInfo_list.append(vehicleInfo)

    return vehInfo_list


def init_agent_list(
    vehIDs: list,
    edgeID_by_shelterID: dict,
    ATTR_RATE: float,

    # ==================================================
    # 経路変更閾値
    # 正規分布: N(mean, std)
    # ==================================================
    ACTIVE_ROUTE_CHANGE_THRESHOLD_CENTER: float,
    ACTIVE_ROUTE_CHANGE_THRESHOLD_SPREAD: float,
    CAUTIOUS_ROUTE_CHANGE_THRESHOLD_CENTER: float,
    CAUTIOUS_ROUTE_CHANGE_THRESHOLD_SPREAD: float,

    # ==================================================
    # 経路変更に対する正常性バイアス
    # 正規分布: N(mean, std)
    # ==================================================
    NORMALCY_VALUE_ABOUT_ROUTE_CHANGE_CENTER: float,
    NORMALCY_VALUE_ABOUT_ROUTE_CHANGE_SPREAD: float,

    # ==================================================
    # 逆走行為に関する閾値
    # 正規分布: N(mean, std)
    # ==================================================
    ACTIVE_WRONG_WAY_DRIVING_THRESHOLD_CENTER: float,
    ACTIVE_WRONG_WAY_DRIVING_THRESHOLD_SPREAD: float,
    CAUTIOUS_WRONG_WAY_DRIVING_THRESHOLD_CENTER: float,
    CAUTIOUS_WRONG_WAY_DRIVING_THRESHOLD_SPREAD: float,

    # ==================================================
    # 逆走行為の最小動機付け値
    # 一様分布: U(start, end)
    # ==================================================
    MIN_MOTIVATION_START: float,
    MIN_MOTIVATION_END: float,

    # ==================================================
    # 津波接近情報に対する正常性バイアス
    # 正規分布: N(mean, std)
    # ==================================================
    ACTIVE_NORMALCY_VALUE_ABOUT_TSUNAMI_PRECURSOR_INFO_CENTER: float,
    ACTIVE_NORMALCY_VALUE_ABOUT_TSUNAMI_PRECURSOR_INFO_SPREAD: float,
    CAUTIOUS_NORMALCY_VALUE_ABOUT_TSUNAMI_PRECURSOR_INFO_CENTER: float,
    CAUTIOUS_NORMALCY_VALUE_ABOUT_TSUNAMI_PRECURSOR_INFO_SPREAD: float,
    
    # ==================================================
    # 経路変更情報に対する正常性バイアス
    # 正規分布: N(mean, std)
    # ==================================================
    ACTIVE_NORMALCY_VALUE_ABOUT_ROUTE_CONGESTION_INFO_CENTER: float,
    ACTIVE_NORMALCY_VALUE_ABOUT_ROUTE_CONGESTION_INFO_SPREAD: float,
    CAUTIOUS_NORMALCY_VALUE_ABOUT_ROUTE_CONGESTION_INFO_CENTER: float,
    CAUTIOUS_NORMALCY_VALUE_ABOUT_ROUTE_CONGESTION_INFO_SPREAD: float,

    # ==================================================
    # 満杯情報に対する正常性バイアス
    # 正規分布: N(mean, std)
    # ==================================================
    ACTIVE_NORMALCY_VALUE_ABOUT_SHELTER_FULL_INFO_CENTER: float,
    ACTIVE_NORMALCY_VALUE_ABOUT_SHELTER_FULL_INFO_SPREAD: float,
    CAUTIOUS_NORMALCY_VALUE_ABOUT_SHELTER_FULL_INFO_CENTER: float,
    CAUTIOUS_NORMALCY_VALUE_ABOUT_SHELTER_FULL_INFO_SPREAD: float,

    # ==================================================
    # 周囲車両による同調性バイアス(増加)
    # ==================================================
    ACTIVE_MAJORITY_INCREASE_VALUE_CENTER: float,
    ACTIVE_MAJORITY_INCREASE_VALUE_SPREAD: float,
    CAUTIOUS_MAJORITY_INCREASE_VALUE_CENTER: float,
    CAUTIOUS_MAJORITY_INCREASE_VALUE_SPREAD: float,
    
    # ==================================================
    # 周囲車両による同調性バイアス(減少)
    # ==================================================
    ACTIVE_MAJORITY_DECREASE_VALUE_CENTER: float,
    ACTIVE_MAJORITY_DECREASE_VALUE_SPREAD: float,
    CAUTIOUS_MAJORITY_DECREASE_VALUE_CENTER: float,
    CAUTIOUS_MAJORITY_DECREASE_VALUE_SPREAD: float,

    # ==================================================
    # 避難所混雑率閾値
    # 一様分布: U(start, end)
    # ==================================================
    ACTIVE_SHELTER_OCCUPANCY_RATE_THRESHOLD_START: float,
    ACTIVE_SHELTER_OCCUPANCY_RATE_THRESHOLD_END: float,
    CAUTIOUS_SHELTER_OCCUPANCY_RATE_THRESHOLD_START: float,
    CAUTIOUS_SHELTER_OCCUPANCY_RATE_THRESHOLD_END: float,

    ):
    """
    Agent の初期リストを作成する。

    処理:
      - ATTR_RATE に基づき active / cautious タイプを分岐
      - 各種閾値・正常性バイアス・同調性バイアス値を生成
      - 目的避難地，候補避難地，避難地混雑率辞書を初期化する

    注意:
      - 既存 utilities.py の init_agent_list の挙動を維持する
      - この関数内では random.seed() を設定しない
      - seed は runner / 実験設定側で明示的に設定する
    """
    agent_list = []

    for vehID in vehIDs:
        target_shelter = vehID.split("_")[1] + "_" + vehID.split("_")[2]
        # せっかちな人はこっち
        if random_true(ATTR_RATE):
            agent: Agent = Agent(
                vehID=vehID,
                target_shelter=target_shelter,
                route_change_threshold=np.random.normal(
                    loc=ACTIVE_ROUTE_CHANGE_THRESHOLD_CENTER,
                    scale=ACTIVE_ROUTE_CHANGE_THRESHOLD_SPREAD,
                ),
                wrong_way_driving_threshold=np.random.normal(
                    ACTIVE_WRONG_WAY_DRIVING_THRESHOLD_CENTER,
                    ACTIVE_WRONG_WAY_DRIVING_THRESHOLD_SPREAD,
                ),
                wrong_way_driving_min_motivation_value=random.uniform(
                    MIN_MOTIVATION_START,
                    MIN_MOTIVATION_END,
                ),

                normalcy_value_about_tsunami_precursor_info=np.random.normal(
                    ACTIVE_NORMALCY_VALUE_ABOUT_TSUNAMI_PRECURSOR_INFO_CENTER,
                    ACTIVE_NORMALCY_VALUE_ABOUT_TSUNAMI_PRECURSOR_INFO_SPREAD,
                ),

                normalcy_value_about_route_congestion_info=np.random.normal(
                    ACTIVE_NORMALCY_VALUE_ABOUT_ROUTE_CONGESTION_INFO_CENTER,
                    ACTIVE_NORMALCY_VALUE_ABOUT_ROUTE_CONGESTION_INFO_SPREAD,
                ),

                normalcy_value_about_shelter_full_info=np.random.normal(
                    ACTIVE_NORMALCY_VALUE_ABOUT_SHELTER_FULL_INFO_CENTER,
                    ACTIVE_NORMALCY_VALUE_ABOUT_SHELTER_FULL_INFO_SPREAD,
                ),

                majority_value_increase=np.random.normal(
                    ACTIVE_MAJORITY_INCREASE_VALUE_CENTER - ACTIVE_MAJORITY_INCREASE_VALUE_SPREAD,
                    ACTIVE_MAJORITY_INCREASE_VALUE_CENTER + ACTIVE_MAJORITY_INCREASE_VALUE_SPREAD,
                ),
                majority_value_decrease=np.random.normal(
                    ACTIVE_MAJORITY_DECREASE_VALUE_CENTER - ACTIVE_MAJORITY_DECREASE_VALUE_SPREAD,
                    ACTIVE_MAJORITY_DECREASE_VALUE_CENTER + ACTIVE_MAJORITY_DECREASE_VALUE_SPREAD,
                ),
                shelter_occupancy_rate_threshold=random.uniform(
                    ACTIVE_SHELTER_OCCUPANCY_RATE_THRESHOLD_START,
                    ACTIVE_SHELTER_OCCUPANCY_RATE_THRESHOLD_END,
                ),
            )

        else:
            agent: Agent = Agent(
                vehID=vehID,
                target_shelter=target_shelter,
                route_change_threshold=np.random.normal(
                    CAUTIOUS_ROUTE_CHANGE_THRESHOLD_CENTER,
                    CAUTIOUS_ROUTE_CHANGE_THRESHOLD_SPREAD,
                ),
                wrong_way_driving_threshold=np.random.normal(
                    CAUTIOUS_WRONG_WAY_DRIVING_THRESHOLD_CENTER,
                    CAUTIOUS_WRONG_WAY_DRIVING_THRESHOLD_SPREAD,
                ),
                wrong_way_driving_min_motivation_value=random.uniform(
                    MIN_MOTIVATION_START,
                    MIN_MOTIVATION_END,
                ),
                normalcy_value_about_tsunami_precursor_info=np.random.normal(
                    CAUTIOUS_NORMALCY_VALUE_ABOUT_TSUNAMI_PRECURSOR_INFO_CENTER,
                    CAUTIOUS_NORMALCY_VALUE_ABOUT_TSUNAMI_PRECURSOR_INFO_SPREAD,
                ),
                normalcy_value_about_route_congestion_info=np.random.normal(
                    CAUTIOUS_NORMALCY_VALUE_ABOUT_ROUTE_CONGESTION_INFO_CENTER,
                    CAUTIOUS_NORMALCY_VALUE_ABOUT_ROUTE_CONGESTION_INFO_SPREAD,
                ),
                normalcy_value_about_shelter_full_info=np.random.normal(
                    CAUTIOUS_NORMALCY_VALUE_ABOUT_SHELTER_FULL_INFO_CENTER,
                    CAUTIOUS_NORMALCY_VALUE_ABOUT_SHELTER_FULL_INFO_SPREAD,
                ),
                majority_value_increase=np.random.normal(
                    CAUTIOUS_MAJORITY_INCREASE_VALUE_CENTER - CAUTIOUS_MAJORITY_INCREASE_VALUE_SPREAD,
                    CAUTIOUS_MAJORITY_INCREASE_VALUE_CENTER + CAUTIOUS_MAJORITY_INCREASE_VALUE_SPREAD,
                ),
                majority_value_decrease=np.random.normal(
                    CAUTIOUS_MAJORITY_DECREASE_VALUE_CENTER - CAUTIOUS_MAJORITY_DECREASE_VALUE_SPREAD,
                    CAUTIOUS_MAJORITY_DECREASE_VALUE_CENTER + CAUTIOUS_MAJORITY_DECREASE_VALUE_SPREAD,
                ),
                shelter_occupancy_rate_threshold=random.uniform(
                    CAUTIOUS_SHELTER_OCCUPANCY_RATE_THRESHOLD_START,
                    CAUTIOUS_SHELTER_OCCUPANCY_RATE_THRESHOLD_END,
                ),
            )

        agent.set_near_edgeID_by_target_shelter(
            edgeID_by_shelterID[target_shelter]
        )
        agent.init_set_candidate_near_shelter(
            shelter_edge_by_IDs=edgeID_by_shelterID
        )
        agent.init_set_shelter_occupancy_rate_dict()
        agent.set_candidate_edge_by_shelterID(edgeID_by_shelterID)

        agent_list.append(agent)

    return agent_list


def init_vehicleInfo_list(vehIDs: list, shelter_list: list, approach_edgeIDs_by_start_edgeID: dict, edgeIDs_within_junction_to_shelter_dict: dict, v2v_capable_vehicle_rate: float):
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
        [vehicleInfo.set_approach_edge_dict(approach_edgeIDs_by_start_edgeID)]
        [vehicleInfo.set_edgeIDs_within_junction_to_shelter_dict(edgeIDs_within_junction_to_shelter_dict)]
        vehInfo_list.append(vehicleInfo)
    # 引数のv2v機能を有する車両の割合に応じて、フラグを設定する（初期は全車両True）
    for vehInfo in vehInfo_list:
        if not random_true(v2v_capable_vehicle_rate):
            vehInfo.set_vehicle_comm_enabled_flag(False)
    return vehInfo_list
