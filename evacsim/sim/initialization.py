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
    EARLY_AGENT_THRESHOLD_LIST: list,
    LATE_AGENT_THRESHOLD_LIST: list,
    ATTR_RATE: float,
    MOTIVATION_THRESHOLD_START: float,
    MOTIVATION_THRESHOLD_END: float,
    MIN_MOTIVATION_START: float,
    MIN_MOTIVATION_END: float,
    ACTIVE_ROUTE_CHANGE_MEAN: float,
    ACTIVE_ROUTE_CHANGE_VAR: float,
    CAUTIOUS_ROUTE_CHANGE_MEAN: float,
    CAUTIOUS_ROUTE_CHANGE_VAR: float,
    POSITIVE_LANECHANGE_START: float,
    POSITIVE_LANECHANGE_END: float,
    NEGATIVE_LANECHANGE_START: float,
    NEGATIVE_LANECHANGE_END: float,
    POSITIVE_MAJORITY_BIAS: float,
    NEGATIVE_MAJORITY_BIAS: float,
    ACTIVE_SHELTER_OCCUPANCY_RATE_THRESHOLD_START: float,
    ACTIVE_SHELTER_OCCUPANCY_RATE_THRESHOLD_END: float,
    CAUTIOUS_SHELTER_OCCUPANCY_RATE_THRESHOLD_START: float,
    CAUTIOUS_SHELTER_OCCUPANCY_RATE_THRESHOLD_END: float,
    ACTIVE_NORMALCY_VALUE_ABOUT_VEHICLE_ABANDONMENT_MEAN: float,
    ACTIVE_NORMALCY_VALUE_ABOUT_VEHICLE_ABANDONMENT_VAR: float,
    ACTIVE_MAJORITY_VALUE_ABOUT_VEHICLE_ABANDONMENT_MEAN: float,
    ACTIVE_MAJORITY_VALUE_ABOUT_VEHICLE_ABANDONMENT_VAR: float,
    ACTIVE_VEHICLE_ABANDONED_THRESHOLD_MEAN: float,
    ACTIVE_VEHICLE_ABANDONED_THRESHOLD_VAR: float,
    CAUTIOUS_NORMALCY_VALUE_ABOUT_VEHICLE_ABANDONMENT_MEAN: float,
    CAUTIOUS_NORMALCY_VALUE_ABOUT_VEHICLE_ABANDONMENT_VAR: float,
    CAUTIOUS_MAJORITY_VALUE_ABOUT_VEHICLE_ABANDONMENT_MEAN: float,
    CAUTIOUS_MAJORITY_VALUE_ABOUT_VEHICLE_ABANDONMENT_VAR: float,
    CAUTIOUS_VEHICLE_ABANDONED_THRESHOLD_MEAN: float,
    CAUTIOUS_VEHICLE_ABANDONED_THRESHOLD_VAR: float,
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
                tunning_threshold=random.randint(
                    EARLY_AGENT_THRESHOLD_LIST[0],
                    EARLY_AGENT_THRESHOLD_LIST[1],
                ),
                route_change_threshold=np.random.normal(
                    loc=ACTIVE_ROUTE_CHANGE_MEAN,
                    scale=ACTIVE_ROUTE_CHANGE_VAR,
                ),
                lane_change_init_threshold=random.uniform(
                    MOTIVATION_THRESHOLD_START,
                    MOTIVATION_THRESHOLD_END,
                ),
                normalcy_motivation_increase=random.uniform(
                    POSITIVE_LANECHANGE_START,
                    POSITIVE_LANECHANGE_END,
                ),
                motivation_decrease_due_to_inactive_neighbors=NEGATIVE_MAJORITY_BIAS,
                motivation_increase_due_to_following_neighbors=POSITIVE_MAJORITY_BIAS,
                lane_minimum_motivation_value=random.uniform(
                    MIN_MOTIVATION_START,
                    MIN_MOTIVATION_END,
                ),
                shelter_occupancy_rate_threshold=random.uniform(
                    ACTIVE_SHELTER_OCCUPANCY_RATE_THRESHOLD_START,
                    ACTIVE_SHELTER_OCCUPANCY_RATE_THRESHOLD_END,
                ),
                vehicle_abandoned_threshold=random.uniform(
                    ACTIVE_VEHICLE_ABANDONED_THRESHOLD_MEAN,
                    ACTIVE_VEHICLE_ABANDONED_THRESHOLD_VAR,
                ),
                normalcy_value_about_vehicle_abandonment=np.random.normal(
                    ACTIVE_NORMALCY_VALUE_ABOUT_VEHICLE_ABANDONMENT_MEAN,
                    ACTIVE_NORMALCY_VALUE_ABOUT_VEHICLE_ABANDONMENT_VAR,
                ),
                majority_value_about_vehicle_abandonment=np.random.normal(
                    ACTIVE_MAJORITY_VALUE_ABOUT_VEHICLE_ABANDONMENT_MEAN,
                    ACTIVE_MAJORITY_VALUE_ABOUT_VEHICLE_ABANDONMENT_VAR,
                ),
            )

        else:
            agent: Agent = Agent(
                vehID=vehID,
                target_shelter=target_shelter,
                tunning_threshold=random.randint(
                    LATE_AGENT_THRESHOLD_LIST[0],
                    LATE_AGENT_THRESHOLD_LIST[1],
                ),
                route_change_threshold=np.random.normal(
                    loc=CAUTIOUS_ROUTE_CHANGE_MEAN,
                    scale=CAUTIOUS_ROUTE_CHANGE_VAR,
                ),
                lane_change_init_threshold=random.uniform(
                    MOTIVATION_THRESHOLD_START,
                    MOTIVATION_THRESHOLD_END,
                ),
                normalcy_motivation_increase=random.uniform(
                    NEGATIVE_LANECHANGE_START,
                    NEGATIVE_LANECHANGE_END,
                ),
                motivation_decrease_due_to_inactive_neighbors=NEGATIVE_MAJORITY_BIAS,
                motivation_increase_due_to_following_neighbors=POSITIVE_MAJORITY_BIAS,
                lane_minimum_motivation_value=random.uniform(
                    MIN_MOTIVATION_START,
                    MIN_MOTIVATION_END,
                ),
                shelter_occupancy_rate_threshold=random.uniform(
                    CAUTIOUS_SHELTER_OCCUPANCY_RATE_THRESHOLD_START,
                    CAUTIOUS_SHELTER_OCCUPANCY_RATE_THRESHOLD_END,
                ),
                vehicle_abandoned_threshold=random.uniform(
                    CAUTIOUS_VEHICLE_ABANDONED_THRESHOLD_MEAN,
                    CAUTIOUS_VEHICLE_ABANDONED_THRESHOLD_VAR,
                ),
                normalcy_value_about_vehicle_abandonment=np.random.normal(
                    CAUTIOUS_NORMALCY_VALUE_ABOUT_VEHICLE_ABANDONMENT_MEAN,
                    CAUTIOUS_NORMALCY_VALUE_ABOUT_VEHICLE_ABANDONMENT_VAR,
                ),
                majority_value_about_vehicle_abandonment=np.random.normal(
                    CAUTIOUS_MAJORITY_VALUE_ABOUT_VEHICLE_ABANDONMENT_MEAN,
                    CAUTIOUS_MAJORITY_VALUE_ABOUT_VEHICLE_ABANDONMENT_VAR,
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
