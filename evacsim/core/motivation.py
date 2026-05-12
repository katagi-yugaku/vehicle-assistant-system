# =========================
# Motivation model helpers
# =========================
#
# 注意:
# このモジュールでは TraCI を import しない。
# 運転者の心理モデル・情報活性化・モチベーション計算だけを扱う。

from __future__ import annotations

from typing import TYPE_CHECKING

import numpy as np
import traci

if TYPE_CHECKING:
    from evacsim.agents.Agent import Agent
    from evacsim.agents.VehicleInfo import VehicleInfo

from evacsim.sim.routing_distance import distance_each_vehIDs

from evacsim.sim.traci_cache import get_vehicle_position_cached

from evacsim.utils.lookup import (
    find_shelter_by_edgeID_connect_target_shelter,
    find_shelterID_by_edgeID_by_shelterID,
    find_agent_by_vehID
)
from evacsim.sim.neighbors import (
    get_around_vehIDs,
)
from evacsim.utilities import get_person_position_cached

from evacsim.maps.edge_utils import (
    is_pre_edgeID_near_shelter,
    get_vehicle_start_edges,
    get_vehicle_end_edges,
    get_opposite_edgeID_by_edgeID,
)

def two_stage_sigmoid(value: float):
    """
    二段階上昇型シグモイド（基礎モチベーション）

    - 第1段階: 渋滞遭遇後，ゆるやかに危機感が上昇
    - 第2段階: 平均渋滞滞在時間付近で急激に上昇

    目標:
      B(368.60) ≒ 4000
      B_max = 5000
    """
    # k1 = 0.012   # 緩やかな上昇
    # x01 = 280    # 第1段階の中心時刻 [sec.]

    # k2 = 0.035   # 急激な上昇
    # x02 = 320    # 第2段階の中心時刻 [sec.]
    k1 = 0.011
    x01 = 310
    k2 = 0.032
    x02 = 350
    ymax = 5000

    ymin = 0
    ymax = 5000

    s1 = 1 / (1 + np.exp(-k1 * (value - x01)))
    s2 = 1 / (1 + np.exp(-k2 * (value - x02)))

    y_norm = 0.5 * (s1 + s2)
    return ymin + (ymax - ymin) * y_norm


def generate_motivation_curve(
    max_time: int = 900,
    step: int = 1,
) -> dict[float, float]:
    """
    経過時間を key，モチベーション値を value とする辞書を生成する。

    既存 utilities.py の挙動を維持する。
    """
    dencimals = 3
    x_values = np.arange(0.0, max_time, step)
    y_values = [
        round(float(two_stage_sigmoid(x)), dencimals)
        for x in x_values
    ]
    x_list = x_values.tolist()
    xy_dict = dict(zip(x_list, y_values))

    return xy_dict


def sigmoid(x: np.ndarray) -> np.ndarray:
    """
    シグモイド関数。

    exp のオーバーフローを避けるために clip している。
    """
    x = np.clip(x, -500, 500)
    return 1.0 / (1.0 + np.exp(-x))


def generate_info_activation_dict(
    rho: float,
    delta: float,
    max_time: int = 900,
    step: int = 1,
) -> dict[float, float]:
    """
    情報取得後の経過時間を key，活性化度 phi を value とする dict を生成する。

    Returns
    -------
    activation_dict:
        {
            情報取得後の経過時間: 活性化度 phi
        }
    """
    base = sigmoid(np.array([-rho * delta]))[0]
    denominator = 1.0 - base

    x_values = np.arange(0, max_time, step)

    activation_dict = {}
    decimals = 3

    for x in x_values:
        numerator = sigmoid(rho * (x - delta)) - base
        phi = numerator / denominator
        phi = float(np.clip(phi, 0.0, 1.0))
        phi = round(phi, decimals)
        activation_dict[float(x)] = phi

    return activation_dict


def multiply_time_dict(
    value_by_time_dict: dict[float, float],
    coefficient: float,
) -> dict[float, float]:
    """
    時刻 -> 値 の dict に対して，全 value に係数を掛けた dict を返す。
    """
    return {
        time: value * coefficient
        for time, value in value_by_time_dict.items()
    }


def set_motivation_curve_dicts_to_agents(agent_list, rho, delta):
    """
    agent_list の各 Agent に，
    base motivation と，正常性バイアス値を掛けた情報由来 motivation 曲線を設定する。

    注意:
    - generate_info_activation_dict() の返り値は 0〜1 の活性化度。
    - Agent に保存する時点で normalcy_value_about_* を掛ける。
    - そのため，保存後の dict は「活性化度」ではなく「motivation 値」として扱う。
    """

    base_motivation_value_for_elapsed_time_dict = generate_motivation_curve(
        max_time=900,
        step=1,
    )

    tsunami_activation_by_elapsed_time = generate_info_activation_dict(
        rho=rho["tsu"],
        delta=delta["tsu"],
        max_time=900,
        step=1,
    )

    jam_activation_by_elapsed_time = generate_info_activation_dict(
        rho=rho["jam"],
        delta=delta["jam"],
        max_time=900,
        step=1,
    )

    full_activation_by_elapsed_time = generate_info_activation_dict(
        rho=rho["full"],
        delta=delta["full"],
        max_time=900,
        step=1,
    )

    for agent in agent_list:
        agent.set_base_motivation_value_by_elapsed_time_dict(
            base_motivation_value_for_elapsed_time_dict
        )

        tsunami_motivation_by_elapsed_time = multiply_time_dict(
            tsunami_activation_by_elapsed_time,
            agent.get_normalcy_value_about_tsunami_precursor_info(),
        )

        jam_motivation_by_elapsed_time = multiply_time_dict(
            jam_activation_by_elapsed_time,
            agent.get_normalcy_value_about_route_congestion_info(),
        )

        full_motivation_by_elapsed_time = multiply_time_dict(
            full_activation_by_elapsed_time,
            agent.get_normalcy_value_about_shelter_full_info(),
        )

        agent.set_tsunami_precursor_normalcy_value_by_elapsed_time_dict(
            tsunami_motivation_by_elapsed_time
        )

        agent.set_route_congestion_normalcy_value_by_elapsed_time_dict(
            jam_motivation_by_elapsed_time
        )

        agent.set_shelter_full_normalcy_value_by_elapsed_time_dict(
            full_motivation_by_elapsed_time
        )


def re_calculate_motivation_value(
    info_activation_dict: dict[float, float],
    elapsed_time: float,
) -> float:
    """
    既存 utilities.py の未完成実装をそのまま残す。

    NOTE:
    現状では値を返していないため、挙動変更を避ける目的でそのままにしている。
    今後、使用箇所を確認したうえで整理する。
    """
    for time, value in info_activation_dict.items():
        time += elapsed_time

    # print(f"elapsed_time: {elapsed_time}")


def get_value_from_time_dict(
    value_by_time_dict: dict,
    elapsed_time: float,
    default: float = 0.0,
) -> float:
    """
    時刻を key とする辞書から，elapsed_time 以下の最大 key に対応する値を取得する。

    - 辞書が空の場合は default を返す
    - elapsed_time 以下の最大 key を使う
    - elapsed_time が最小 key より小さい場合は，最小 key の値を返す
    - elapsed_time が最大 key より大きい場合は，最大 key の値を返す
    - key は int / float を想定する
    """
    if not value_by_time_dict:
        return float(default)

    numeric_keys = [
        key for key in value_by_time_dict.keys()
        if isinstance(key, (int, float))
    ]

    if not numeric_keys:
        return float(default)

    elapsed_time = float(elapsed_time)

    selected_key = None

    for key in numeric_keys:
        if key <= elapsed_time:
            if selected_key is None or key > selected_key:
                selected_key = key

    if selected_key is None:
        selected_key = min(numeric_keys)

    return float(value_by_time_dict[selected_key])


def calculate_motivation_for_evacuation_action(
    agent: Agent,
    agent_list: list[Agent],
    vehInfo: VehicleInfo,
    current_time: float,
    action: str,
    agent_by_vehID_dict: dict,
    custome_edge_list: list,
    pedestrianID_list: list,
    debug: bool = False,
) -> float:
    """
    指定された運転者 agent について，現時点の避難行動モチベーションを計算して返す。

    今回は最小実装として，車両乗り捨て va のみを対象とする。
    この関数内では agent の状態更新は行わない。
    """
    base_motivation_value_by_elapsed_time: dict = (
        agent.get_base_motivation_value_by_elapsed_time_dict()
    )

    encounted_congestion_time = agent.get_encounted_congestion_time()
    base_value = 0.0
    base_value = get_value_from_time_dict(
        value_by_time_dict=base_motivation_value_by_elapsed_time,
        elapsed_time=current_time-encounted_congestion_time,
        default=0.0,
    )
    # 情報取得による増加
    info_motivation_value = 0.0
    if vehInfo.get_vehicle_comm_enabled_flag():
        info_motivation_value = calculate_motivation_for_obtained_info(agent=agent, current_time=current_time, action=action)

    majority_motivation_value = 0.0
    # 同調性バイアスによる増減
    majority_motivation_value = calculate_motivation_for_majority_tunning_bias(
        agent=agent, 
        vehInfo=vehInfo, 
        agent_list=agent_list, 
        candidate_action=action, 
        agent_by_vehID_dict=agent_by_vehID_dict, 
        custome_edge_list=custome_edge_list,
        pedestrianID_list=pedestrianID_list)

    if base_value + majority_motivation_value < 0:
        majority_motivation_value = 0.0


    motivation = base_value + info_motivation_value + majority_motivation_value

    if debug:
        print(
            "[calculate_motivation_for_evacuation_action] "
            f"vehID={agent.get_vehID()}, "
            f"action={action}, "
            f"current_time={current_time}, "
            f"base={base_value}, "
            f"motivation={motivation}"
        )

    return float(motivation)


def calculate_motivation_for_obtained_info(
    agent: Agent,
    current_time: float,
    action: str,
) -> float:
    Info_motivation_value = 0.0
    encounted_congestion_time = agent.get_encounted_congestion_time()
    # 満杯情報に対するモチベーション増加のシグモイド関数
    shelter_full_activation_value_by_elapsed_time: dict = (
        agent.get_shelter_full_normalcy_value_by_elapsed_time_dict()
    )
    # その情報の取得時間
    shelter_full_info_obtained_time = agent.get_shelter_full_info_obtained_time()

    # 津波接近情報に対するモチベーション増加のシグモイド関数
    tsunami_precursor_normalcy_value_by_elapsed_time: dict = (
        agent.get_tsunami_precursor_normalcy_value_by_elapsed_time_dict()
    )
    # その情報の取得時間
    tsunami_precursor_info_obtained_time = agent.get_tsunami_info_obtained_time()

    # 経路の混雑情報に対するモチベーション増加のシグモイド関数
    route_congestion_normalcy_value_by_elapsed_time: dict = (
        agent.get_route_congestion_normalcy_value_by_elapsed_time_dict()
    )
    # その情報の取得時間
    route_congestion_info_obtained_time = agent.get_route_congestion_info_obtained_time()

    if encounted_congestion_time > shelter_full_info_obtained_time:
        shelter_full_info_obtained_time = encounted_congestion_time
        agent.set_shelter_full_info_obtained_time(shelter_full_info_obtained_time)
    if encounted_congestion_time > tsunami_precursor_info_obtained_time:
        tsunami_precursor_info_obtained_time = encounted_congestion_time
        agent.set_tsunami_info_obtained_time(tsunami_precursor_info_obtained_time)
    if encounted_congestion_time > route_congestion_info_obtained_time:
        route_congestion_info_obtained_time = encounted_congestion_time
        agent.set_route_congestion_info_obtained_time(route_congestion_info_obtained_time)

    if action not in {"rc", "ww", "va"}:
        raise ValueError(f"Invalid action: {action}. Expected one of 'rc', 'ww', 'va'.")
    info_motivation_value = 0.0
    if action == "rc":
        if agent.get_route_congestion_info_obtained_flg():
            info_motivation_value += get_value_from_time_dict(
                route_congestion_normalcy_value_by_elapsed_time,
                current_time - route_congestion_info_obtained_time,
                default=0.0,
            )
        if agent.get_shelter_full_info_obtained_flg():
            info_motivation_value += get_value_from_time_dict(
                shelter_full_activation_value_by_elapsed_time,
                current_time - shelter_full_info_obtained_time,
                default=0.0,
            )


    if action == "ww":
        if agent.get_tsunami_info_obtained_flg():
            info_motivation_value += get_value_from_time_dict(
                    value_by_time_dict=tsunami_precursor_normalcy_value_by_elapsed_time,
                    elapsed_time=current_time - tsunami_precursor_info_obtained_time,
                    default=0.0,
                )
            
        if agent.get_route_congestion_info_obtained_flg():
            info_motivation_value += get_value_from_time_dict(
                    value_by_time_dict=route_congestion_normalcy_value_by_elapsed_time,
                    elapsed_time=current_time - route_congestion_info_obtained_time,
                    default=0.0,
                )
    
    if action == "va":
        if agent.get_shelter_full_info_obtained_flg(): 
            info_motivation_value += get_value_from_time_dict(
                value_by_time_dict=shelter_full_activation_value_by_elapsed_time,
                elapsed_time=current_time - shelter_full_info_obtained_time,
                default=0.0,
            )

        if agent.get_tsunami_info_obtained_flg():
            info_motivation_value += get_value_from_time_dict(
                        value_by_time_dict=tsunami_precursor_normalcy_value_by_elapsed_time,
                        elapsed_time=current_time - tsunami_precursor_info_obtained_time,
                        default=0.0,
                    )
    return float(info_motivation_value)


def calculate_motivation_for_majority_tunning_bias(
    agent: Agent,
    vehInfo: VehicleInfo,
    agent_list: list[Agent],
    candidate_action: str,
    agent_by_vehID_dict: dict,
    custome_edge_list: list,
    pedestrianID_list: list,
) -> float:
    same_action_count = count_same_action_drivers(
            agent=agent, vehInfo=vehInfo, 
            agent_list=agent_list, 
            candidate_action=candidate_action, 
            agent_by_vehID_dict=agent_by_vehID_dict,
            custome_edge_list=custome_edge_list,
            pedestrianID_list=pedestrianID_list,)
    if vehInfo.get_vehicle_comm_enabled_flag():
        if same_action_count == 0:
            return -agent.get_majority_value_decrease()
        return same_action_count * agent.get_majority_value_increase()
    else:
        # print(f"same_action_count {same_action_count}")
        if same_action_count > 5:
            same_action_count = 5
        return same_action_count * agent.get_majority_value_increase()


def count_same_action_drivers(
    agent: Agent,
    vehInfo: VehicleInfo,
    agent_list: list[Agent],
    candidate_action: str,
    agent_by_vehID_dict: dict,
    custome_edge_list: list,
    pedestrianID_list: list,
):
    """
    同調性バイアスの計算

    - agent: 対象の運転者エージェント
    - vehInfo: 周囲の車両情報（例: 近くの車両の避難行動割合）

    Returns
    -------
    
    """
    step_cache = None
    around_vehIDs: list = get_around_vehIDs(
                    target_vehID=agent.get_vehID(),
                    custome_edge_list=custome_edge_list,
                    step_cache=step_cache,
                )
    target_position = get_vehicle_position_cached(
            agent.get_vehID(),
            step_cache=step_cache,
        )
    same_action_count = 0
    current_edgeID = traci.vehicle.getRoadID(agent.get_vehID())
    # 車両乗り捨てをしている人が近くにいるか
    if candidate_action == "va":
        for pedestrianID in pedestrianID_list:
            ped_position = get_person_position_cached(pedestrianID, step_cache=step_cache)
            if ped_position is None:
                continue
            distance = distance_each_vehIDs(target_position, ped_position)
            if distance <= 50:
                # print(f"va same action found: {pedestrianID} at distance {distance}")
                same_action_count += 1
    # 車両乗り捨てをしている人が近くにいるか
    if candidate_action == "ww":
        # print(f"current_edgeID: {current_edgeID} current_lane_ID: {traci.vehicle.getLaneID(agent.get_vehID())}")
        current_lane_ID = traci.vehicle.getLaneID(agent.get_vehID())
        edge_id, lane_index = current_lane_ID.rsplit("_", 1)
        lane_index = int(lane_index)

        if lane_index == 2:
            target_lane_index = 1
        else:
            target_lane_index = lane_index + 1
        ww_lane_ID = f"{edge_id}_{target_lane_index}"
        ww_moviing_vehIDs = traci.lane.getLastStepVehicleIDs(ww_lane_ID)
        for ww_moving_vehID in ww_moviing_vehIDs:
            if ww_moving_vehID == agent.get_vehID():
                continue
            other_agent = find_agent_by_vehID(vehID=ww_moving_vehID, agent_list=agent_list, agent_by_vehID_dict=agent_by_vehID_dict)
            if other_agent == agent or other_agent.get_shelter_changed_flg() or other_agent.get_vehicle_abandoned_flg():
                continue
            if other_agent is None:
                continue
            if other_agent.get_agent_action_name() == "":
                continue

            other_agent_position = get_vehicle_position_cached(
                other_agent.get_vehID(),
                step_cache=step_cache,
            )
            distance = distance_each_vehIDs(target_position, other_agent_position)
            if distance <= 50:
                print(f"ww same action found: {other_agent.get_vehID()} at distance {distance}")
                if other_agent.get_agent_action_name() == candidate_action:
                    print(f"ww same action found: {other_agent.get_vehID()} at distance {distance}")
                    same_action_count += 1

    # 車両を変更している人が近くにいるか
    if candidate_action == "rc":
        opposite_edgeID = get_opposite_edgeID_by_edgeID(current_edgeID)
        opposite_moving_vehIDs = traci.edge.getLastStepVehicleIDs(opposite_edgeID)
        # if len(opposite_moving_vehIDs):
        #     print(f"rc opposite_edgeID: {opposite_edgeID}, opposite_moving_vehIDs: {opposite_moving_vehIDs}")
        for opposite_moving_vehID in opposite_moving_vehIDs:
            other_agent = find_agent_by_vehID(vehID=opposite_moving_vehID, agent_list=agent_list, agent_by_vehID_dict=agent_by_vehID_dict)
            if other_agent == agent or other_agent.get_vehicle_abandoned_flg():
                continue
            # elif other_agent.get_agent_action_name() == "":
            #     print(f"rc other_agent {other_agent.get_vehID()} has no action")
            #     continue
            elif other_agent is not None:
                other_agent_position = get_vehicle_position_cached(
                    other_agent.get_vehID(),
                    step_cache=step_cache,
                )
                distance = distance_each_vehIDs(target_position, other_agent_position)
                print(f"rc other_agent: {other_agent.get_vehID()}, distance: {distance}")
                if distance <= 50:
                    print(f"rc same action found: {other_agent.get_vehID()} at distance {distance}")
                    if other_agent.get_agent_action_name() == candidate_action:
                        print(f"rc same action found: {other_agent.get_vehID()} at distance {distance}")
                        same_action_count += 1
    return same_action_count

