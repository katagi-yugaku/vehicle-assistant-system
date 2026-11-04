# =========================
# Vehicle abandonment helpers
# =========================
#
# 注意:
# このモジュールは TraCI に依存する。
# 車両乗り捨て，歩行者生成，放棄車両による閉塞判定を扱う。
#
# NOTE:
# 既存コード互換のため、関数名中の abandant などの typo は維持する。

from __future__ import annotations

from math import sqrt
from typing import TYPE_CHECKING

from evacsim.sim.sumo_env import ensure_sumo_tools_on_path

ensure_sumo_tools_on_path()

import traci

from evacsim.sim.traci_cache import (
    get_vehicle_position_cached,
    get_person_position_cached,
    get_vehicle_road_id_cached,
    get_vehicle_lane_index_cached,
    get_vehicle_lane_position_cached,
    get_edge_vehicle_ids_cached,
    get_vehicle_leader_cached,
)
from evacsim.utils.route_utils import get_remaining_edges
from evacsim.utils.lookup import find_agent_by_vehID
from evacsim.sim.routing_distance import calculate_remaining_route_distance

if TYPE_CHECKING:
    from evacsim.agents.Agent import Agent
    from evacsim.agents.VehicleInfo import VehicleInfo
    from evacsim.agents.Shelter import Shelter


def count_near_abandoned_vehicle_in_right_lane(
    vehID: str,
    agent_list: list,
    pedestrianID_list: list,
    veh_position=None,
    step_cache=None,
) -> int:
    """
    対象車両の近傍にいる pedestrian 数を数える。

    既存 utilities.py の count_near_abandoned_vehicle_in_right_lane の挙動を維持する。
    agent_list は既存シグネチャ互換のため残す。
    """
    count = 0

    if veh_position is None:
        veh_position = get_vehicle_position_cached(
            vehID,
            step_cache=step_cache,
        )

    veh_x, veh_y = veh_position

    for pedestrianID in pedestrianID_list:
        ped_pos = get_person_position_cached(
            pedestrianID,
            step_cache=step_cache,
        )

        if ped_pos is None:
            continue

        ped_x, ped_y = ped_pos

        distance = sqrt(
            (ped_x - veh_x) ** 2
            + (ped_y - veh_y) ** 2
        )

        if distance < 30.0:
            count += 1

    return count


def is_vehicle_abandant_this_position(
    current_vehID: str,
    current_edgeID: str,
    agent_by_current_vehID: Agent,
    vehInfo_by_target_vehID: VehicleInfo,
    PEDESTRIAN_COUNT: int,
    STOPPING_TIME_IN_SHELTER: int,
    shelter: Shelter,
):
    """
    現在位置で車両を停止・乗り捨て可能かを判定し、
    可能なら traci.vehicle.setStop を実行する。

    既存 utilities.py の is_vehicle_abandant_this_position の挙動を維持する。
    未使用引数も既存シグネチャ互換のため残す。
    """
    edge_position = traci.vehicle.getLanePosition(current_vehID)
    current_lane_index = traci.vehicle.getLaneIndex(current_vehID)

    print(f"current_lane_index: {current_lane_index}")

    current_lane_length = traci.lane.getLength(
        f"{current_edgeID}_{current_lane_index}"
    )

    stop_pos = edge_position + 5.0

    # 念のため downstream 条件を保証
    if stop_pos >= current_lane_length - 2.0:
        print(
            f"停止位置がレーンの終端に近すぎるため、車両 {current_vehID} "
            f"は放棄されませんでした。 "
            f"stop_pos: {stop_pos}, lane_length: {current_lane_length}"
        )
        return False

    if stop_pos <= edge_position + 2.0:
        print(
            f"停止位置が車両の直近すぎるため、車両 {current_vehID} "
            f"は放棄されませんでした。 "
            f"stop_pos: {stop_pos}, edge_position: {edge_position}"
        )
        return False

    try:
        traci.vehicle.setStop(
            vehID=current_vehID,
            edgeID=current_edgeID,
            pos=stop_pos,
            laneIndex=current_lane_index,
            duration=STOPPING_TIME_IN_SHELTER,
        )
        return True

    except traci.exceptions.TraCIException as e:
        print(f"[WARN] setStop failed for {current_vehID}: {e}")
        return False


def vehicle_abandant_behavior(
    current_vehID: str,
    current_edgeID: str,
    agent_by_current_vehID: Agent,
    vehInfo_by_target_vehID: VehicleInfo,
    PEDESTRIAN_COUNT: int,
    STOPPING_TIME_IN_SHELTER: int,
    shelter: Shelter,
):
    """
    車両乗り捨て時の挙動を実行する。

    処理:
      - 現在位置に pedestrian を生成
      - 現在 edge 以降の route を walking stage に設定
      - 元車両を灰色に変更
      - 残り歩行距離を計算
      - Agent に vehicle_abandoned_flg を立てる

    既存 utilities.py の vehicle_abandant_behavior の挙動を維持する。
    """
    edge_position = traci.vehicle.getLanePosition(current_vehID)

    try:
        person_id = f"ped_{current_vehID}_{PEDESTRIAN_COUNT}"

        traci.person.add(
            personID=person_id,
            edgeID=current_edgeID,
            pos=edge_position,
            depart=traci.simulation.getTime(),
            typeID="DEFAULT_PEDTYPE",
        )

        route_edges_for_pedestrian: list = get_remaining_edges(
            route_edges=traci.vehicle.getRoute(current_vehID),
            current_edgeID=current_edgeID,
        )

        traci.person.appendWalkingStage(
            personID=person_id,
            edges=route_edges_for_pedestrian,
            arrivalPos=100.0,
        )

        traci.vehicle.setColor(
            current_vehID,
            (128, 128, 128, 255),
        )

        walking_distance = calculate_remaining_route_distance(
            vehID=current_vehID,
            to_edge=vehInfo_by_target_vehID.get_edgeID_connect_target_shelter(),
            shelter=shelter,
        )

        PEDESTRIAN_COUNT += 1
        agent_by_current_vehID.set_vehicle_abandoned_flg(True)

        return PEDESTRIAN_COUNT, person_id, walking_distance

    except traci.exceptions.TraCIException as e:
        print(f"[WARN] setStop failed for {current_vehID}: {e}")
        return PEDESTRIAN_COUNT, None, None


def vehicle_abandant_behavior_with_vehicle_remove(
    current_vehID: str,
    current_edgeID: str,
    agent_by_current_vehID: Agent,
    vehInfo_by_target_vehID: VehicleInfo,
    PEDESTRIAN_COUNT: int,
    STOPPING_TIME_IN_SHELTER: int,
    shelter: Shelter,
):
    """
    車両乗り捨て時の歩行者生成処理。

    NOTE:
    関数名は with_vehicle_remove だが、
    既存 utilities.py の実装では traci.vehicle.remove は実行していない。
    挙動変更を避けるため、そのまま維持する。
    """
    edge_position = traci.vehicle.getLanePosition(current_vehID)

    person_id = f"ped_{current_vehID}_{PEDESTRIAN_COUNT}"

    traci.person.add(
        personID=person_id,
        edgeID=current_edgeID,
        pos=edge_position,
        depart=traci.simulation.getTime(),
        typeID="DEFAULT_PEDTYPE",
    )

    route_edges_for_pedestrian: list = get_remaining_edges(
        route_edges=traci.vehicle.getRoute(current_vehID),
        current_edgeID=current_edgeID,
    )

    traci.person.appendWalkingStage(
        personID=person_id,
        edges=route_edges_for_pedestrian,
        arrivalPos=100.0,
    )

    walking_distance = calculate_remaining_route_distance(
        vehID=current_vehID,
        to_edge=vehInfo_by_target_vehID.get_edgeID_connect_target_shelter(),
        shelter=shelter,
    )

    PEDESTRIAN_COUNT += 1
    agent_by_current_vehID.set_vehicle_abandoned_flg(True)

    return PEDESTRIAN_COUNT, person_id, walking_distance


def has_abandoned_vehicle_within_front_n(
    vehID: str,
    agent_list: list,
    front_n: int = 5,
    agent_by_vehID_dict: dict = None,
    step_cache=None,
) -> bool:
    """
    同一 edge・同一 lane の前方 front_n 台以内に
    乗り捨て済み車両が存在するかを判定する。

    さらに leader 車両が乗り捨て済みの場合も True とする。

    既存 utilities.py の has_abandoned_vehicle_within_front_n の挙動を維持する。
    """
    current_edgeID = get_vehicle_road_id_cached(
        vehID,
        step_cache=step_cache,
    )
    current_lane_index = get_vehicle_lane_index_cached(
        vehID,
        step_cache=step_cache,
    )
    current_pos = get_vehicle_lane_position_cached(
        vehID,
        step_cache=step_cache,
    )

    front_vehicles = []

    current_edge_vehicle_ids = get_edge_vehicle_ids_cached(
        current_edgeID,
        step_cache=step_cache,
    )

    for other_vehID in current_edge_vehicle_ids:
        if other_vehID == vehID:
            continue

        if (
            get_vehicle_lane_index_cached(
                other_vehID,
                step_cache=step_cache,
            )
            != current_lane_index
        ):
            continue

        other_pos = get_vehicle_lane_position_cached(
            other_vehID,
            step_cache=step_cache,
        )

        if other_pos > current_pos:
            front_vehicles.append((other_vehID, other_pos))

    front_vehicles.sort(key=lambda x: x[1])

    for other_vehID, _ in front_vehicles[:front_n]:
        other_agent = find_agent_by_vehID(
            other_vehID,
            agent_list=agent_list,
            agent_by_vehID_dict=agent_by_vehID_dict,
        )

        if other_agent is not None and other_agent.get_vehicle_abandoned_flg():
            return True

    leader_veh = get_vehicle_leader_cached(
        vehID,
        step_cache=step_cache,
    )

    leader_agent = None

    if leader_veh is not None:
        leader_agent = find_agent_by_vehID(
            leader_veh[0],
            agent_list=agent_list,
            agent_by_vehID_dict=agent_by_vehID_dict,
        )

    if leader_agent.get_vehicle_abandoned_flg() if leader_agent is not None else False:
        return True

    return False


def is_vehID_blocked_by_abandoned_vehicle(
    vehID: str,
    agent_list: list,
    agent_by_vehID_dict: dict = None,
    step_cache=None,
) -> bool:
    """
    対象車両が前方の乗り捨て車両により閉塞されているかを判定する。

    既存 utilities.py と同じく、
    has_abandoned_vehicle_within_front_n(front_n=5) の結果を返す。
    """
    has_abandoned_ahead = has_abandoned_vehicle_within_front_n(
        vehID=vehID,
        agent_list=agent_list,
        front_n=5,
        agent_by_vehID_dict=agent_by_vehID_dict,
        step_cache=step_cache,
    )

    return has_abandoned_ahead
