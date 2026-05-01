# =========================
# Congestion / traffic condition helpers
# =========================
#
# 注意:
# このモジュールは TraCI に依存する。
# 道路混雑，周辺密度，反対車線上の車両検知，右車線退避可否などを扱う。

from __future__ import annotations

from typing import TYPE_CHECKING

from evacsim.sim.sumo_env import ensure_sumo_tools_on_path

ensure_sumo_tools_on_path()

import traci

from evacsim.sim.traci_cache import (
    get_vehicle_road_id_cached,
    get_vehicle_route_edges_cached,
    get_edge_vehicle_ids_cached,
    get_lane_length_cached,
    get_vehicle_speed_cached,
    get_vehicle_lane_position_cached,
    get_vehicle_lane_index_cached,
    get_vehicle_leader_cached,
)
from evacsim.utils.route_utils import get_next_edge, get_prev_edge
from evacsim.utils.lookup import find_vehInfo_by_vehID
from evacsim.sim.routing_distance import distance_each_vehIDs
from evacsim.maps.edge_utils import get_opposite_edgeID_by_edgeID
from evacsim.utils.random_utils import random_true

if TYPE_CHECKING:
    from evacsim.agents.Agent import Agent
    from evacsim.agents.VehicleInfo import VehicleInfo


def is_candidate_shelter_full(
    agent: Agent,
    vehInfo: VehicleInfo,
):
    """
    候補避難地のうち，現在の目的地以外が満杯に近いかを判定する。

    既存 utilities.py の is_candidate_shelter_full の挙動を維持する。
    """
    for shelterID, near_edgeID in agent.get_candidate_shelter().items():
        # 目的の避難地以外の避難地が混雑している場合
        if not agent.get_target_shelter() == shelterID:
            if vehInfo.get_congestion_level_by_shelter(shelterID) > 0.99:
                # traci.vehicle.setColor(agent.get_vehID(), (255, 255, 80, 255))
                print(
                    f'避難地{shelterID}の混雑率は、'
                    f'{vehInfo.get_congestion_level_by_shelter(shelterID)}'
                )
                return True

    return False


def is_vehIDs_changed_evaciation(
    target_vehID: str,
    vehInfo_list: list,
):
    """
    反対車線にいる車両のうち，避難地を変更した車両がいるかを判定する。

    NOTE:
    関数名 evaciation の typo は既存コード互換のため維持する。
    """
    target_vehInfo: VehicleInfo = find_vehInfo_by_vehID(
        target_vehID,
        vehInfo_list,
    )

    if target_vehInfo.get_agent_changed_flag():
        return False

    try:
        current_edge_ID = traci.vehicle.getRoadID(target_vehID)
    except traci.TraCIException as e:
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
                        if (
                            distance_each_vehIDs(
                                traci.vehicle.getPosition(target_vehID),
                                traci.vehicle.getPosition(opposite_vehID),
                            )
                            < 30
                        ):
                            return True

            return False

    except Exception:
        return False


def is_vehIDs_another_lane(
    target_vehID: str,
    vehInfo_list: list,
    DRIVER_VISIBILITY_DISTANCE: int,
):
    """
    反対車線を走行中で，かつ避難地を変更済みの車両が
    視界内 DRIVER_VISIBILITY_DISTANCE にいるかを判定する。
    """
    # 1. 自分の車両情報を取得
    target_vehInfo: VehicleInfo = find_vehInfo_by_vehID(
        target_vehID,
        vehInfo_list,
    )

    if not target_vehInfo:
        return False

    # 2. 自分が既に変更済みなら、この判定は不要
    if target_vehInfo.get_agent_changed_flag():
        return False

    try:
        # 3. 自分の現在地と、反対車線 edge を取得
        target_pos = traci.vehicle.getPosition(target_vehID)
        current_edge_ID = traci.vehicle.getRoadID(target_vehID)
        opposite_edge_ID = get_opposite_edgeID_by_edgeID(current_edge_ID)

        if not opposite_edge_ID or opposite_edge_ID == current_edge_ID:
            return False

        # 4. 反対車線の車線数を取得
        num_lanes = traci.edge.getLaneNumber(opposite_edge_ID)

        if num_lanes == 0:
            return False

        # 5. laneID を edgeID_0, edgeID_1 ... の形式で構築
        all_opposite_lane_IDs = [
            f"{opposite_edge_ID}_{i}"
            for i in range(num_lanes)
        ]

    except traci.TraCIException as e:
        print(f'Error: {target_vehID} の車両情報取得に失敗: {e}')
        return False

    # 6. 反対車線の全レーンをスキャン
    for lane_id in all_opposite_lane_IDs:
        try:
            vehicles_on_opposite_lane = traci.lane.getLastStepVehicleIDs(lane_id)
        except traci.TraCIException:
            continue

        # 7. 反対車線の各車両をチェック
        for other_vehID in vehicles_on_opposite_lane:
            other_vehInfo: VehicleInfo = find_vehInfo_by_vehID(
                other_vehID,
                vehInfo_list,
            )

            if other_vehInfo is not None:
                try:
                    other_pos = traci.vehicle.getPosition(other_vehID)
                    distance = distance_each_vehIDs(target_pos, other_pos)

                    if distance < float(DRIVER_VISIBILITY_DISTANCE):
                        return True
                    else:
                        return False

                except traci.TraCIException:
                    continue

    return False


def is_vehIDs_changed_evaciation_with_random_true(
    target_vehID: str,
):
    """
    反対車線にいる車両のうち，避難地を変更した車両がいるかを確率的に判定する。

    NOTE:
    関数名 evaciation の typo は既存コード互換のため維持する。
    """
    try:
        current_edge_ID = traci.vehicle.getRoadID(target_vehID)
    except traci.TraCIException:
        return False

    try:
        # 交差点Junction は除く
        if not current_edge_ID.startswith(':J'):
            if current_edge_ID.startswith("-"):
                opposite_edgeID: str = current_edge_ID.lstrip('-')
            else:
                opposite_edgeID: str = "-" + current_edge_ID

            opposite_vehIDs = traci.edge.getLastStepVehicleIDs(opposite_edgeID)

            if opposite_edgeID is not None and len(opposite_vehIDs) > 0:
                for opposite_vehID in opposite_vehIDs:
                    if (
                        distance_each_vehIDs(
                            traci.vehicle.getPosition(target_vehID),
                            traci.vehicle.getPosition(opposite_vehID),
                        )
                        < 5
                    ):
                        if random_true(0.1):
                            return True
            else:
                return False

        else:
            return False

    except Exception:
        return False


def get_local_density(
    vehID: str,
    radius: float = 100.0,
) -> float:
    """
    指定車両の周囲 radius[m] 内にいる車両数を数え，
    最大10台で正規化する。

    10台で密度 1.0。
    """
    ego_pos = traci.vehicle.getPosition(vehID)
    cur_edge = traci.vehicle.getRoadID(vehID)

    nearby_vehIDs = list(traci.edge.getLastStepVehicleIDs(cur_edge))

    # NOTE:
    # 既存 utilities.py では nearby_vehIDs に対して get_prev_edge / get_next_edge を呼んでいる。
    # edge 列ではなく vehID 列に対する呼び出しになっているが、
    # 挙動変更を避けるため、そのまま維持する。
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


def is_vehID_in_congested_edge(
    vehID: str,
    threshold_speed: float,
    step_cache=None,
) -> bool:
    """
    車両が渋滞状態にあるかを判定する。

    判定条件:
      - 現在 edge / 前後 edge の車両密度が閾値を超える
      - 自車速度が threshold_speed 未満
      - 同一 lane 前方に一定台数以上の車両がいる
      - leader が近い

    既存 utilities.py の is_vehID_in_congested_edge の挙動を維持する。
    """
    leader_search_dist: float = 100.0
    leader_gap_threshold: float = 20.0
    density_threshold: float = 0.03
    non_congested_front_vehicle_threshold: int = 5

    try:
        current_edgeID = get_vehicle_road_id_cached(
            vehID,
            step_cache=step_cache,
        )
        route_edges = get_vehicle_route_edges_cached(
            vehID,
            step_cache=step_cache,
        )

        next_edge = get_next_edge(
            edgeIDs=route_edges,
            current_edgeID=current_edgeID,
        )
        prev_edge = get_prev_edge(
            edgeIDs=route_edges,
            current_edgeID=current_edgeID,
        )

        current_edge_vehicle_ids = get_edge_vehicle_ids_cached(
            current_edgeID,
            step_cache=step_cache,
        )
        next_edge_vehicle_ids = (
            get_edge_vehicle_ids_cached(next_edge, step_cache=step_cache)
            if next_edge
            else ()
        )
        prev_edge_vehicle_ids = (
            get_edge_vehicle_ids_cached(prev_edge, step_cache=step_cache)
            if prev_edge
            else ()
        )

        current_num = len(current_edge_vehicle_ids)
        next_num = len(next_edge_vehicle_ids)
        prev_num = len(prev_edge_vehicle_ids)

        current_length = get_lane_length_cached(
            f"{current_edgeID}_0",
            step_cache=step_cache,
        )
        next_length = (
            get_lane_length_cached(f"{next_edge}_0", step_cache=step_cache)
            if next_edge
            else 0.0
        )
        prev_length = (
            get_lane_length_cached(f"{prev_edge}_0", step_cache=step_cache)
            if prev_edge
            else 0.0
        )

        total_vehicle_count = current_num + next_num + prev_num
        total_length = current_length + next_length + prev_length

        if total_length <= 0:
            return False

        vehicle_density = total_vehicle_count / total_length
        many_vehicles = vehicle_density > density_threshold

        low_speed = (
            get_vehicle_speed_cached(vehID, step_cache=step_cache)
            < threshold_speed
        )

        current_pos = get_vehicle_lane_position_cached(
            vehID,
            step_cache=step_cache,
        )
        current_lane_index = get_vehicle_lane_index_cached(
            vehID,
            step_cache=step_cache,
        )

        front_vehicle_count = 0

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
                front_vehicle_count += 1

        if front_vehicle_count <= non_congested_front_vehicle_threshold:
            return False

        leader = get_vehicle_leader_cached(
            vehID,
            dist=leader_search_dist,
            step_cache=step_cache,
        )

        if leader is None:
            return False

        leader_id, leader_gap = leader
        close_leader = leader_gap < leader_gap_threshold

        return low_speed and many_vehicles and close_leader

    except Exception as e:
        print(f"Error in is_vehID_in_congested_edge: {e}")
        return False


def can_escape_to_right_lane(
    vehID: str,
    front_threshold: float = 15.0,
    back_threshold: float = 10.0,
) -> bool:
    """
    右車線へ退避可能かを判定する。

    - 右前方車両が近すぎる場合は False
    - 右後方車両が近すぎる場合は False
    - それ以外は True
    """
    leaders = traci.vehicle.getNeighbors(vehID, 0b011)   # 右前
    followers = traci.vehicle.getNeighbors(vehID, 0b001) # 右後

    if leaders:
        _, dist = leaders[0]

        if dist < front_threshold:
            return False

    if followers:
        _, dist = followers[0]

        if dist > -back_threshold:
            return False

    return True
