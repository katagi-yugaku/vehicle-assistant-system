# =========================
# Routing distance helpers
# =========================
#
# 注意:
# このモジュールは TraCI に依存する。
# 経路距離・迂回距離・edge 間距離など、
# SUMO ネットワーク上の距離計算を扱う。

from __future__ import annotations

from math import sqrt
from typing import TYPE_CHECKING

from evacsim.sim.sumo_env import ensure_sumo_tools_on_path

ensure_sumo_tools_on_path()

import traci

if TYPE_CHECKING:
    from evacsim.agents.Shelter import Shelter


def _get_shelter_group(shelter_id: str) -> str:
    """
    避難所IDからグループ名を抽出する。

    例:
        "ShelterB_2" -> "ShelterB"

    既存 utilities.py の挙動を維持する。
    """
    try:
        return shelter_id.split("_")[0]
    except IndexError:
        return shelter_id


def _get_full_edge_length(edge_ID: str) -> float:
    """
    1 edge の完全な長さを取得する。

    交差点内の internal edge などで TraCIException が出た場合は 0.0 を返す。
    """
    try:
        num_lanes = traci.edge.getLaneNumber(edge_ID)

        if num_lanes > 0:
            first_lane_ID = f"{edge_ID}_0"
            return traci.lane.getLength(first_lane_ID)

    except traci.TraCIException:
        pass

    return 0.0


def _sum_route_distance_from_second_edge(
    route_edges: list,
    shelter: Shelter,
    initial_distance: float = 0.0,
) -> float:
    """
    findRoute で得た経路リストについて、
    2番目以降の edge 長を合計する。

    最初の edge のコストは initial_distance として計算済みとみなす。
    避難所 edge は半分の長さとして計算する。

    既存 utilities.py の挙動を維持する。
    """
    total_distance = initial_distance
    shelter_edge_ID = shelter.get_near_edgeID()

    for edge_ID in route_edges[1:]:
        if edge_ID == shelter_edge_ID:
            total_distance += _get_full_edge_length(edge_ID) / 2
        else:
            total_distance += _get_full_edge_length(edge_ID)

    return total_distance


def calculate_remaining_route_distance(
    vehID: str,
    to_edge: str,
    shelter: Shelter,
) -> float:
    """
    現在位置から to_edge までの残り走行距離を計算する。

    既存 utilities.py の calculate_remaining_route_distance の挙動を維持する。
    """
    try:
        current_edgeID = traci.vehicle.getRoadID(vehID)

        if current_edgeID.startswith(":"):
            return 0.0

        stage = traci.simulation.findRoute(current_edgeID, to_edge)
        route_edges = stage.edges

        if not route_edges:
            print(
                f"Warning:2 No route found from {current_edgeID} "
                f"to {to_edge} for vehicle {vehID}."
            )
            return 0.0

        # 1. 最初の edge の残り距離を計算
        try:
            current_laneID = traci.vehicle.getLaneID(vehID)
            current_lane_len = traci.lane.getLength(current_laneID)
            edge_position = traci.vehicle.getLanePosition(vehID)
            initial_distance = current_lane_len - edge_position

        except traci.TraCIException as e:
            print(
                f"TraCI Error calculating remaining length for "
                f"{vehID} on {current_edgeID}: {e}"
            )
            initial_distance = _get_full_edge_length(route_edges[0])

        # 2. 2番目以降の edge 長を加算
        return _sum_route_distance_from_second_edge(
            route_edges,
            shelter,
            initial_distance,
        )

    except traci.TraCIException as e:
        print(f"TraCI Error calculating remaining distance for {vehID}: {e}")
        return float("inf")


def get_opposite_edgeID_by_edgeID(edgeID: str):
    """
    edgeID に対応する反対方向 edgeID を返す。

    NOTE:
    本来は evacsim/maps/edge_utils.py に置くべき関数だが、
    calculate_reroute_distance が依存しているため、
    現段階では routing_distance.py に一時的に同居させる。
    後続ステップで edge_utils.py に切り出す。
    """
    if edgeID.startswith("-"):
        opposite_edgeID: str = edgeID.lstrip("-")
    else:
        opposite_edgeID: str = "-" + edgeID

    if opposite_edgeID in traci.edge.getIDList():
        return opposite_edgeID
    else:
        return edgeID


def calculate_reroute_distance(
    vehID: str,
    from_edgeID: str,
    to_edgeID: str,
    shelter: Shelter,
    approach_edgeIDs_by_start_edgeID: dict,
) -> float:
    """
    車両が Uターンして新しい避難所 to_edgeID に向かう場合の
    総走行距離を計算する。

    approach_edgeIDs_by_start_edgeID は既存関数の引数として残す。
    現状の既存実装では、この関数内では直接使用していない。
    """
    try:
        total_distance = 0.0

        # 1. Uターンコスト
        try:
            edge_position = traci.vehicle.getLanePosition(vehID)
            total_distance += edge_position

        except traci.TraCIException as e:
            print(f"TraCI Error calculating U-turn cost for {vehID}: {e}")
            return float("inf")

        # 2. Uターン後の経路を検索
        stage = traci.simulation.findRoute(from_edgeID, to_edgeID)
        route_edges = stage.edges

        if not route_edges:
            from_edgeID = get_opposite_edgeID_by_edgeID(from_edgeID)
            stage = traci.simulation.findRoute(from_edgeID, to_edgeID)
            route_edges = stage.edges

            if not route_edges:
                return float("inf")

        # 3. 2番目以降の edge 長を加算
        shelter_edge_ID = shelter.get_near_edgeID()

        for edge_ID in route_edges[1:]:
            if edge_ID == shelter_edge_ID or edge_ID == to_edgeID:
                total_distance += _get_full_edge_length(edge_ID) / 2
            else:
                total_distance += _get_full_edge_length(edge_ID)

        return total_distance

    except traci.TraCIException as e:
        print(f"TraCI Error in calculate_reroute_distance for {vehID}: {e}")
        return float("inf")


def calculate_distance_between_edgeIDs(
    current_edgeID: str,
    target_edgeID: str,
):
    """
    2つの edgeID の中心点間距離を計算する。

    例外時は既存 utilities.py と同じく 100000 を返す。
    """
    try:
        start_edge_shape = traci.lane.getShape(f"{current_edgeID}_0")
        start_edge_center = (
            (start_edge_shape[0][0] + start_edge_shape[1][0]) / 2,
            (start_edge_shape[0][1] + start_edge_shape[1][1]) / 2,
        )

        end_edge_shape = traci.lane.getShape(f"{target_edgeID}_0")
        end_edge_center = (
            (end_edge_shape[0][0] + end_edge_shape[1][0]) / 2,
            (end_edge_shape[0][1] + end_edge_shape[1][1]) / 2,
        )

        distance = sqrt(
            (start_edge_center[0] - end_edge_center[0]) ** 2
            + (start_edge_center[1] - end_edge_center[1]) ** 2
        )

        return distance

    except Exception:
        return 100000


def distance_each_vehIDs(
    one_veh_pos,
    other_veh_pos,
) -> float:
    """
    2点間のユークリッド距離を返す。

    既存 utilities.py の distance_each_vehIDs の挙動を維持する。
    """
    distance = sqrt(
        (one_veh_pos[0] - other_veh_pos[0]) ** 2
        + (one_veh_pos[1] - other_veh_pos[1]) ** 2
    )

    return distance
