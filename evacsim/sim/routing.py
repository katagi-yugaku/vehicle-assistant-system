# =========================
# Routing / shelter choice helpers
# =========================
#
# 注意:
# このモジュールは TraCI に依存する。
# 避難地変更先の選択，代替経路探索，経路変更判断などを扱う。
#
# まずは generate_new_veh 系で必要になる
# get_new_shelterID_and_near_edgeID 系だけを切り出す。

from __future__ import annotations

from typing import TYPE_CHECKING
from dataclasses import dataclass
from typing import Any, Callable, Optional, Literal


from evacsim.sim.sumo_env import ensure_sumo_tools_on_path

ensure_sumo_tools_on_path()

import traci
import math
from evacsim.maps.edge_utils import (
    get_opposite_edgeID_by_edgeID,
    is_has_middle_edge,
    is_near_shelterID_on_opposite_edges,
    is_route_exist,
)
from evacsim.sim.routing_distance import (
    calculate_distance_between_edgeIDs,
    calculate_remaining_route_distance,
    calculate_reroute_distance,
    distance_each_vehIDs,
)
from evacsim.utils.lookup import find_shelter_by_edgeID_connect_target_shelter

if TYPE_CHECKING:
    from evacsim.agents.Agent import Agent
    from evacsim.agents.Shelter import Shelter
    from evacsim.agents.VehicleInfo import VehicleInfo
    from evacsim.agents.CustomeEdge import CustomeEdge

FREE_FLOW_SPEED = 11.0

V2V_PREFIX_INITIAL_COMMON_PREFIX = ("E0", "E1", "E20")

V2V_ROUTE_FAMILY_MARKERS = {
    # E0_E16_0 / E0_E13_1 系（short-side corridor）
    "short_side": {
        "E43", "E25", "E26", "E7", "E42", "E41",
    },

    # E0_E13_0 / E0_E16_1 系（detour-side corridor）
    "detour_side": {
        "E22", "E21", "E23", "E29", "E8", "E30",
        "E31", "E9", "E17", "E32", "E18",
    },
}



def get_new_shelterID_and_near_edgeID_by_vehID(
    current_edgeID,
    opposite_edgeID,
    agent_by_target_vehID: Agent,
    vehInfo_by_target_vehID: VehicleInfo,
    shelter_list: list,
):
    """
    V2V で得た経路時間情報に基づき，新しい避難地と接続 edge を決定する。

    既存 utilities.py の get_new_shelterID_and_near_edgeID_by_vehID の挙動を維持する。
    """
    from_edgeID = ""
    new_shelterID = ""
    to_edgeID = ""

    shelter_for_vehInfo: Shelter = find_shelter_by_edgeID_connect_target_shelter(
        edgeID=agent_by_target_vehID.get_near_edgeID_by_target_shelter(),
        shelter_list=shelter_list,
    )

    # 現在地から避難地までの距離・時間計算
    remaining_distance = distance_each_vehIDs(
        one_veh_pos=shelter_for_vehInfo.get_position(),
        other_veh_pos=traci.vehicle.getPosition(agent_by_target_vehID.get_vehID()),
    )

    veh_speed = traci.vehicle.getSpeed(agent_by_target_vehID.get_vehID()) or 1
    remaining_time = remaining_distance / veh_speed

    route_info_with_receive_time = (
        vehInfo_by_target_vehID.get_avg_evac_time_by_route_by_recive_time()
    )

    current_route_edgeIDs = tuple(
        traci.vehicle.getRoute(agent_by_target_vehID.get_vehID())
    )

    routes_dict = list(route_info_with_receive_time.values())[0]

    # 現在ルート以外で最も avg_time が短いルートを探す
    min_avg_time = float("inf")
    best_route = None

    for route, info in routes_dict.items():
        if route == current_route_edgeIDs:
            continue

        if info["avg_time"] < min_avg_time:
            min_avg_time = info["avg_time"]

            if route[-1] != current_route_edgeIDs[-1]:
                best_route = route

    if current_route_edgeIDs not in routes_dict:
        return from_edgeID, new_shelterID, to_edgeID

    if best_route is None:
        return from_edgeID, new_shelterID, to_edgeID

    reverse_route_time = (
        distance_each_vehIDs(
            one_veh_pos=traci.vehicle.getPosition(agent_by_target_vehID.get_vehID()),
            other_veh_pos=(100.0, 0.0),
        )
        / 8.0
    )

    if (
        remaining_time + agent_by_target_vehID.get_route_change_threshold()
        > min_avg_time + reverse_route_time
    ):
        from_edgeID = opposite_edgeID
        to_edgeID = best_route[-1]

        new_shelter: Shelter = find_shelter_by_edgeID_connect_target_shelter(
            edgeID=to_edgeID,
            shelter_list=shelter_list,
        )

        if new_shelter is None:
            return from_edgeID, new_shelterID, to_edgeID

        new_shelterID = new_shelter.get_shelterID()

        return from_edgeID, new_shelterID, to_edgeID

    return from_edgeID, new_shelterID, to_edgeID


def get_new_shelterID_and_near_edgeID_by_vehID_based_on_distance(
    current_edgeID,
    opposite_edgeID,
    agent_by_target_vehID: Agent,
    connected_edges_list: list,
):
    """
    現在地から最も近い候補避難地を距離ベースで選択する。

    generate_new_veh で使われる関数。
    既存 utilities.py の get_new_shelterID_and_near_edgeID_by_vehID_based_on_distance の挙動を維持する。
    """
    new_shelterID = ""
    new_edgeID_near_shelter = ""
    from_edgeID = ""
    min_distance = 10000000

    # 2つの候補 edge から探す（現在地と逆方向）
    for edgeID in [current_edgeID, opposite_edgeID]:
        for shelterID, near_edgeID in agent_by_target_vehID.get_candidate_shelter().items():
            # 現在のターゲット避難所とは異なり、かつ逆方向にはない候補だけ
            if (
                shelterID != agent_by_target_vehID.get_target_shelter()
                and is_near_shelterID_on_opposite_edges(edgeID, near_edgeID)
            ):
                try:
                    if is_route_exist(edgeID, near_edgeID, connected_edges_list):
                        distance = calculate_distance_between_edgeIDs(
                            edgeID,
                            near_edgeID,
                        )

                        if distance < min_distance:
                            min_distance = distance
                            new_shelterID = shelterID
                            new_edgeID_near_shelter = near_edgeID
                            from_edgeID = edgeID

                except Exception as e:
                    print(f"[経路探索エラー] {edgeID} → {near_edgeID}, 理由: {e}")

    # junction上だった場合、接続される edge からも再探索する
    if current_edgeID.startswith(":J"):
        junctionID = current_edgeID.split(":")[-1].split("_")[0]

        next_edgeIDs = (
            traci.junction.getOutgoingEdges(junctionID)
            + traci.junction.getIncomingEdges(junctionID)
        )

        for next_edgeID in next_edgeIDs:
            if not next_edgeID.startswith(":J"):
                for shelterID, near_edgeID in (
                    agent_by_target_vehID.get_candidate_shelter().items()
                ):
                    if (
                        shelterID != agent_by_target_vehID.get_target_shelter()
                        and not is_near_shelterID_on_opposite_edges(
                            next_edgeID,
                            near_edgeID,
                        )
                    ):
                        try:
                            if is_route_exist(
                                next_edgeID,
                                near_edgeID,
                                connected_edges_list,
                            ):
                                distance = calculate_distance_between_edgeIDs(
                                    next_edgeID,
                                    near_edgeID,
                                )

                                if distance < min_distance:
                                    min_distance = distance
                                    new_shelterID = shelterID
                                    new_edgeID_near_shelter = near_edgeID
                                    from_edgeID = next_edgeID

                        except Exception as e:
                            print(
                                f"[Junction経由の経路探索エラー] "
                                f"{next_edgeID} → {near_edgeID}, 理由: {e}"
                            )

    return from_edgeID, new_shelterID, new_edgeID_near_shelter


def find_route_name_by_edge(edgeID, routes_dict):
    """
    edgeID をキーとして、それが含まれる最初の経路名を検索する。

    既存 utilities.py の find_route_name_by_edge の挙動を維持する。
    """
    for route_name, edge_list in routes_dict.items():
        if edgeID in edge_list:
            return route_name

    return None


def find_alternative_route_calculated_time(
    current_edgeID: str,
    vehInfo,
    agent,
    shelter_list: list,
    custome_edge_list: list,
    route_edges_by_routeID_dict: dict = None,
):
    """
    V2V で得た route 平均避難時間情報を使い、
    現在 route より十分短い代替 route があるかを判定する。

    戻り値:
        条件を満たす routeID があれば routeID
        なければ None

    NOTE:
    current_edgeID, shelter_list, custome_edge_list は
    既存シグネチャ互換のため残している。
    現在の実装では直接使用していない。
    """
    route_info_with_receive_time = (
        vehInfo.get_avg_evac_time_by_route_by_recive_time()
    )

    if route_edges_by_routeID_dict is None:
        route_iter = (
            (routeID, tuple(traci.route.getEdges(routeID)))
            for routeID in traci.route.getIDList()
        )
    else:
        route_iter = route_edges_by_routeID_dict.items()

    for routeID, edges_for_routeID in route_iter:
        current_route_time, best_alternative_time, best_alternative_route = (
            extract_current_and_best_alternative_time(
                route_info_with_receive_time,
                current_route=tuple(edges_for_routeID),
            )
        )

        if current_route_time is not None and best_alternative_time is not None:
            if (
                current_route_time - best_alternative_time
                > agent.get_route_change_threshold()
            ):
                return routeID

    return None


def extract_current_and_best_alternative_time(
    route_info_with_receive_time,
    current_route,
):
    """
    route_info_with_receive_time から、
    現在 route の平均時間と、最良代替 route の平均時間を取り出す。

    既存 utilities.py の extract_current_and_best_alternative_time の挙動を維持する。

    NOTE:
    route_info_with_receive_time が空の場合、既存コードは
    return None, None, None, None
    となっているため、そのまま維持している。
    """
    if not route_info_with_receive_time:
        return None, None, None, None

    # 受信時刻が1つだけ入っている前提
    receive_time = next(iter(route_info_with_receive_time.keys()))
    route_dict = route_info_with_receive_time[receive_time]

    current_route_time = None
    best_alternative_time = None
    best_alternative_route = None

    for route, info in route_dict.items():
        avg_time = info.get("avg_time")

        if route == current_route:
            current_route_time = avg_time
        else:
            if best_alternative_time is None or avg_time < best_alternative_time:
                best_alternative_time = avg_time
                best_alternative_route = route

    return current_route_time, best_alternative_time, best_alternative_route


def find_alternative_route_better(
    current_edgeID: str,
    vehInfo,
    agent,
    shelter_list: list,
    custome_edge_list: list,
    MIDDLE_EDGE_ID_LIST: list,
    NEAR_EDGE_MIDDLE_EDGE_LIST: list,
):
    """
    V2V で取得した経路別平均避難時間情報を用いて、
    現在経路より良い代替経路があるかを判定する。

    Returns:
        from_edgeID, to_edgeID, shelterID

    NOTE:
    - 既存 utilities.py の find_alternative_route_better の挙動を維持する。
    - custome_edge_list, NEAR_EDGE_MIDDLE_EDGE_LIST は既存シグネチャ互換のため残す。
    - 関数内の細かい条件分岐や早期 return は、挙動変更を避けるため元実装に合わせる。
    """
    from_edgeID = ""
    to_edgeID = ""
    shelterID = ""

    shelter_for_vehInfo = find_shelter_by_edgeID_connect_target_shelter(
        edgeID=agent.get_near_edgeID_by_target_shelter(),
        shelter_list=shelter_list,
    )

    route_info_with_receive_time = (
        vehInfo.get_avg_evac_time_by_route_by_recive_time()
    )

    current_route_edgeIDs = tuple(
        traci.vehicle.getRoute(agent.get_vehID())
    )

    routes_dict = list(route_info_with_receive_time.values())[0]

    # 現在地から避難地までの距離・時間計算
    remaining_distance = distance_each_vehIDs(
        one_veh_pos=shelter_for_vehInfo.get_position(),
        other_veh_pos=traci.vehicle.getPosition(agent.get_vehID()),
    )

    if remaining_distance < 50:
        return from_edgeID, to_edgeID, shelterID

    veh_speed = traci.vehicle.getSpeed(agent.get_vehID()) or 1
    remaining_time: float = remaining_distance / veh_speed

    # 現在ルート以外で最も avg_time が短いルートを探す
    min_avg_time = float("inf")
    best_route = None

    for route, info in routes_dict.items():
        if route == current_route_edgeIDs:
            continue

        if info["avg_time"] < min_avg_time:
            min_avg_time = info["avg_time"]

            if route[-1] != current_route_edgeIDs[-1]:
                best_route = route

    if current_route_edgeIDs not in routes_dict:
        return from_edgeID, to_edgeID, shelterID

    if best_route is None:
        return from_edgeID, to_edgeID, shelterID

    # 最短路が交差点1以前を含む場合
    if not "-E2" in best_route:
        if remaining_time + agent.get_route_change_threshold() > min_avg_time:
            from_edgeID = get_opposite_edgeID_by_edgeID(current_edgeID)
            to_edgeID = best_route[-1]

            new_shelter = find_shelter_by_edgeID_connect_target_shelter(
                edgeID=to_edgeID,
                shelter_list=shelter_list,
            )

            shelterID = new_shelter.get_shelterID()

            return from_edgeID, to_edgeID, shelterID

    if is_has_middle_edge(current_route_edgeIDs, MIDDLE_EDGE_ID_LIST):
        nearest_junctionID = ""

        for route_edgeID in current_route_edgeIDs:
            aroud_junctions_of_edgeID = []
            aroud_junctions_of_edgeID.append(
                traci.edge.getFromJunction(route_edgeID)
            )
            aroud_junctions_of_edgeID.append(
                traci.edge.getToJunction(route_edgeID)
            )

            for middle_edgeID in MIDDLE_EDGE_ID_LIST:
                junctions_of_middle_edgeID = []
                junctions_of_middle_edgeID.append(
                    traci.edge.getFromJunction(middle_edgeID)
                )
                junctions_of_middle_edgeID.append(
                    traci.edge.getToJunction(middle_edgeID)
                )

                if any(
                    j in aroud_junctions_of_edgeID
                    for j in junctions_of_middle_edgeID
                ):
                    common = (
                        set(junctions_of_middle_edgeID)
                        & set(aroud_junctions_of_edgeID)
                    )
                    nearest_junctionID = list(common)[0]

            if nearest_junctionID != "":
                return from_edgeID, to_edgeID, shelterID

        nearest_junctionID_positon = traci.junction.getPosition(
            nearest_junctionID
        )
        current_positon = traci.vehicle.getPosition(agent.get_vehID())

        distance_from_current_positon_to_nearest_junc = distance_each_vehIDs(
            one_veh_pos=current_positon,
            other_veh_pos=nearest_junctionID_positon,
        )

        time_to_nearest_junc = (
            distance_from_current_positon_to_nearest_junc / 8.0
        )

        time_from_start_junc_to_nearest_junct = (
            routes_dict[current_route_edgeIDs]["avg_time"]
            - remaining_time
            - time_to_nearest_junc
        )

        candidate_route_time: float = (
            min_avg_time
            - time_from_start_junc_to_nearest_junct
            + time_to_nearest_junc
        )

        if (
            remaining_time + agent.get_route_change_threshold()
            > candidate_route_time
        ):
            from_edgeID = get_opposite_edgeID_by_edgeID(current_edgeID)
            to_edgeID = best_route[-1]

            new_shelter = find_shelter_by_edgeID_connect_target_shelter(
                edgeID=to_edgeID,
                shelter_list=shelter_list,
            )

            shelterID = new_shelter.get_shelterID()

            return from_edgeID, to_edgeID, shelterID

    else:
        # 候補となる経路に中間経路がない場合
        reverse_route_time = (
            distance_each_vehIDs(
                one_veh_pos=traci.vehicle.getPosition(agent.get_vehID()),
                other_veh_pos=(100.0, 0.0),
            )
            / 8.0
        )

        candidate_route_time: float = min_avg_time + reverse_route_time

        if (
            remaining_time + agent.get_route_change_threshold()
            > candidate_route_time
        ):
            from_edgeID = get_opposite_edgeID_by_edgeID(current_edgeID)
            to_edgeID = best_route[-1]

            new_shelter = find_shelter_by_edgeID_connect_target_shelter(
                edgeID=to_edgeID,
                shelter_list=shelter_list,
            )

            shelterID = new_shelter.get_shelterID()

            return from_edgeID, to_edgeID, shelterID

    return from_edgeID, to_edgeID, shelterID


# def find_alternative_better_choice_fixed(
#     current_edgeID: str, 
#     vehInfo: VehicleInfo, 
#     agent: Agent, 
#     shelter: Shelter,
#     shelter_list: list, 
#     custome_edge_list: list,
#     debug: bool = False,
#     ):
#     """
#     現在の経路と，V2V情報および自由速度計算に基づく迂回経路を比較し，
#     より良い選択肢があれば経路変更情報を返す。

#     修正点:
#     - vehInfo.get_avg_evac_time_by_route_by_recive_time() の外側キーである receive_time は，
#       情報取得時刻であり，経路時間推定の計算には使わない。
#     - receive_time ごとの内側辞書 {route_tuple: route_info} を統合し，
#       すべてのV2V経路情報を候補として利用する。
#     - debug=True のときだけ，判断に必要な中間値を出力する。
#     """

#     def dprint(message: str):
#         if debug:
#             print(f"[find_alternative_better_choice] {message}")

#     # --- 0. 定数・初期化 ---
#     FREE_FLOW_SPEED = 11.0

#     shelterID = ""
#     shelterID_to_return = vehInfo.get_target_shelter()
#     to_edge_list_to_return = []
#     congestion_flg = False

#     dprint(
#         f"start vehID={agent.get_vehID()}, "
#         f"current_edgeID={current_edgeID}, "
#     )

#     if current_edgeID.startswith(':'):
#         dprint(f"skip: current_edgeID is internal junction edge: {current_edgeID}")
#         return "", shelterID_to_return, to_edge_list_to_return, 0.0, congestion_flg

#     # --- 1. 現在の経路での残り時間を計算 ---
#     current_target_shelterID = agent.get_target_shelter()

#     shelter_for_vehInfo: Shelter = shelter

#     current_route_tuple = tuple(traci.vehicle.getRoute(agent.get_vehID()))
#     current_destination_edge = current_route_tuple[-1]

#     distance_to_current_shelter = calculate_remaining_route_distance(
#         agent.get_vehID(),
#         current_destination_edge,
#         shelter=shelter_for_vehInfo
#     )
#     try:
#         # 念のため初期値を入れておく
#         current_speed = FREE_FLOW_SPEED

#         if vehInfo.get_vehicle_comm_enabled_flag:
#             # システム有り: 現在edgeの平均速度を使う
#             current_speed = traci.edge.getLastStepMeanSpeed(current_edgeID)
#         else:
#             # システム無し: 自由速度を使う
#             current_speed = FREE_FLOW_SPEED

#         if current_speed < 1.8:
#             dprint(
#                 f"current_speed is too low: {current_speed:.3f}. "
#                 f"clamped to 1.8"
#             )
#             current_speed = 1.8

#     except traci.TraCIException as e:
#         dprint(f"Error getting speed for {agent.get_vehID()}: {e}")
#         current_speed = 1.0

#     estimated_current_route_evacuation_time = (
#         distance_to_current_shelter / current_speed
#     )

#     dprint(
#         "current route: "
#         f"target_shelter={current_target_shelterID}, "
#         f"destination_edge={current_destination_edge}, "
#         f"distance_to_current_shelter={distance_to_current_shelter:.3f}, "
#         f"current_speed={current_speed:.3f}, "
#         f"estimated_current_time={estimated_current_route_evacuation_time:.3f}"
#     )
#     # --- 2. 迂回検討のための準備 ---
#     approach_edgeIDs_by_start_edgeID = vehInfo.get_approach_edge_dict()
#     # print(f"approach_edgeIDs_by_start_edgeID: {approach_edgeIDs_by_start_edgeID}")
#     is_in_approach_list = any(
#         current_edgeID in edge_list
#         for edge_list in approach_edgeIDs_by_start_edgeID.values()
#     )
#     approach_edge_flg = False
#     base_reroute_start_edgeID = ""

#     if not is_in_approach_list:
#         # アプローチエッジにいない場合，デフォルトは反対車線
#         base_reroute_start_edgeID = get_opposite_edgeID_by_edgeID(
#             edgeID=current_edgeID
#         )
#     else:
#         # アプローチエッジにいる場合，デフォルトは現在車線
#         base_reroute_start_edgeID = current_edgeID
#         approach_edge_flg = True

#         if not vehInfo.get_vehicle_comm_enabled_flag():
#             dprint(
#                 "system_mode=3 and vehicle is in approach edge. "
#                 "return without reroute calculation."
#             )
#             return base_reroute_start_edgeID, shelterID_to_return, [], 0.0

#     dprint(
#         f"is_in_approach_list={is_in_approach_list}, "
#         f"approach_edge_flg={approach_edge_flg}, "
#         f"base_reroute_start_edgeID={base_reroute_start_edgeID}"
#     )
#     # --- 2.5. V2V情報の取得 ---
#     avg_evac_time_data = vehInfo.get_avg_evac_time_by_route_by_recive_time()

#     # 修正点:
#     # avg_evac_time_data は {receive_time: {route_tuple: route_info}} の構造。
#     # receive_time は情報取得時刻なので，計算には使わない。
#     # 中の route 情報だけを統合する。
#     routes_dict = {}

#     if avg_evac_time_data:
#         for receive_time in sorted(avg_evac_time_data.keys()):
#             route_dict_at_time = avg_evac_time_data.get(receive_time, {})

#             if not route_dict_at_time:
#                 dprint(f"V2V info receive_time={receive_time}: empty")
#                 continue

#             duplicate_count = sum(
#                 1 for route_tuple in route_dict_at_time.keys()
#                 if route_tuple in routes_dict
#             )

#             routes_dict.update(route_dict_at_time)

#             dprint(
#                 f"V2V info receive_time={receive_time}: "
#                 f"routes={len(route_dict_at_time)}, "
#                 f"duplicates={duplicate_count}, "
#                 f"merged_routes={len(routes_dict)}"
#             )
#     else:
#         dprint("V2V info is empty")

#     dprint(f"total merged V2V routes={len(routes_dict)}")
#     # --- 3. 全ての候補避難所について所要時間を計算 ---
#     candidate_results_list = []

#     edgeIDs_within_junction_to_shelter_dict = (
#         vehInfo.get_edgeIDs_within_junction_to_shelter_dict()
#     )

#     route_name = find_route_name_by_edge(
#         edgeID=current_edgeID,
#         routes_dict=edgeIDs_within_junction_to_shelter_dict
#     )

#     if route_name is None:
#         route_name = ""

#     current_group = _get_shelter_group(current_target_shelterID)

#     dprint(
#         f"route_name={route_name}, "
#         f"current_group={current_group}, "
#         f"candidate_shelters={list(agent.get_candidate_edge_by_shelterID().keys())}"
#     )

#     for candidate_shelterID, candidate_edgeID in (
#         agent.get_candidate_edge_by_shelterID().items()
#     ):
#         if candidate_shelterID == current_target_shelterID:
#             dprint(
#                 f"skip candidate_shelter={candidate_shelterID}: "
#                 "same as current target shelter"
#             )
#             continue

#         candidate_shelter = find_shelter_by_edgeID_connect_target_shelter(
#             edgeID=candidate_edgeID,
#             shelter_list=shelter_list
#         )

#         if not candidate_shelter:
#             dprint(
#                 f"skip candidate_shelter={candidate_shelterID}: "
#                 f"candidate_edgeID={candidate_edgeID}, shelter not found"
#             )
#             continue

#         candidate_group = _get_shelter_group(candidate_shelterID)

#         # 毎候補で必ずベースから始める
#         edge_to_search_from = base_reroute_start_edgeID

#         # --- 3.A. 迂回開始edgeの補正 ---
#         if route_name.startswith('intermediate'):
#             if current_group == candidate_group:
#                 edge_to_search_from = current_edgeID

#         elif route_name.startswith('ShelterA'):
#             if route_name.endswith("_opposite"):
#                 if current_group == candidate_group:
#                     edge_to_search_from = get_opposite_edgeID_by_edgeID(
#                         edgeID=current_edgeID
#                     )
#                 else:
#                     edge_to_search_from = current_edgeID

#         elif route_name.startswith('ShelterB'):
#             if route_name.endswith("_opposite"):
#                 if current_group == candidate_group:
#                     edge_to_search_from = current_edgeID
#                 else:
#                     edge_to_search_from = get_opposite_edgeID_by_edgeID(
#                         edgeID=current_edgeID
#                     )

#         dprint(
#             f"candidate start: shelter={candidate_shelterID}, "
#             f"candidate_edgeID={candidate_edgeID}, "
#             f"candidate_group={candidate_group}, "
#             f"edge_to_search_from={edge_to_search_from}"
#         ) 
#         if current_edgeID in ["E20", "E1"]:
#             edge_to_search_from = current_edgeID
#         # ---------------------------------------------------------
#         # A. 自由速度での迂回所要時間
#         # ---------------------------------------------------------
#         # print(f"8current_edgeID: {current_edgeID}, from_edgeID: {edge_to_search_from}, to_edgeID: {candidate_edgeID}, shelter: {candidate_shelterID}")
#         distance_free_flow = calculate_reroute_distance(
#             vehID=agent.get_vehID(),
#             from_edgeID=edge_to_search_from,
#             to_edgeID=candidate_edgeID,
#             shelter=candidate_shelter,
#             approach_edgeIDs_by_start_edgeID=approach_edgeIDs_by_start_edgeID
#         )

#         time_free_flow = distance_free_flow / FREE_FLOW_SPEED

#         dprint(
#             f"candidate={candidate_shelterID}: "
#             f"free_flow distance={distance_free_flow:.3f}, "
#             f"time_free_flow={time_free_flow:.3f}"
#         )
#         # ---------------------------------------------------------
#         # B. V2V情報に基づく所要時間
#         # ---------------------------------------------------------
#         time_v2v_based = float('inf')
#         best_v2v_debug_info = None
#         # ここをレビュー
#         if routes_dict:
#             for route_tuple, route_info in routes_dict.items():

#                 # 候補避難所につながるedgeで終わる経路だけを見る
#                 if route_tuple[-1] != candidate_edgeID:
#                     continue

#                 # 現在経路そのものは迂回候補から除外
#                 if route_tuple == current_route_tuple:
#                     continue

#                 v2v_avg_time = route_info.get('avg_time')
#                 # print(f"Checking V2V route for candidate={candidate_shelterID}: route={route_tuple}, v2v_avg_time={v2v_avg_time}")

#                 if v2v_avg_time is None:
#                     dprint(
#                         f"candidate={candidate_shelterID}: "
#                         f"skip V2V route because avg_time is missing: {route_tuple}"
#                     )
#                     continue

#                 # 現在経路とV2V経路の先頭一致部分を探す
#                 lcp_length = 0
#                 min_len = min(len(current_route_tuple), len(route_tuple))

#                 for i in range(min_len):
#                     if current_route_tuple[i] == route_tuple[i]:
#                         lcp_length += 1
#                     else:
#                         break

#                 target_index = lcp_length

#                 if target_index >= len(route_tuple):
#                     target_index = 0

#                 if target_index == 0:
#                     dprint(
#                         f"candidate={candidate_shelterID}: "
#                         f"skip V2V route because branch target is invalid. "
#                         f"lcp_length={lcp_length}, route={route_tuple}"
#                     )
#                     continue

#                 branch_target_edge = route_tuple[target_index]
#                 distance_to_branch = calculate_reroute_distance(
#                     vehID=agent.get_vehID(),
#                     from_edgeID=edge_to_search_from,
#                     to_edgeID=branch_target_edge,
#                     shelter=candidate_shelter,
#                     approach_edgeIDs_by_start_edgeID=approach_edgeIDs_by_start_edgeID
#                 )

#                 time_to_branch = distance_to_branch / FREE_FLOW_SPEED

#                 # 既存実装の近似を踏襲:
#                 # 現在地から分岐先までは自由速度で補い，
#                 # その先の参考値としてV2Vのavg_timeを足す。
#                 calculated_time = time_to_branch + v2v_avg_time

#                 dprint(
#                     f"candidate={candidate_shelterID}: V2V route matched. "
#                     f"route_last_edge={route_tuple[-1]}, "
#                     f"lcp_length={lcp_length}, "
#                     f"branch_target_edge={branch_target_edge}, "
#                     f"distance_to_branch={distance_to_branch:.3f}, "
#                     f"time_to_branch={time_to_branch:.3f}, "
#                     f"v2v_avg_time={v2v_avg_time:.3f}, "
#                     f"calculated_time={calculated_time:.3f}, "
#                     f"vehicles={route_info.get('vehicles')}"
#                 )

#                 if calculated_time < time_v2v_based:
#                     time_v2v_based = calculated_time
#                     best_v2v_debug_info = {
#                         "route_tuple": route_tuple,
#                         "branch_target_edge": branch_target_edge,
#                         "distance_to_branch": distance_to_branch,
#                         "time_to_branch": time_to_branch,
#                         "v2v_avg_time": v2v_avg_time,
#                         "calculated_time": calculated_time,
#                         "vehicles": route_info.get("vehicles"),
#                     }

#         if time_v2v_based == float('inf'):
#             dprint(
#                 f"candidate={candidate_shelterID}: "
#                 "no usable V2V route. use free_flow only."
#             )
#         else:
#             dprint(
#                 f"candidate={candidate_shelterID}: "
#                 f"best V2V time={time_v2v_based:.3f}, "
#                 f"best_v2v_info={best_v2v_debug_info}"
#             )

#         # ---------------------------------------------------------
#         # C. 最良時間の採用
#         # ---------------------------------------------------------
#         best_time_for_this_candidate = min(time_free_flow, time_v2v_based)

#         if time_v2v_based < time_free_flow:
#             best_source = "v2v_based"
#         else:
#             best_source = "free_flow"

#         dprint(
#             f"candidate result: shelter={candidate_shelterID}, "
#             f"candidate_edgeID={candidate_edgeID}, "
#             f"best_time={best_time_for_this_candidate:.3f}, "
#             f"best_source={best_source}"
#         )

#         candidate_results_list.append(
#             (
#                 best_time_for_this_candidate,
#                 candidate_shelterID,
#                 candidate_edgeID
#             )
#         )
#     # --- 4. ソートと結果の生成 ---
#     candidate_results_list.sort(key=lambda x: x[0])
#     to_edge_list = [item[2] for item in candidate_results_list]

#     dprint(f"candidate_results_list={candidate_results_list}")
#     dprint(f"to_edge_list={to_edge_list}")

#     if not candidate_results_list:
#         dprint("no candidate results. keep current route.")
#         return base_reroute_start_edgeID, shelterID, [], 0.0, congestion_flg

#     # --- 5. 最終判断 ---
#     best_candidate_time = candidate_results_list[0][0]
#     best_candidate_shelterID = candidate_results_list[0][1]
#     best_candidate_edgeID = candidate_results_list[0][2]

#     time_gain = estimated_current_route_evacuation_time - best_candidate_time
#     threshold = agent.get_route_change_threshold()
#     # print(f"Check estimated_current_route_evacuation_time: {estimated_current_route_evacuation_time:.3f}, best_candidate_time: {best_candidate_time:.3f}, time_gain: {time_gain:.3f}, threshold: {threshold:.3f}")
#     if time_gain > threshold:
#         to_edge_decision_list = [best_candidate_edgeID]
#         congestion_flg = True

#         if not approach_edge_flg:
#             return (
#                 base_reroute_start_edgeID,
#                 best_candidate_shelterID,
#                 to_edge_decision_list,
#                 time_gain,
#                 congestion_flg
#             )
#         print("check num 4")
#         route = traci.simulation.findRoute(
#             base_reroute_start_edgeID,
#             best_candidate_edgeID
#         )

#         if route.edges:
#             traci.vehicle.setRoute(agent.get_vehID(), route.edges)
#             return (
#                 base_reroute_start_edgeID,
#                 best_candidate_shelterID,
#                 [],
#                 time_gain,
#                 congestion_flg
#             )

#         return (
#             base_reroute_start_edgeID,
#             best_candidate_shelterID,
#             [],
#             time_gain,
#             congestion_flg
#         )
#     return base_reroute_start_edgeID, shelterID, to_edge_list, time_gain, congestion_flg

def find_alternative_better_choice_fixed(
    current_edgeID: str, 
    vehInfo: VehicleInfo, 
    agent: Agent, 
    shelter: Shelter,
    shelter_list: list, 
    custome_edge_list: list,
    debug: bool = False,
):
    """
    現在の経路と，V2V情報および自由速度計算に基づく迂回経路を比較し，
    より良い選択肢があれば経路変更情報を返す。

    修正点:
    - vehInfo.get_avg_evac_time_by_route_by_recive_time() の外側キーである receive_time は，
      情報取得時刻であり，経路時間推定の計算には使わない。
    - receive_time ごとの内側辞書 {route_tuple: route_info} を統合し，
      すべてのV2V経路情報を候補として利用する。
    - debug=True のときだけ，判断に必要な中間値を出力する。
    - 現在目的地 edge と同じ candidate_edgeID は代替候補から除外する。
    - V2V候補でも現在目的地 edge に到達する経路は防御的に除外する。
    """

    def dprint(message: str):
        if debug:
            print(f"[find_alternative_better_choice] {message}")

    # --- 0. 定数・初期化 ---
    FREE_FLOW_SPEED = 11.0

    shelterID = ""
    shelterID_to_return = vehInfo.get_target_shelter()
    to_edge_list_to_return = []
    congestion_flg = False

    dprint(
        f"start vehID={agent.get_vehID()}, "
        f"current_edgeID={current_edgeID}, "
    )

    if current_edgeID.startswith(':'):
        dprint(f"skip: current_edgeID is internal junction edge: {current_edgeID}")
        return "", shelterID_to_return, to_edge_list_to_return, 0.0, congestion_flg

    # --- 1. 現在の経路での残り時間を計算 ---
    current_target_shelterID = agent.get_target_shelter()

    shelter_for_vehInfo: Shelter = shelter
    dprint(f"current_target_shelterID={current_target_shelterID}, shelter_for_vehInfo={shelter_for_vehInfo.get_shelterID()}")
    current_route_tuple = tuple(traci.vehicle.getRoute(agent.get_vehID()))
    dprint(f"current_route_tuple={current_route_tuple}")
    current_destination_edge = current_route_tuple[-1]

    distance_to_current_shelter = calculate_remaining_route_distance(
        agent.get_vehID(),
        current_destination_edge,
        shelter=shelter_for_vehInfo
    )
    try:
        # 念のため初期値を入れておく
        current_speed = FREE_FLOW_SPEED

        if vehInfo.get_vehicle_comm_enabled_flag():
            # システム有り: 現在edgeの平均速度を使う
            current_speed = traci.edge.getLastStepMeanSpeed(current_edgeID)
        else:
            # システム無し: 自由速度を使う
            current_speed = FREE_FLOW_SPEED

        if current_speed < 1.8:
            dprint(
                f"current_speed is too low: {current_speed:.3f}. "
                f"clamped to 1.8"
            )
            current_speed = 1.8

    except traci.TraCIException as e:
        print(f"Error getting speed for {agent.get_vehID()}: {e}")
        current_speed = 1.0

    estimated_current_route_evacuation_time = (
        distance_to_current_shelter / current_speed
    )

    dprint(
        "current route: "
        f"target_shelter={current_target_shelterID}, "
        f"destination_edge={current_destination_edge}, "
        f"distance_to_current_shelter={distance_to_current_shelter:.3f}, "
        f"current_speed={current_speed:.3f}, "
        f"estimated_current_time={estimated_current_route_evacuation_time:.3f}"
    )
    # --- 2. 迂回検討のための準備 ---
    approach_edgeIDs_by_start_edgeID = vehInfo.get_approach_edge_dict()
    # print(f"approach_edgeIDs_by_start_edgeID: {approach_edgeIDs_by_start_edgeID}")
    is_in_approach_list = any(
        current_edgeID in edge_list
        for edge_list in approach_edgeIDs_by_start_edgeID.values()
    )
    approach_edge_flg = False
    base_reroute_start_edgeID = ""

    if not is_in_approach_list:
        # アプローチエッジにいない場合，デフォルトは反対車線
        base_reroute_start_edgeID = get_opposite_edgeID_by_edgeID(
            edgeID=current_edgeID
        )
    else:
        # アプローチエッジにいる場合，デフォルトは現在車線
        base_reroute_start_edgeID = current_edgeID
        approach_edge_flg = True

        if not vehInfo.get_vehicle_comm_enabled_flag():
            dprint(
                "system_mode=3 and vehicle is in approach edge. "
                "return without reroute calculation."
            )
            return base_reroute_start_edgeID, shelterID_to_return, [], 0.0, congestion_flg

    dprint(
        f"is_in_approach_list={is_in_approach_list}, "
        f"approach_edge_flg={approach_edge_flg}, "
        f"base_reroute_start_edgeID={base_reroute_start_edgeID}"
    )
    # --- 2.5. V2V情報の取得 ---
    avg_evac_time_data = vehInfo.get_avg_evac_time_by_route_by_recive_time()

    # 修正点:
    # avg_evac_time_data は {receive_time: {route_tuple: route_info}} の構造。
    # receive_time は情報取得時刻なので，計算には使わない。
    # 中の route 情報だけを統合する。
    routes_dict = {}

    if avg_evac_time_data:
        for receive_time in sorted(avg_evac_time_data.keys()):
            route_dict_at_time = avg_evac_time_data.get(receive_time, {})

            if not route_dict_at_time:
                dprint(f"V2V info receive_time={receive_time}: empty")
                continue

            duplicate_count = sum(
                1 for route_tuple in route_dict_at_time.keys()
                if route_tuple in routes_dict
            )

            routes_dict.update(route_dict_at_time)

            dprint(
                f"V2V info receive_time={receive_time}: "
                f"routes={len(route_dict_at_time)}, "
                f"duplicates={duplicate_count}, "
                f"merged_routes={len(routes_dict)}"
            )
    else:
        dprint("V2V info is empty")

    dprint(f"total merged V2V routes={len(routes_dict)}")
    # --- 3. 全ての候補避難所について所要時間を計算 ---
    candidate_results_list = []

    edgeIDs_within_junction_to_shelter_dict = (
        vehInfo.get_edgeIDs_within_junction_to_shelter_dict()
    )

    route_name = find_route_name_by_edge(
        edgeID=current_edgeID,
        routes_dict=edgeIDs_within_junction_to_shelter_dict
    )

    if route_name is None:
        route_name = ""

    current_group = _get_shelter_group(current_target_shelterID)

    dprint(
        f"route_name={route_name}, "
        f"current_group={current_group}, "
        f"candidate_shelters={list(agent.get_candidate_edge_by_shelterID().keys())}"
    )

    for candidate_shelterID, candidate_edgeID in (
        agent.get_candidate_edge_by_shelterID().items()
    ):
        dprint(
            f"current_target_shelterID={current_target_shelterID}, "
            f"current_destination_edge={current_destination_edge}, "
            f"candidate_shelterID={candidate_shelterID}, "
            f"candidate_edgeID={candidate_edgeID}"
        )

        # 完全に現在の候補IDと同じなら除外する。
        # 例: current_target_shelterID=ShelterA_1, candidate_shelterID=ShelterA_1
        if candidate_shelterID == current_target_shelterID:
            dprint(
                f"skip candidate_shelter={candidate_shelterID}: "
                "same as current target shelter ID"
            )
            continue

        # shelterID が異なっていても，目的地 edge が現在目的地と同じなら除外する。
        # 例: ShelterA_1 -> E16 が現在目的地で，候補にも別IDで E16 が入る場合。
        # 一方，ShelterA_2 -> E13 のような同一避難地・別経路は評価対象として残る。
        if candidate_edgeID == current_destination_edge:
            dprint(
                f"skip candidate because candidate_edgeID is current destination: "
                f"candidate_edgeID={candidate_edgeID}, "
                f"current_destination_edge={current_destination_edge}"
            )
            continue

        candidate_shelter = find_shelter_by_edgeID_connect_target_shelter(
            edgeID=candidate_edgeID,
            shelter_list=shelter_list
        )

        if not candidate_shelter:
            dprint(
                f"skip candidate_shelter={candidate_shelterID}: "
                f"candidate_edgeID={candidate_edgeID}, shelter not found"
            )
            continue

        candidate_group = _get_shelter_group(candidate_shelterID)

        # 毎候補で必ずベースから始める
        edge_to_search_from = base_reroute_start_edgeID

        # --- 3.A. 迂回開始edgeの補正 ---
        if route_name.startswith('intermediate'):
            if current_group == candidate_group:
                edge_to_search_from = current_edgeID

        elif route_name.startswith('ShelterA'):
            if route_name.endswith("_opposite"):
                if current_group == candidate_group:
                    edge_to_search_from = get_opposite_edgeID_by_edgeID(
                        edgeID=current_edgeID
                    )
                else:
                    edge_to_search_from = current_edgeID

        elif route_name.startswith('ShelterB'):
            if route_name.endswith("_opposite"):
                if current_group == candidate_group:
                    edge_to_search_from = current_edgeID
                else:
                    edge_to_search_from = get_opposite_edgeID_by_edgeID(
                        edgeID=current_edgeID
                    )

        dprint(
            f"candidate start: shelter={candidate_shelterID}, "
            f"candidate_edgeID={candidate_edgeID}, "
            f"candidate_group={candidate_group}, "
            f"edge_to_search_from={edge_to_search_from}"
        ) 
        if current_edgeID in ["E20", "E1"]:
            edge_to_search_from = current_edgeID
        # ---------------------------------------------------------
        # A. 自由速度での迂回所要時間
        # ---------------------------------------------------------
        # print(f"8current_edgeID: {current_edgeID}, from_edgeID: {edge_to_search_from}, to_edgeID: {candidate_edgeID}, shelter: {candidate_shelterID}")
        distance_free_flow = calculate_reroute_distance(
            vehID=agent.get_vehID(),
            from_edgeID=edge_to_search_from,
            to_edgeID=candidate_edgeID,
            shelter=candidate_shelter,
            approach_edgeIDs_by_start_edgeID=approach_edgeIDs_by_start_edgeID
        )

        time_free_flow = distance_free_flow / FREE_FLOW_SPEED

        dprint(
            f"candidate={candidate_shelterID}: "
            f"free_flow distance={distance_free_flow:.3f}, "
            f"time_free_flow={time_free_flow:.3f}"
        )
        # ---------------------------------------------------------
        # B. V2V情報に基づく所要時間
        # ---------------------------------------------------------
        time_v2v_based = float('inf')
        best_v2v_debug_info = None
        # ここをレビュー
        if routes_dict:
            for route_tuple, route_info in routes_dict.items():

                # 候補避難所につながるedgeで終わる経路だけを見る
                if route_tuple[-1] != candidate_edgeID:
                    continue

                # 現在目的地 edge に向かう V2V 経路は代替候補から除外する。
                # route_tuple == current_route_tuple だけでは，
                # 同じ目的地 edge に向かう別経路を除外できない。
                if route_tuple[-1] == current_destination_edge:
                    dprint(
                        f"candidate={candidate_shelterID}: "
                        f"skip V2V route because destination edge is current destination. "
                        f"route_last_edge={route_tuple[-1]}, "
                        f"current_destination_edge={current_destination_edge}, "
                        f"route={route_tuple}"
                    )
                    continue

                # 現在経路そのものは迂回候補から除外
                if route_tuple == current_route_tuple:
                    dprint(
                        f"candidate={candidate_shelterID}: "
                        f"skip V2V route because it is exactly current route: "
                        f"route={route_tuple}"
                    )
                    continue

                v2v_avg_time = route_info.get('avg_time')
                # print(f"Checking V2V route for candidate={candidate_shelterID}: route={route_tuple}, v2v_avg_time={v2v_avg_time}")

                if v2v_avg_time is None:
                    dprint(
                        f"candidate={candidate_shelterID}: "
                        f"skip V2V route because avg_time is missing: {route_tuple}"
                    )
                    continue

                # 現在経路とV2V経路の先頭一致部分を探す
                lcp_length = 0
                min_len = min(len(current_route_tuple), len(route_tuple))

                for i in range(min_len):
                    if current_route_tuple[i] == route_tuple[i]:
                        lcp_length += 1
                    else:
                        break

                target_index = lcp_length

                if target_index >= len(route_tuple):
                    target_index = 0

                if target_index == 0:
                    dprint(
                        f"candidate={candidate_shelterID}: "
                        f"skip V2V route because branch target is invalid. "
                        f"lcp_length={lcp_length}, route={route_tuple}"
                    )
                    continue
                # print(f"route_tuple={route_tuple}, lcp_length={lcp_length}, target_index={target_index}")
                branch_target_edge = route_tuple[target_index]
                # ここに問題あり
                # print(f"candidate={candidate_shelterID}: V2V route matched.from_edgeID={edge_to_search_from}, branch_target_edge={branch_target_edge}, route={route_tuple}")
                distance_to_branch = calculate_reroute_distance(
                    vehID=agent.get_vehID(),
                    from_edgeID=edge_to_search_from,
                    to_edgeID=branch_target_edge,
                    shelter=candidate_shelter,
                    approach_edgeIDs_by_start_edgeID=approach_edgeIDs_by_start_edgeID
                )

                time_to_branch = distance_to_branch / FREE_FLOW_SPEED

                # 既存実装の近似を踏襲:
                # 現在地から分岐先までは自由速度で補い，
                # その先の参考値としてV2Vのavg_timeを足す。
                calculated_time = time_to_branch + v2v_avg_time

                dprint(
                    f"candidate={candidate_shelterID}: V2V route matched. "
                    f"route_last_edge={route_tuple[-1]}, "
                    f"lcp_length={lcp_length}, "
                    f"branch_target_edge={branch_target_edge}, "
                    f"distance_to_branch={distance_to_branch:.3f}, "
                    f"time_to_branch={time_to_branch:.3f}, "
                    f"v2v_avg_time={v2v_avg_time:.3f}, "
                    f"calculated_time={calculated_time:.3f}, "
                    f"vehicles={route_info.get('vehicles')}"
                )

                if calculated_time < time_v2v_based:
                    time_v2v_based = calculated_time
                    best_v2v_debug_info = {
                        "route_tuple": route_tuple,
                        "branch_target_edge": branch_target_edge,
                        "distance_to_branch": distance_to_branch,
                        "time_to_branch": time_to_branch,
                        "v2v_avg_time": v2v_avg_time,
                        "calculated_time": calculated_time,
                        "vehicles": route_info.get("vehicles"),
                    }

        if time_v2v_based == float('inf'):
            dprint(
                f"candidate={candidate_shelterID}: "
                "no usable V2V route. use free_flow only."
            )
        else:
            dprint(
                f"candidate={candidate_shelterID}: "
                f"best V2V time={time_v2v_based:.3f}, "
                f"best_v2v_info={best_v2v_debug_info}"
            )

        # ---------------------------------------------------------
        # C. 最良時間の採用
        # ---------------------------------------------------------
        best_time_for_this_candidate = min(time_free_flow, time_v2v_based)

        if time_v2v_based < time_free_flow:
            best_source = "v2v_based"
        else:
            best_source = "free_flow"

        dprint(
            f"candidate result: shelter={candidate_shelterID}, "
            f"candidate_edgeID={candidate_edgeID}, "
            f"best_time={best_time_for_this_candidate:.3f}, "
            f"best_source={best_source}"
        )

        candidate_results_list.append(
            (
                best_time_for_this_candidate,
                candidate_shelterID,
                candidate_edgeID
            )
        )
    # --- 4. ソートと結果の生成 ---
    # 防御的チェック: 万一，現在目的地 edge が残っていてもここで落とす。
    candidate_results_list = [
        item for item in candidate_results_list
        if item[2] != current_destination_edge
    ]
    candidate_results_list.sort(key=lambda x: x[0])
    to_edge_list = [item[2] for item in candidate_results_list]

    dprint(f"candidate_results_list={candidate_results_list}")
    dprint(f"to_edge_list={to_edge_list}")

    if not candidate_results_list:
        dprint("no candidate results. keep current route.")
        return base_reroute_start_edgeID, shelterID, [], 0.0, congestion_flg

    # --- 5. 最終判断 ---
    best_candidate_time = candidate_results_list[0][0]
    best_candidate_shelterID = candidate_results_list[0][1]
    best_candidate_edgeID = candidate_results_list[0][2]

    time_gain = estimated_current_route_evacuation_time - best_candidate_time
    threshold = agent.get_route_change_threshold()
    # print(f"Check estimated_current_route_evacuation_time: {estimated_current_route_evacuation_time:.3f}, best_candidate_time: {best_candidate_time:.3f}, time_gain: {time_gain:.3f}, threshold: {threshold:.3f}")
    if time_gain > threshold:
        to_edge_decision_list = [best_candidate_edgeID]
        congestion_flg = True

        if not approach_edge_flg:
            return (
                base_reroute_start_edgeID,
                best_candidate_shelterID,
                to_edge_decision_list,
                time_gain,
                congestion_flg
            )
        dprint("approach edge: apply best route directly with traci.vehicle.setRoute")
        pass  # removed temporary debug print
        route = traci.simulation.findRoute(
            base_reroute_start_edgeID,
            best_candidate_edgeID
        )
        pass  # removed temporary debug print

        if route.edges:
            traci.vehicle.setRoute(agent.get_vehID(), route.edges)
            return (
                base_reroute_start_edgeID,
                best_candidate_shelterID,
                [],
                time_gain,
                congestion_flg
            )

        return (
            base_reroute_start_edgeID,
            best_candidate_shelterID,
            [],
            time_gain,
            congestion_flg
        )
    return base_reroute_start_edgeID, shelterID, to_edge_list, time_gain, congestion_flg


def _get_shelter_group(shelter_id: str) -> str:
    """
    避難所IDからグループ名（例: "ShelterA"）を抽出する。
    例: "ShelterB_2" -> "ShelterB"
    """
    try:
        return shelter_id.split('_')[0]
    except IndexError:
        return shelter_id # '_' がない場合はID全体をグループ名とみなす



def is_route_time_difference_exceeding_threshold(current_edgeID, agent_by_target_vehID: Agent, vehInfo_by_target_vehID: VehicleInfo, shelter: Shelter, shelter_list: list[Shelter], custome_edge_list: list):
    base_reroute_start_edgeID, shelterID, to_edge_list, time_gain, congestion_flg = find_alternative_better_choice_fixed(
                                                                                                            current_edgeID=current_edgeID,
                                                                                                            vehInfo=vehInfo_by_target_vehID,
                                                                                                            agent=agent_by_target_vehID,
                                                                                                            shelter=shelter,
                                                                                                            shelter_list=shelter_list,
                                                                                                            custome_edge_list=custome_edge_list,
                                                                                                            debug=False
                                                                                                            )
    return base_reroute_start_edgeID, to_edge_list, congestion_flg

def get_route_time_difference_exceeding_threshold(current_edgeID, agent_by_target_vehID: Agent, vehInfo_by_target_vehID: VehicleInfo, shelter: Shelter, shelter_list: list[Shelter], custome_edge_list: list):
    from_edgeID, shelterID, to_edge_list, time_gain, congestion_flg, route_edges_for_sumo= find_alternative_better_choice_fixed_divide(
                                                                                                            current_edgeID=current_edgeID,
                                                                                                            vehInfo=vehInfo_by_target_vehID,
                                                                                                            agent=agent_by_target_vehID,
                                                                                                            shelter=shelter,
                                                                                                            shelter_list=shelter_list,
                                                                                                            custome_edge_list=custome_edge_list,
                                                                                                            debug=False
                                                                                                            )
    # if congestion_flg:
    #     print(f"from_edgeID: {from_edgeID}, shelterID: {shelterID}, to_edge_list: {to_edge_list}, time_gain: {time_gain:.3f}, congestion_flg: {congestion_flg}, route_edges_for_sumo: {route_edges_for_sumo}")
    return from_edgeID, shelterID, to_edge_list, time_gain, congestion_flg, route_edges_for_sumo


def find_alternative_better_choice(current_edgeID: str, 
                                   vehInfo:VehicleInfo, 
                                   agent:Agent, 
                                   shelter_list:list, 
                                   custome_edge_list:List[CustomeEdge],
                                   system_mode:int,
                                   decision_mode:int
                                   ):
    """
    現在の経路と、V2V情報および自由速度計算に基づく迂回経路を比較し、
    より良い選択肢があれば経路変更情報を返す。
    """
    # --- 0. 定数・初期化 ---
    FREE_FLOW_SPEED = 11.0
    shelterID = ""
    shelterID_to_return = vehInfo.get_target_shelter() # 最初は現在の避難地
    to_edge_list_to_return = []

    if current_edgeID.startswith(':'):
        # 交差点内では計算しない
        return "", shelterID_to_return, to_edge_list_to_return, 0.0 

    # Uターンロジック
    # 現在目指している避難所の情報を取得
    current_target_shelterID = agent.get_target_shelter()
    shelter_for_vehInfo: Shelter = find_shelter_by_edgeID_connect_target_shelter(
        edgeID=agent.get_near_edgeID_by_target_shelter(),
        shelter_list=shelter_list
        )

    # --- 1. 現在の経路での残り時間 (Estimated Current Time) の計算 ---
    current_route_tuple = tuple(traci.vehicle.getRoute(agent.get_vehID()))
    current_destination_edge = current_route_tuple[-1]
    
    distance_to_current_shelter = calculate_remaining_route_distance(
        agent.get_vehID(), 
        current_destination_edge, 
        shelter=shelter_for_vehInfo
    )

    # 現在の速度（0除算防止）
    try:
        if system_mode == 1: # システム有の場合
            current_speed = traci.edge.getLastStepMeanSpeed(current_edgeID)
        if system_mode == 3 : # システム無の場合
            current_speed = FREE_FLOW_SPEED
        if current_speed < 1.8:
            current_speed = 1.8
    except traci.TraCIException as e:
        print(f"Error getting speed for {agent.get_vehID()}: {e}")
        current_speed = 1.0
    estimated_current_route_evacuation_time = distance_to_current_shelter / current_speed
    
    # --- 2. 迂回検討のための準備 (Uターン地点の決定) ---
    approach_edgeIDs_by_start_edgeID = vehInfo.get_approach_edge_dict()
    is_in_approach_list = any(current_edgeID in edge_list for edge_list in approach_edgeIDs_by_start_edgeID.values())
    approach_edge_flg = False
    base_reroute_start_edgeID = ""
    if not is_in_approach_list:
        # アプローチエッジにいない場合、デフォルトは反対車線
        base_reroute_start_edgeID = get_opposite_edgeID_by_edgeID(edgeID=current_edgeID)
    else:
        # アプローチエッジにいる場合、デフォルトは現在車線
        base_reroute_start_edgeID = current_edgeID
        
        if system_mode == 3:
            return base_reroute_start_edgeID, shelterID_to_return, [], 0.0

    # V2V情報の取得
    avg_evac_time_data = vehInfo.get_avg_evac_time_by_route_by_recive_time()
    routes_dict = list(avg_evac_time_data.values())[0] if avg_evac_time_data else {}

    # --- 3. 全ての候補避難所について所要時間を計算 ---
    candidate_results_list = []
    
    # ★★★ 修正（ループ外へ移動）★★★
    # 経路辞書とルート名はループ前に1回だけ取得すればよい
    edgeIDs_within_junction_to_shelter_dict = vehInfo.get_edgeIDs_within_junction_to_shelter_dict()
    route_name = find_route_name_by_edge(edgeID=current_edgeID, routes_dict=edgeIDs_within_junction_to_shelter_dict)
    current_group = _get_shelter_group(current_target_shelterID)

    for candidate_shelterID, candidate_edgeID in agent.get_candidate_edge_by_shelterID().items():
        if candidate_shelterID == current_target_shelterID:
            continue

        candidate_shelter = find_shelter_by_edgeID_connect_target_shelter(
            edgeID=candidate_edgeID,
            shelter_list=shelter_list
        )
        if not candidate_shelter:
            continue

        # ---------------------------------------------------------
        # A. [実測] 自由速度でのUターン所要時間の計算 (Baseline)
        # ---------------------------------------------------------
        candidate_group = _get_shelter_group(candidate_shelterID)

        # ★★★ 修正（バグ修正） ★★★
        # 毎回のループで、必ず「ベース」の値から計算を始める
        edge_to_search_from = base_reroute_start_edgeID

        # print(f" find_222alternative_better_choice vehID:{agent.get_vehID()} current_edgeID {current_edgeID} from_edgeID: {edge_to_search_from}, to_edgeID: {candidate_edgeID}, shelterID: {candidate_shelterID} current_route {current_route_tuple}")

        # --- ▼▼▼ ユーザー指定のロジックを挿入 ▼▼▼ ---
        # rerouteは反対車線設定がデフォルト
        if route_name.startswith('intermediate'):
            # 避難地グループが一緒の場合は再度反転させて、元のedgeから探索
            if current_group == candidate_group:
                edge_to_search_from = current_edgeID
            
        elif route_name.startswith('ShelterA'):
            if route_name.endswith("_opposite"):
                if current_group == candidate_group:
                    edge_to_search_from = get_opposite_edgeID_by_edgeID(edgeID=current_edgeID)
                else:
                    edge_to_search_from = current_edgeID
        elif route_name.startswith('ShelterB'):
            if route_name.endswith("_opposite"):
                if current_group == candidate_group:
                    edge_to_search_from = current_edgeID 
                else:
                    edge_to_search_from = get_opposite_edgeID_by_edgeID(edgeID=current_edgeID)
        # --- ▲▲▲ ユーザー指定のロジックここまで ▲▲▲ ---
        distance_free_flow = calculate_reroute_distance(
            vehID=agent.get_vehID(),
            from_edgeID=edge_to_search_from, # 変更された可能性のあるエッジを使用
            to_edgeID=candidate_edgeID,
            shelter=candidate_shelter,
            approach_edgeIDs_by_start_edgeID=approach_edgeIDs_by_start_edgeID
        )
        time_free_flow = distance_free_flow / FREE_FLOW_SPEED

        # ---------------------------------------------------------
        # B. [V2V] 情報に基づく所要時間の計算 (あれば)
        # ---------------------------------------------------------
        time_v2v_based = float('inf')
        
        if routes_dict:
            for route_tuple, route_info in routes_dict.items():
                if route_tuple[-1] != candidate_edgeID or route_tuple == current_route_tuple:
                    continue
                
                v2v_avg_time = route_info['avg_time']
                lcp_length = 0
                min_len = min(len(current_route_tuple), len(route_tuple))
                for i in range(min_len):
                    if current_route_tuple[i] == route_tuple[i]:
                        lcp_length += 1
                    else:
                        break
                
                target_index = lcp_length
                if target_index >= len(route_tuple):
                    target_index = 0
                if target_index == 0:
                    continue
                else:
                    branch_target_edge = route_tuple[target_index]
                    
                    # ★★★ 修正（バグ修正） ★★★
                    # 分岐点までの距離計算も、Aで決定した edge_to_search_from を使う
                    distance_to_branch = calculate_reroute_distance(
                        vehID=agent.get_vehID(),
                        from_edgeID=edge_to_search_from, 
                        to_edgeID=branch_target_edge,
                        shelter=candidate_shelter, 
                        approach_edgeIDs_by_start_edgeID=approach_edgeIDs_by_start_edgeID
                    )
                    time_to_branch = distance_to_branch / FREE_FLOW_SPEED
                    calculated_time = time_to_branch + v2v_avg_time

                    if calculated_time < time_v2v_based:
                        time_v2v_based = calculated_time

        # ---------------------------------------------------------
        # C. 最良時間の採用
        # ---------------------------------------------------------
        best_time_for_this_candidate = min(time_free_flow, time_v2v_based)
        
        candidate_results_list.append(
            (best_time_for_this_candidate, candidate_shelterID, candidate_edgeID)
        )

    # --- 4. ソートと結果の生成 ---
    candidate_results_list.sort(key=lambda x: x[0])
    to_edge_list = [item[2] for item in candidate_results_list]

    if not candidate_results_list:
        return base_reroute_start_edgeID, shelterID, [], 0.0
    
    if decision_mode == 2:
        best_candidate_edgeID = candidate_results_list[0][2]
        best_candidate_shelterID = candidate_results_list[0][1]
        to_edge_decision_list = [best_candidate_edgeID]
        return base_reroute_start_edgeID, best_candidate_shelterID, to_edge_list, 0.0

    # --- 5. 最終判断 ---
    best_candidate_time = candidate_results_list[0][0]
    best_candidate_shelterID = candidate_results_list[0][1]
    best_candidate_edgeID = candidate_results_list[0][2]

    time_gain = estimated_current_route_evacuation_time - best_candidate_time
    threshold = agent.get_route_change_threshold()
    
    if time_gain > threshold:
        to_edge_decision_list = [best_candidate_edgeID]
        if not approach_edge_flg:
            return base_reroute_start_edgeID, best_candidate_shelterID, to_edge_decision_list, time_gain
        else:
            route = traci.simulation.findRoute(base_reroute_start_edgeID, best_candidate_edgeID)
            if route.edges:
                traci.vehicle.setRoute(agent.get_vehID(), route.edges) 
                traci.vehicle.setColor(agent.get_vehID(), (255, 0, 0, 255))
                return base_reroute_start_edgeID, best_candidate_shelterID, [], time_gain
            return base_reroute_start_edgeID, best_candidate_shelterID, [], time_gain
    else:
        # 経路維持
        return base_reroute_start_edgeID, shelterID, to_edge_list, time_gain


def find_alternative_shelter_choice(
                                        current_target_shelterID: str, 
                                        vehID: str, 
                                        current_edgeID: str,
                                        vehInfo: VehicleInfo,
                                        shelter_list: list,
                                        agent: Agent
                                    ) -> tuple: # (str, str, list) のタプルを返す
    
    FREE_FLOW_SPEED = 11.0
    shelterID_to_return = vehInfo.get_target_shelter() # 最初は現在の避難地
    to_edge_list_to_return = []
    # ここはOK
    # --- 1. Uターン地点の決定 ---
    if current_edgeID.startswith(':'):
        # 交差点内では計算しない
        return "", shelterID_to_return, to_edge_list_to_return 

    # Uターンロジック
    # 進入路にいる場合は、そのまま、そうでなければ反対車線からスタート
    approach_edgeIDs_by_start_edgeID = vehInfo.get_approach_edge_dict()
    is_in_approach_list = any(current_edgeID in edge_list for edge_list in approach_edgeIDs_by_start_edgeID.values())
    reroute_start_edgeID = current_edgeID
    approach_edge_flg = False
    if not is_in_approach_list:
        reroute_start_edgeID = get_opposite_edgeID_by_edgeID(edgeID=current_edgeID)
    else:
        approach_edge_flg = True

    # --- 2. 現在の避難地のグループ名を取得 ---
    current_group = _get_shelter_group(current_target_shelterID)
    # --- 3. 候補の計算と比較 ---
    # (時間, 避難所ID, 目的エッジID) を格納
    candidate_results_list = [] 

    # shelter_list (エージェントが知っている候補リストではない) を全検索
    for candidate_shelter in shelter_list:
        if candidate_shelter.get_near_edgeID() != agent.get_near_edgeID_by_target_shelter():
            candidate_shelterID = candidate_shelter.get_shelterID()
            ## ここが違う
            candidate_group = _get_shelter_group(candidate_shelterID)
            
            # --- 3a. フィルタリング ---
            # (A) 現在の目的地そのものはスキップ
            if candidate_shelterID == current_target_shelterID:
                continue
                
            # (B) 現在のグループと同じグループはスキップ
            if candidate_group == current_group:
                continue
                
            # --- 3b. 自由速度での所要時間を計算 ---
            candidate_edgeID = candidate_shelter.get_near_edgeID()
            distance_free_flow = calculate_reroute_distance(
                vehID=vehID,
                from_edgeID=reroute_start_edgeID,
                to_edgeID=candidate_edgeID,
                shelter=candidate_shelter,
                approach_edgeIDs_by_start_edgeID=approach_edgeIDs_by_start_edgeID
            )
            
            time_free_flow = distance_free_flow / FREE_FLOW_SPEED
            
            candidate_results_list.append(
                (time_free_flow, candidate_shelterID, candidate_edgeID)
            )
        # else:
        #     return reroute_start_edgeID, shelterID_to_return, to_edge_list_to_return

    # --- 4. ソートと結果の返却 ---
    if not candidate_results_list:
        # 比較対象(別グループ)が一つもなかった場合
        return reroute_start_edgeID, shelterID_to_return, to_edge_list_to_return

    # 時間（タプルの0番目）でソート
    candidate_results_list.sort(key=lambda x: x[0])

    # 戻り値用のソート済み「エッジIDリスト」を作成
    to_edge_list_to_return = [item[2] for item in candidate_results_list]
    found_shelterID = next(shelterID for shelterID, edgeID in agent.get_candidate_edge_by_shelterID().items() if edgeID == to_edge_list_to_return[0])
    shelterID_to_return = found_shelterID
    if not approach_edge_flg:
        return reroute_start_edgeID, shelterID_to_return, to_edge_list_to_return
    else:
        route = traci.simulation.findRoute(reroute_start_edgeID, to_edge_list_to_return[0])
        traci.vehicle.setRoute(vehID, route.edges) # 現在のルートを再設定してUターンを防止
        return reroute_start_edgeID, shelterID_to_return, []


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

def find_uturn_shortest_route_to_current_shelter_group(
    current_edgeID: str,
    vehID: str,
    vehInfo: VehicleInfo,
    agent: Agent,
    shelter_list: list[Shelter],
    debug: bool = False,
) -> tuple[str, str, list]:
    """
    同調バイアス用:
    現在の避難地グループ、例 ShelterA、は維持する。
    ただし ShelterA_1 / ShelterA_2 のどちらに向かうかは、
    U-turn後の開始edgeから最短距離になるものを選ぶ。

    重要:
        現在目的地 edge は候補から除外する。
        例:
            現在 target_shelter = ShelterA_1
            現在目的地 edge = E16
            候補:
                ShelterA_1 -> E16
                ShelterA_2 -> E13

            この場合:
                ShelterA_1 / E16 は除外
                ShelterA_2 / E13 のみ評価
    """

    def dprint(message: str):
        if debug:
            print(f"[majority bias shortest route] {message}")

    current_target_shelterID = agent.get_target_shelter()
    current_group = _get_shelter_group(current_target_shelterID)

    if current_edgeID.startswith(":"):
        dprint(
            f"skip: current_edgeID is internal junction edge: {current_edgeID}"
        )
        return "", current_target_shelterID, []

    # 現在の実際の目的地 edge を TraCI の現在経路から取得する
    try:
        current_route_tuple = tuple(traci.vehicle.getRoute(vehID))
        current_destination_edge = (
            current_route_tuple[-1] if current_route_tuple else ""
        )
    except Exception as e:
        dprint(
            f"failed to get current route from TraCI: "
            f"vehID={vehID}, error={e}"
        )
        current_route_tuple = tuple()
        current_destination_edge = ""

    # TraCI から取れない場合の保険:
    # current_target_shelterID に対応する shelter の near_edgeID を使う
    if not current_destination_edge:
        for shelter in shelter_list:
            if shelter.get_shelterID() == current_target_shelterID:
                current_destination_edge = shelter.get_near_edgeID()
                dprint(
                    f"fallback current_destination_edge from shelter_list: "
                    f"{current_destination_edge}"
                )
                break

    approach_edgeIDs_by_start_edgeID = vehInfo.get_approach_edge_dict()

    is_in_approach_list = any(
        current_edgeID in edge_list
        for edge_list in approach_edgeIDs_by_start_edgeID.values()
    )

    if not is_in_approach_list:
        reroute_start_edgeID = get_opposite_edgeID_by_edgeID(
            edgeID=current_edgeID
        )
    else:
        reroute_start_edgeID = current_edgeID

    dprint(
        f"vehID={vehID}, "
        f"current_edgeID={current_edgeID}, "
        f"reroute_start_edgeID={reroute_start_edgeID}, "
        f"current_target_shelterID={current_target_shelterID}, "
        f"current_group={current_group}, "
        f"current_destination_edge={current_destination_edge}, "
        f"current_route={current_route_tuple}"
    )

    candidate_results_list = []

    for candidate_shelter in shelter_list:
        candidate_shelterID = candidate_shelter.get_shelterID()
        candidate_group = _get_shelter_group(candidate_shelterID)
        candidate_edgeID = candidate_shelter.get_near_edgeID()

        dprint(
            f"check candidate: "
            f"candidate_shelterID={candidate_shelterID}, "
            f"candidate_group={candidate_group}, "
            f"candidate_edgeID={candidate_edgeID}"
        )

        # ShelterA_1, ShelterA_2 のように、
        # 同じ避難地グループだけ候補にする
        if candidate_group != current_group:
            dprint(
                f"skip candidate={candidate_shelterID}: "
                f"different group. "
                f"candidate_group={candidate_group}, "
                f"current_group={current_group}"
            )
            continue

        # 現在の target_shelterID そのものは除外
        # 例: current_target_shelterID=ShelterA_1,
        #     candidate_shelterID=ShelterA_1
        if candidate_shelterID == current_target_shelterID:
            dprint(
                f"skip candidate={candidate_shelterID}: "
                "same as current target shelter ID"
            )
            continue

        # 現在目的地 edge そのものは除外
        # 例: current_destination_edge=E16,
        #     candidate_edgeID=E16
        if candidate_edgeID == current_destination_edge:
            dprint(
                f"skip candidate because candidate_edgeID is current destination: "
                f"candidate_edgeID={candidate_edgeID}, "
                f"current_destination_edge={current_destination_edge}"
            )
            continue

        try:
            distance = calculate_reroute_distance(
                vehID=vehID,
                from_edgeID=reroute_start_edgeID,
                to_edgeID=candidate_edgeID,
                shelter=candidate_shelter,
                approach_edgeIDs_by_start_edgeID=approach_edgeIDs_by_start_edgeID,
            )
        except Exception as e:
            print(
                f"[majority bias route search error] "
                f"vehID={vehID}, "
                f"from_edgeID={reroute_start_edgeID}, "
                f"to_edgeID={candidate_edgeID}, "
                f"shelterID={candidate_shelterID}, "
                f"error={e}"
            )
            continue

        dprint(
            f"candidate result: "
            f"shelterID={candidate_shelterID}, "
            f"edgeID={candidate_edgeID}, "
            f"distance={distance:.2f}"
        )

        candidate_results_list.append(
            (
                distance,
                candidate_shelterID,
                candidate_edgeID,
            )
        )

    if not candidate_results_list:
        dprint(
            "no alternative candidate in the same shelter group. "
            "keep current target."
        )
        return reroute_start_edgeID, current_target_shelterID, []

    candidate_results_list.sort(key=lambda x: x[0])

    best_distance = candidate_results_list[0][0]
    best_shelterID = candidate_results_list[0][1]
    best_edgeID = candidate_results_list[0][2]

    dprint(
        f"selected: "
        f"vehID={vehID}, "
        f"current_edgeID={current_edgeID}, "
        f"reroute_start_edgeID={reroute_start_edgeID}, "
        f"current_target={current_target_shelterID}, "
        f"current_destination_edge={current_destination_edge}, "
        f"selected_shelter={best_shelterID}, "
        f"selected_edge={best_edgeID}, "
        f"distance={best_distance:.2f}, "
        f"candidates={candidate_results_list}"
    )

    return reroute_start_edgeID, best_shelterID, [best_edgeID]
# def find_uturn_shortest_route_to_current_shelter_group(
#     current_edgeID: str,
#     vehID: str,
#     vehInfo: VehicleInfo,
#     agent: Agent,
#     shelter_list: list[Shelter],
# ) -> tuple[str, str, list]:
#     """
#     同調バイアス用:
#     現在の避難地グループ、例 ShelterA、は維持する。
#     ただし ShelterA_1 / ShelterA_2 のどちらに向かうかは、
#     U-turn後の開始edgeから最短距離になるものを選ぶ。

#     例:
#         現在 target_shelter = ShelterA_1
#         候補:
#             ShelterA_1 -> E16
#             ShelterA_2 -> E13

#         -E41, -E42 から E13 の方が近ければ、
#         ShelterA_2, ["E13"] を返す。
#     """
#     FREE_FLOW_SPEED = 11.0

#     current_target_shelterID = agent.get_target_shelter()
#     current_group = _get_shelter_group(current_target_shelterID)

#     if current_edgeID.startswith(":"):
#         return "", current_target_shelterID, []

#     approach_edgeIDs_by_start_edgeID = vehInfo.get_approach_edge_dict()

#     is_in_approach_list = any(
#         current_edgeID in edge_list
#         for edge_list in approach_edgeIDs_by_start_edgeID.values()
#     )

#     if not is_in_approach_list:
#         reroute_start_edgeID = get_opposite_edgeID_by_edgeID(
#             edgeID=current_edgeID
#         )
#     else:
#         reroute_start_edgeID = current_edgeID

#     candidate_results_list = []

#     for candidate_shelter in shelter_list:
#         candidate_shelterID = candidate_shelter.get_shelterID()
#         candidate_group = _get_shelter_group(candidate_shelterID)

#         # ShelterA_1, ShelterA_2 のように、同じ避難地グループだけ候補にする
#         if candidate_group != current_group:
#             continue

#         candidate_edgeID = candidate_shelter.get_near_edgeID()

#         try:
#             distance = calculate_reroute_distance(
#                 vehID=vehID,
#                 from_edgeID=reroute_start_edgeID,
#                 to_edgeID=candidate_edgeID,
#                 shelter=candidate_shelter,
#                 approach_edgeIDs_by_start_edgeID=approach_edgeIDs_by_start_edgeID,
#             )
#         except Exception as e:
#             print(
#                 f"[majority bias route search error] "
#                 f"vehID={vehID}, "
#                 f"from_edgeID={reroute_start_edgeID}, "
#                 f"to_edgeID={candidate_edgeID}, "
#                 f"shelterID={candidate_shelterID}, "
#                 f"error={e}"
#             )
#             continue

#         candidate_results_list.append(
#             (
#                 distance,
#                 candidate_shelterID,
#                 candidate_edgeID,
#             )
#         )

#     if not candidate_results_list:
#         return reroute_start_edgeID, current_target_shelterID, []

#     candidate_results_list.sort(key=lambda x: x[0])

#     best_distance = candidate_results_list[0][0]
#     best_shelterID = candidate_results_list[0][1]
#     best_edgeID = candidate_results_list[0][2]

#     # print(
#     #     f"[majority bias shortest route] "
#     #     f"vehID={vehID}, "
#     #     f"current_edgeID={current_edgeID}, "
#     #     f"reroute_start_edgeID={reroute_start_edgeID}, "
#     #     f"current_target={current_target_shelterID}, "
#     #     f"selected_shelter={best_shelterID}, "
#     #     f"selected_edge={best_edgeID}, "
#     #     f"distance={best_distance:.2f}, "
#     #     f"candidates={candidate_results_list}"
#     # )

#     return reroute_start_edgeID, best_shelterID, [best_edgeID]


@dataclass(frozen=True)
class CurrentRouteEstimate:
    target_shelter_id: str
    route_tuple: tuple[str, ...]
    destination_edge_id: str
    estimated_time: float

    # SUMOの現在位置から先だけを切り出した実走行予定経路。
    # route_tuple全体ではなく、この値と候補route_edges_for_sumoを比較する。
    remaining_route_tuple: tuple[str, ...] = tuple()

    # traci.vehicle.getRouteIndex()で取得した現在edgeの位置。
    # route内に同じedgeが複数回現れてもtuple.index()の誤選択を避ける。
    current_edge_index: int = -1

    remaining_distance: float = float("inf")
    current_speed: float = FREE_FLOW_SPEED


@dataclass(frozen=True)
class RerouteStartContext:
    approach_edgeIDs_by_start_edgeID: dict
    approach_edge_flg: bool
    base_reroute_start_edgeID: str
    should_return_without_calculation: bool = False


ManeuverType = Literal["forward_reroute", "uturn"]
CandidateSource = Literal[
    "forward_free_flow",
    "v2v_prefix_adjusted",
    "u_turn",
]
TIME_COMPARISON_EPSILON = 1e-6
MIN_ROUTE_EVALUATION_SPEED = 1.8


@dataclass(frozen=True)
class CandidateEvaluation:
    """同じ時間単位（秒）で比較可能な経路変更候補。"""

    best_time: float
    shelter_id: str
    edge_id: str
    from_edge_id: str
    best_source: CandidateSource
    maneuver_type: ManeuverType
    route_edges_for_sumo: tuple[str, ...]

    # 新しい明示フィールド
    time_forward_free_flow: float = float("inf")
    time_v2v_prefix_adjusted: float = float("inf")
    time_u_turn: float = float("inf")
    best_debug_info: Optional[dict] = None

    # 既存コードとの互換フィールド
    time_free_flow: float = float("inf")
    time_v2v_based: float = float("inf")
    best_v2v_debug_info: Optional[dict] = None


@dataclass(frozen=True)
class RouteManeuverDecision:
    current_route: CurrentRouteEstimate
    best_candidate: Optional[CandidateEvaluation]
    candidate_results: tuple[CandidateEvaluation, ...]
    time_gain: float
    route_change_threshold: float
    threshold_exceeded: bool
    apply_normalcy_threshold: bool
    corridor_family: Optional[str]

    @property
    def has_candidate(self) -> bool:
        return self.best_candidate is not None

    @property
    def approved(self) -> bool:
        if self.best_candidate is None:
            return False
        if self.apply_normalcy_threshold:
            return self.threshold_exceeded
        return True

    @property
    def maneuver_type(self) -> Optional[ManeuverType]:
        if self.best_candidate is None:
            return None
        return self.best_candidate.maneuver_type

    def to_legacy_tuple(self) -> tuple[
        str,
        str,
        list[str],
        float,
        bool,
        tuple[str, ...],
    ]:
        """既存の6値インターフェースへ変換する。"""
        if not self.approved or self.best_candidate is None:
            candidate_edges = list(dict.fromkeys(
                candidate.edge_id for candidate in self.candidate_results
            ))
            return "", "", candidate_edges, self.time_gain, False, tuple()

        candidate = self.best_candidate
        return (
            candidate.from_edge_id,
            candidate.shelter_id,
            [candidate.edge_id],
            self.time_gain,
            True,
            candidate.route_edges_for_sumo,
        )


def _make_dprint(debug: bool) -> Callable[[str], None]:
    def dprint(message: str) -> None:
        if debug:
            print(f"[ROUTING_DEBUG] {message}")

    return dprint


def _is_internal_junction_edge(edge_id: str) -> bool:
    return edge_id.startswith(":")


def classify_corridor(current_edge_id: str) -> Optional[str]:
    """現在edgeが属する主要経路ファミリを返す。"""
    for family_name, marker_edges in V2V_ROUTE_FAMILY_MARKERS.items():
        if current_edge_id in marker_edges:
            return family_name
    return None


def _classify_v2v_route_family(route_tuple: tuple[str, ...]) -> str:
    """
    route IDの末尾ではなく、実際に含まれるedge列から分類する。

    正しい対応:
        short_side:  E0_E16_0 / E0_E13_1
        detour_side: E0_E13_0 / E0_E16_1
    """
    route_edges = set(route_tuple)
    match_counts = {
        family_name: len(route_edges & marker_edges)
        for family_name, marker_edges in V2V_ROUTE_FAMILY_MARKERS.items()
    }
    positive_matches = {
        family_name: count
        for family_name, count in match_counts.items()
        if count > 0
    }
    if not positive_matches:
        return ""

    max_count = max(positive_matches.values())
    winners = [
        family_name
        for family_name, count in positive_matches.items()
        if count == max_count
    ]
    if len(winners) != 1:
        return ""
    return winners[0]


def _get_current_speed(
    current_edge_id: str,
    veh_info: "VehicleInfo",
    agent: "Agent",
    dprint: Callable[[str], None],
) -> float:
    try:
        communication_enabled = veh_info.get_vehicle_comm_enabled_flag()
        if communication_enabled:
            current_speed = float(
                traci.edge.getLastStepMeanSpeed(current_edge_id)
            )
        else:
            current_speed = FREE_FLOW_SPEED
    except Exception as exc:
        dprint(
            "failed to obtain current edge speed; "
            f"vehID={agent.get_vehID()}, edge={current_edge_id}, error={exc}"
        )
        current_speed = MIN_ROUTE_EVALUATION_SPEED

    if not math.isfinite(current_speed) or current_speed < MIN_ROUTE_EVALUATION_SPEED:
        current_speed = MIN_ROUTE_EVALUATION_SPEED
    return current_speed


def _extract_current_remaining_route(
    *,
    route_tuple: tuple[str, ...],
    current_edge_id: str,
    route_index: int,
    dprint: Callable[[str], None],
) -> tuple[int, tuple[str, ...]]:
    """
    SUMOの現在位置から先のrouteを返す。

    原則としてtraci.vehicle.getRouteIndex()を使用する。
    route indexが取得できない、または現在edgeと整合しない場合だけ、
    current_edge_idによる検索へフォールバックする。
    """
    if not route_tuple:
        return -1, tuple()

    if 0 <= route_index < len(route_tuple):
        indexed_edge = route_tuple[route_index]
        if indexed_edge == current_edge_id:
            return route_index, tuple(route_tuple[route_index:])

        dprint(
            "route index and current edge disagree; "
            f"route_index={route_index}, "
            f"indexed_edge={indexed_edge}, "
            f"current_edge={current_edge_id}, "
            "fall back to edge search"
        )

    matching_indices = [
        index
        for index, edge_id in enumerate(route_tuple)
        if edge_id == current_edge_id
    ]
    if not matching_indices:
        dprint(
            "current edge is not contained in route; "
            f"current_edge={current_edge_id}, route={route_tuple}"
        )
        return -1, tuple()

    # routeIndexが壊れている場合、同一edgeが複数存在すると完全には特定できない。
    # ただし通常はgetRouteIndex()が利用できるため、これは防御的フォールバック。
    fallback_index = (
        min(matching_indices, key=lambda index: abs(index - route_index))
        if route_index >= 0
        else matching_indices[0]
    )
    dprint(
        "use current-edge fallback for remaining route; "
        f"current_edge={current_edge_id}, "
        f"fallback_index={fallback_index}, "
        f"matching_indices={matching_indices}"
    )
    return fallback_index, tuple(route_tuple[fallback_index:])


def _estimate_current_route_time(
    current_edge_id: str,
    veh_info: "VehicleInfo",
    agent: "Agent",
    shelter: "Shelter",
    dprint: Callable[[str], None],
) -> CurrentRouteEstimate:
    veh_id = agent.get_vehID()

    try:
        current_route_tuple = tuple(traci.vehicle.getRoute(veh_id))
    except Exception as exc:
        dprint(f"failed to get current route: {exc}")
        current_route_tuple = tuple()

    try:
        route_index = int(traci.vehicle.getRouteIndex(veh_id))
    except Exception as exc:
        dprint(f"failed to get current route index: {exc}")
        route_index = -1

    (
        current_edge_index,
        remaining_route_tuple,
    ) = _extract_current_remaining_route(
        route_tuple=current_route_tuple,
        current_edge_id=current_edge_id,
        route_index=route_index,
        dprint=dprint,
    )

    current_target_shelter_id = agent.get_target_shelter()
    current_destination_edge = (
        remaining_route_tuple[-1]
        if remaining_route_tuple
        else (current_route_tuple[-1] if current_route_tuple else "")
    )

    if not current_destination_edge:
        return CurrentRouteEstimate(
            target_shelter_id=current_target_shelter_id,
            route_tuple=current_route_tuple,
            destination_edge_id="",
            estimated_time=float("inf"),
            remaining_route_tuple=remaining_route_tuple,
            current_edge_index=current_edge_index,
        )

    try:
        remaining_distance = float(
            calculate_remaining_route_distance(
                veh_id,
                current_destination_edge,
                shelter=shelter,
            )
        )
    except Exception as exc:
        dprint(f"failed to calculate current remaining distance: {exc}")
        remaining_distance = float("inf")

    current_speed = _get_current_speed(
        current_edge_id=current_edge_id,
        veh_info=veh_info,
        agent=agent,
        dprint=dprint,
    )
    estimated_time = (
        remaining_distance / current_speed
        if math.isfinite(remaining_distance) and remaining_distance >= 0.0
        else float("inf")
    )

    dprint(
        "current route snapshot: "
        f"vehID={veh_id}, "
        f"route_index={current_edge_index}, "
        f"current_edge={current_edge_id}, "
        f"destination={current_destination_edge}, "
        f"remaining_route={remaining_route_tuple}"
    )

    return CurrentRouteEstimate(
        target_shelter_id=current_target_shelter_id,
        route_tuple=current_route_tuple,
        destination_edge_id=current_destination_edge,
        estimated_time=estimated_time,
        remaining_route_tuple=remaining_route_tuple,
        current_edge_index=current_edge_index,
        remaining_distance=remaining_distance,
        current_speed=current_speed,
    )


def _build_reroute_start_context(
    current_edge_id: str,
    veh_info: "VehicleInfo",
    dprint: Callable[[str], None],
) -> RerouteStartContext:
    approach_edge_ids_by_start = veh_info.get_approach_edge_dict()
    is_in_approach_list = any(
        current_edge_id in edge_list
        for edge_list in approach_edge_ids_by_start.values()
    )

    if is_in_approach_list:
        base_start_edge = current_edge_id
        should_return = not veh_info.get_vehicle_comm_enabled_flag()
    else:
        try:
            base_start_edge = get_opposite_edgeID_by_edgeID(
                edgeID=current_edge_id
            )
        except TypeError:
            base_start_edge = get_opposite_edgeID_by_edgeID(current_edge_id)
        should_return = False

    dprint(
        f"approach_edge={is_in_approach_list}, "
        f"base_start_edge={base_start_edge}, should_return={should_return}"
    )
    return RerouteStartContext(
        approach_edgeIDs_by_start_edgeID=approach_edge_ids_by_start,
        approach_edge_flg=is_in_approach_list,
        base_reroute_start_edgeID=base_start_edge or "",
        should_return_without_calculation=should_return,
    )


def _merge_v2v_routes(
    veh_info: "VehicleInfo",
    dprint: Callable[[str], None],
) -> dict[tuple[str, ...], dict]:
    """受信時刻を除き、最新側で上書きしながらroute情報を統合する。"""
    merged: dict[tuple[str, ...], dict] = {}
    raw = veh_info.get_avg_evac_time_by_route_by_recive_time() or {}
    for receive_time in sorted(raw):
        route_dict = raw.get(receive_time) or {}
        for route, info in route_dict.items():
            route_tuple = tuple(route)
            if route_tuple:
                merged[route_tuple] = dict(info or {})
    dprint(f"merged V2V route count={len(merged)}")
    return merged


def _collect_route_options(
    v2v_routes: dict[tuple[str, ...], dict],
    dprint: Callable[[str], None],
) -> dict[tuple[str, ...], dict]:
    """V2V経路にSUMO登録済みrouteを加え、edge列で重複排除する。"""
    route_options = dict(v2v_routes)
    try:
        route_ids = tuple(traci.route.getIDList())
    except Exception as exc:
        dprint(f"failed to list SUMO routes: {exc}")
        route_ids = tuple()

    for route_id in route_ids:
        try:
            route_tuple = tuple(traci.route.getEdges(route_id))
        except Exception as exc:
            dprint(f"failed to read routeID={route_id}: {exc}")
            continue
        if not route_tuple:
            continue
        route_options.setdefault(route_tuple, {"route_id": route_id})
    return route_options


def _is_valid_positive_time(value: float) -> bool:
    return math.isfinite(value) and value > 0.0


def _get_edge_length(edge_id: str) -> float:
    if not edge_id or _is_internal_junction_edge(edge_id):
        return float("inf")
    try:
        lane_count = int(traci.edge.getLaneNumber(edge_id))
        if lane_count <= 0:
            return float("inf")
        return float(traci.lane.getLength(f"{edge_id}_0"))
    except Exception:
        return float("inf")


def _calculate_route_distance_from_edges(
    edge_ids: tuple[str, ...],
    dprint: Callable[[str], None],
) -> float:
    if not edge_ids:
        return 0.0
    total_distance = 0.0
    for edge_id in edge_ids:
        edge_length = _get_edge_length(edge_id)
        if not math.isfinite(edge_length) or edge_length <= 0.0:
            dprint(f"invalid edge length: edge={edge_id}, length={edge_length}")
            return float("inf")
        total_distance += edge_length
    return total_distance


def _remaining_length_on_current_edge(
    veh_id: str,
    current_edge_id: str,
    dprint: Callable[[str], None],
) -> float:
    edge_length = _get_edge_length(current_edge_id)
    if not _is_valid_positive_time(edge_length):
        return float("inf")
    try:
        lane_position = float(traci.vehicle.getLanePosition(veh_id))
    except Exception as exc:
        dprint(f"failed to get lane position; use full edge length: {exc}")
        return edge_length
    return max(0.0, edge_length - lane_position)


def _longest_common_prefix_length(
    route_a: tuple[str, ...],
    route_b: tuple[str, ...],
) -> int:
    length = 0
    for edge_a, edge_b in zip(route_a, route_b):
        if edge_a != edge_b:
            break
        length += 1
    return length


def _route_starts_with_prefix(
    route_tuple: tuple[str, ...],
    prefix: tuple[str, ...],
) -> bool:
    return tuple(route_tuple[:len(prefix)]) == prefix


def _has_only_initial_common_prefix(
    current_route: tuple[str, ...],
    alternative_route: tuple[str, ...],
) -> bool:
    lcp_length = _longest_common_prefix_length(
        current_route,
        alternative_route,
    )
    return (
        _route_starts_with_prefix(
            current_route,
            V2V_PREFIX_INITIAL_COMMON_PREFIX,
        )
        and _route_starts_with_prefix(
            alternative_route,
            V2V_PREFIX_INITIAL_COMMON_PREFIX,
        )
        and lcp_length <= len(V2V_PREFIX_INITIAL_COMMON_PREFIX)
    )


def _validate_route_edge_sequence(
    route_edges: tuple[str, ...],
    dprint: Callable[[str], None],
) -> bool:
    """junction接続による事前検証。最終的な可否はsetRouteでも検証する。"""
    if not route_edges:
        return False
    for from_edge, to_edge in zip(route_edges, route_edges[1:]):
        try:
            if (
                traci.edge.getToJunction(from_edge)
                != traci.edge.getFromJunction(to_edge)
            ):
                dprint(
                    "route connection mismatch: "
                    f"from={from_edge}, to={to_edge}"
                )
                return False
        except Exception as exc:
            dprint(
                "route connection validation unavailable; "
                f"from={from_edge}, to={to_edge}, error={exc}"
            )
            # 検証API失敗だけでは候補を捨てず、setRouteの例外捕捉へ委ねる。
            return True
    return True


def _time_from_current_to_divergence(
    current_edge_id: str,
    divergence_index: int,
    current_route: CurrentRouteEstimate,
    agent: "Agent",
    dprint: Callable[[str], None],
) -> tuple[float, float, tuple[str, ...]]:
    current_index = current_route.current_edge_index
    if (
        current_index < 0
        or current_index >= len(current_route.route_tuple)
        or current_route.route_tuple[current_index] != current_edge_id
    ):
        return float("inf"), float("inf"), tuple()

    if divergence_index < current_index:
        return float("inf"), float("inf"), tuple()
    if divergence_index == current_index:
        return 0.0, 0.0, tuple()

    edges = tuple(
        current_route.route_tuple[current_index:divergence_index + 1]
    )
    current_remaining = _remaining_length_on_current_edge(
        agent.get_vehID(),
        current_edge_id,
        dprint,
    )
    following_distance = _calculate_route_distance_from_edges(
        tuple(edges[1:]),
        dprint,
    )
    if not math.isfinite(current_remaining) or not math.isfinite(following_distance):
        return float("inf"), float("inf"), edges

    distance = current_remaining + following_distance
    time_value = distance / current_route.current_speed
    return time_value, distance, edges


def _evaluate_forward_route(
    *,
    current_edge_id: str,
    current_route: CurrentRouteEstimate,
    alternative_route: tuple[str, ...],
    route_info: dict,
    candidate_shelter_id: str,
    candidate_edge_id: str,
    agent: "Agent",
    dprint: Callable[[str], None],
) -> Optional[CandidateEvaluation]:
    if not alternative_route or alternative_route == current_route.route_tuple:
        return None
    if alternative_route[-1] != candidate_edge_id:
        return None
    if alternative_route[-1] == current_route.destination_edge_id:
        return None
    if not current_route.route_tuple:
        return None
    if not current_route.remaining_route_tuple:
        return None

    current_family = _classify_v2v_route_family(current_route.route_tuple)
    alternative_family = _classify_v2v_route_family(alternative_route)
    if not current_family or current_family != alternative_family:
        return None
    if _has_only_initial_common_prefix(
        current_route.route_tuple,
        alternative_route,
    ):
        return None

    common_prefix_length = _longest_common_prefix_length(
        current_route.route_tuple,
        alternative_route,
    )
    if common_prefix_length <= 0:
        return None
    common_prefix = tuple(alternative_route[:common_prefix_length])
    if current_edge_id not in common_prefix:
        return None

    divergence_index = common_prefix_length - 1
    current_index = current_route.current_edge_index
    if (
        current_index < 0
        or current_index >= len(current_route.route_tuple)
        or current_route.route_tuple[current_index] != current_edge_id
    ):
        return None
    if divergence_index < current_index:
        return None

    route_edges_for_sumo = tuple(alternative_route[current_index:])

    # 重要:
    # route_tuple全体ではなく、現在位置から先の実走行予定経路と比較する。
    # 通過済みprefixが異なるだけの「見かけ上の変更」を候補にしない。
    if route_edges_for_sumo == current_route.remaining_route_tuple:
        dprint(
            "skip forward candidate because it equals current remaining route: "
            f"current_edge={current_edge_id}, "
            f"candidate_shelter={candidate_shelter_id}, "
            f"candidate_edge={candidate_edge_id}, "
            f"remaining_route={current_route.remaining_route_tuple}"
        )
        return None

    if (
        not route_edges_for_sumo
        or route_edges_for_sumo[0] != current_edge_id
        or route_edges_for_sumo[-1] != candidate_edge_id
        or not _validate_route_edge_sequence(route_edges_for_sumo, dprint)
    ):
        return None

    (
        time_to_divergence,
        distance_to_divergence,
        edges_to_divergence,
    ) = _time_from_current_to_divergence(
        current_edge_id=current_edge_id,
        divergence_index=divergence_index,
        current_route=current_route,
        agent=agent,
        dprint=dprint,
    )
    if not math.isfinite(time_to_divergence) or time_to_divergence < 0.0:
        return None

    edges_after_divergence = tuple(
        alternative_route[divergence_index + 1:]
    )
    distance_after_divergence = _calculate_route_distance_from_edges(
        edges_after_divergence,
        dprint,
    )
    if not _is_valid_positive_time(distance_after_divergence):
        return None

    time_after_divergence_free_flow = (
        distance_after_divergence / FREE_FLOW_SPEED
    )
    forward_free_flow_time = (
        time_to_divergence + time_after_divergence_free_flow
    )

    alternative_avg_time = route_info.get("avg_time")
    v2v_prefix_adjusted_time = float("inf")
    v2v_time_after_divergence_raw = float("inf")
    alternative_total_distance = _calculate_route_distance_from_edges(
        alternative_route,
        dprint,
    )
    if (
        isinstance(alternative_avg_time, (int, float))
        and _is_valid_positive_time(float(alternative_avg_time))
        and _is_valid_positive_time(alternative_total_distance)
    ):
        v2v_time_after_divergence_raw = (
            float(alternative_avg_time)
            * distance_after_divergence
            / alternative_total_distance
        )
        v2v_time_after_divergence = max(
            v2v_time_after_divergence_raw,
            time_after_divergence_free_flow,
        )
        v2v_prefix_adjusted_time = (
            time_to_divergence + v2v_time_after_divergence
        )

    if (
        _is_valid_positive_time(v2v_prefix_adjusted_time)
        and v2v_prefix_adjusted_time
        < forward_free_flow_time - TIME_COMPARISON_EPSILON
    ):
        best_time = v2v_prefix_adjusted_time
        best_source: CandidateSource = "v2v_prefix_adjusted"
    else:
        best_time = forward_free_flow_time
        best_source = "forward_free_flow"

    if not _is_valid_positive_time(best_time):
        return None

    debug_info = {
        "source": best_source,
        "current_route": current_route.route_tuple,
        "current_remaining_route": current_route.remaining_route_tuple,
        "current_route_index": current_route.current_edge_index,
        "alternative_route": alternative_route,
        "forward_route": route_edges_for_sumo,
        "forward_route_valid": True,
        "current_family": current_family,
        "alternative_family": alternative_family,
        "common_prefix": common_prefix,
        "common_prefix_length": common_prefix_length,
        "divergence_edge_index": divergence_index,
        "divergence_edge_id": alternative_route[divergence_index],
        "current_edge_index": current_index,
        "edges_from_current_to_divergence": edges_to_divergence,
        "distance_from_current_to_divergence": distance_to_divergence,
        "time_from_current_to_divergence": time_to_divergence,
        "alternative_edges_after_divergence": edges_after_divergence,
        "alternative_distance_after_divergence": distance_after_divergence,
        "alternative_total_distance": alternative_total_distance,
        "alternative_avg_time": alternative_avg_time,
        "v2v_time_after_divergence_raw": v2v_time_after_divergence_raw,
        "forward_free_flow_time": forward_free_flow_time,
        "v2v_prefix_adjusted_time": v2v_prefix_adjusted_time,
        "route_edges_for_sumo": route_edges_for_sumo,
    }
    return CandidateEvaluation(
        best_time=best_time,
        shelter_id=candidate_shelter_id,
        edge_id=candidate_edge_id,
        from_edge_id=current_edge_id,
        best_source=best_source,
        maneuver_type="forward_reroute",
        route_edges_for_sumo=route_edges_for_sumo,
        time_forward_free_flow=forward_free_flow_time,
        time_v2v_prefix_adjusted=v2v_prefix_adjusted_time,
        time_u_turn=float("inf"),
        best_debug_info=debug_info,
        time_free_flow=forward_free_flow_time,
        time_v2v_based=v2v_prefix_adjusted_time,
        best_v2v_debug_info=(
            debug_info if best_source == "v2v_prefix_adjusted" else None
        ),
    )


def _evaluate_best_forward_candidate(
    *,
    current_edge_id: str,
    current_route: CurrentRouteEstimate,
    route_options: dict[tuple[str, ...], dict],
    candidate_shelter_id: str,
    candidate_edge_id: str,
    agent: "Agent",
    dprint: Callable[[str], None],
) -> Optional[CandidateEvaluation]:
    candidates: list[CandidateEvaluation] = []
    for route_tuple, route_info in route_options.items():
        candidate = _evaluate_forward_route(
            current_edge_id=current_edge_id,
            current_route=current_route,
            alternative_route=tuple(route_tuple),
            route_info=route_info or {},
            candidate_shelter_id=candidate_shelter_id,
            candidate_edge_id=candidate_edge_id,
            agent=agent,
            dprint=dprint,
        )
        if candidate is not None:
            candidates.append(candidate)
    if not candidates:
        return None
    return _sort_candidates(candidates)[0]


def _evaluate_u_turn_candidate(
    *,
    current_edge_id: str,
    candidate_shelter_id: str,
    candidate_edge_id: str,
    dprint: Callable[[str], None],
) -> Optional[CandidateEvaluation]:
    try:
        u_turn_start_edge = get_opposite_edgeID_by_edgeID(
            edgeID=current_edge_id
        )
    except TypeError:
        u_turn_start_edge = get_opposite_edgeID_by_edgeID(current_edge_id)
    if not u_turn_start_edge:
        return None

    try:
        route = traci.simulation.findRoute(
            u_turn_start_edge,
            candidate_edge_id,
        )
    except Exception as exc:
        dprint(
            "U-turn findRoute failed: "
            f"from={u_turn_start_edge}, to={candidate_edge_id}, error={exc}"
        )
        return None

    route_edges = tuple(getattr(route, "edges", tuple()))
    if not route_edges:
        return None
    route_distance = float(getattr(route, "length", 0.0) or 0.0)
    if not _is_valid_positive_time(route_distance):
        route_distance = _calculate_route_distance_from_edges(
            route_edges,
            dprint,
        )
    if not _is_valid_positive_time(route_distance):
        return None

    u_turn_time = route_distance / FREE_FLOW_SPEED
    if not _is_valid_positive_time(u_turn_time):
        return None

    debug_info = {
        "source": "u_turn",
        "u_turn_start_edge": u_turn_start_edge,
        "u_turn_route": route_edges,
        "u_turn_route_distance": route_distance,
        "u_turn_time": u_turn_time,
    }
    return CandidateEvaluation(
        best_time=u_turn_time,
        shelter_id=candidate_shelter_id,
        edge_id=candidate_edge_id,
        from_edge_id=u_turn_start_edge,
        best_source="u_turn",
        maneuver_type="uturn",
        route_edges_for_sumo=tuple(),
        time_forward_free_flow=float("inf"),
        time_v2v_prefix_adjusted=float("inf"),
        time_u_turn=u_turn_time,
        best_debug_info=debug_info,
        time_free_flow=float("inf"),
        time_v2v_based=float("inf"),
        best_v2v_debug_info=None,
    )


def _candidate_compare(
    left: CandidateEvaluation,
    right: CandidateEvaluation,
) -> int:
    delta = left.best_time - right.best_time
    if abs(delta) > TIME_COMPARISON_EPSILON:
        return -1 if delta < 0 else 1

    maneuver_priority = {
        "forward_reroute": 0,
        "uturn": 1,
    }
    left_key = (
        maneuver_priority[left.maneuver_type],
        left.shelter_id,
        left.edge_id,
        left.best_source,
        left.route_edges_for_sumo,
    )
    right_key = (
        maneuver_priority[right.maneuver_type],
        right.shelter_id,
        right.edge_id,
        right.best_source,
        right.route_edges_for_sumo,
    )
    return (left_key > right_key) - (left_key < right_key)


def _sort_candidates(
    candidates: list[CandidateEvaluation],
) -> list[CandidateEvaluation]:
    from functools import cmp_to_key

    usable = [
        candidate
        for candidate in candidates
        if _is_valid_positive_time(candidate.best_time)
    ]
    return sorted(usable, key=cmp_to_key(_candidate_compare))


def _is_same_forward_candidate_as_current_route(
    *,
    candidate: CandidateEvaluation,
    current_route: CurrentRouteEstimate,
) -> bool:
    """現在位置から先の経路が完全一致するforward候補かを判定する。"""
    return (
        candidate.maneuver_type == "forward_reroute"
        and bool(candidate.route_edges_for_sumo)
        and candidate.route_edges_for_sumo
        == current_route.remaining_route_tuple
    )


def _evaluate_all_candidates(
    *,
    current_edge_id: str,
    agent: "Agent",
    current_route: CurrentRouteEstimate,
    route_options: dict[tuple[str, ...], dict],
    dprint: Callable[[str], None],
) -> list[CandidateEvaluation]:
    results: list[CandidateEvaluation] = []
    candidate_map = agent.get_candidate_edge_by_shelterID() or {}

    for candidate_shelter_id, candidate_edge_id in candidate_map.items():
        if candidate_shelter_id == current_route.target_shelter_id:
            continue
        if candidate_edge_id == current_route.destination_edge_id:
            continue
        if not candidate_edge_id:
            continue

        forward = _evaluate_best_forward_candidate(
            current_edge_id=current_edge_id,
            current_route=current_route,
            route_options=route_options,
            candidate_shelter_id=candidate_shelter_id,
            candidate_edge_id=candidate_edge_id,
            agent=agent,
            dprint=dprint,
        )
        if forward is not None:
            if _is_same_forward_candidate_as_current_route(
                candidate=forward,
                current_route=current_route,
            ):
                dprint(
                    "discard forward candidate equal to current remaining route: "
                    f"shelter={forward.shelter_id}, "
                    f"edge={forward.edge_id}, "
                    f"route={forward.route_edges_for_sumo}"
                )
            else:
                results.append(forward)

        u_turn = _evaluate_u_turn_candidate(
            current_edge_id=current_edge_id,
            candidate_shelter_id=candidate_shelter_id,
            candidate_edge_id=candidate_edge_id,
            dprint=dprint,
        )
        if u_turn is not None:
            results.append(u_turn)

    return _sort_candidates(results)


def _debug_route_maneuver_comparison(
    *,
    debug: bool,
    logical_vehicle_id: str,
    current_edge_id: str,
    current_route: CurrentRouteEstimate,
    corridor_family: Optional[str],
    candidates: list[CandidateEvaluation],
    selected: Optional[CandidateEvaluation],
    time_gain: float,
    threshold: float,
) -> None:
    if not debug:
        return

    selected_edge = selected.edge_id if selected else ""
    same_destination = [
        candidate
        for candidate in candidates
        if candidate.edge_id == selected_edge
    ]
    forward = next(
        (
            candidate
            for candidate in same_destination
            if candidate.maneuver_type == "forward_reroute"
        ),
        None,
    )
    u_turn = next(
        (
            candidate
            for candidate in same_destination
            if candidate.maneuver_type == "uturn"
        ),
        None,
    )

    try:
        current_time = float(traci.simulation.getTime())
    except Exception:
        current_time = float("nan")

    forward_debug = forward.best_debug_info if forward else {}
    u_turn_debug = u_turn.best_debug_info if u_turn else {}
    payload = {
        "vehID": logical_vehicle_id,
        "logical_vehicle_id": logical_vehicle_id,
        "current_time": current_time,
        "current_edgeID": current_edge_id,
        "corridor_family": corridor_family,
        "current_route": current_route.route_tuple,
        "current_remaining_route": current_route.remaining_route_tuple,
        "current_route_index": current_route.current_edge_index,
        "current_destination_edge": current_route.destination_edge_id,
        "current_route_estimated_time": current_route.estimated_time,
        "candidate_shelter_id": selected.shelter_id if selected else "",
        "candidate_edge_id": selected_edge,
        "forward_route": (
            forward.route_edges_for_sumo if forward else tuple()
        ),
        "forward_route_valid": forward is not None,
        "forward_free_flow_time": (
            forward.time_forward_free_flow if forward else float("inf")
        ),
        "v2v_prefix_adjusted_time": (
            forward.time_v2v_prefix_adjusted if forward else float("inf")
        ),
        "u_turn_start_edge": u_turn_debug.get("u_turn_start_edge", ""),
        "u_turn_route": u_turn_debug.get("u_turn_route", tuple()),
        "u_turn_time": u_turn.time_u_turn if u_turn else float("inf"),
        "selected_source": selected.best_source if selected else "",
        "selected_maneuver_type": (
            selected.maneuver_type if selected else ""
        ),
        "selected_time": selected.best_time if selected else float("inf"),
        "time_gain": time_gain,
        "route_change_threshold": threshold,
        "route_edges_for_sumo": (
            selected.route_edges_for_sumo if selected else tuple()
        ),
        "forward_debug": forward_debug,
    }
    print("[ROUTE_MANEUVER_COMPARISON]", payload)


def evaluate_route_maneuver_decision(
    current_edgeID: str,
    vehInfo: "VehicleInfo",
    agent: "Agent",
    shelter: "Shelter",
    shelter_list: list,
    custome_edge_list: list,
    *,
    apply_normalcy_threshold: bool,
    debug: bool = False,
    logical_vehicle_id: Optional[str] = None,
) -> RouteManeuverDecision:
    """
    順方向変更とUターンを同じ秒単位で評価する。

    apply_normalcy_threshold=True:
        time_gain > 個人閾値の場合だけapproved。

    apply_normalcy_threshold=False:
        同調性ですでに変更意思が成立しているため、最短の実行可能候補を
        thresholdなしでapprovedとする。
    """
    del shelter_list, custome_edge_list  # 既存シグネチャ互換
    dprint = _make_dprint(debug)
    logical_id = logical_vehicle_id or agent.get_vehID()

    empty_current_route = CurrentRouteEstimate(
        target_shelter_id=agent.get_target_shelter(),
        route_tuple=tuple(),
        destination_edge_id="",
        estimated_time=float("inf"),
        remaining_route_tuple=tuple(),
        current_edge_index=-1,
    )
    if _is_internal_junction_edge(current_edgeID):
        return RouteManeuverDecision(
            current_route=empty_current_route,
            best_candidate=None,
            candidate_results=tuple(),
            time_gain=0.0,
            route_change_threshold=agent.get_route_change_threshold(),
            threshold_exceeded=False,
            apply_normalcy_threshold=apply_normalcy_threshold,
            corridor_family=None,
        )

    current_route = _estimate_current_route_time(
        current_edge_id=current_edgeID,
        veh_info=vehInfo,
        agent=agent,
        shelter=shelter,
        dprint=dprint,
    )
    reroute_context = _build_reroute_start_context(
        current_edge_id=current_edgeID,
        veh_info=vehInfo,
        dprint=dprint,
    )
    if reroute_context.should_return_without_calculation:
        return RouteManeuverDecision(
            current_route=current_route,
            best_candidate=None,
            candidate_results=tuple(),
            time_gain=0.0,
            route_change_threshold=agent.get_route_change_threshold(),
            threshold_exceeded=False,
            apply_normalcy_threshold=apply_normalcy_threshold,
            corridor_family=classify_corridor(current_edgeID),
        )

    v2v_routes = _merge_v2v_routes(vehInfo, dprint)
    route_options = _collect_route_options(v2v_routes, dprint)
    candidates = _evaluate_all_candidates(
        current_edge_id=current_edgeID,
        agent=agent,
        current_route=current_route,
        route_options=route_options,
        dprint=dprint,
    )

    # 最終防御:
    # 下位関数の変更や外部候補生成があっても、現在の残り経路と同一の
    # forward_rerouteを最終選択へ進めない。
    candidate_count_before_same_route_filter = len(candidates)
    candidates = [
        candidate
        for candidate in candidates
        if not _is_same_forward_candidate_as_current_route(
            candidate=candidate,
            current_route=current_route,
        )
    ]
    removed_same_route_count = (
        candidate_count_before_same_route_filter - len(candidates)
    )
    if removed_same_route_count > 0:
        dprint(
            "removed candidates equal to current remaining route: "
            f"count={removed_same_route_count}, "
            f"remaining_route={current_route.remaining_route_tuple}"
        )

    candidates = _sort_candidates(candidates)

    best_candidate = candidates[0] if candidates else None
    if (
        best_candidate is not None
        and math.isfinite(current_route.estimated_time)
    ):
        time_gain = current_route.estimated_time - best_candidate.best_time
    else:
        time_gain = 0.0

    threshold = float(agent.get_route_change_threshold())
    threshold_exceeded = bool(
        best_candidate is not None
        and math.isfinite(time_gain)
        and time_gain > threshold
    )
    corridor_family = classify_corridor(current_edgeID)

    _debug_route_maneuver_comparison(
        debug=debug,
        logical_vehicle_id=logical_id,
        current_edge_id=current_edgeID,
        current_route=current_route,
        corridor_family=corridor_family,
        candidates=candidates,
        selected=best_candidate,
        time_gain=time_gain,
        threshold=threshold,
    )

    return RouteManeuverDecision(
        current_route=current_route,
        best_candidate=best_candidate,
        candidate_results=tuple(candidates),
        time_gain=time_gain,
        route_change_threshold=threshold,
        threshold_exceeded=threshold_exceeded,
        apply_normalcy_threshold=apply_normalcy_threshold,
        corridor_family=corridor_family,
    )


def find_alternative_better_choice_fixed_divide(
    current_edgeID: str,
    vehInfo: "VehicleInfo",
    agent: "Agent",
    shelter: "Shelter",
    shelter_list: list,
    custome_edge_list: list,
    debug: bool = False,
):
    """正常性バイアス用。既存の6値形式を維持する。"""
    decision = evaluate_route_maneuver_decision(
        current_edgeID=current_edgeID,
        vehInfo=vehInfo,
        agent=agent,
        shelter=shelter,
        shelter_list=shelter_list,
        custome_edge_list=custome_edge_list,
        apply_normalcy_threshold=True,
        debug=debug,
    )
    return decision.to_legacy_tuple()


def get_best_route_maneuver_without_threshold(
    current_edgeID: str,
    agent_by_target_vehID: "Agent",
    vehInfo_by_target_vehID: "VehicleInfo",
    shelter: "Shelter",
    shelter_list: list["Shelter"],
    custome_edge_list: list,
    debug: bool = False,
):
    """同調性バイアス用。個人の正常性閾値を再適用しない。"""
    decision = evaluate_route_maneuver_decision(
        current_edgeID=current_edgeID,
        vehInfo=vehInfo_by_target_vehID,
        agent=agent_by_target_vehID,
        shelter=shelter,
        shelter_list=shelter_list,
        custome_edge_list=custome_edge_list,
        apply_normalcy_threshold=False,
        debug=debug,
    )
    return decision.to_legacy_tuple()
