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
from typing import Any, Callable, Optional


from evacsim.sim.sumo_env import ensure_sumo_tools_on_path

ensure_sumo_tools_on_path()

import traci

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
#         print(f"Error getting speed for {agent.get_vehID()}: {e}")
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
    print(f"current_target_shelterID: {current_target_shelterID}, shelter_for_vehInfo: {shelter_for_vehInfo.get_shelterID()}")
    current_route_tuple = tuple(traci.vehicle.getRoute(agent.get_vehID()))
    print(f"current_route_tuple: {current_route_tuple}")
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
        print("test5")
        route = traci.simulation.findRoute(
            base_reroute_start_edgeID,
            best_candidate_edgeID
        )
        print("test6")

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
    if congestion_flg:
        print(f"base_reroute_start_edgeID: {base_reroute_start_edgeID}, shelterID: {shelterID}, to_edge_list: {to_edge_list}, time_gain: {time_gain:.3f}, congestion_flg: {congestion_flg}")
    return base_reroute_start_edgeID, to_edge_list, congestion_flg

def get_route_time_difference_exceeding_threshold(current_edgeID, agent_by_target_vehID: Agent, vehInfo_by_target_vehID: VehicleInfo, shelter: Shelter, shelter_list: list[Shelter], custome_edge_list: list):
    base_reroute_start_edgeID, shelterID, to_edge_list, time_gain, congestion_flg = find_alternative_better_choice_fixed_divide(
                                                                                                            current_edgeID=current_edgeID,
                                                                                                            vehInfo=vehInfo_by_target_vehID,
                                                                                                            agent=agent_by_target_vehID,
                                                                                                            shelter=shelter,
                                                                                                            shelter_list=shelter_list,
                                                                                                            custome_edge_list=custome_edge_list,
                                                                                                            debug=False
                                                                                                            )
    if congestion_flg:
        print(f"base_reroute_start_edgeID: {base_reroute_start_edgeID}, shelterID: {shelterID}, to_edge_list: {to_edge_list}, time_gain: {time_gain:.3f}, congestion_flg: {congestion_flg}")
    return base_reroute_start_edgeID, shelterID, to_edge_list, congestion_flg


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
    route_tuple: tuple
    destination_edge_id: str
    estimated_time: float


@dataclass(frozen=True)
class RerouteStartContext:
    approach_edgeIDs_by_start_edgeID: dict
    approach_edge_flg: bool
    base_reroute_start_edgeID: str
    should_return_without_calculation: bool = False


@dataclass(frozen=True)
class CandidateEvaluation:
    best_time: float
    shelter_id: str
    edge_id: str
    from_edge_id: str
    best_source: str
    time_free_flow: float
    time_v2v_based: float = float("inf")
    best_v2v_debug_info: Optional[dict] = None
    time_v2v_prefix_adjusted: float = float("inf")
    time_u_turn: float = float("inf")
    best_debug_info: Optional[dict] = None


def _make_dprint(debug: bool) -> Callable[[str], None]:
    def dprint(message: str) -> None:
        if debug:
            print(f"[find_alternative_better_choice] {message}")

    return dprint


def _is_internal_junction_edge(edgeID: str) -> bool:
    return edgeID.startswith(":")


def _get_current_speed(
    current_edgeID: str,
    vehInfo: "VehicleInfo",
    agent: "Agent",
    dprint: Callable[[str], None],
) -> float:
    try:
        if vehInfo.get_vehicle_comm_enabled_flag():
            current_speed = traci.edge.getLastStepMeanSpeed(current_edgeID)
        else:
            current_speed = FREE_FLOW_SPEED

        if current_speed < 1.8:
            dprint(
                f"current_speed is too low: {current_speed:.3f}. "
                "clamped to 1.8"
            )
            current_speed = 1.8

        return current_speed

    except traci.TraCIException as e:
        dprint(f"failed to get speed for {agent.get_vehID()}: {e}. use 1.8")
        return 1.8


def _estimate_current_route_time(
    current_edgeID: str,
    vehInfo: "VehicleInfo",
    agent: "Agent",
    shelter: "Shelter",
    dprint: Callable[[str], None],
) -> CurrentRouteEstimate:
    current_target_shelterID = agent.get_target_shelter()
    current_route_tuple = tuple(traci.vehicle.getRoute(agent.get_vehID()))
    current_destination_edge = current_route_tuple[-1]

    distance_to_current_shelter = calculate_remaining_route_distance(
        agent.get_vehID(),
        current_destination_edge,
        shelter=shelter,
    )

    current_speed = _get_current_speed(
        current_edgeID=current_edgeID,
        vehInfo=vehInfo,
        agent=agent,
        dprint=dprint,
    )

    estimated_current_time = distance_to_current_shelter / current_speed

    dprint(
        "current route: "
        f"target_shelter={current_target_shelterID}, "
        f"destination_edge={current_destination_edge}, "
        f"distance_to_current_shelter={distance_to_current_shelter:.3f}, "
        f"current_speed={current_speed:.3f}, "
        f"estimated_current_time={estimated_current_time:.3f}"
    )

    return CurrentRouteEstimate(
        target_shelter_id=current_target_shelterID,
        route_tuple=current_route_tuple,
        destination_edge_id=current_destination_edge,
        estimated_time=estimated_current_time,
    )


def _build_reroute_start_context(
    current_edgeID: str,
    vehInfo: "VehicleInfo",
    dprint: Callable[[str], None],
) -> RerouteStartContext:
    approach_edgeIDs_by_start_edgeID = vehInfo.get_approach_edge_dict()

    is_in_approach_list = any(
        current_edgeID in edge_list
        for edge_list in approach_edgeIDs_by_start_edgeID.values()
    )

    if not is_in_approach_list:
        base_reroute_start_edgeID = get_opposite_edgeID_by_edgeID(
            edgeID=current_edgeID
        )
        approach_edge_flg = False
        should_return_without_calculation = False
    else:
        base_reroute_start_edgeID = current_edgeID
        approach_edge_flg = True
        should_return_without_calculation = not vehInfo.get_vehicle_comm_enabled_flag()

    dprint(
        f"is_in_approach_list={is_in_approach_list}, "
        f"approach_edge_flg={approach_edge_flg}, "
        f"base_reroute_start_edgeID={base_reroute_start_edgeID}, "
        f"should_return_without_calculation={should_return_without_calculation}"
    )

    return RerouteStartContext(
        approach_edgeIDs_by_start_edgeID=approach_edgeIDs_by_start_edgeID,
        approach_edge_flg=approach_edge_flg,
        base_reroute_start_edgeID=base_reroute_start_edgeID,
        should_return_without_calculation=should_return_without_calculation,
    )


def _merge_v2v_routes(
    vehInfo: "VehicleInfo",
    dprint: Callable[[str], None],
) -> dict:
    """
    vehInfo.get_avg_evac_time_by_route_by_recive_time() は
    {receive_time: {route_tuple: route_info}} の構造なので，
    receive_time は計算に使わず，内側の route 情報だけを統合する。
    """
    avg_evac_time_data = vehInfo.get_avg_evac_time_by_route_by_recive_time()
    routes_dict = {}

    if not avg_evac_time_data:
        dprint("V2V info is empty")
        return routes_dict

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

    dprint(f"total merged V2V routes={len(routes_dict)}")
    return routes_dict


def _get_current_route_name(current_edgeID: str, vehInfo: "VehicleInfo") -> str:
    edgeIDs_within_junction_to_shelter_dict = (
        vehInfo.get_edgeIDs_within_junction_to_shelter_dict()
    )

    route_name = find_route_name_by_edge(
        edgeID=current_edgeID,
        routes_dict=edgeIDs_within_junction_to_shelter_dict,
    )

    return route_name or ""


def _should_skip_candidate(
    candidate_shelterID: str,
    candidate_edgeID: str,
    current_route: CurrentRouteEstimate,
    dprint: Callable[[str], None],
) -> bool:
    if candidate_shelterID == current_route.target_shelter_id:
        dprint(
            f"skip candidate_shelter={candidate_shelterID}: "
            "same as current target shelter ID"
        )
        return True

    if candidate_edgeID == current_route.destination_edge_id:
        dprint(
            "skip candidate because candidate_edgeID is current destination: "
            f"candidate_edgeID={candidate_edgeID}, "
            f"current_destination_edge={current_route.destination_edge_id}"
        )
        return True

    return False


def _resolve_candidate_shelter(
    candidate_shelterID: str,
    candidate_edgeID: str,
    shelter_list: list,
    dprint: Callable[[str], None],
) -> Optional["Shelter"]:
    candidate_shelter = find_shelter_by_edgeID_connect_target_shelter(
        edgeID=candidate_edgeID,
        shelter_list=shelter_list,
    )

    if not candidate_shelter:
        dprint(
            f"skip candidate_shelter={candidate_shelterID}: "
            f"candidate_edgeID={candidate_edgeID}, shelter not found"
        )
        return None

    return candidate_shelter


def _select_base_start_edge_for_candidate(
    current_edgeID: str,
    base_reroute_start_edgeID: str,
    route_name: str,
    current_group: str,
    candidate_group: str,
) -> str:
    """
    既存の迂回開始 edge 補正ロジックをそのまま隔離する。
    """
    edge_to_search_from = base_reroute_start_edgeID

    if route_name.startswith("intermediate"):
        if current_group == candidate_group:
            edge_to_search_from = current_edgeID

    elif route_name.startswith("ShelterA"):
        if route_name.endswith("_opposite"):
            if current_group == candidate_group:
                edge_to_search_from = get_opposite_edgeID_by_edgeID(
                    edgeID=current_edgeID
                )
            else:
                edge_to_search_from = current_edgeID

    elif route_name.startswith("ShelterB"):
        if route_name.endswith("_opposite"):
            if current_group == candidate_group:
                edge_to_search_from = current_edgeID
            else:
                edge_to_search_from = get_opposite_edgeID_by_edgeID(
                    edgeID=current_edgeID
                )

    # 既存の特殊ケースを維持
    if current_edgeID in ["E20", "E1"]:
        edge_to_search_from = current_edgeID

    return edge_to_search_from


def _unique_non_empty(values: list[str]) -> list[str]:
    result = []
    for value in values:
        if value and value not in result:
            result.append(value)
    return result


def _get_start_edge_options_for_candidate(
    current_edgeID: str,
    base_start_edge_for_candidate: str,
) -> list[str]:
    """
    自由速度ベース評価に使う開始 edge 候補を返す。

    以前は特定 edge ID に依存して current_edgeID を追加していたが，
    順方向分岐候補は V2V prefix 補正で，Uターン候補は
    _estimate_u_turn_reroute_time() でそれぞれ一般的に評価する。
    そのため，ここでは既存の base_start_edge_for_candidate のみを返す。
    """
    start_edge_options = _unique_non_empty([base_start_edge_for_candidate])
    if start_edge_options:
        return start_edge_options

    return [current_edgeID]


def _safe_calculate_reroute_distance(
    vehID: str,
    from_edgeID: str,
    to_edgeID: str,
    shelter: "Shelter",
    approach_edgeIDs_by_start_edgeID: dict,
    dprint: Callable[[str], None],
) -> float:
    try:
        return calculate_reroute_distance(
            vehID=vehID,
            from_edgeID=from_edgeID,
            to_edgeID=to_edgeID,
            shelter=shelter,
            approach_edgeIDs_by_start_edgeID=approach_edgeIDs_by_start_edgeID,
        )
    except traci.TraCIException as e:
        dprint(
            "skip route because calculate_reroute_distance failed: "
            f"from_edgeID={from_edgeID}, to_edgeID={to_edgeID}, error={e}"
        )
        return float("inf")


def _estimate_free_flow_time(
    agent: "Agent",
    from_edgeID: str,
    to_edgeID: str,
    shelter: "Shelter",
    approach_edgeIDs_by_start_edgeID: dict,
    dprint: Callable[[str], None],
) -> tuple[float, float]:
    distance_free_flow = _safe_calculate_reroute_distance(
        vehID=agent.get_vehID(),
        from_edgeID=from_edgeID,
        to_edgeID=to_edgeID,
        shelter=shelter,
        approach_edgeIDs_by_start_edgeID=approach_edgeIDs_by_start_edgeID,
        dprint=dprint,
    )

    return distance_free_flow, distance_free_flow / FREE_FLOW_SPEED


def _longest_common_prefix_length(route_a: tuple, route_b: tuple) -> int:
    lcp_length = 0
    min_len = min(len(route_a), len(route_b))

    for i in range(min_len):
        if route_a[i] == route_b[i]:
            lcp_length += 1
        else:
            break

    return lcp_length


def _get_v2v_branch_target_edge(
    current_route_tuple: tuple,
    route_tuple: tuple,
    candidate_shelterID: str,
    dprint: Callable[[str], None],
) -> Optional[str]:
    lcp_length = _longest_common_prefix_length(current_route_tuple, route_tuple)
    target_index = lcp_length

    if target_index >= len(route_tuple):
        target_index = 0

    if target_index == 0:
        dprint(
            f"candidate={candidate_shelterID}: "
            "skip V2V route because branch target is invalid. "
            f"lcp_length={lcp_length}, route={route_tuple}"
        )
        return None

    return route_tuple[target_index]


def _is_unusable_distance(distance: float) -> bool:
    return distance == float("inf") or distance <= 0.0


def _calculate_route_distance_from_edges(
    edge_ids: tuple[str, ...],
    dprint: Callable[[str], None],
) -> float:
    """
    edge ID の列から総距離を計算する。

    まず edge/lane の長さを合計し，それが失敗した場合は
    traci.simulation.findRoute(edge_ids[0], edge_ids[-1]) の length に
    フォールバックする。どちらでも取得できない場合は float("inf") を返す。
    """
    if not edge_ids:
        dprint("route distance calculation failed: edge_ids is empty")
        return float("inf")

    total_distance = 0.0
    can_use_lane_lengths = True

    for edge_id in edge_ids:
        if not edge_id or _is_internal_junction_edge(edge_id):
            can_use_lane_lengths = False
            dprint(
                "route distance lane-length calculation skipped: "
                f"edge_id={edge_id}"
            )
            break

        try:
            lane_count = traci.edge.getLaneNumber(edge_id)
            if lane_count <= 0:
                can_use_lane_lengths = False
                dprint(
                    "route distance lane-length calculation failed: "
                    f"edge_id={edge_id}, lane_count={lane_count}"
                )
                break

            lane_id = f"{edge_id}_0"
            edge_length = traci.lane.getLength(lane_id)
            if edge_length <= 0:
                can_use_lane_lengths = False
                dprint(
                    "route distance lane-length calculation failed: "
                    f"edge_id={edge_id}, lane_id={lane_id}, "
                    f"edge_length={edge_length}"
                )
                break

            total_distance += edge_length

        except traci.TraCIException as e:
            can_use_lane_lengths = False
            dprint(
                "route distance lane-length calculation failed: "
                f"edge_id={edge_id}, error={e}"
            )
            break

    if can_use_lane_lengths and total_distance > 0:
        return total_distance

    try:
        fallback_route = traci.simulation.findRoute(edge_ids[0], edge_ids[-1])
        fallback_length = getattr(fallback_route, "length", float("inf"))

        if fallback_length > 0 and fallback_length != float("inf"):
            dprint(
                "route distance calculated by findRoute fallback: "
                f"from_edgeID={edge_ids[0]}, to_edgeID={edge_ids[-1]}, "
                f"length={fallback_length:.3f}"
            )
            return fallback_length

        dprint(
            "route distance findRoute fallback returned unusable length: "
            f"from_edgeID={edge_ids[0]}, to_edgeID={edge_ids[-1]}, "
            f"length={fallback_length}"
        )

    except traci.TraCIException as e:
        dprint(
            "route distance findRoute fallback failed: "
            f"from_edgeID={edge_ids[0]}, to_edgeID={edge_ids[-1]}, error={e}"
        )

    return float("inf")


def _extract_edges_from_current_to_divergence(
    current_route_tuple: tuple,
    current_edgeID: str,
    divergence_edge_index: int,
    dprint: Callable[[str], None],
) -> Optional[tuple]:
    """
    current_route_tuple 上で，現在 edge から分岐 edge までの edge 群を返す。
    分岐 edge が現在 edge より前にある場合，または current_edgeID が
    current_route_tuple に存在しない場合は None を返す。
    """
    try:
        current_edge_index = current_route_tuple.index(current_edgeID)
    except ValueError:
        dprint(
            "skip V2V prefix candidate because current edge is not in "
            f"current route: current_edgeID={current_edgeID}, "
            f"current_route_tuple={current_route_tuple}"
        )
        return None

    if divergence_edge_index < current_edge_index:
        dprint(
            "skip V2V prefix candidate because divergence edge is behind "
            "the current edge: "
            f"current_edge_index={current_edge_index}, "
            f"divergence_edge_index={divergence_edge_index}"
        )
        return None

    return tuple(current_route_tuple[current_edge_index:divergence_edge_index + 1])


def _estimate_time_from_current_to_divergence(
    current_edgeID: str,
    vehInfo: "VehicleInfo",
    agent: "Agent",
    edges_from_current_to_divergence: tuple,
    dprint: Callable[[str], None],
) -> float:
    """
    現在 edge から分岐 edge までの推定時間を現在速度で計算する。
    """
    distance_from_current_to_divergence = _calculate_route_distance_from_edges(
        tuple(edges_from_current_to_divergence),
        dprint=dprint,
    )
    if _is_unusable_distance(distance_from_current_to_divergence):
        return float("inf")

    current_speed = _get_current_speed(
        current_edgeID=current_edgeID,
        vehInfo=vehInfo,
        agent=agent,
        dprint=dprint,
    )

    return distance_from_current_to_divergence / current_speed


def _estimate_time_after_divergence_from_v2v(
    alternative_route_tuple: tuple,
    divergence_edge_index: int,
    alternative_avg_time: float,
    dprint: Callable[[str], None],
) -> float:
    """
    代替 route 全体の avg_time を，距離比により分岐 edge 以降の時間へ補正する。
    """
    if alternative_avg_time is None:
        return float("inf")

    if divergence_edge_index < 0 or divergence_edge_index >= len(alternative_route_tuple):
        return float("inf")

    alternative_total_distance = _calculate_route_distance_from_edges(
        tuple(alternative_route_tuple),
        dprint=dprint,
    )
    if _is_unusable_distance(alternative_total_distance):
        return float("inf")

    alternative_edges_after_divergence = tuple(
        alternative_route_tuple[divergence_edge_index:]
    )
    alternative_distance_after_divergence = _calculate_route_distance_from_edges(
        alternative_edges_after_divergence,
        dprint=dprint,
    )
    if _is_unusable_distance(alternative_distance_after_divergence):
        return float("inf")

    return (
        alternative_avg_time
        * alternative_distance_after_divergence
        / alternative_total_distance
    )


def _estimate_forward_reroute_time_from_v2v_prefix(
    current_edgeID: str,
    vehInfo: "VehicleInfo",
    agent: "Agent",
    current_route: CurrentRouteEstimate,
    alternative_route_tuple: tuple,
    route_info: dict,
    candidate_edgeID: str,
    dprint: Callable[[str], None],
) -> tuple[float, Optional[dict]]:
    """
    現在 route と代替 route の共通 prefix を使い，
    順方向分岐による経路変更後の推定所要時間を計算する。

    return:
        forward_reroute_estimated_time, debug_info
    """
    if not current_route.route_tuple or not alternative_route_tuple:
        return float("inf"), None

    if alternative_route_tuple[-1] != candidate_edgeID:
        return float("inf"), None

    if alternative_route_tuple[-1] == current_route.destination_edge_id:
        dprint(
            "skip V2V prefix candidate because destination edge is current "
            f"destination: candidate_edgeID={candidate_edgeID}, "
            f"current_destination_edge={current_route.destination_edge_id}"
        )
        return float("inf"), None

    if alternative_route_tuple == current_route.route_tuple:
        dprint(
            "skip V2V prefix candidate because it is exactly current route: "
            f"alternative_route_tuple={alternative_route_tuple}"
        )
        return float("inf"), None

    if alternative_route_tuple[0] != current_route.route_tuple[0]:
        dprint(
            "skip V2V prefix candidate because start edge differs: "
            f"current_start_edge={current_route.route_tuple[0]}, "
            f"alternative_start_edge={alternative_route_tuple[0]}, "
            f"alternative_route_tuple={alternative_route_tuple}"
        )
        return float("inf"), None

    alternative_avg_time = route_info.get("avg_time")
    if alternative_avg_time is None:
        dprint(
            "skip V2V prefix candidate because avg_time is missing: "
            f"alternative_route_tuple={alternative_route_tuple}"
        )
        return float("inf"), None

    common_prefix_length = _longest_common_prefix_length(
        current_route.route_tuple,
        alternative_route_tuple,
    )
    if common_prefix_length <= 0:
        dprint(
            "skip V2V prefix candidate because common prefix is empty: "
            f"alternative_route_tuple={alternative_route_tuple}"
        )
        return float("inf"), None

    divergence_edge_index = common_prefix_length - 1
    divergence_edge_id = alternative_route_tuple[divergence_edge_index]

    try:
        current_edge_index = current_route.route_tuple.index(current_edgeID)
    except ValueError:
        dprint(
            "skip V2V prefix candidate because current edge is not in "
            f"current route: current_edgeID={current_edgeID}, "
            f"current_route_tuple={current_route.route_tuple}"
        )
        return float("inf"), None

    if divergence_edge_index < current_edge_index:
        dprint(
            "skip V2V prefix candidate because divergence edge is behind "
            "current edge: "
            f"current_edge_index={current_edge_index}, "
            f"divergence_edge_index={divergence_edge_index}, "
            f"divergence_edge_id={divergence_edge_id}"
        )
        return float("inf"), None

    edges_from_current_to_divergence = _extract_edges_from_current_to_divergence(
        current_route_tuple=current_route.route_tuple,
        current_edgeID=current_edgeID,
        divergence_edge_index=divergence_edge_index,
        dprint=dprint,
    )
    if edges_from_current_to_divergence is None:
        return float("inf"), None

    distance_from_current_to_divergence = _calculate_route_distance_from_edges(
        tuple(edges_from_current_to_divergence),
        dprint=dprint,
    )
    if _is_unusable_distance(distance_from_current_to_divergence):
        dprint(
            "skip V2V prefix candidate because distance from current to "
            "divergence is unusable: "
            f"edges_from_current_to_divergence={edges_from_current_to_divergence}, "
            f"distance_from_current_to_divergence="
            f"{distance_from_current_to_divergence}"
        )
        return float("inf"), None

    current_speed = _get_current_speed(
        current_edgeID=current_edgeID,
        vehInfo=vehInfo,
        agent=agent,
        dprint=dprint,
    )
    time_from_current_to_divergence = (
        distance_from_current_to_divergence / current_speed
    )

    alternative_total_distance = _calculate_route_distance_from_edges(
        tuple(alternative_route_tuple),
        dprint=dprint,
    )
    if _is_unusable_distance(alternative_total_distance):
        dprint(
            "skip V2V prefix candidate because alternative total distance "
            f"is unusable: alternative_total_distance={alternative_total_distance}, "
            f"alternative_route_tuple={alternative_route_tuple}"
        )
        return float("inf"), None

    alternative_edges_after_divergence = tuple(
        alternative_route_tuple[divergence_edge_index:]
    )
    alternative_distance_after_divergence = _calculate_route_distance_from_edges(
        alternative_edges_after_divergence,
        dprint=dprint,
    )
    if _is_unusable_distance(alternative_distance_after_divergence):
        dprint(
            "skip V2V prefix candidate because alternative distance after "
            "divergence is unusable: "
            f"alternative_edges_after_divergence="
            f"{alternative_edges_after_divergence}, "
            f"alternative_distance_after_divergence="
            f"{alternative_distance_after_divergence}"
        )
        return float("inf"), None

    estimated_time_after_divergence = (
        alternative_avg_time
        * alternative_distance_after_divergence
        / alternative_total_distance
    )

    forward_reroute_estimated_time = (
        time_from_current_to_divergence
        + estimated_time_after_divergence
    )

    debug_info = {
        "source": "v2v_prefix_adjusted",
        "from_edgeID": current_edgeID,
        "candidate_edgeID": candidate_edgeID,
        "alternative_route_tuple": alternative_route_tuple,
        "alternative_avg_time": alternative_avg_time,
        "vehicles": route_info.get("vehicles"),
        "common_prefix_length": common_prefix_length,
        "divergence_edge_index": divergence_edge_index,
        "divergence_edge_id": divergence_edge_id,
        "current_edge_index": current_edge_index,
        "edges_from_current_to_divergence": edges_from_current_to_divergence,
        "distance_from_current_to_divergence": distance_from_current_to_divergence,
        "time_from_current_to_divergence": time_from_current_to_divergence,
        "alternative_edges_after_divergence": alternative_edges_after_divergence,
        "alternative_total_distance": alternative_total_distance,
        "alternative_distance_after_divergence": alternative_distance_after_divergence,
        "estimated_time_after_divergence": estimated_time_after_divergence,
        "forward_reroute_estimated_time": forward_reroute_estimated_time,
    }

    dprint(
        "V2V prefix candidate evaluated: "
        f"candidate_edgeID={candidate_edgeID}, "
        f"alternative_avg_time={alternative_avg_time:.3f}, "
        f"common_prefix_length={common_prefix_length}, "
        f"divergence_edge_index={divergence_edge_index}, "
        f"divergence_edge_id={divergence_edge_id}, "
        f"current_edge_index={current_edge_index}, "
        f"edges_from_current_to_divergence={edges_from_current_to_divergence}, "
        f"distance_from_current_to_divergence="
        f"{distance_from_current_to_divergence:.3f}, "
        f"time_from_current_to_divergence="
        f"{time_from_current_to_divergence:.3f}, "
        f"alternative_edges_after_divergence="
        f"{alternative_edges_after_divergence}, "
        f"alternative_total_distance={alternative_total_distance:.3f}, "
        f"alternative_distance_after_divergence="
        f"{alternative_distance_after_divergence:.3f}, "
        f"estimated_time_after_divergence="
        f"{estimated_time_after_divergence:.3f}, "
        f"forward_reroute_estimated_time="
        f"{forward_reroute_estimated_time:.3f}"
    )

    return forward_reroute_estimated_time, debug_info


def _estimate_best_v2v_prefix_adjusted_time(
    current_edgeID: str,
    vehInfo: "VehicleInfo",
    agent: "Agent",
    candidate_shelterID: str,
    candidate_edgeID: str,
    current_route: CurrentRouteEstimate,
    routes_dict: dict,
    dprint: Callable[[str], None],
) -> tuple[float, Optional[dict]]:
    """
    routes_dict 内の V2V route を走査し，
    現在 route と共通 prefix を持つ代替 route の中から，
    最も短い forward_reroute_estimated_time を返す。
    """
    best_v2v_prefix_adjusted_time = float("inf")
    best_v2v_prefix_adjusted_debug_info = None

    if not routes_dict:
        return best_v2v_prefix_adjusted_time, best_v2v_prefix_adjusted_debug_info

    for route_tuple, route_info in routes_dict.items():
        alternative_route_tuple = tuple(route_tuple)

        if not alternative_route_tuple:
            continue

        if alternative_route_tuple[-1] != candidate_edgeID:
            continue

        if alternative_route_tuple[-1] == current_route.destination_edge_id:
            dprint(
                f"candidate={candidate_shelterID}: "
                "skip V2V prefix route because destination edge is current "
                f"destination: route_last_edge={alternative_route_tuple[-1]}, "
                f"current_destination_edge={current_route.destination_edge_id}, "
                f"route={alternative_route_tuple}"
            )
            continue

        if alternative_route_tuple == current_route.route_tuple:
            dprint(
                f"candidate={candidate_shelterID}: "
                "skip V2V prefix route because it is exactly current route: "
                f"route={alternative_route_tuple}"
            )
            continue

        alternative_avg_time = route_info.get("avg_time")
        if alternative_avg_time is None:
            dprint(
                f"candidate={candidate_shelterID}: "
                "skip V2V prefix route because avg_time is missing: "
                f"{alternative_route_tuple}"
            )
            continue

        if not current_route.route_tuple:
            continue

        if alternative_route_tuple[0] != current_route.route_tuple[0]:
            dprint(
                f"candidate={candidate_shelterID}: "
                "skip V2V prefix route because start edge differs: "
                f"current_start_edge={current_route.route_tuple[0]}, "
                f"alternative_start_edge={alternative_route_tuple[0]}, "
                f"route={alternative_route_tuple}"
            )
            continue

        forward_reroute_estimated_time, debug_info = (
            _estimate_forward_reroute_time_from_v2v_prefix(
                current_edgeID=current_edgeID,
                vehInfo=vehInfo,
                agent=agent,
                current_route=current_route,
                alternative_route_tuple=alternative_route_tuple,
                route_info=route_info,
                candidate_edgeID=candidate_edgeID,
                dprint=dprint,
            )
        )

        if forward_reroute_estimated_time < best_v2v_prefix_adjusted_time:
            best_v2v_prefix_adjusted_time = forward_reroute_estimated_time
            best_v2v_prefix_adjusted_debug_info = debug_info

    return best_v2v_prefix_adjusted_time, best_v2v_prefix_adjusted_debug_info


def _estimate_u_turn_reroute_time(
    current_edgeID: str,
    candidate_edgeID: str,
    vehInfo: "VehicleInfo",
    agent: "Agent",
    dprint: Callable[[str], None],
) -> tuple[float, Optional[dict]]:
    """
    現在 edge の反対車線から候補避難地 edge へ向かう Uターン候補の推定時間を計算する。

    return:
        u_turn_estimated_time, debug_info
    """
    try:
        u_turn_start_edge_id = get_opposite_edgeID_by_edgeID(edgeID=current_edgeID)
    except TypeError:
        u_turn_start_edge_id = get_opposite_edgeID_by_edgeID(current_edgeID)

    if not u_turn_start_edge_id:
        dprint(
            "skip U-turn candidate because opposite edge is empty: "
            f"current_edgeID={current_edgeID}"
        )
        return float("inf"), None

    try:
        u_turn_route = traci.simulation.findRoute(
            u_turn_start_edge_id,
            candidate_edgeID,
        )
    except traci.TraCIException as e:
        dprint(
            "skip U-turn candidate because findRoute failed: "
            f"u_turn_start_edge_id={u_turn_start_edge_id}, "
            f"candidate_edgeID={candidate_edgeID}, error={e}"
        )
        return float("inf"), None

    u_turn_route_edges = tuple(getattr(u_turn_route, "edges", tuple()))
    if not u_turn_route_edges:
        dprint(
            "skip U-turn candidate because route edges are empty: "
            f"u_turn_start_edge_id={u_turn_start_edge_id}, "
            f"candidate_edgeID={candidate_edgeID}"
        )
        return float("inf"), None

    u_turn_route_distance = _calculate_route_distance_from_edges(
        u_turn_route_edges,
        dprint=dprint,
    )
    if _is_unusable_distance(u_turn_route_distance):
        dprint(
            "skip U-turn candidate because distance is unusable: "
            f"u_turn_route_edges={u_turn_route_edges}, "
            f"u_turn_route_distance={u_turn_route_distance}"
        )
        return float("inf"), None

    current_speed = _get_current_speed(
        current_edgeID=current_edgeID,
        vehInfo=vehInfo,
        agent=agent,
        dprint=dprint,
    )
    u_turn_estimated_time = u_turn_route_distance / current_speed

    debug_info = {
        "source": "u_turn",
        "from_edgeID": u_turn_start_edge_id,
        "candidate_edgeID": candidate_edgeID,
        "u_turn_start_edge_id": u_turn_start_edge_id,
        "u_turn_route_edges": u_turn_route_edges,
        "u_turn_route_distance": u_turn_route_distance,
        "u_turn_estimated_time": u_turn_estimated_time,
    }

    dprint(
        "U-turn candidate evaluated: "
        f"u_turn_start_edge_id={u_turn_start_edge_id}, "
        f"candidate_edgeID={candidate_edgeID}, "
        f"u_turn_route_edges={u_turn_route_edges}, "
        f"u_turn_route_distance={u_turn_route_distance:.3f}, "
        f"u_turn_estimated_time={u_turn_estimated_time:.3f}"
    )

    return u_turn_estimated_time, debug_info

def _estimate_best_v2v_time(
    agent: "Agent",
    candidate_shelterID: str,
    candidate_edgeID: str,
    candidate_shelter: "Shelter",
    from_edgeID: str,
    current_route: CurrentRouteEstimate,
    routes_dict: dict,
    approach_edgeIDs_by_start_edgeID: dict,
    dprint: Callable[[str], None],
) -> tuple[float, Optional[dict]]:
    time_v2v_based = float("inf")
    best_v2v_debug_info = None

    if not routes_dict:
        return time_v2v_based, best_v2v_debug_info

    for route_tuple, route_info in routes_dict.items():
        # 候補避難所につながる edge で終わる経路だけを見る
        if route_tuple[-1] != candidate_edgeID:
            continue

        # 現在目的地 edge に向かう V2V 経路は代替候補から除外する
        if route_tuple[-1] == current_route.destination_edge_id:
            dprint(
                f"candidate={candidate_shelterID}: "
                "skip V2V route because destination edge is current destination. "
                f"route_last_edge={route_tuple[-1]}, "
                f"current_destination_edge={current_route.destination_edge_id}, "
                f"route={route_tuple}"
            )
            continue

        # 現在経路そのものは迂回候補から除外
        if route_tuple == current_route.route_tuple:
            dprint(
                f"candidate={candidate_shelterID}: "
                "skip V2V route because it is exactly current route: "
                f"route={route_tuple}"
            )
            continue

        v2v_avg_time = route_info.get("avg_time")
        if v2v_avg_time is None:
            dprint(
                f"candidate={candidate_shelterID}: "
                f"skip V2V route because avg_time is missing: {route_tuple}"
            )
            continue

        branch_target_edge = _get_v2v_branch_target_edge(
            current_route_tuple=current_route.route_tuple,
            route_tuple=route_tuple,
            candidate_shelterID=candidate_shelterID,
            dprint=dprint,
        )

        if branch_target_edge is None:
            continue

        distance_to_branch = _safe_calculate_reroute_distance(
            vehID=agent.get_vehID(),
            from_edgeID=from_edgeID,
            to_edgeID=branch_target_edge,
            shelter=candidate_shelter,
            approach_edgeIDs_by_start_edgeID=approach_edgeIDs_by_start_edgeID,
            dprint=dprint,
        )
        time_to_branch = distance_to_branch / FREE_FLOW_SPEED

        # 既存実装の近似を踏襲:
        # 現在地から分岐先までは自由速度で補い，その先に V2V の avg_time を足す。
        calculated_time = time_to_branch + v2v_avg_time

        dprint(
            f"candidate={candidate_shelterID}: V2V route matched. "
            f"from_edgeID={from_edgeID}, "
            f"route_last_edge={route_tuple[-1]}, "
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
                "from_edgeID": from_edgeID,
                "branch_target_edge": branch_target_edge,
                "distance_to_branch": distance_to_branch,
                "time_to_branch": time_to_branch,
                "v2v_avg_time": v2v_avg_time,
                "calculated_time": calculated_time,
                "vehicles": route_info.get("vehicles"),
            }

    return time_v2v_based, best_v2v_debug_info


def _evaluate_candidate_from_start_edge(
    current_edgeID: str,
    agent: "Agent",
    vehInfo: "VehicleInfo",
    current_route: CurrentRouteEstimate,
    candidate_shelterID: str,
    candidate_edgeID: str,
    candidate_shelter: "Shelter",
    from_edgeID: str,
    routes_dict: dict,
    approach_edgeIDs_by_start_edgeID: dict,
    dprint: Callable[[str], None],
) -> CandidateEvaluation:
    distance_free_flow, time_free_flow = _estimate_free_flow_time(
        agent=agent,
        from_edgeID=from_edgeID,
        to_edgeID=candidate_edgeID,
        shelter=candidate_shelter,
        approach_edgeIDs_by_start_edgeID=approach_edgeIDs_by_start_edgeID,
        dprint=dprint,
    )

    free_flow_debug_info = {
        "source": "free_flow",
        "from_edgeID": from_edgeID,
        "candidate_edgeID": candidate_edgeID,
        "distance_free_flow": distance_free_flow,
        "time_free_flow": time_free_flow,
    }

    dprint(
        f"candidate={candidate_shelterID}: "
        f"from_edgeID={from_edgeID}, "
        f"free_flow distance={distance_free_flow:.3f}, "
        f"time_free_flow={time_free_flow:.3f}"
    )

    time_v2v_prefix_adjusted, best_v2v_prefix_adjusted_debug_info = (
        _estimate_best_v2v_prefix_adjusted_time(
            current_edgeID=current_edgeID,
            vehInfo=vehInfo,
            agent=agent,
            candidate_shelterID=candidate_shelterID,
            candidate_edgeID=candidate_edgeID,
            current_route=current_route,
            routes_dict=routes_dict,
            dprint=dprint,
        )
    )

    if time_v2v_prefix_adjusted == float("inf"):
        dprint(
            f"candidate={candidate_shelterID}: "
            "no usable V2V prefix-adjusted route."
        )
    else:
        dprint(
            f"candidate={candidate_shelterID}: "
            f"time_v2v_prefix_adjusted={time_v2v_prefix_adjusted:.3f}, "
            f"best_v2v_prefix_adjusted_debug_info="
            f"{best_v2v_prefix_adjusted_debug_info}"
        )

    time_u_turn, u_turn_debug_info = _estimate_u_turn_reroute_time(
        current_edgeID=current_edgeID,
        candidate_edgeID=candidate_edgeID,
        vehInfo=vehInfo,
        agent=agent,
        dprint=dprint,
    )

    if time_u_turn == float("inf"):
        dprint(
            f"candidate={candidate_shelterID}: "
            "no usable U-turn route."
        )
    else:
        dprint(
            f"candidate={candidate_shelterID}: "
            f"time_u_turn={time_u_turn:.3f}, "
            f"u_turn_debug_info={u_turn_debug_info}"
        )

    best_time = min(
        time_free_flow,
        time_v2v_prefix_adjusted,
        time_u_turn,
    )

    if (
        best_time == time_v2v_prefix_adjusted
        and best_v2v_prefix_adjusted_debug_info is not None
    ):
        best_source = "v2v_prefix_adjusted"
        best_debug_info = best_v2v_prefix_adjusted_debug_info
    elif best_time == time_u_turn and u_turn_debug_info is not None:
        best_source = "u_turn"
        best_debug_info = u_turn_debug_info
    else:
        best_source = "free_flow"
        best_debug_info = free_flow_debug_info

    best_from_edge_id = best_debug_info.get("from_edgeID", from_edgeID)

    dprint(
        f"candidate={candidate_shelterID}: "
        f"time_free_flow={time_free_flow:.3f}, "
        f"time_v2v_prefix_adjusted={time_v2v_prefix_adjusted:.3f}, "
        f"time_u_turn={time_u_turn:.3f}, "
        f"best_source={best_source}, "
        f"best_time={best_time:.3f}, "
        f"best_debug_info={best_debug_info}"
    )

    return CandidateEvaluation(
        best_time=best_time,
        shelter_id=candidate_shelterID,
        edge_id=candidate_edgeID,
        from_edge_id=best_from_edge_id,
        best_source=best_source,
        time_free_flow=time_free_flow,
        time_v2v_based=time_v2v_prefix_adjusted,
        best_v2v_debug_info=best_v2v_prefix_adjusted_debug_info,
        time_v2v_prefix_adjusted=time_v2v_prefix_adjusted,
        time_u_turn=time_u_turn,
        best_debug_info=best_debug_info,
    )


def _evaluate_candidate(
    current_edgeID: str,
    agent: "Agent",
    vehInfo: "VehicleInfo",
    current_route: CurrentRouteEstimate,
    candidate_shelterID: str,
    candidate_edgeID: str,
    shelter_list: list,
    route_name: str,
    current_group: str,
    reroute_start_context: RerouteStartContext,
    routes_dict: dict,
    dprint: Callable[[str], None],
) -> Optional[CandidateEvaluation]:
    dprint(
        f"current_edgeID={current_edgeID}, "
        f"current_route.route_tuple={current_route.route_tuple}, "
        f"current_route.destination_edge_id={current_route.destination_edge_id}, "
        f"current_target_shelterID={current_route.target_shelter_id}, "
        f"candidate_shelterID={candidate_shelterID}, "
        f"candidate_edgeID={candidate_edgeID}"
    )

    if _should_skip_candidate(
        candidate_shelterID=candidate_shelterID,
        candidate_edgeID=candidate_edgeID,
        current_route=current_route,
        dprint=dprint,
    ):
        return None

    candidate_shelter = _resolve_candidate_shelter(
        candidate_shelterID=candidate_shelterID,
        candidate_edgeID=candidate_edgeID,
        shelter_list=shelter_list,
        dprint=dprint,
    )
    if candidate_shelter is None:
        return None

    candidate_group = _get_shelter_group(candidate_shelterID)

    base_start_edge_for_candidate = _select_base_start_edge_for_candidate(
        current_edgeID=current_edgeID,
        base_reroute_start_edgeID=reroute_start_context.base_reroute_start_edgeID,
        route_name=route_name,
        current_group=current_group,
        candidate_group=candidate_group,
    )

    start_edge_options = _get_start_edge_options_for_candidate(
        current_edgeID=current_edgeID,
        base_start_edge_for_candidate=base_start_edge_for_candidate,
    )

    dprint(
        f"candidate start: shelter={candidate_shelterID}, "
        f"candidate_edgeID={candidate_edgeID}, "
        f"candidate_group={candidate_group}, "
        f"start_edge_options={start_edge_options}"
    )

    evaluations = [
        _evaluate_candidate_from_start_edge(
            current_edgeID=current_edgeID,
            agent=agent,
            vehInfo=vehInfo,
            current_route=current_route,
            candidate_shelterID=candidate_shelterID,
            candidate_edgeID=candidate_edgeID,
            candidate_shelter=candidate_shelter,
            from_edgeID=from_edgeID,
            routes_dict=routes_dict,
            approach_edgeIDs_by_start_edgeID=(
                reroute_start_context.approach_edgeIDs_by_start_edgeID
            ),
            dprint=dprint,
        )
        for from_edgeID in start_edge_options
    ]

    if not evaluations:
        dprint(
            f"candidate={candidate_shelterID}: "
            "no start edge options. skip candidate."
        )
        return None

    best_evaluation = min(evaluations, key=lambda item: item.best_time)

    dprint(
        f"candidate result: shelter={best_evaluation.shelter_id}, "
        f"candidate_edgeID={best_evaluation.edge_id}, "
        f"from_edgeID={best_evaluation.from_edge_id}, "
        f"best_time={best_evaluation.best_time:.3f}, "
        f"best_source={best_evaluation.best_source}, "
        f"best_debug_info={best_evaluation.best_debug_info}"
    )

    return best_evaluation


def _evaluate_all_candidates(
    current_edgeID: str,
    agent: "Agent",
    vehInfo: "VehicleInfo",
    current_route: CurrentRouteEstimate,
    shelter_list: list,
    reroute_start_context: RerouteStartContext,
    routes_dict: dict,
    dprint: Callable[[str], None],
) -> list[CandidateEvaluation]:
    route_name = _get_current_route_name(current_edgeID=current_edgeID, vehInfo=vehInfo)
    current_group = _get_shelter_group(current_route.target_shelter_id)

    dprint(
        f"route_name={route_name}, "
        f"current_group={current_group}, "
        f"candidate_shelters={list(agent.get_candidate_edge_by_shelterID().keys())}"
    )
    # 出発地が同じで、routeが一部同じものに関しては、その差分で、交差点から避難地のまでの差分がわかる
    # 現在のedge_IDからrouteが異なるところまでお
    candidate_results = []

    for candidate_shelterID, candidate_edgeID in (
        agent.get_candidate_edge_by_shelterID().items()
    ):
        evaluation = _evaluate_candidate(
            current_edgeID=current_edgeID,
            agent=agent,
            vehInfo=vehInfo,
            current_route=current_route,
            candidate_shelterID=candidate_shelterID,
            candidate_edgeID=candidate_edgeID,
            shelter_list=shelter_list,
            route_name=route_name,
            current_group=current_group,
            reroute_start_context=reroute_start_context,
            routes_dict=routes_dict,
            dprint=dprint,
        )

        if evaluation is not None:
            candidate_results.append(evaluation)

    candidate_results = [
        item for item in candidate_results
        if item.edge_id != current_route.destination_edge_id
    ]
    candidate_results.sort(key=lambda item: item.best_time)

    dprint(f"candidate_results_list={candidate_results}")
    dprint(f"to_edge_list={[item.edge_id for item in candidate_results]}")

    return candidate_results


def _apply_approach_edge_route_if_needed(
    agent: "Agent",
    from_edgeID: str,
    to_edgeID: str,
    dprint: Callable[[str], None],
) -> None:
    dprint("approach edge: apply best route directly with traci.vehicle.setRoute")

    route = traci.simulation.findRoute(from_edgeID, to_edgeID)
    if route.edges:
        traci.vehicle.setRoute(agent.get_vehID(), route.edges)


def _build_final_decision(
    agent: "Agent",
    estimated_current_route_time: float,
    candidate_results: list[CandidateEvaluation],
    base_reroute_start_edgeID: str,
    approach_edge_flg: bool,
    dprint: Callable[[str], None],
) -> tuple:
    shelterID = ""
    congestion_flg = False

    if not candidate_results:
        dprint("no candidate results. keep current route.")
        return base_reroute_start_edgeID, shelterID, [], 0.0, congestion_flg

    best_candidate = candidate_results[0]
    time_gain = estimated_current_route_time - best_candidate.best_time
    threshold = agent.get_route_change_threshold()

    dprint(
        f"final decision: estimated_current_time={estimated_current_route_time:.3f}, "
        f"best_candidate_time={best_candidate.best_time:.3f}, "
        f"time_gain={time_gain:.3f}, "
        f"threshold={threshold:.3f}, "
        f"best_candidate={best_candidate}"
    )

    if time_gain > threshold:
        congestion_flg = True

        if not approach_edge_flg:
            return (
                best_candidate.from_edge_id,
                best_candidate.shelter_id,
                [best_candidate.edge_id],
                time_gain,
                congestion_flg,
            )

        _apply_approach_edge_route_if_needed(
            agent=agent,
            from_edgeID=best_candidate.from_edge_id,
            to_edgeID=best_candidate.edge_id,
            dprint=dprint,
        )
        return (
            best_candidate.from_edge_id,
            best_candidate.shelter_id,
            [],
            time_gain,
            congestion_flg,
        )

    to_edge_list = [item.edge_id for item in candidate_results]
    return base_reroute_start_edgeID, shelterID, to_edge_list, time_gain, congestion_flg


def find_alternative_better_choice_fixed_divide(
    current_edgeID: str,
    vehInfo: "VehicleInfo",
    agent: "Agent",
    shelter: "Shelter",
    shelter_list: list,
    custome_edge_list: list,
    debug: bool = False,
):
    """
    現在の経路と，V2V情報および自由速度計算に基づく迂回経路を比較し，
    より良い選択肢があれば経路変更情報を返す。

    返り値の形式は既存と同じ:
        return from_edgeID, shelterID, to_edge_list, time_gain, congestion_flg
    """
    dprint = _make_dprint(debug)
    shelterID_to_return = vehInfo.get_target_shelter()
    to_edge_list_to_return = []
    congestion_flg = False

    dprint(
        f"start vehID={agent.get_vehID()}, "
        f"current_edgeID={current_edgeID}"
    )

    if _is_internal_junction_edge(current_edgeID):
        dprint(f"skip: current_edgeID is internal junction edge: {current_edgeID}")
        return "", shelterID_to_return, to_edge_list_to_return, 0.0, congestion_flg

    current_route = _estimate_current_route_time(
        current_edgeID=current_edgeID,
        vehInfo=vehInfo,
        agent=agent,
        shelter=shelter,
        dprint=dprint,
    )

    reroute_start_context = _build_reroute_start_context(
        current_edgeID=current_edgeID,
        vehInfo=vehInfo,
        dprint=dprint,
    )

    if reroute_start_context.should_return_without_calculation:
        dprint(
            "vehicle is in approach edge and communication is disabled. "
            "return without reroute calculation."
        )
        return (
            reroute_start_context.base_reroute_start_edgeID,
            shelterID_to_return,
            [],
            0.0,
            congestion_flg,
        )

    routes_dict = _merge_v2v_routes(vehInfo=vehInfo, dprint=dprint)
    if len(routes_dict) > 2:
        print(f"current_route={current_route}")
        print(f"routes_dict={routes_dict}")

    candidate_results = _evaluate_all_candidates(
        current_edgeID=current_edgeID,
        agent=agent,
        vehInfo=vehInfo,
        current_route=current_route,
        shelter_list=shelter_list,
        reroute_start_context=reroute_start_context,
        routes_dict=routes_dict,
        dprint=dprint,
    )

    return _build_final_decision(
        agent=agent,
        estimated_current_route_time=current_route.estimated_time,
        candidate_results=candidate_results,
        base_reroute_start_edgeID=reroute_start_context.base_reroute_start_edgeID,
        approach_edge_flg=reroute_start_context.approach_edge_flg,
        dprint=dprint,
    )
