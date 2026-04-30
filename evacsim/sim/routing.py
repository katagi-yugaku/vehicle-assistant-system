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
    distance_each_vehIDs,
)
from evacsim.utils.lookup import find_shelter_by_edgeID_connect_target_shelter

if TYPE_CHECKING:
    from evacsim.agents.Agent import Agent
    from evacsim.agents.Shelter import Shelter
    from evacsim.agents.VehicleInfo import VehicleInfo


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
