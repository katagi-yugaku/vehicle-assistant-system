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
    calculate_remaining_route_distance,
    calculate_reroute_distance,
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


def find_alternative_better_choice_fixed(
    current_edgeID: str, 
    vehInfo: VehicleInfo, 
    agent: Agent, 
    shelter: Shelter,
    shelter_list: list, 
    custome_edge_list: List[CustomeEdge],
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

    current_route_tuple = tuple(traci.vehicle.getRoute(agent.get_vehID()))
    current_destination_edge = current_route_tuple[-1]

    distance_to_current_shelter = calculate_remaining_route_distance(
        agent.get_vehID(),
        current_destination_edge,
        shelter=shelter_for_vehInfo
    )
    ################ ここまでおK ################

    try:
        # 念のため初期値を入れておく
        current_speed = FREE_FLOW_SPEED

        if vehInfo.get_vehicle_comm_enabled_flag:
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
            return base_reroute_start_edgeID, shelterID_to_return, [], 0.0

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
        if candidate_shelterID == current_target_shelterID:
            dprint(
                f"skip candidate_shelter={candidate_shelterID}: "
                "same as current target shelter"
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

        # ---------------------------------------------------------
        # A. 自由速度での迂回所要時間
        # ---------------------------------------------------------
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

                # 現在経路そのものは迂回候補から除外
                if route_tuple == current_route_tuple:
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

                branch_target_edge = route_tuple[target_index]

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
    # print(f"estimated_current_route_evacuation_time: {estimated_current_route_evacuation_time:.3f}, best_candidate_time: {best_candidate_time:.3f}, time_gain: {time_gain:.3f}, threshold: {threshold:.3f}")

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

        route = traci.simulation.findRoute(
            base_reroute_start_edgeID,
            best_candidate_edgeID
        )

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
    return congestion_flg


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
            # print(f"canditate_sget_near_edgeIDhelterID: {candidate_shelter.get_near_edgeID()} agent_edgeID: {agent.get_near_edgeID_by_target_shelter()}")
            # print(f"vehID: {vehID} current_edgeID: {current_edgeID} reroute_start_edgeID: {reroute_start_edgeID}")
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
