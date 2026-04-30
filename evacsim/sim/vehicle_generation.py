# =========================
# Vehicle generation helpers
# =========================
#
# 注意:
# このモジュールは TraCI に依存する。
# 車両生成・ルート生成・車両置換処理を扱う。
#
# まずは低リスクな初期車両生成関数のみを切り出す。
# generate_new_veh 系は Agent / VehicleInfo の状態継承が重いため次ステップで追加する。

from __future__ import annotations

import copy

from evacsim.sim.sumo_env import ensure_sumo_tools_on_path

ensure_sumo_tools_on_path()

import traci

from evacsim.agents.Agent import Agent
from evacsim.agents.VehicleInfo import VehicleInfo
from evacsim.maps.edge_utils import get_opposite_edgeID_by_edgeID
from evacsim.sim.congestion import is_candidate_shelter_full
from evacsim.sim.initialization import init_driver_behavior
from evacsim.sim.routing import (
    get_new_shelterID_and_near_edgeID_by_vehID_based_on_distance,
)


def generate_init_vehID(
    from_edgeID: str,
    to_edgeID: str,
    shelterID: str,
    VEHICLE_FOR_SHELTER: int,
    start_time: float,
    generate_interval: float,
    generate_route_count: int,
    generate_veh_count: int,
):
    """
    初期車両を複数台生成する。

    処理:
      - from_edgeID から to_edgeID までの route を findRoute で取得
      - route を traci.route.add で登録
      - vehicle を traci.vehicle.add で生成
      - parkingAreaStop を shelterID に設定

    既存 utilities.py の generate_init_vehID の挙動を維持する。
    """
    vehID_list = []

    for vehicle_count in range(VEHICLE_FOR_SHELTER):
        # 新しい車両IDを生成と出発時間を設定
        new_veh_ID: str = "{}_{}_{}".format(
            "init",
            shelterID,
            generate_veh_count,
        )

        vehID_list.append(new_veh_ID)

        deparet_time: float = start_time + generate_interval * vehicle_count

        # 経由地点を取得する
        via_edgeIDs_with_intial_end_edge: list = list(
            traci.simulation.findRoute(
                from_edgeID,
                to_edgeID,
            ).edges
        )

        # ルートを設定する
        new_route_ID: str = "{}_{}_{}".format(
            "initroute",
            shelterID,
            generate_route_count,
        )

        traci.route.add(
            routeID=new_route_ID,
            edges=via_edgeIDs_with_intial_end_edge,
        )

        # 新しい車両を生成
        traci.vehicle.add(
            vehID=new_veh_ID,
            routeID=new_route_ID,
            depart=deparet_time,
        )

        # 新規の避難地を設定
        traci.vehicle.setParkingAreaStop(
            vehID=new_veh_ID,
            stopID=shelterID,
            duration=100000,
        )

        generate_veh_count += 1
        generate_route_count += 1

    return generate_veh_count, generate_route_count, vehID_list


def generate_simple_init_vehID(
    from_edgeID: str,
    to_edgeID: str,
    shelterID: str,
    generate_interval: float,
    generate_route_count: int,
    generate_veh_count: int,
    depart_time: float,
):
    """
    初期車両を1台生成する。

    既存 utilities.py の generate_simple_init_vehID の挙動を維持する。
    """
    vehID_list = []

    # 新しい車両IDを生成と出発時間を設定
    new_veh_ID: str = "{}_{}_{}".format(
        "init",
        shelterID,
        generate_veh_count,
    )

    vehID_list.append(new_veh_ID)

    # 経由地点を取得する
    via_edgeIDs_with_intial_end_edge: list = list(
        traci.simulation.findRoute(
            from_edgeID,
            to_edgeID,
        ).edges
    )

    # ルートを設定する
    new_route_ID: str = "{}_{}_{}".format(
        "initroute",
        shelterID,
        generate_route_count,
    )

    traci.route.add(
        routeID=new_route_ID,
        edges=via_edgeIDs_with_intial_end_edge,
    )

    # 新しい車両を生成
    traci.vehicle.add(
        vehID=new_veh_ID,
        routeID=new_route_ID,
        depart=depart_time,
    )

    depart_time = depart_time + generate_interval

    # 新規の避難地を設定
    traci.vehicle.setParkingAreaStop(
        vehID=new_veh_ID,
        stopID=shelterID,
        duration=100000,
    )

    generate_veh_count += 1
    generate_route_count += 1

    return generate_veh_count, generate_route_count, vehID_list, depart_time


def generate_init_vehID_with_route_edges(
    from_edgeID: str,
    to_edgeID: str,
    shelterID: str,
    generate_interval: float,
    generate_route_count: int,
    generate_veh_count: int,
    depart_time: float,
    route_edges: list,
):
    """
    route_edges が与えられている場合はそれを使って初期車両を1台生成する。

    route_edges が None の場合は、
    from_edgeID から to_edgeID までの route を findRoute で取得する。

    既存 utilities.py の generate_init_vehID_with_route_edges の挙動を維持する。
    """
    vehID_list = []

    # 新しい車両IDを生成と出発時間を設定
    new_veh_ID: str = "{}_{}_{}".format(
        "init",
        shelterID,
        generate_veh_count,
    )

    vehID_list.append(new_veh_ID)

    # 経由地点を取得する
    via_edgeIDs_with_intial_end_edge: list = (
        route_edges
        if route_edges is not None
        else list(
            traci.simulation.findRoute(
                from_edgeID,
                to_edgeID,
            ).edges
        )
    )

    # ルートを設定する
    new_route_ID: str = "{}_{}_{}".format(
        "initroute",
        shelterID,
        generate_route_count,
    )

    traci.route.add(
        routeID=new_route_ID,
        edges=via_edgeIDs_with_intial_end_edge,
    )

    # 新しい車両を生成
    traci.vehicle.add(
        vehID=new_veh_ID,
        routeID=new_route_ID,
        depart=depart_time,
    )

    depart_time = depart_time + generate_interval

    # 新規の避難地を設定
    traci.vehicle.setParkingAreaStop(
        vehID=new_veh_ID,
        stopID=shelterID,
        duration=100000,
    )

    generate_veh_count += 1
    generate_route_count += 1

    return generate_veh_count, generate_route_count, vehID_list, depart_time


def generate_new_veh_based_on_route_time(
    target_vehID: str,
    NEW_VEHICLE_COUNT: int,
    agent_list: list,
    vehInfo_list: list,
    vehInfo_by_target_vehID: VehicleInfo,
    agent_by_target_vehID: Agent,
    from_edgeID: str,
    new_shelterID: str,
    to_edgeID: str,
    color_mode: int,
):
    """
    経路時間ベースで決定済みの from_edgeID / new_shelterID / to_edgeID に従い、
    既存車両 target_vehID を新しい避難地へ向かう車両に置き換える。

    既存 utilities.py の generate_new_veh_based_on_route_time の挙動を維持する。
    """
    if from_edgeID == "" and new_shelterID == "" and to_edgeID == "":
        return NEW_VEHICLE_COUNT

    # 経由地点を取得する
    via_edgeIDs_with_intial_end_edge: list = list(
        traci.simulation.findRoute(from_edgeID, to_edgeID).edges
    )

    if (
        via_edgeIDs_with_intial_end_edge is None
        or len(via_edgeIDs_with_intial_end_edge) == 0
    ):
        via_edgeIDs_with_intial_end_edge: list = list(
            traci.simulation.findRoute(
                get_opposite_edgeID_by_edgeID(from_edgeID),
                to_edgeID,
            ).edges
        )

        if (
            via_edgeIDs_with_intial_end_edge is None
            or len(via_edgeIDs_with_intial_end_edge) == 0
        ):
            print(f"避難経路が存在しません from: {from_edgeID} to: {to_edgeID}")
            return NEW_VEHICLE_COUNT

    # target_vehIDから数字をとる
    vehID_num = target_vehID.split("_")[3]

    # 新しい車両IDを生成と出発時間を設定
    new_veh_ID: str = "{}_{}_{}_{}".format(
        "newveh",
        new_shelterID,
        vehID_num,
        NEW_VEHICLE_COUNT,
    )

    deparet_time: float = traci.simulation.getTime()

    # 現在のagentを無効にする
    agent_by_target_vehID.set_shelter_changed_flg(True)

    # 候補地を更新
    # TODO 満杯情報を受け取った後は、当該避難地を候補地から削除
    updated_candidate_shelter = agent_by_target_vehID.get_candidate_shelter()

    # 元の vehID を削除し、新しい Agent を追加
    agent: Agent = Agent(
        vehID=new_veh_ID,
        target_shelter=new_shelterID,
        tunning_threshold=copy.deepcopy(
            agent_by_target_vehID.get_tunning_threshold()
        ),
        route_change_threshold=copy.deepcopy(
            agent_by_target_vehID.get_route_change_threshold()
        ),
        lane_change_init_threshold=copy.deepcopy(
            agent_by_target_vehID.get_lane_change_decision_threshold()
        ),
        normalcy_motivation_increase=copy.deepcopy(
            agent_by_target_vehID.get_motivation_increase_from_info_receive()
        ),
        motivation_decrease_due_to_inactive_neighbors=copy.deepcopy(
            agent_by_target_vehID.get_motivation_decrease_due_to_inactive_neighbors()
        ),
        motivation_increase_due_to_following_neighbors=copy.deepcopy(
            agent_by_target_vehID.get_motivation_increase_due_to_following_neighbors()
        ),
        lane_minimum_motivation_value=copy.deepcopy(
            agent_by_target_vehID.get_minimum_motivation_value()
        ),
        shelter_occupancy_rate_threshold=copy.deepcopy(
            agent_by_target_vehID.get_shelter_occupancy_rate_threshold()
        ),
        vehicle_abandoned_threshold=copy.deepcopy(
            agent_by_target_vehID.get_vehicle_abandoned_threshold()
        ),
        normalcy_value_about_vehicle_abandonment=copy.deepcopy(
            agent_by_target_vehID.get_normalcy_value_about_vehicle_abandonment()
        ),
        majority_value_about_vehicle_abandonment=copy.deepcopy(
            agent_by_target_vehID.get_majority_value_about_vehicle_abandonment()
        ),
    )

    agent.set_near_edgeID_by_target_shelter(
        copy.deepcopy(agent_by_target_vehID.get_near_edgeID_by_target_shelter())
    )
    agent.set_candidate_edge_by_shelterID(updated_candidate_shelter)
    agent.init_set_candidate_near_shelter(
        shelter_edge_by_IDs=updated_candidate_shelter
    )

    agent.set_x_elapsed_time_for_lane_change_list(
        copy.deepcopy(agent_by_target_vehID.get_x_elapsed_time_for_lane_change_list())
    )
    # 既存 utilities.py では y_motivation_value_for_lane_change_list はコピーしていない
    agent.set_lane_change_xy_dict(
        copy.deepcopy(agent_by_target_vehID.get_lane_change_xy_dict())
    )

    agent.set_motivation_decrease_due_to_inactive_neighbors(
        copy.deepcopy(
            agent_by_target_vehID.get_motivation_decrease_due_to_inactive_neighbors()
        )
    )
    agent.set_motivation_increase_due_to_following_neighbors(
        copy.deepcopy(
            agent_by_target_vehID.get_motivation_increase_due_to_following_neighbors()
        )
    )
    agent.set_calculated_motivation_value(
        copy.deepcopy(agent_by_target_vehID.get_calculated_motivation_value())
    )
    agent.set_near_edgeID_by_target_shelter(to_edgeID)
    agent.set_shelter_full_flg(
        copy.deepcopy(agent_by_target_vehID.get_shelter_full_flg())
    )

    agent_list.append(agent)

    # VehicleInfo を継承して新規作成
    new_vehInfo_by_target_vehID: VehicleInfo = VehicleInfo(
        vehID=new_veh_ID,
        target_shelter=new_shelterID,
        edgeID_connect_target_shelter=to_edgeID,
        create_time=deparet_time,
    )

    new_vehInfo_by_target_vehID.set_shelter_congestion_info(
        copy.deepcopy(vehInfo_by_target_vehID.get_shelter_congestion_info())
    )
    new_vehInfo_by_target_vehID.set_avg_evac_time_by_route_by_recive_time(
        copy.deepcopy(
            vehInfo_by_target_vehID.get_avg_evac_time_by_route_by_recive_time()
        )
    )
    new_vehInfo_by_target_vehID.set_approach_edge_dict(
        copy.deepcopy(vehInfo_by_target_vehID.get_approach_edge_dict())
    )
    new_vehInfo_by_target_vehID.set_edgeIDs_within_junction_to_shelter_dict(
        copy.deepcopy(
            vehInfo_by_target_vehID.get_edgeIDs_within_junction_to_shelter_dict()
        )
    )

    vehInfo_list.append(new_vehInfo_by_target_vehID)
    vehInfo_by_target_vehID.set_agent_changed_flag(True)

    # edge上の始点からどこにいるのかを取得する
    edge_position = traci.vehicle.getLanePosition(target_vehID)
    current_laneID = traci.vehicle.getLaneID(target_vehID)
    current_edgeID_len = traci.lane.getLength(current_laneID)

    if current_edgeID_len > 100:
        depart_position = current_edgeID_len - edge_position - 10

    if current_edgeID_len <= 100:
        depart_position = current_edgeID_len - edge_position - 10

    traci.vehicle.remove(target_vehID)

    new_route_ID: str = "{}_{}_{}".format(
        "newroute",
        new_shelterID,
        NEW_VEHICLE_COUNT,
    )

    traci.route.add(
        routeID=new_route_ID,
        edges=via_edgeIDs_with_intial_end_edge,
    )

    traci.vehicle.add(
        vehID=new_veh_ID,
        routeID=new_route_ID,
        depart=deparet_time,
        departPos=depart_position,
    )

    traci.vehicle.setParkingAreaStop(
        vehID=new_veh_ID,
        stopID=new_shelterID,
        duration=100000,
    )

    # color_mode は既存コードでもコメントアウトされているため未使用のまま維持

    NEW_VEHICLE_COUNT += 1

    return NEW_VEHICLE_COUNT


def generate_new_veh(
    target_vehID: str,
    NEW_VEHICLE_COUNT: int,
    agent_list: list,
    vehInfo_list: list,
    vehInfo_by_target_vehID: VehicleInfo,
    agent_by_target_vehID: Agent,
    shelter_list: list,
    connected_edges_list: list,
    LATE_AGENT_THRESHOLD_LIST: list,
    lane_change_mode: int,
):
    """
    既存車両 target_vehID を基点に、候補避難地のうち新たな避難地へ向かう
    新規車両を生成して置き換える。

    既存 utilities.py の generate_new_veh の挙動を維持する。
    """
    # === 現在位置と対向情報の取得 ===
    current_edgeID = traci.vehicle.getRoadID(target_vehID)
    opposite_edgeID = get_opposite_edgeID_by_edgeID(current_edgeID)

    # 同一路線判定（対向が同じなら生成不要）
    if current_edgeID == opposite_edgeID:
        return NEW_VEHICLE_COUNT

    # 候補地満杯チェック（全候補が満杯なら生成しない）
    if is_candidate_shelter_full(
        agent_by_target_vehID,
        vehInfo_by_target_vehID,
    ):
        return NEW_VEHICLE_COUNT

    # === 新しい避難地・経路端点を決定 ===
    from_edgeID, new_shelterID, to_edgeID = (
        get_new_shelterID_and_near_edgeID_by_vehID_based_on_distance(
            current_edgeID=current_edgeID,
            opposite_edgeID=opposite_edgeID,
            agent_by_target_vehID=agent_by_target_vehID,
            connected_edges_list=connected_edges_list,
        )
    )

    # 避難経路が存在しない場合は生成しない
    if from_edgeID == "" and new_shelterID == "" and to_edgeID == "":
        return NEW_VEHICLE_COUNT

    # === 経路探索（from → to） ===
    via_edgeIDs_with_intial_end_edge: list = list(
        traci.simulation.findRoute(from_edgeID, to_edgeID).edges
    )

    # === 新車両ID・時刻などの準備 ===
    vehID_num = target_vehID.split("_")[3]

    new_veh_ID: str = "{}_{}_{}_{}".format(
        "newveh",
        new_shelterID,
        vehID_num,
        NEW_VEHICLE_COUNT,
    )

    deparet_time: float = traci.simulation.getTime()

    # === 既存Agentの状態更新 & 新Agent生成 ===
    agent_by_target_vehID.set_shelter_changed_flg(True)

    # TODO: 満杯情報を受け取った後は、当該避難地を候補から削除するロジックを入れる
    updated_candidate_shelter = agent_by_target_vehID.get_candidate_shelter()

    agent: Agent = Agent(
        vehID=new_veh_ID,
        target_shelter=new_shelterID,
        tunning_threshold=copy.deepcopy(
            agent_by_target_vehID.get_tunning_threshold()
        ),
        route_change_threshold=copy.deepcopy(
            agent_by_target_vehID.get_route_change_threshold()
        ),
        lane_change_init_threshold=copy.deepcopy(
            agent_by_target_vehID.get_lane_change_decision_threshold()
        ),
        normalcy_motivation_increase=copy.deepcopy(
            agent_by_target_vehID.get_motivation_increase_from_info_receive()
        ),
        motivation_decrease_due_to_inactive_neighbors=copy.deepcopy(
            agent_by_target_vehID.get_motivation_decrease_due_to_inactive_neighbors()
        ),
        motivation_increase_due_to_following_neighbors=copy.deepcopy(
            agent_by_target_vehID.get_motivation_increase_due_to_following_neighbors()
        ),
        lane_minimum_motivation_value=copy.deepcopy(
            agent_by_target_vehID.get_minimum_motivation_value()
        ),
        shelter_occupancy_rate_threshold=copy.deepcopy(
            agent_by_target_vehID.get_shelter_occupancy_rate_threshold()
        ),
    )

    agent.set_near_edgeID_by_target_shelter(
        copy.deepcopy(agent_by_target_vehID.get_near_edgeID_by_target_shelter())
    )
    agent.set_candidate_edge_by_shelterID(updated_candidate_shelter)
    agent.init_set_candidate_near_shelter(
        shelter_edge_by_IDs=updated_candidate_shelter
    )

    agent.set_x_elapsed_time_for_lane_change_list(
        copy.deepcopy(agent_by_target_vehID.get_x_elapsed_time_for_lane_change_list())
    )
    agent.set_y_motivation_value_for_lane_change_list(
        copy.deepcopy(agent_by_target_vehID.get_y_motivation_value_for_lane_change_list())
    )
    agent.set_lane_change_xy_dict(
        copy.deepcopy(agent_by_target_vehID.get_lane_change_xy_dict())
    )

    agent.set_motivation_decrease_due_to_inactive_neighbors(
        copy.deepcopy(
            agent_by_target_vehID.get_motivation_decrease_due_to_inactive_neighbors()
        )
    )
    agent.set_motivation_increase_due_to_following_neighbors(
        copy.deepcopy(
            agent_by_target_vehID.get_motivation_increase_due_to_following_neighbors()
        )
    )
    agent.set_calculated_motivation_value(
        copy.deepcopy(agent_by_target_vehID.get_calculated_motivation_value())
    )

    agent_list.append(agent)

    # === VehicleInfo を継承して新規作成 ===
    new_vehInfo_by_target_vehID: VehicleInfo = VehicleInfo(
        vehID=new_veh_ID,
        target_shelter=new_shelterID,
        edgeID_connect_target_shelter=to_edgeID,
        create_time=deparet_time,
    )

    new_vehInfo_by_target_vehID.set_shelter_congestion_info(
        copy.deepcopy(vehInfo_by_target_vehID.get_shelter_congestion_info())
    )
    new_vehInfo_by_target_vehID.set_avg_evac_time_by_route_by_recive_time(
        copy.deepcopy(
            vehInfo_by_target_vehID.get_avg_evac_time_by_route_by_recive_time()
        )
    )
    new_vehInfo_by_target_vehID.set_tsunami_precursor_info(
        copy.deepcopy(vehInfo_by_target_vehID.get_tsunami_precursor_info())
    )

    vehInfo_list.append(new_vehInfo_by_target_vehID)

    # === 置換のための出発位置計算 ===
    edge_position = traci.vehicle.getLanePosition(target_vehID)
    current_laneID = traci.vehicle.getLaneID(target_vehID)
    current_lane_len = traci.lane.getLength(current_laneID)

    if current_lane_len > 100:
        depart_position = 200 - edge_position
    else:
        depart_position = 100 - edge_position

    # === 旧車両を削除し、新車両を追加 ===
    traci.vehicle.remove(target_vehID)

    new_route_ID: str = "{}_{}_{}".format(
        "newroute",
        new_shelterID,
        NEW_VEHICLE_COUNT,
    )

    traci.route.add(
        routeID=new_route_ID,
        edges=via_edgeIDs_with_intial_end_edge,
    )

    traci.vehicle.add(
        vehID=new_veh_ID,
        routeID=new_route_ID,
        depart=deparet_time,
        departPos=depart_position,
    )

    traci.vehicle.setParkingAreaStop(
        vehID=new_veh_ID,
        stopID=new_shelterID,
        duration=100000,
    )

    init_driver_behavior(
        vehIDs=[new_veh_ID],
        lane_change_mode=lane_change_mode,
    )

    # === カウンタ更新 ===
    NEW_VEHICLE_COUNT += 1

    return NEW_VEHICLE_COUNT
