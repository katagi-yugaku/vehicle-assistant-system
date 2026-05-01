# =========================
# Communication helpers
# =========================
#
# 注意:
# このモジュールは TraCI に依存する。
# V2Shelter / V2V / 津波情報共有など、情報伝達処理を扱う。

from __future__ import annotations

from typing import TYPE_CHECKING

from evacsim.sim.sumo_env import ensure_sumo_tools_on_path

ensure_sumo_tools_on_path()

import traci

from evacsim.sim.traci_cache import get_vehicle_position_cached
from evacsim.utils.lookup import (
    find_agent_by_vehID,
    find_vehInfo_by_vehID,
    find_shelter_by_edgeID_connect_target_shelter,
)
from evacsim.sim.routing_distance import distance_each_vehIDs

if TYPE_CHECKING:
    from evacsim.agents.Agent import Agent
    from evacsim.agents.Shelter import Shelter
    from evacsim.agents.VehicleInfo import VehicleInfo


def merge_route_info_within_shelters(
    one_shelter: Shelter,
    another_shelter: Shelter,
):
    """
    2つの避難所が持つ route ごとの平均避難時間情報を統合する。

    同じ route が存在する場合は、vehicles 数で重み付き平均を取る。
    既存 utilities.py の merge_route_info_within_shelters の挙動を維持する。
    """
    one_shelter_has_route_info = one_shelter.get_avg_evac_time_by_route()
    another_shelter_has_route_info = another_shelter.get_avg_evac_time_by_route()

    merged_info = {}

    # shelterA の情報をまずコピー
    for route, data in one_shelter_has_route_info.items():
        merged_info[route] = data.copy()

    # shelterB の情報をマージ
    for route, data in another_shelter_has_route_info.items():
        if route in merged_info:
            # 同じルートがある場合は、重み付き平均
            total_vehicles = (
                merged_info[route]["vehicles"]
                + data["vehicles"]
            )

            if total_vehicles == 0:
                avg_time = 0
            else:
                avg_time = (
                    merged_info[route]["avg_time"]
                    * merged_info[route]["vehicles"]
                    + data["avg_time"] * data["vehicles"]
                ) / total_vehicles

            merged_info[route]["avg_time"] = avg_time
            merged_info[route]["vehicles"] = total_vehicles

        else:
            # 違うルートならそのまま追加
            merged_info[route] = data.copy()

    one_shelter.set_avg_evac_time_by_route(merged_info)
    another_shelter.set_avg_evac_time_by_route(merged_info)


def v2shelter_communication(
    target_vehID: str,
    shelterID: str,
    vehInfo_list: list,
    shelter_list: list,
    COMMUNICATION_RANGE: float,
    target_vehInfo: VehicleInfo = None,
    target_position=None,
    vehInfo_by_vehID_dict: dict = None,
    step_cache=None,
):
    """
    車両と避難所の通信処理。

    処理:
      - 対象車両が避難所の通信範囲内にいるか判定
      - 避難所混雑率を VehicleInfo に反映
      - 避難所が持つ経路別平均避難時間を VehicleInfo に反映
      - 受信時刻つき route 情報を更新

    既存 utilities.py の v2shelter_communication の挙動を維持する。
    """
    if target_vehInfo is None:
        target_vehInfo = find_vehInfo_by_vehID(
            target_vehID,
            vehInfo_list=vehInfo_list,
            vehInfo_by_vehID_dict=vehInfo_by_vehID_dict,
        )

    if target_vehInfo is None:
        return

    shelter_for_target_vehID: Shelter = (
        find_shelter_by_edgeID_connect_target_shelter(
            target_vehInfo.get_edgeID_connect_target_shelter(),
            shelter_list,
        )
    )

    if target_position is None:
        target_position = get_vehicle_position_cached(
            target_vehID,
            step_cache=step_cache,
        )

    if (
        distance_each_vehIDs(
            target_position,
            shelter_for_target_vehID.get_position(),
        )
        < COMMUNICATION_RANGE
    ):
        current_time = traci.simulation.getTime()

        current_congestion_rate_by_shelterID = (
            shelter_for_target_vehID.get_congestion_rate()
        )

        target_vehInfo.update_shelter_congestion_info(
            shelterID=shelterID,
            congestion=current_congestion_rate_by_shelterID,
            time_stamp=current_time,
        )

        avg_evac_time_by_route = (
            shelter_for_target_vehID.get_avg_evac_time_by_route()
        )

        target_vehInfo.v2shelter_update_avg_evac_time_by_route(
            avg_evac_time_by_route
        )

        target_vehInfo.v2v_avg_evac_time_by_route_by_recive_time(
            current_time=current_time
        )


def v2v_communication(
    target_vehID: str,
    target_vehInfo: VehicleInfo,
    around_vehIDs: list,
    agent_list: list,
    vehInfo_list: list,
    COMMUNICATION_RANGE: float,
    target_position=None,
    agent_by_vehID_dict: dict = None,
    vehInfo_by_vehID_dict: dict = None,
    step_cache=None,
):
    """
    車両間通信処理。

    処理:
      - 通信範囲内の周辺車両を対象にする
      - 避難地混雑情報について、新しい timestamp の情報を伝播する
      - 経路別平均避難時間情報について、新しい受信時刻の情報を伝播する

    既存 utilities.py の v2v_communication の挙動を維持する。
    """
    target_agent: Agent = find_agent_by_vehID(
        target_vehID,
        agent_list=agent_list,
        agent_by_vehID_dict=agent_by_vehID_dict,
    )

    if target_agent is None:
        return

    if target_position is None:
        target_position = get_vehicle_position_cached(
            target_vehID,
            step_cache=step_cache,
        )

    for around_vehID in around_vehIDs:
        around_position = get_vehicle_position_cached(
            around_vehID,
            step_cache=step_cache,
        )

        if (
            distance_each_vehIDs(target_position, around_position)
            < COMMUNICATION_RANGE
        ):
            around_vehInfo: VehicleInfo = find_vehInfo_by_vehID(
                around_vehID,
                vehInfo_list=vehInfo_list,
                vehInfo_by_vehID_dict=vehInfo_by_vehID_dict,
            )

            if around_vehInfo is None:
                continue

            for candidate_shelter, near_edgeID in (
                target_agent.get_candidate_edge_by_shelterID().items()
            ):
                target_of_congestion_info_time = (
                    target_vehInfo.get_latest_time_stamp_of_shelter(
                        candidate_shelter
                    )
                )

                around_of_congestion_info_time = (
                    around_vehInfo.get_latest_time_stamp_of_shelter(
                        candidate_shelter
                    )
                )

                if (
                    target_of_congestion_info_time
                    != around_of_congestion_info_time
                ):
                    info_source = (
                        target_vehInfo
                        if target_of_congestion_info_time
                        > around_of_congestion_info_time
                        else around_vehInfo
                    )

                    info_target = (
                        around_vehInfo
                        if target_of_congestion_info_time
                        > around_of_congestion_info_time
                        else target_vehInfo
                    )

                    info_target.set_shelter_congestion_info(
                        info_source.get_shelter_congestion_info()
                    )

            target_of_route_info_tuple, = (
                target_vehInfo
                .get_avg_evac_time_by_route_by_recive_time()
                .items()
            )
            target_of_route_info_time = target_of_route_info_tuple[0]

            around_of_route_info_tuple, = (
                around_vehInfo
                .get_avg_evac_time_by_route_by_recive_time()
                .items()
            )
            around_of_route_info_time = around_of_route_info_tuple[0]

            if (
                (
                    target_of_route_info_tuple[1]
                    or around_of_route_info_tuple[1]
                )
                and target_of_route_info_time != around_of_route_info_time
            ):
                info_source = (
                    target_vehInfo
                    if target_of_route_info_time > around_of_route_info_time
                    else around_vehInfo
                )

                info_target = (
                    around_vehInfo
                    if target_of_route_info_time > around_of_route_info_time
                    else target_vehInfo
                )

                info_target.set_avg_evac_time_by_route_by_recive_time(
                    info_source.get_avg_evac_time_by_route_by_recive_time()
                )


def v2v_communication_about_tsunami_info(
    target_vehID: str,
    target_vehInfo: VehicleInfo,
    around_vehIDs: list,
    vehInfo_list: list,
    COMMUNICATION_RANGE: float,
    target_position=None,
    vehInfo_by_vehID_dict: dict = None,
    step_cache=None,
):
    """
    車両間で津波前兆情報を共有する。

    処理:
      - 通信範囲内の車両同士で津波情報を比較
      - 片方だけ情報を持つ場合は相手へ伝播
      - 両方が情報を持つ場合は、より早い時刻の情報を採用

    既存 utilities.py の v2v_communication_about_tsunami_info の挙動を維持する。
    """
    if target_position is None:
        target_position = get_vehicle_position_cached(
            target_vehID,
            step_cache=step_cache,
        )

    for around_vehID in around_vehIDs:
        if target_vehID == around_vehID:
            continue

        around_position = get_vehicle_position_cached(
            around_vehID,
            step_cache=step_cache,
        )

        if (
            distance_each_vehIDs(target_position, around_position)
            < COMMUNICATION_RANGE
        ):
            around_vehInfo: VehicleInfo = find_vehInfo_by_vehID(
                around_vehID,
                vehInfo_list=vehInfo_list,
                vehInfo_by_vehID_dict=vehInfo_by_vehID_dict,
            )

            if around_vehInfo is None:
                continue

            target_of_tsunami_info_tuple = (
                target_vehInfo.get_tsunami_precursor_info()
            )
            target_of_tsunami_info_time_with_flag = list(
                target_of_tsunami_info_tuple.values()
            )[0]

            around_of_tsunami_info_tuple = (
                around_vehInfo.get_tsunami_precursor_info()
            )
            around_of_tsunami_info_time_with_flag = list(
                around_of_tsunami_info_tuple.values()
            )[0]

            target_flag = target_of_tsunami_info_time_with_flag[0]
            around_flag = around_of_tsunami_info_time_with_flag[0]

            if target_flag and not around_flag:
                around_vehInfo.set_tsunami_precursor_info(
                    target_vehInfo.get_tsunami_precursor_info()
                )

            elif not target_flag and around_flag:
                target_vehInfo.set_tsunami_precursor_info(
                    around_vehInfo.get_tsunami_precursor_info()
                )

            elif target_flag and around_flag:
                target_time = target_of_tsunami_info_time_with_flag[1]
                around_time = around_of_tsunami_info_time_with_flag[1]

                if target_time < around_time:
                    around_vehInfo.set_tsunami_precursor_info(
                        target_vehInfo.get_tsunami_precursor_info()
                    )

                elif around_time < target_time:
                    target_vehInfo.set_tsunami_precursor_info(
                        around_vehInfo.get_tsunami_precursor_info()
                    )
