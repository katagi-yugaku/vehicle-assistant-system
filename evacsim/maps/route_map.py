# =========================
# Route map utilities
# =========================
#
# 注意:
# このモジュールは TraCI に依存する。
# start edge / end edge 間の到達可能性判定や、
# start edge から近い end edge の並び替えを扱う。

from __future__ import annotations

from math import sqrt

from evacsim.sim.sumo_env import ensure_sumo_tools_on_path

ensure_sumo_tools_on_path()

import traci


def get_vehicle_end_list_by_start_edge_dict(
    vehicle_start_edges: list,
    vehicle_end_edges: list,
) -> dict[str, list[str]]:
    """
    start edge ごとに、到達可能な end edge のリストを作成する。

    既存 utilities.py の get_vehicle_end_list_by_start_edge_dict の挙動を維持する。
    """
    vehicle_end_list_by_start_edge_dict: dict = {}

    for start_edge in vehicle_start_edges:
        tmp_list = []

        for end_edge in vehicle_end_edges:
            route_edges = list(
                traci.simulation.findRoute(
                    start_edge.get_current_edgeID(),
                    end_edge.get_current_edgeID(),
                ).edges
            )

            if len(route_edges) > 0:
                tmp_list.append(end_edge.get_current_edgeID())

        vehicle_end_list_by_start_edge_dict[
            start_edge.get_current_edgeID()
        ] = tmp_list

    return vehicle_end_list_by_start_edge_dict


def get_nearest_end_edgeID_by_start_edgeID(
    vehicle_end_list_by_start_edge_dict: dict,
):
    """
    start edge ごとに、到達可能な end edge を距離が近い順に並べる。

    距離は、start edge の lane shape の中心点と、
    end edge の lane shape の中心点のユークリッド距離で計算する。

    既存 utilities.py の get_nearest_end_edgeID_by_start_edgeID の挙動を維持する。
    """
    nearest_end_edgeID_by_start_edgeID_dict: dict = {}

    for vehicle_start_edgeID, vehicle_end_edgeID_list in (
        vehicle_end_list_by_start_edge_dict.items()
    ):
        start_edge_shape = traci.lane.getShape(f"{vehicle_start_edgeID}_0")
        start_edge_center = (
            (start_edge_shape[0][0] + start_edge_shape[1][0]) / 2,
            (start_edge_shape[0][1] + start_edge_shape[1][1]) / 2,
        )

        disatnce_by_start_edge_ID: dict = {}

        for vehicle_end_edgeID in vehicle_end_edgeID_list:
            end_edge_shape = traci.lane.getShape(f"{vehicle_end_edgeID}_0")
            end_edge_center = (
                (end_edge_shape[0][0] + end_edge_shape[1][0]) / 2,
                (end_edge_shape[0][1] + end_edge_shape[1][1]) / 2,
            )

            distance = sqrt(
                (start_edge_center[0] - end_edge_center[0]) ** 2
                + (start_edge_center[1] - end_edge_center[1]) ** 2
            )

            disatnce_by_start_edge_ID[vehicle_end_edgeID] = distance

        sorted_disatnce_by_start_edgeID = sorted(
            disatnce_by_start_edge_ID.items(),
            key=lambda x: x[1],
        )

        nearest_end_edgeID_by_start_edgeID_dict[vehicle_start_edgeID] = [
            edgeID
            for edgeID, _ in sorted_disatnce_by_start_edgeID
        ]

    return nearest_end_edgeID_by_start_edgeID_dict
