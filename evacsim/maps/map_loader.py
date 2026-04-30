# =========================
# Map / network loader helpers
# =========================
#
# 注意:
# このモジュールは TraCI に依存する。
# SUMO network から CustomeEdge, Shelter, connected_edges_list を初期化する処理を扱う。
#
# NOTE:
# 既存コード互換のため、CustomeEdge / init_custome_edge の typo は維持する。

from __future__ import annotations

from typing import List

from evacsim.sim.sumo_env import ensure_sumo_tools_on_path

ensure_sumo_tools_on_path()

import traci

from evacsim.agents.CustomeEdge import CustomeEdge
from evacsim.agents.Shelter import Shelter
from evacsim.maps.edge_utils import is_near_shelterID_on_opposite_edges


def init_custome_edge() -> List["CustomeEdge"]:
    """
    SUMO network 上の全 edgeID から CustomeEdge のリストを生成する。

    処理:
      - traci.edge.getIDList() で全 edgeID を取得
      - 各 edgeID について CustomeEdge を生成
      - 反対方向 edgeID を初期設定
      - start / end junction を初期設定

    既存 utilities.py の init_custome_edge の挙動を維持する。
    """
    custome_edge_list: List[CustomeEdge] = []

    # 全ての edgeID を取得する
    edgeIDs: list[str] = traci.edge.getIDList()

    # edgeID をもとに CustomEdge を生成
    for edgeID in edgeIDs:
        current_edgeIDs = [
            custome_edge.get_current_edgeID()
            for custome_edge in custome_edge_list
        ]

        if edgeID not in current_edgeIDs:
            custome_edge: CustomeEdge = CustomeEdge(edgeID)

            custome_edge.setting_init_opposite_edgeID(
                edgeIDs=edgeIDs,
            )

            custome_edge.setting_init_start_end_junctions()

            custome_edge_list.append(custome_edge)

    return custome_edge_list


def init_shelter(
    shelterID: str,
    shelter_capacity_by_ID: dict,
    near_edgeID: str,
    shelter_list: list,
) -> list:
    """
    Shelter を生成し、位置情報を設定して shelter_list に追加する。

    処理:
      - shelterID と capacity と near_edgeID から Shelter を生成
      - parkingArea の laneID を取得
      - lane shape の中心を避難所位置として設定
      - shelter_list に append

    既存 utilities.py の init_shelter の挙動を維持する。
    """
    shelter: Shelter = Shelter(
        shelterID=shelterID,
        capacity=shelter_capacity_by_ID[shelterID],
        near_edgeID=near_edgeID,
    )

    # 避難所のレーンIDを取得
    laneID_on_sheler = traci.parkingarea.getLaneID(shelterID)

    shelter_position_elements = traci.lane.getShape(laneID_on_sheler)

    shelter_position = (
        (
            shelter_position_elements[0][0]
            + shelter_position_elements[-1][0]
        )
        / 2,
        (
            shelter_position_elements[0][1]
            + shelter_position_elements[-1][1]
        )
        / 2,
    )

    shelter.set_position(shelter_position)

    shelter_list.append(shelter)

    return shelter_list


def init_connected_edges_list(
    custome_edge_list: list,
):
    """
    CustomEdge のリストから到達可能な edge 間接続リストを生成する。

    戻り値は既存 utilities.py と同じく、
    ConnectedEdges オブジェクトではなく以下の tuple list とする。

        [
            (one_edge, other_edge, via_edges),
            ...
        ]

    既存 utilities.py の init_connected_edges_list の挙動を維持する。
    """
    connected_edges_list = []

    edgeIDs = [
        custome_edge.get_current_edgeID()
        for custome_edge in custome_edge_list
    ]

    for one_edge in edgeIDs:
        for other_edge in edgeIDs:
            # 同一 edge はスキップ
            if (
                not one_edge == other_edge
                and is_near_shelterID_on_opposite_edges(one_edge, other_edge)
                and not one_edge.startswith(":J")
                and not other_edge.startswith(":J")
            ):
                via_edges = list(
                    traci.simulation.findRoute(
                        one_edge,
                        other_edge,
                    ).edges
                )

                if len(via_edges) > 0:
                    connected_edges_list.append(
                        (
                            one_edge,
                            other_edge,
                            via_edges,
                        )
                    )

    return connected_edges_list
