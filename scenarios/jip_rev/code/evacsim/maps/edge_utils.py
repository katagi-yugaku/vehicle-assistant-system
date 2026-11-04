# =========================
# Edge / junction utilities
# =========================
#
# 注意:
# このモジュールは一部 TraCI に依存する。
# edgeID の文字列処理、CustomeEdge 探索、反対方向 edge の取得、
# route 存在判定などを扱う。

from __future__ import annotations

from evacsim.sim.sumo_env import ensure_sumo_tools_on_path

ensure_sumo_tools_on_path()

import traci


def get_custome_edge_by_edgeID(
    edgeID: str,
    custome_edge_list: list,
):
    """
    CustomEdge リストから特定の edgeID を持つ CustomEdge を取得する。

    NOTE:
    元の関数名 get_custome_edge_by_edgeID の typo は維持する。
    """
    for custome_edge in custome_edge_list:
        if custome_edge.get_current_edgeID() == edgeID:
            return custome_edge


def find_customedge_by_edgeID(
    edgeID: str,
    custome_edge_list: list,
):
    """
    edgeID から CustomEdge を取得する。

    get_custome_edge_by_edgeID と同等だが、
    既存 utilities.py に両方の名前があるため互換のため残す。
    """
    for custome_edge in custome_edge_list:
        if custome_edge.get_current_edgeID() == edgeID:
            return custome_edge


def get_edgeIDs_without_junction(customeEdge):
    """
    CustomeEdge の incoming / outgoing edgeID から junction edge を除外する。
    """
    inco_edgeIDs = [
        inco_edgeID
        for inco_edgeID in customeEdge.obtain_neighbour_incom_edgeIDs_with_junc_by_start_junc()
        if not inco_edgeID.startswith(":")
    ]

    outgo_edgeIDs = [
        outgo_edgeID
        for outgo_edgeID in customeEdge.obtain_neighbour_outcom_edgeIDs_with_junc_by_end_junc()
        if not outgo_edgeID.startswith(":")
    ]

    return inco_edgeIDs, outgo_edgeIDs


def remove_junction_from_edgeID(edgeIDs: list):
    """
    junction edge を除外した edgeID list を返す。

    SUMO の internal edge は ':' で始まるため、それを除外する。
    """
    return [edgeID for edgeID in edgeIDs if not edgeID.startswith(":")]


def get_vehicle_start_edges(custome_edge_list: list) -> list:
    """
    開始エッジ CustomEdge だけを抽出して返す。
    """
    vehicle_start_edges = []

    for custome_edge in custome_edge_list:
        if custome_edge.is_current_edgeID_start_edge():
            vehicle_start_edges.append(custome_edge)

    return vehicle_start_edges


def get_vehicle_end_edges(custome_edge_list: list) -> list:
    """
    終了エッジ CustomEdge だけを抽出して返す。
    """
    vehicle_end_edges = []

    for custome_edge in custome_edge_list:
        if custome_edge.is_current_edgeID_end_edge():
            vehicle_end_edges.append(custome_edge)

    return vehicle_end_edges


def get_opposite_edgeID_by_edgeID(edgeID: str):
    """
    edgeID に対応する反対方向 edgeID を返す。

    反対方向 edge が SUMO network 上に存在しない場合は、元の edgeID を返す。
    """
    if edgeID.startswith("-"):
        opposite_edgeID: str = edgeID.lstrip("-")
    else:
        opposite_edgeID: str = "-" + edgeID

    if opposite_edgeID in traci.edge.getIDList():
        return opposite_edgeID
    else:
        return edgeID


def is_has_middle_edge(
    current_route_edgeIDs,
    middle_edge_ID_list,
):
    """
    current_route_edgeIDs に middle_edge_ID_list のいずれかが含まれるかを返す。
    """
    return any(edge in current_route_edgeIDs for edge in middle_edge_ID_list)


def is_near_shelterID_on_opposite_edges(
    current_ID,
    near_edgeID,
):
    """
    current_ID と near_edgeID が反対方向側の edge かを判定する。

    - '-' を除いた ID が同じ場合は False
    - 片方だけ '-' で始まる場合は True
    - それ以外は False

    既存 utilities.py の挙動を維持する。
    """
    # -を除いたIDが同じ場合は False（反対車線だが同じ場所なので対象外）
    if current_ID.lstrip("-") == near_edgeID.lstrip("-"):
        return False

    # currentとnearの片方が負、片方が正なら True（反対側）
    if current_ID.startswith("-") and not near_edgeID.startswith("-"):
        return True
    elif not current_ID.startswith("-") and near_edgeID.startswith("-"):
        return True
    else:
        return False


def is_pre_edgeID_near_shelter(
    current_edgeID: str,
    edgeID_near_shelter: str,
    custome_edge_list: list,
):
    """
    current_edgeID が、避難地近傍 edge の手前 edge かどうかを判定する。

    既存 utilities.py の挙動を維持する。
    見つからない場合は None を返す。
    """
    edge_near_shelter = find_customedge_by_edgeID(
        edgeID_near_shelter,
        custome_edge_list,
    )

    try:
        pre_edge_tuple: tuple = (
            edge_near_shelter.obtain_neighbour_outgo_edgeIDs_with_junc_by_end_junc()
        )
    except Exception:
        pre_edge_tuple = ()

    for pre_edge in pre_edge_tuple:
        if current_edgeID == pre_edge:
            return True


def is_route_exist(
    current_edgeID: str,
    near_edgeID: str,
    connected_edges_list: list,
):
    """
    connected_edges_list 内に current_edgeID -> near_edgeID の接続があるか判定する。

    connected_edges は get_start_edgeID(), get_end_edgeID() を持つ想定。
    """
    for connected_edges in connected_edges_list:
        if (
            connected_edges.get_start_edgeID() == current_edgeID
            and connected_edges.get_end_edgeID() == near_edgeID
        ):
            return True

    return False
