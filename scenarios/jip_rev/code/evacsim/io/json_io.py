# =========================
# JSON I/O helpers
# =========================
#
# 注意:
# このモジュールの import 時点では TraCI を import しない。
# ConnectedEdges が必要な関数内でのみ遅延 import する。

import json


def export_start_end_edgeIDs_to_json(
    start_end_edgeIDs: dict,
    file_path: str,
):
    """
    start/end edgeIDs の辞書を JSON ファイルへ出力する。

    既存 utilities.py の挙動を維持する。
    """
    with open(file_path, "w") as f:
        json.dump(start_end_edgeIDs, f, indent=4)

    print(f"JSONファイルをエクスポートしました: {file_path}")


def import_start_end_edgeIDs_from_json(file_path: str):
    """
    start/end edgeIDs の辞書を JSON ファイルから読み込む。

    既存 utilities.py の挙動を維持する。
    """
    with open(file_path, "r") as f:
        start_end_edgeIDs = json.load(f)

    return start_end_edgeIDs


def export_connected_edges_to_json(
    connected_edges_list: list,
    file_path: str,
):
    """
    ConnectedEdges 相当のリストを JSON ファイルへ出力する。

    既存 utilities.py の挙動では、connected_edges_list の各要素を
    start, end, path に unpack しているため、その仕様を維持する。
    """
    connected_edges_json = [
        {
            "start": start,
            "end": end,
            "path": path,
        }
        for start, end, path in connected_edges_list
    ]

    with open(file_path, "w", encoding="utf-8") as f:
        json.dump(connected_edges_json, f, ensure_ascii=False, indent=4)

    print("JSONファイルに出力しました")


def import_connected_edges_from_json(file_path: str):
    """
    JSON ファイルから ConnectedEdges のリストを復元する。

    ConnectedEdges は evacsim.agents.CustomeEdge に定義されているが、
    CustomeEdge.py が traci を import するため、ここでは関数内で遅延 import する。
    """
    from evacsim.sim.sumo_env import ensure_sumo_tools_on_path

    ensure_sumo_tools_on_path()

    from evacsim.agents.CustomeEdge import ConnectedEdges

    connected_edges_list = []

    with open(file_path, "r", encoding="utf-8") as f:
        data = json.load(f)

        for item in data:
            one_edge = item["start"]
            other_edge = item["end"]
            via_edges = item["path"]

            connected_edges = ConnectedEdges(
                start_edgeID=one_edge,
                end_edgeID=other_edge,
                via_edgeIDs=via_edges,
            )

            connected_edges_list.append(connected_edges)

    return connected_edges_list
