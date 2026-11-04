# =========================
# Route file / XML helpers
# =========================
#
# 注意:
# このモジュールでは TraCI を import しない。
# SUMO の route XML / trip XML の読み書きだけを扱う。

from __future__ import annotations

from typing import TYPE_CHECKING
import xml.etree.ElementTree as ET

if TYPE_CHECKING:
    from evacsim.agents.Shelter import Shelter


VEHICLE_SHELTER_DURATION_TIME = 10000


def generate_initial_vehIDs_for_row_xml(
    start_edge: str,
    end_edge: str,
    via_edges: str,
    depart_time: float,
    interval: float,
    veh_count: int,
    vehs_written_list: list,
    shelter: Shelter,
):
    """
    初期車両の <trip> 行を生成し、vehs_written_list に追加する。

    既存 utilities.py の generate_initial_vehIDs_for_row_xml の挙動を維持する。
    """
    for num in range(veh_count):
        veh_line: str = (
            f'<trip id="nt_{shelter.get_shelterID()}_{num}" '
            f'depart="{depart_time}" '
            f'from="{start_edge}" '
            f'to="{end_edge}" '
            f'via="{via_edges}"> '
            f'<stop parkingArea="{shelter.get_shelterID()}" '
            f'duration="{VEHICLE_SHELTER_DURATION_TIME}"/></trip>\n'
        )

        tmp_tuple: tuple = (depart_time, veh_line)
        vehs_written_list.append(tmp_tuple)

        veh_count += 1
        depart_time += interval

    return vehs_written_list


def write_initial_vehIDs_for_row_xml(
    file_path: str,
    veh_written_list: list,
):
    """
    veh_written_list の <trip> 行を XML ファイル末尾直前に挿入する。

    既存 utilities.py の write_initial_vehIDs_for_row_xml の挙動を維持する。
    """
    sorted_by_key = sorted(veh_written_list, key=lambda x: x[0])
    veh_written_list = [value for key, value in sorted_by_key]

    print("書き込みを開始します")

    with open(file_path, "r") as file:
        current_content = file.readlines()

    for park_written in veh_written_list:
        try:
            with open(file_path, "r") as file:
                current_content = file.readlines()

            lines_num = len(current_content)
            current_content.insert(lines_num - 1, park_written)

            with open(file_path, "w") as file:
                file.writelines(current_content)

        except Exception as e:
            print(f"ファイルの書き込み中にエラーが発生しました: {e}")


def clean_vehIDs_for_row_xml(
    file_path: str,
    START_WRITTEN_LINE: int,
    END_WRITTEN_LINE: int,
):
    """
    XML ファイルから指定行範囲を削除する。

    既存 utilities.py の clean_vehIDs_for_row_xml の挙動を維持する。
    行番号は 1 始まり。
    """
    with open(file_path, "r") as file:
        lines = file.readlines()

    lines_to_keep = [
        line
        for i, line in enumerate(lines, start=1)
        if not (START_WRITTEN_LINE <= i <= END_WRITTEN_LINE)
    ]

    with open(file_path, "w") as file:
        file.writelines(lines_to_keep)


def convert_routefile_to_routes_by_id(file_path):
    """
    SUMO route file から routeID ごとの edges list を作る。

    Returns:
        {
            route_id: [edgeID, edgeID, ...]
        }
    """
    tree = ET.parse(file_path)
    root = tree.getroot()

    route_dict = {}

    for route in root.findall("route"):
        route_id = route.get("id")
        edges_str = route.get("edges")

        # スペース区切り → リスト化
        edges_list = edges_str.split()

        route_dict[route_id] = edges_list

    return route_dict
