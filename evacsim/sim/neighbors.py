# =========================
# Neighbor vehicle helpers
# =========================
#
# 注意:
# このモジュールは TraCI に依存する。
# 対象車両の周辺 edge / 周辺車両 ID の取得を扱う。

from __future__ import annotations

from evacsim.sim.sumo_env import ensure_sumo_tools_on_path

ensure_sumo_tools_on_path()

import traci

from evacsim.maps.edge_utils import (
    get_custome_edge_by_edgeID,
    remove_junction_from_edgeID,
)
from evacsim.sim.traci_cache import get_edge_vehicle_ids_cached


def get_around_edgeIDs(
    target_vehID: str,
    custome_edge_list: list,
):
    """
    車両IDを元に、周辺 edgeID を取得する。

    既存 utilities.py の get_around_edgeIDs の挙動を維持する。
    """
    # 車両がいる edgeID を取得
    current_edgeID = traci.vehicle.getRoadID(target_vehID)

    # 車両がいる edgeID をもとに、周辺 edgeID を取得する
    current_edge = get_custome_edge_by_edgeID(
        current_edgeID,
        custome_edge_list,
    )

    around_edgeIDs_with_junction = remove_junction_from_edgeID(
        current_edge.around_edgeIDs()
    )

    return around_edgeIDs_with_junction


def get_around_vehIDs(
    target_vehID: str,
    custome_edge_list: list,
    step_cache=None,
):
    """
    車両IDを元に周辺 edgeID を取得し、さらに周辺車両IDを取得する。

    get_edge_vehicle_ids_cached を使うため、
    step_cache が渡された場合は TraCI 呼び出しをキャッシュする。
    """
    around_edgeIDs_for_target_vehID = get_around_edgeIDs(
        target_vehID,
        custome_edge_list,
    )

    around_vehIDs = []

    for around_edgeID in around_edgeIDs_for_target_vehID:
        tmp_vehIDs = get_edge_vehicle_ids_cached(
            around_edgeID,
            step_cache=step_cache,
        )
        around_vehIDs.extend(tmp_vehIDs)

    return around_vehIDs
