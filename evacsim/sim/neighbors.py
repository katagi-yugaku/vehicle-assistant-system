# =========================
# Neighbor vehicle helpers
# =========================
#
# 注意:
# このモジュールは TraCI に依存する。
# 対象車両の周辺 edge / 周辺車両 ID の取得を扱う。

from __future__ import annotations

from typing import TYPE_CHECKING

import numpy as np
import traci

if TYPE_CHECKING:
    from evacsim.agents.Agent import Agent
    from evacsim.agents.VehicleInfo import VehicleInfo

from evacsim.sim.sumo_env import ensure_sumo_tools_on_path

ensure_sumo_tools_on_path()

from evacsim.maps.edge_utils import (
    get_custome_edge_by_edgeID,
    remove_junction_from_edgeID,
)
from evacsim.utils.lookup import (
    find_shelter_by_edgeID_connect_target_shelter,
    find_shelterID_by_edgeID_by_shelterID,
    find_agent_by_vehID
)

from evacsim.maps.edge_utils import (
    is_pre_edgeID_near_shelter,
    get_vehicle_start_edges,
    get_vehicle_end_edges,
    get_opposite_edgeID_by_edgeID,
)
from evacsim.sim.traci_cache import(
    get_edge_vehicle_ids_cached,
    get_vehicle_position_cached,
)

from evacsim.sim.routing_distance import distance_each_vehIDs


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

def count_rc_around_vehicles(
    agent: Agent,
    vehInfo: VehicleInfo,
    agent_list: list[Agent],
    candidate_action: str,
    agent_by_vehID_dict: dict,
    custome_edge_list: list,
    current_edgeID: str,
    distance_threshold: float = 10.0,
):
    step_cache = None
    target_vehID = agent.get_vehID()

    try:
        target_position = get_vehicle_position_cached(
            target_vehID,
            step_cache=step_cache,
        )
    except Exception as e:
        print(f"Failed to get target vehicle position: vehID={target_vehID}, error={e}")
        return 0
    opposite_moving_vehIDs = []
    try:
        opposite_edgeID = get_opposite_edgeID_by_edgeID(current_edgeID)

        if opposite_edgeID:
            opposite_moving_vehIDs = list(
                traci.edge.getLastStepVehicleIDs(opposite_edgeID)
            )
            # if len(opposite_moving_vehIDs) > 0:
            #     print(f"Found opposite moving vehicles: vehID={target_vehID}, current_edgeID:{current_edgeID} opposite_edgeID:{opposite_edgeID} opposite_moving_vehIDs={opposite_moving_vehIDs}, count={len(opposite_moving_vehIDs)}")
    except Exception as e:
        print(
            f"Failed to get opposite moving vehicles: "
            f"vehID={target_vehID}, current_edgeID={current_edgeID}, error={e}"
        )
        opposite_moving_vehIDs = []

    try:
        around_vehIDs = get_around_vehIDs(
            target_vehID=target_vehID,
            custome_edge_list=custome_edge_list,
            step_cache=step_cache,
        )
    except Exception as e:
        print(f"Failed to get around vehicles: vehID={target_vehID}, error={e}")
        around_vehIDs = []

    # tuple + list 問題を避ける
    # 同じ車両が重複して数えられるのも防ぐ
    candidate_vehIDs = set(opposite_moving_vehIDs) | set(around_vehIDs)
    same_action_count = 0
    for other_vehID in candidate_vehIDs:
        if other_vehID == target_vehID:
            continue

        other_agent = find_agent_by_vehID(
            vehID=other_vehID,
            agent_list=agent_list,
            agent_by_vehID_dict=agent_by_vehID_dict,
        )
        if other_agent is None:
            continue

        try:
            other_agent_position = get_vehicle_position_cached(
                other_agent.get_vehID(),
                step_cache=step_cache,
            )
        except Exception:
            continue

        distance = distance_each_vehIDs(
            target_position,
            other_agent_position,
        )

        if distance > distance_threshold:
            continue

        if (
            other_agent.get_evacuation_route_changed_flg()
        ):
            same_action_count += 1
            # print(f"Found same action vehicle: target_vehID={target_vehID}, other_vehID={other_vehID}, distance={distance:.2f}")
        # else:
        #     print(f"Found different action vehicle: target_vehID={target_vehID}, other_vehID={other_vehID}, distance={distance:.2f} simulation_step={traci.simulation.getTime()}")

    return same_action_count