# =========================
# Arrival handlers
# =========================
#
# 注意:
# このモジュールは TraCI に依存する。
# 車両・歩行者の避難地到着時処理を扱う。

from __future__ import annotations

from typing import TYPE_CHECKING

from evacsim.sim.sumo_env import ensure_sumo_tools_on_path

ensure_sumo_tools_on_path()

import traci

from evacsim.utils.lookup import find_shelter_by_edgeID_connect_target_shelter

# B6(改訂): 共有する旅行時間から差し引く値。元のコードは 100 s 固定。
# runner が設定 travel_time_offset に応じて書き換える。
TRAVEL_TIME_OFFSET: float = 100.0

# B5(改訂): 到着時の記録先(None なら何もしない)。
# note_arrival(sumo_id, arrival_time, route, departure) を持つオブジェクト。
ARRIVAL_RECORDER = None

if TYPE_CHECKING:
    from evacsim.agents.Agent import Agent
    from evacsim.agents.Shelter import Shelter
    from evacsim.agents.VehicleInfo import VehicleInfo


def handle_arrival(
    current_vehID,
    vehInfo_by_current_vehID: VehicleInfo,
    agent_by_current_vehID: Agent,
    shelter_for_current_vehID: Shelter,
    shelter_list: list,
    arrival_time_list: list,
    arrival_time_by_vehID_dict: dict,
    elapsed_time_list: list,
):
    """
    避難地到着時の処理。

    既存 utilities.py の handle_arrival の挙動を維持する。
    """
    arrival_time_list.append(traci.simulation.getTime())
    arrival_time_by_vehID_dict[f"{current_vehID}"] = traci.simulation.getTime()

    traci.vehicle.setSpeed(current_vehID, 9.0)

    # 避難地オブジェクトに登録
    shelter_for_current_vehID.add_arrival_vehID(current_vehID)

    vehInfo_by_current_vehID.set_evac_end_time(
        traci.simulation.getTime()
    )

    # 近傍エッジから避難地オブジェクトを取得して避難時間を更新
    shelter: Shelter = find_shelter_by_edgeID_connect_target_shelter(
        agent_by_current_vehID.get_near_edgeID_by_target_shelter(),
        shelter_list,
    )

    departure_time = traci.vehicle.getDeparture(current_vehID) + TRAVEL_TIME_OFFSET

    if ARRIVAL_RECORDER is not None:
        ARRIVAL_RECORDER.note_arrival(
            current_vehID,
            traci.simulation.getTime(),
            traci.vehicle.getRoute(current_vehID),
            traci.vehicle.getDeparture(current_vehID),
        )

    shelter.update_evac_time_default_dict(
        vehID=current_vehID,
        route=traci.vehicle.getRoute(current_vehID),
        evac_time=vehInfo_by_current_vehID.get_evac_end_time() - departure_time,
    )

    # 到着フラグと駐車フラグを更新
    vehInfo_by_current_vehID.set_parked_flag(True)
    agent_by_current_vehID.set_arrival_time(
        traci.simulation.getTime()
    )
    vehInfo_by_current_vehID.set_arrival_flag(True)

    elapsed_time_list.append(
        traci.simulation.getTime()
        - agent_by_current_vehID.get_created_time()
    )


def handle_arrival_for_pedestrian(
    pedestrianID: str,
    vehInfo_by_current_vehID: VehicleInfo,
    agent_by_current_vehID: Agent,
    shelter_for_current_vehID: Shelter,
    shelter_list: list,
    arrival_time_list: list,
    arrival_time_by_vehID_dict: dict,
    elapsed_time_list: list,
):
    """
    歩行者として避難地に到着した場合の処理。

    既存 utilities.py の handle_arrival_for_pedestrian の挙動を維持する。
    """
    arrival_time_list.append(traci.simulation.getTime())
    arrival_time_by_vehID_dict[f"{pedestrianID}"] = traci.simulation.getTime()

    # 避難地オブジェクトに登録
    shelter_for_current_vehID.add_arrival_vehID(pedestrianID)

    vehInfo_by_current_vehID.set_evac_end_time(
        traci.simulation.getTime()
    )

    # 近傍エッジから避難地オブジェクトを取得して避難時間を更新
    shelter: Shelter = find_shelter_by_edgeID_connect_target_shelter(
        agent_by_current_vehID.get_near_edgeID_by_target_shelter(),
        shelter_list,
    )

    # 到着フラグと駐車フラグを更新
    agent_by_current_vehID.set_arrival_shelter_flg(True)
    agent_by_current_vehID.set_arrival_time(
        traci.simulation.getTime()
    )

    elapsed_time_list.append(
        traci.simulation.getTime()
        - agent_by_current_vehID.get_created_time()
    )


def extract_vehicle_id(pedestrian_id: str) -> str:
    """
    pedestrianID から元の vehicleID を復元する。

    例:
        ped_init_ShelterA_1_0_3 -> init_ShelterA_1_0
    """
    return pedestrian_id.replace("ped_", "").rsplit("_", 1)[0]
