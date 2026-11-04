# from __future__ import annotations

# from collections import deque
# from copy import deepcopy
# from dataclasses import dataclass
# from enum import Enum
# from math import hypot
# from typing import Callable, Hashable, Iterable, Mapping, Sequence
# import warnings

# try:
#     from evacsim.agents.VehicleInfo import (
#         InformationKind,
#         InformationRecord,
#         VehicleInfo,
#         information_record_is_preferred,
#     )
# except ImportError:  # 単体テスト・スタンドアロン実行用
#     from VehicleInfo import (  # type: ignore
#         InformationKind,
#         InformationRecord,
#         VehicleInfo,
#         information_record_is_preferred,
#     )


# class CommunicationMode(str, Enum):
#     V2V_ONLY = "v2v_only"
#     V2V_DTN = "v2v_dtn"


# ALL_INFORMATION_KINDS: set[InformationKind] = {
#     "shelter_congestion",
#     "route_average_time",
#     "tsunami_precursor",
# }


# @dataclass(frozen=True)
# class CommunicationRoundResult:
#     communication_round: Hashable
#     mode: CommunicationMode
#     connected_components: tuple[tuple[str, ...], ...]
#     updated_vehicle_ids: tuple[str, ...]
#     applied_record_count: int


# @dataclass(frozen=True)
# class _HeldRecord:
#     holder_vehicle_id: str
#     record: InformationRecord


# def normalize_communication_mode(
#     communication_mode: CommunicationMode | str,
# ) -> CommunicationMode:
#     if isinstance(communication_mode, CommunicationMode):
#         return communication_mode
#     try:
#         return CommunicationMode(communication_mode)
#     except ValueError as exc:
#         valid = ", ".join(mode.value for mode in CommunicationMode)
#         raise ValueError(
#             f"Unsupported communication mode {communication_mode!r}. "
#             f"Expected one of: {valid}"
#         ) from exc


# def merge_route_info_within_shelters(one_shelter, another_shelter) -> None:
#     """2つの避難所が持つ経路別平均避難時間を重み付き平均で統合する。"""
#     one_info = one_shelter.get_avg_evac_time_by_route()
#     another_info = another_shelter.get_avg_evac_time_by_route()
#     merged_info: dict = {}

#     for route, data in one_info.items():
#         merged_info[route] = deepcopy(data)

#     for route, data in another_info.items():
#         if route not in merged_info:
#             merged_info[route] = deepcopy(data)
#             continue

#         one_vehicles = merged_info[route]["vehicles"]
#         another_vehicles = data["vehicles"]
#         total_vehicles = one_vehicles + another_vehicles
#         avg_time = 0.0
#         if total_vehicles:
#             avg_time = (
#                 merged_info[route]["avg_time"] * one_vehicles
#                 + data["avg_time"] * another_vehicles
#             ) / total_vehicles
#         merged_info[route] = {
#             "avg_time": avg_time,
#             "vehicles": total_vehicles,
#         }

#     # 2避難所間で同じ辞書オブジェクトを共有しない。
#     one_shelter.set_avg_evac_time_by_route(deepcopy(merged_info))
#     another_shelter.set_avg_evac_time_by_route(deepcopy(merged_info))


# def v2shelter_communication(
#     target_vehID: str,
#     shelterID: str,
#     vehInfo_list: list,
#     shelter_list: list,
#     COMMUNICATION_RANGE: float,
#     target_vehInfo: VehicleInfo | None = None,
#     target_position=None,
#     vehInfo_by_vehID_dict: dict | None = None,
#     step_cache=None,
#     *,
#     communication_round: Hashable | None = None,
#     current_time: float | None = None,
# ) -> bool:
#     """
#     車両と避難所の直接通信を行う。

#     直接取得した情報は、意思決定用に永続保存される。同時に、V2V-onlyでは
#     ``communication_round`` と同じラウンドに限りV2V共有可能として記録される。
#     """
#     if COMMUNICATION_RANGE < 0:
#         raise ValueError("COMMUNICATION_RANGE must be non-negative")

#     from evacsim.sim.sumo_env import ensure_sumo_tools_on_path

#     ensure_sumo_tools_on_path()
#     import traci
#     from evacsim.sim.routing_distance import distance_each_vehIDs
#     from evacsim.sim.traci_cache import get_vehicle_position_cached
#     from evacsim.utils.lookup import (
#         find_shelter_by_edgeID_connect_target_shelter,
#         find_vehInfo_by_vehID,
#     )

#     if target_vehInfo is None:
#         target_vehInfo = find_vehInfo_by_vehID(
#             target_vehID,
#             vehInfo_list=vehInfo_list,
#             vehInfo_by_vehID_dict=vehInfo_by_vehID_dict,
#         )
#     if target_vehInfo is None:
#         return False

#     shelter = find_shelter_by_edgeID_connect_target_shelter(
#         target_vehInfo.get_edgeID_connect_target_shelter(),
#         shelter_list,
#     )
#     if shelter is None:
#         return False

#     if target_position is None:
#         target_position = get_vehicle_position_cached(
#             target_vehID,
#             step_cache=step_cache,
#         )

#     if (
#         distance_each_vehIDs(target_position, shelter.get_position())
#         >= COMMUNICATION_RANGE
#     ):
#         return False

#     resolved_time = (
#         float(traci.simulation.getTime())
#         if current_time is None
#         else float(current_time)
#     )
#     resolved_round = (
#         resolved_time if communication_round is None else communication_round
#     )

#     target_vehInfo.record_direct_shelter_information(
#         shelter_id=shelterID,
#         congestion=shelter.get_congestion_rate(),
#         avg_evac_time_by_route=shelter.get_avg_evac_time_by_route(),
#         current_time=resolved_time,
#         communication_round=resolved_round,
#         source_id=shelterID,
#     )
#     return True


# def record_direct_tsunami_precursor_info(
#     target_vehInfo: VehicleInfo,
#     *,
#     detected_time: float,
#     communication_round: Hashable,
#     source_id: str | None = None,
# ) -> None:
#     """津波前兆を直接検知した車両に、当該ラウンドだけ転送可能な情報を登録する。"""
#     target_vehInfo.record_direct_tsunami_precursor_information(
#         detected_time=detected_time,
#         communication_round=communication_round,
#         source_id=source_id,
#         precursor_flag=True,
#     )


# def execute_v2v_communication_round(
#     *,
#     vehicle_info_by_id: Mapping[str, VehicleInfo],
#     positions_by_vehicle_id: Mapping[str, Sequence[float]],
#     communication_range: float,
#     communication_mode: CommunicationMode | str,
#     communication_round: Hashable,
#     current_time: float,
#     candidate_neighbors_by_vehicle_id: Mapping[str, Iterable[str]] | None = None,
#     information_kinds: set[InformationKind] | None = None,
#     distance_function: Callable[[Sequence[float], Sequence[float]], float] | None = None,
# ) -> CommunicationRoundResult:
#     """
#     1通信ラウンド分のV2V情報共有を、スナップショット方式で一括実行する。

#     V2V-only:
#       - 指定ラウンドに避難所・車載センサから直接取得した情報だけを送信元にする。
#       - 連結成分内の同一ラウンド・マルチホップ共有を許可する。
#       - V2V受信情報は次回ラウンドの送信元にはならない。

#     V2V+DTN:
#       - 車両が永続保持する情報を送信元にできる。
#       - したがって時間的に切断された接触間でもstore-carry-forwardが成立する。
#     """
#     mode = normalize_communication_mode(communication_mode)
#     if communication_range < 0:
#         raise ValueError("communication_range must be non-negative")

#     selected_kinds = (
#         set(ALL_INFORMATION_KINDS)
#         if information_kinds is None
#         else set(information_kinds)
#     )
#     unknown_kinds = selected_kinds - ALL_INFORMATION_KINDS
#     if unknown_kinds:
#         raise ValueError(f"Unknown information kinds: {sorted(unknown_kinds)}")

#     distance = distance_function or _euclidean_distance
#     participant_ids = sorted(
#         vehicle_id
#         for vehicle_id, vehicle_info in vehicle_info_by_id.items()
#         if vehicle_id in positions_by_vehicle_id
#         and vehicle_info.get_vehicle_comm_enabled_flag()
#     )

#     adjacency = _build_undirected_communication_graph(
#         participant_ids=participant_ids,
#         positions_by_vehicle_id=positions_by_vehicle_id,
#         communication_range=communication_range,
#         candidate_neighbors_by_vehicle_id=candidate_neighbors_by_vehicle_id,
#         distance_function=distance,
#     )
#     components = _connected_components(adjacency)

#     # 第1段階: 通信開始時点の送信可能情報をスナップショット化する。
#     source_snapshot: dict[str, tuple[InformationRecord, ...]] = {}
#     for vehicle_id in participant_ids:
#         vehicle_info = vehicle_info_by_id[vehicle_id]
#         if mode is CommunicationMode.V2V_ONLY:
#             records = vehicle_info.get_forwardable_information_records(
#                 communication_round,
#                 information_kinds=selected_kinds,
#             )
#         else:
#             records = vehicle_info.get_dtn_transferable_information_records(
#                 information_kinds=selected_kinds,
#             )
#         source_snapshot[vehicle_id] = tuple(record.detached() for record in records)

#     # 第2段階: スナップショットだけを使って、各連結成分の共有結果を決定する。
#     component_records: list[
#         tuple[tuple[str, ...], dict[tuple[str, str], _HeldRecord]]
#     ] = []
#     for component in components:
#         best_by_key: dict[tuple[str, str], _HeldRecord] = {}
#         for holder_vehicle_id in component:
#             for record in source_snapshot[holder_vehicle_id]:
#                 record_key = (record.kind, record.information_key)
#                 current = best_by_key.get(record_key)
#                 if current is None or _held_record_is_preferred(
#                     _HeldRecord(holder_vehicle_id, record),
#                     current,
#                 ):
#                     best_by_key[record_key] = _HeldRecord(
#                         holder_vehicle_id,
#                         record.detached(),
#                     )
#         component_records.append((component, best_by_key))

#     # 第3段階: 計算済み結果を全車両へ一括反映する。
#     applied_count = 0
#     updated_vehicle_ids: set[str] = set()
#     for component, best_by_key in component_records:
#         hop_cache: dict[str, dict[str, int]] = {}
#         for held_record in best_by_key.values():
#             source_holder = held_record.holder_vehicle_id
#             if source_holder not in hop_cache:
#                 hop_cache[source_holder] = _shortest_hops_from(
#                     source_holder,
#                     adjacency,
#                     allowed_vertices=set(component),
#                 )
#             hops_from_holder = hop_cache[source_holder]

#             for receiver_vehicle_id in component:
#                 path_hops = hops_from_holder[receiver_vehicle_id]
#                 total_hops = held_record.record.relay_hops + path_hops
#                 applied = vehicle_info_by_id[
#                     receiver_vehicle_id
#                 ].apply_received_information_record(
#                     held_record.record,
#                     current_time=current_time,
#                     communication_round=communication_round,
#                     relay_hops=total_hops,
#                     communication_mode=mode.value,
#                 )
#                 if applied:
#                     applied_count += 1
#                     updated_vehicle_ids.add(receiver_vehicle_id)

#     return CommunicationRoundResult(
#         communication_round=communication_round,
#         mode=mode,
#         connected_components=tuple(components),
#         updated_vehicle_ids=tuple(sorted(updated_vehicle_ids)),
#         applied_record_count=applied_count,
#     )


# def execute_v2v_communication_round_traci(
#     *,
#     active_vehicle_ids: Iterable[str],
#     vehicle_info_by_id: Mapping[str, VehicleInfo],
#     communication_range: float,
#     communication_mode: CommunicationMode | str,
#     communication_round: Hashable,
#     current_time: float | None = None,
#     candidate_neighbors_by_vehicle_id: Mapping[str, Iterable[str]] | None = None,
#     information_kinds: set[InformationKind] | None = None,
#     step_cache=None,
# ) -> CommunicationRoundResult:
#     """TraCIから位置を取得して、1ラウンド分のV2V通信を一括実行する。"""
#     from evacsim.sim.sumo_env import ensure_sumo_tools_on_path

#     ensure_sumo_tools_on_path()
#     import traci
#     from evacsim.sim.traci_cache import get_vehicle_position_cached

#     resolved_time = (
#         float(traci.simulation.getTime())
#         if current_time is None
#         else float(current_time)
#     )
#     positions: dict[str, Sequence[float]] = {}
#     selected_vehicle_info: dict[str, VehicleInfo] = {}

#     for vehicle_id in sorted(set(active_vehicle_ids)):
#         vehicle_info = vehicle_info_by_id.get(vehicle_id)
#         if vehicle_info is None:
#             continue
#         try:
#             positions[vehicle_id] = get_vehicle_position_cached(
#                 vehicle_id,
#                 step_cache=step_cache,
#             )
#         except Exception:
#             # シミュレーションから消えた車両はこのラウンドの対象外とする。
#             continue
#         selected_vehicle_info[vehicle_id] = vehicle_info

#     return execute_v2v_communication_round(
#         vehicle_info_by_id=selected_vehicle_info,
#         positions_by_vehicle_id=positions,
#         communication_range=communication_range,
#         communication_mode=communication_mode,
#         communication_round=communication_round,
#         current_time=resolved_time,
#         candidate_neighbors_by_vehicle_id=candidate_neighbors_by_vehicle_id,
#         information_kinds=information_kinds,
#     )


# def v2v_communication(
#     target_vehID: str,
#     target_vehInfo: VehicleInfo,
#     around_vehIDs: list,
#     agent_list: list,
#     vehInfo_list: list,
#     COMMUNICATION_RANGE: float,
#     target_position=None,
#     agent_by_vehID_dict: dict | None = None,
#     vehInfo_by_vehID_dict: dict | None = None,
#     step_cache=None,
#     *,
#     communication_mode: CommunicationMode | str = CommunicationMode.V2V_DTN,
#     communication_round: Hashable | None = None,
#     current_time: float | None = None,
# ):
#     """
#     旧runner向け互換ラッパー。

#     V2V-onlyでは全車両のスナップショットが必要なため、この1台単位APIは使用禁止。
#     ``execute_v2v_communication_round_traci`` へ移行すること。
#     """
#     mode = normalize_communication_mode(communication_mode)
#     if mode is CommunicationMode.V2V_ONLY:
#         raise RuntimeError(
#             "v2v_only must be executed with "
#             "execute_v2v_communication_round_traci() once per communication round"
#         )

#     warnings.warn(
#         "v2v_communication() is a legacy per-vehicle API. "
#         "Use execute_v2v_communication_round_traci() for deterministic results.",
#         DeprecationWarning,
#         stacklevel=2,
#     )
#     return _execute_legacy_subset_round(
#         target_vehID=target_vehID,
#         target_vehInfo=target_vehInfo,
#         around_vehIDs=around_vehIDs,
#         vehInfo_list=vehInfo_list,
#         COMMUNICATION_RANGE=COMMUNICATION_RANGE,
#         target_position=target_position,
#         vehInfo_by_vehID_dict=vehInfo_by_vehID_dict,
#         step_cache=step_cache,
#         communication_mode=mode,
#         communication_round=communication_round,
#         current_time=current_time,
#         information_kinds={"shelter_congestion", "route_average_time"},
#     )


# def v2v_communication_about_tsunami_info(
#     target_vehID: str,
#     target_vehInfo: VehicleInfo,
#     around_vehIDs: list,
#     vehInfo_list: list,
#     COMMUNICATION_RANGE: float,
#     target_position=None,
#     vehInfo_by_vehID_dict: dict | None = None,
#     step_cache=None,
#     *,
#     communication_mode: CommunicationMode | str = CommunicationMode.V2V_DTN,
#     communication_round: Hashable | None = None,
#     current_time: float | None = None,
# ):
#     """旧津波情報通信API向け互換ラッパー。"""
#     mode = normalize_communication_mode(communication_mode)
#     if mode is CommunicationMode.V2V_ONLY:
#         raise RuntimeError(
#             "v2v_only tsunami sharing must be included in "
#             "execute_v2v_communication_round_traci()"
#         )

#     warnings.warn(
#         "v2v_communication_about_tsunami_info() is deprecated. "
#         "Use execute_v2v_communication_round_traci().",
#         DeprecationWarning,
#         stacklevel=2,
#     )
#     return _execute_legacy_subset_round(
#         target_vehID=target_vehID,
#         target_vehInfo=target_vehInfo,
#         around_vehIDs=around_vehIDs,
#         vehInfo_list=vehInfo_list,
#         COMMUNICATION_RANGE=COMMUNICATION_RANGE,
#         target_position=target_position,
#         vehInfo_by_vehID_dict=vehInfo_by_vehID_dict,
#         step_cache=step_cache,
#         communication_mode=mode,
#         communication_round=communication_round,
#         current_time=current_time,
#         information_kinds={"tsunami_precursor"},
#     )


# def _execute_legacy_subset_round(
#     *,
#     target_vehID: str,
#     target_vehInfo: VehicleInfo,
#     around_vehIDs: list,
#     vehInfo_list: list,
#     COMMUNICATION_RANGE: float,
#     target_position,
#     vehInfo_by_vehID_dict: dict | None,
#     step_cache,
#     communication_mode: CommunicationMode,
#     communication_round: Hashable | None,
#     current_time: float | None,
#     information_kinds: set[InformationKind],
# ):
#     from evacsim.sim.sumo_env import ensure_sumo_tools_on_path

#     ensure_sumo_tools_on_path()
#     import traci
#     from evacsim.sim.traci_cache import get_vehicle_position_cached
#     from evacsim.utils.lookup import find_vehInfo_by_vehID

#     resolved_time = (
#         float(traci.simulation.getTime())
#         if current_time is None
#         else float(current_time)
#     )
#     resolved_round = (
#         resolved_time if communication_round is None else communication_round
#     )

#     vehicle_info_by_id: dict[str, VehicleInfo] = {target_vehID: target_vehInfo}
#     positions: dict[str, Sequence[float]] = {
#         target_vehID: (
#             target_position
#             if target_position is not None
#             else get_vehicle_position_cached(target_vehID, step_cache=step_cache)
#         )
#     }

#     for around_id in around_vehIDs:
#         around_info = find_vehInfo_by_vehID(
#             around_id,
#             vehInfo_list=vehInfo_list,
#             vehInfo_by_vehID_dict=vehInfo_by_vehID_dict,
#         )
#         if around_info is None:
#             continue
#         try:
#             positions[around_id] = get_vehicle_position_cached(
#                 around_id,
#                 step_cache=step_cache,
#             )
#         except Exception:
#             continue
#         vehicle_info_by_id[around_id] = around_info

#     return execute_v2v_communication_round(
#         vehicle_info_by_id=vehicle_info_by_id,
#         positions_by_vehicle_id=positions,
#         communication_range=COMMUNICATION_RANGE,
#         communication_mode=communication_mode,
#         communication_round=resolved_round,
#         current_time=resolved_time,
#         information_kinds=information_kinds,
#     )


# def _euclidean_distance(
#     one_position: Sequence[float],
#     another_position: Sequence[float],
# ) -> float:
#     if len(one_position) < 2 or len(another_position) < 2:
#         raise ValueError("Each position must contain at least x and y")
#     return hypot(
#         float(one_position[0]) - float(another_position[0]),
#         float(one_position[1]) - float(another_position[1]),
#     )


# def _build_undirected_communication_graph(
#     *,
#     participant_ids: list[str],
#     positions_by_vehicle_id: Mapping[str, Sequence[float]],
#     communication_range: float,
#     candidate_neighbors_by_vehicle_id: Mapping[str, Iterable[str]] | None,
#     distance_function: Callable[[Sequence[float], Sequence[float]], float],
# ) -> dict[str, set[str]]:
#     adjacency = {vehicle_id: set() for vehicle_id in participant_ids}
#     participant_set = set(participant_ids)

#     if candidate_neighbors_by_vehicle_id is None:
#         candidate_pairs = (
#             (participant_ids[i], participant_ids[j])
#             for i in range(len(participant_ids))
#             for j in range(i + 1, len(participant_ids))
#         )
#     else:
#         normalized_pairs: set[tuple[str, str]] = set()
#         for one_id, neighbor_ids in candidate_neighbors_by_vehicle_id.items():
#             if one_id not in participant_set:
#                 continue
#             for another_id in neighbor_ids:
#                 if another_id not in participant_set or one_id == another_id:
#                     continue
#                 normalized_pairs.add(tuple(sorted((one_id, another_id))))
#         candidate_pairs = iter(sorted(normalized_pairs))

#     for one_id, another_id in candidate_pairs:
#         if (
#             distance_function(
#                 positions_by_vehicle_id[one_id],
#                 positions_by_vehicle_id[another_id],
#             )
#             < communication_range
#         ):
#             adjacency[one_id].add(another_id)
#             adjacency[another_id].add(one_id)

#     return adjacency


# def _connected_components(
#     adjacency: Mapping[str, set[str]],
# ) -> list[tuple[str, ...]]:
#     visited: set[str] = set()
#     components: list[tuple[str, ...]] = []
#     for start in sorted(adjacency):
#         if start in visited:
#             continue
#         queue = deque([start])
#         visited.add(start)
#         component: list[str] = []
#         while queue:
#             current = queue.popleft()
#             component.append(current)
#             for neighbor in sorted(adjacency[current]):
#                 if neighbor not in visited:
#                     visited.add(neighbor)
#                     queue.append(neighbor)
#         components.append(tuple(sorted(component)))
#     return components


# def _shortest_hops_from(
#     source: str,
#     adjacency: Mapping[str, set[str]],
#     *,
#     allowed_vertices: set[str],
# ) -> dict[str, int]:
#     hops = {source: 0}
#     queue = deque([source])
#     while queue:
#         current = queue.popleft()
#         for neighbor in sorted(adjacency[current]):
#             if neighbor not in allowed_vertices or neighbor in hops:
#                 continue
#             hops[neighbor] = hops[current] + 1
#             queue.append(neighbor)
#     if set(hops) != allowed_vertices:
#         raise RuntimeError("Component contains unreachable vertices")
#     return hops


# def _held_record_is_preferred(
#     candidate: _HeldRecord,
#     current: _HeldRecord,
# ) -> bool:
#     if information_record_is_preferred(candidate.record, current.record):
#         return True
#     if information_record_is_preferred(current.record, candidate.record):
#         return False
#     return candidate.holder_vehicle_id < current.holder_vehicle_id


# =========================
# Communication helpers
# =========================
#
# 注意:
# このモジュールは TraCI に依存する。
# V2Shelter / V2V / 津波情報共有など、情報伝達処理を扱う。

from __future__ import annotations

from typing import TYPE_CHECKING
from copy import deepcopy
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
                        deepcopy(
                        info_source.get_shelter_congestion_info()
                        )
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
                    deepcopy(info_source.get_avg_evac_time_by_route_by_recive_time())
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
