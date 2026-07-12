# from __future__ import annotations

# from collections import defaultdict
# from copy import deepcopy
# from dataclasses import dataclass, replace
# from typing import Any, Hashable, Literal, Mapping


# InformationKind = Literal[
#     "shelter_congestion",
#     "route_average_time",
#     "tsunami_precursor",
# ]
# OriginType = Literal[
#     "shelter",
#     "vehicle",
#     "vehicle_sensor",
#     "legacy",
# ]
# CommunicationRound = Hashable


# @dataclass(frozen=True)
# class InformationRecord:
#     """通信情報と、その生成・受信・転送可否を表す不変レコード。"""

#     kind: InformationKind
#     information_key: str
#     payload: Any
#     source_time: float
#     received_time: float
#     received_round: CommunicationRound | None
#     origin_type: OriginType
#     origin_id: str
#     relay_hops: int = 0
#     forwardable_in_round: CommunicationRound | None = None

#     def detached(self) -> "InformationRecord":
#         """payloadを複製し、送受信車両間の参照共有を防ぐ。"""
#         return replace(self, payload=deepcopy(self.payload))

#     def as_received(
#         self,
#         *,
#         received_time: float,
#         received_round: CommunicationRound,
#         relay_hops: int,
#         forwardable_in_round: CommunicationRound | None,
#     ) -> "InformationRecord":
#         """受信側に保存する独立したレコードを作る。"""
#         if relay_hops < self.relay_hops:
#             raise ValueError("relay_hops must not decrease")
#         return replace(
#             self,
#             payload=deepcopy(self.payload),
#             received_time=float(received_time),
#             received_round=received_round,
#             relay_hops=int(relay_hops),
#             forwardable_in_round=forwardable_in_round,
#         )


# def information_record_is_preferred(
#     candidate: InformationRecord,
#     current: InformationRecord | None,
# ) -> bool:
#     """同じ情報種別・キーについてcandidateを採用すべきか判定する。"""
#     if current is None:
#         return True
#     if (
#         candidate.kind != current.kind
#         or candidate.information_key != current.information_key
#     ):
#         raise ValueError("Only records with the same kind and key are comparable")
#     if candidate == current:
#         return False

#     if candidate.kind == "tsunami_precursor":
#         candidate_flag = bool(candidate.payload)
#         current_flag = bool(current.payload)
#         if candidate_flag != current_flag:
#             return candidate_flag
#         if candidate.source_time != current.source_time:
#             # 津波前兆情報は、より早い検知時刻を優先する。
#             return candidate.source_time < current.source_time
#     else:
#         if candidate.source_time != current.source_time:
#             # 混雑情報・経路時間情報は、より新しい生成時刻を優先する。
#             return candidate.source_time > current.source_time

#     if candidate.relay_hops != current.relay_hops:
#         return candidate.relay_hops < current.relay_hops

#     candidate_tie_breaker = (
#         candidate.origin_type,
#         candidate.origin_id,
#         repr(candidate.payload),
#     )
#     current_tie_breaker = (
#         current.origin_type,
#         current.origin_id,
#         repr(current.payload),
#     )
#     return candidate_tie_breaker < current_tie_breaker


# class VehicleInfo:
#     """
#     車両が保持する状態と通信情報を管理する。

#     従来の辞書はドライバー意思決定用として維持する。
#     ``_information_records`` は通信元・受信ラウンド・転送可否を管理し、
#     V2V-onlyにおけるstore-carry-forwardを防止する。
#     """

#     multi_map = defaultdict(list)

#     def __init__(
#         self,
#         vehID: str,
#         target_shelter: str,
#         edgeID_connect_target_shelter: str,
#         create_time: float,
#     ) -> None:
#         self._vehID = vehID
#         self._target_shelter = target_shelter
#         self._edgeID_connect_target_shelter = edgeID_connect_target_shelter
#         self._multiDict_around_vehInfos: defaultdict = defaultdict(list)
#         self._shelter_congestion_info: defaultdict = defaultdict(list)
#         self._tsunami_precursor_info: defaultdict = defaultdict(list)
#         self._create_time = float(create_time)
#         self._stopping_time = 0
#         self._evac_start_time = 0.0
#         self._evac_end_time = 0.0
#         self._arrival_flag = False
#         self._arrival_time = 0.0
#         self._decline_edge_arrival_flag = False
#         self._parked_flag = False
#         self._avg_evac_time_by_route_from_shelter: dict = {}
#         self._avg_evac_time_by_route_by_recive_time: dict = {}
#         self._stop_flag = False
#         self._start_time_measured_flag = False
#         self._agent_changed_flag = False
#         self._vehicle_comm_enabled_flag = True
#         self._approach_edge_dict: dict = {}
#         self._edgeIDs_within_junction_to_shelter_dict: dict = {}

#         self._information_records: dict[
#             InformationKind,
#             dict[str, InformationRecord],
#         ] = {
#             "shelter_congestion": {},
#             "route_average_time": {},
#             "tsunami_precursor": {},
#         }

#     # ------------------------------------------------------------------
#     # 通信メタデータ管理
#     # ------------------------------------------------------------------
#     def record_direct_shelter_information(
#         self,
#         *,
#         shelter_id: str,
#         congestion: float,
#         avg_evac_time_by_route: Mapping,
#         current_time: float,
#         communication_round: CommunicationRound,
#         source_id: str | None = None,
#     ) -> None:
#         """避難所から直接受信した情報を意思決定用領域と当該ラウンド送信用領域へ記録する。"""
#         origin_id = source_id or shelter_id
#         current_time = float(current_time)

#         congestion_record = InformationRecord(
#             kind="shelter_congestion",
#             information_key=shelter_id,
#             payload=float(congestion),
#             source_time=current_time,
#             received_time=current_time,
#             received_round=communication_round,
#             origin_type="shelter",
#             origin_id=origin_id,
#             relay_hops=0,
#             forwardable_in_round=communication_round,
#         )
#         self._store_record_and_update_legacy(congestion_record)

#         route_record = InformationRecord(
#             kind="route_average_time",
#             information_key="route_average_time",
#             payload=deepcopy(dict(avg_evac_time_by_route)),
#             source_time=current_time,
#             received_time=current_time,
#             received_round=communication_round,
#             origin_type="shelter",
#             origin_id=origin_id,
#             relay_hops=0,
#             forwardable_in_round=communication_round,
#         )
#         self._store_record_and_update_legacy(route_record)

#     def record_direct_tsunami_precursor_information(
#         self,
#         *,
#         detected_time: float,
#         communication_round: CommunicationRound,
#         source_id: str | None = None,
#         precursor_flag: bool = True,
#     ) -> None:
#         """車載観測などにより直接得た津波前兆情報を記録する。"""
#         if not precursor_flag:
#             self.update_tsunami_precursor_info(
#                 source_id or self._vehID,
#                 False,
#                 float(detected_time),
#             )
#             return

#         origin_id = source_id or self._vehID
#         record = InformationRecord(
#             kind="tsunami_precursor",
#             information_key="tsunami_precursor",
#             payload=True,
#             source_time=float(detected_time),
#             received_time=float(detected_time),
#             received_round=communication_round,
#             origin_type="vehicle_sensor",
#             origin_id=origin_id,
#             relay_hops=0,
#             forwardable_in_round=communication_round,
#         )
#         self._store_record_and_update_legacy(record)

#     def get_forwardable_information_records(
#         self,
#         communication_round: CommunicationRound,
#         *,
#         information_kinds: set[InformationKind] | None = None,
#     ) -> list[InformationRecord]:
#         """V2V-only用に、指定ラウンドでのみ転送可能な直接取得情報を返す。"""
#         records: list[InformationRecord] = []
#         for kind, by_key in self._information_records.items():
#             if information_kinds is not None and kind not in information_kinds:
#                 continue
#             for record in by_key.values():
#                 if record.forwardable_in_round == communication_round:
#                     records.append(record.detached())
#         return records

#     def get_dtn_transferable_information_records(
#         self,
#         *,
#         information_kinds: set[InformationKind] | None = None,
#     ) -> list[InformationRecord]:
#         """V2V+DTN用に、永続保持中のすべての有効情報を返す。"""
#         self._synthesize_missing_records_from_legacy()
#         records: list[InformationRecord] = []
#         for kind, by_key in self._information_records.items():
#             if information_kinds is not None and kind not in information_kinds:
#                 continue
#             records.extend(record.detached() for record in by_key.values())
#         return records

#     def apply_received_information_record(
#         self,
#         record: InformationRecord,
#         *,
#         current_time: float,
#         communication_round: CommunicationRound,
#         relay_hops: int,
#         communication_mode: str,
#     ) -> bool:
#         """
#         V2V受信情報を永続領域へ反映する。

#         V2V-onlyで受信した情報は、同一ラウンドを含め送信可能レコードにはしない。
#         同一ラウンドのマルチホップは通信グラフの連結成分計算で実現する。
#         """
#         if communication_mode not in {"v2v_only", "v2v_dtn"}:
#             raise ValueError(f"Unsupported communication_mode: {communication_mode}")

#         received_record = record.as_received(
#             received_time=float(current_time),
#             received_round=communication_round,
#             relay_hops=relay_hops,
#             forwardable_in_round=None,
#         )
#         current = self._information_records[received_record.kind].get(
#             received_record.information_key
#         )
#         if not information_record_is_preferred(received_record, current):
#             return False

#         self._store_record_and_update_legacy(received_record)
#         return True

#     def get_information_record(
#         self,
#         kind: InformationKind,
#         information_key: str,
#     ) -> InformationRecord | None:
#         record = self._information_records[kind].get(information_key)
#         return None if record is None else record.detached()

#     def _store_record_and_update_legacy(self, record: InformationRecord) -> None:
#         current = self._information_records[record.kind].get(record.information_key)
#         if current is not None and not information_record_is_preferred(record, current):
#             return

#         detached = record.detached()
#         self._information_records[detached.kind][detached.information_key] = detached

#         if detached.kind == "shelter_congestion":
#             self._shelter_congestion_info[detached.information_key] = [
#                 deepcopy(detached.payload),
#                 float(detached.source_time),
#             ]
#         elif detached.kind == "route_average_time":
#             route_info = deepcopy(detached.payload)
#             self._avg_evac_time_by_route_from_shelter = deepcopy(route_info)
#             # 既存コードが情報鮮度の比較に使うキーは、転送時刻ではなく元情報の時刻を保持する。
#             self._avg_evac_time_by_route_by_recive_time = {
#                 float(detached.source_time): deepcopy(route_info)
#             }
#         elif detached.kind == "tsunami_precursor":
#             self._tsunami_precursor_info = defaultdict(list)
#             self._tsunami_precursor_info[detached.origin_id] = [
#                 bool(detached.payload),
#                 float(detached.source_time),
#             ]
#         else:
#             raise ValueError(f"Unknown information kind: {detached.kind}")

#     def _synthesize_missing_records_from_legacy(self) -> None:
#         """旧APIだけで設定された情報をDTNモード向けレコードに変換する。"""
#         for shelter_id, value in self._shelter_congestion_info.items():
#             if not isinstance(value, (list, tuple)) or len(value) < 2:
#                 continue
#             congestion, source_time = value[0], float(value[1])
#             current = self._information_records["shelter_congestion"].get(
#                 str(shelter_id)
#             )
#             # 旧APIで後から更新された、より新しい情報もDTN送信対象へ反映する。
#             if current is not None and source_time <= current.source_time:
#                 continue
#             self._information_records["shelter_congestion"][str(shelter_id)] = (
#                 InformationRecord(
#                     kind="shelter_congestion",
#                     information_key=str(shelter_id),
#                     payload=deepcopy(congestion),
#                     source_time=source_time,
#                     received_time=source_time,
#                     received_round=None,
#                     origin_type="legacy",
#                     origin_id=self._vehID,
#                     relay_hops=0,
#                     forwardable_in_round=None,
#                 )
#             )

#         if self._avg_evac_time_by_route_by_recive_time:
#             latest_time, route_info = max(
#                 self._avg_evac_time_by_route_by_recive_time.items(),
#                 key=lambda item: float(item[0]),
#             )
#             latest_time = float(latest_time)
#             current = self._information_records["route_average_time"].get(
#                 "route_average_time"
#             )
#             if (
#                 route_info
#                 and (current is None or latest_time > current.source_time)
#             ):
#                 self._information_records["route_average_time"][
#                     "route_average_time"
#                 ] = InformationRecord(
#                     kind="route_average_time",
#                     information_key="route_average_time",
#                     payload=deepcopy(route_info),
#                     source_time=latest_time,
#                     received_time=latest_time,
#                     received_round=None,
#                     origin_type="legacy",
#                     origin_id=self._vehID,
#                     relay_hops=0,
#                     forwardable_in_round=None,
#                 )

#         true_entries: list[tuple[str, float]] = []
#         for origin_id, value in self._tsunami_precursor_info.items():
#             if (
#                 isinstance(value, (list, tuple))
#                 and len(value) >= 2
#                 and bool(value[0])
#             ):
#                 true_entries.append((str(origin_id), float(value[1])))
#         if true_entries:
#             origin_id, source_time = min(true_entries, key=lambda item: item[1])
#             current = self._information_records["tsunami_precursor"].get(
#                 "tsunami_precursor"
#             )
#             if current is None or source_time < current.source_time:
#                 self._information_records["tsunami_precursor"][
#                     "tsunami_precursor"
#                 ] = InformationRecord(
#                     kind="tsunami_precursor",
#                     information_key="tsunami_precursor",
#                     payload=True,
#                     source_time=source_time,
#                     received_time=source_time,
#                     received_round=None,
#                     origin_type="legacy",
#                     origin_id=origin_id,
#                     relay_hops=0,
#                     forwardable_in_round=None,
#                 )

#     # ------------------------------------------------------------------
#     # 既存API（意思決定処理との互換性を維持）
#     # ------------------------------------------------------------------
#     def update_around_vehInfos(
#         self,
#         around_vehID: str,
#         speed: float,
#         time_stamp: float,
#     ) -> None:
#         self._multiDict_around_vehInfos[around_vehID] = [speed, time_stamp]

#     def get_latest_speed(self, around_vehID: str):
#         return self._multiDict_around_vehInfos[around_vehID][0]

#     def get_latest_time_stamp(self, around_vehID: str):
#         return self._multiDict_around_vehInfos[around_vehID][1]

#     def update_shelter_congestion_info(
#         self,
#         shelterID: str,
#         congestion: float,
#         time_stamp: float,
#     ) -> None:
#         self._shelter_congestion_info[shelterID] = [
#             deepcopy(congestion),
#             float(time_stamp),
#         ]

#     def v2shelter_update_avg_evac_time_by_route(
#         self,
#         avg_evac_time_by_route: Mapping,
#     ) -> None:
#         self._avg_evac_time_by_route_from_shelter = deepcopy(
#             dict(avg_evac_time_by_route)
#         )

#     def v2v_avg_evac_time_by_route_by_recive_time(
#         self,
#         current_time: float,
#     ) -> None:
#         self._avg_evac_time_by_route_by_recive_time = {
#             float(current_time): deepcopy(
#                 self.get_avg_evac_time_by_route_from_shelter()
#             )
#         }

#     def update_tsunami_precursor_info(
#         self,
#         vehID: str,
#         tsunami_precursor_flag: int | bool,
#         current_time: float,
#     ) -> None:
#         self._tsunami_precursor_info[vehID] = [
#             bool(tsunami_precursor_flag),
#             float(current_time),
#         ]

#     def has_tsunami_precursor_info(self) -> bool:
#         for value in self.get_tsunami_precursor_info().values():
#             if isinstance(value, list) and value and value[0] is True:
#                 return True
#         return False

#     def get_latest_time_stamp_of_shelter(self, shelter: str):
#         try:
#             return self._shelter_congestion_info[shelter][1]
#         except (KeyError, IndexError, TypeError):
#             return 0

#     def get_congestion_level_by_shelter(self, shelter: str):
#         try:
#             return self._shelter_congestion_info[shelter][0]
#         except (KeyError, IndexError, TypeError):
#             return 0

#     def has_shelter_full_info(
#         self,
#         shelterID: str,
#         threshold: float = 0.98,
#     ) -> bool:
#         try:
#             congestion, _time_stamp = self._shelter_congestion_info[shelterID]
#             return congestion >= threshold
#         except (KeyError, ValueError, TypeError):
#             return False

#     def init_set_congestion_level_by_shelter(
#         self,
#         shelter: str,
#         congestion: float,
#         time_stamp: float,
#     ) -> None:
#         self._shelter_congestion_info[shelter] = [congestion, time_stamp]

#     def init_set_avg_evac_time_by_route_by_recive_time(self) -> None:
#         self._avg_evac_time_by_route_by_recive_time[0.0] = {}

#     def init_set_tsunami_precursor_info(self) -> None:
#         self._tsunami_precursor_info[self.get_vehID()] = [False, 0.0]

#     def update_stopping_time(self) -> None:
#         self._stopping_time += 1

#     def clear_stopping_time(self) -> None:
#         self._stopping_time = 0

#     def get_vehID(self):
#         return self._vehID

#     def set_vehID(self, vehID: str) -> None:
#         self._vehID = vehID

#     def get_target_shelter(self):
#         return self._target_shelter

#     def set_target_shelter(self, target_shelter: str) -> None:
#         self._target_shelter = target_shelter

#     def get_edgeID_connect_target_shelter(self):
#         return self._edgeID_connect_target_shelter

#     def set_edgeID_connect_target_shelter(
#         self,
#         edgeID_connect_target_shelter: str,
#     ) -> None:
#         self._edgeID_connect_target_shelter = edgeID_connect_target_shelter

#     def get_multiDict_around_vehIDs(self):
#         return self._multiDict_around_vehInfos

#     def set_multiDict_around_vehIDs(self, around_vehInfo: defaultdict) -> None:
#         self._multiDict_around_vehInfos = deepcopy(around_vehInfo)

#     def get_shelter_congestion_info(self):
#         return self._shelter_congestion_info

#     def set_shelter_congestion_info(self, shelter_congestion_info) -> None:
#         self._shelter_congestion_info = defaultdict(list)
#         self._shelter_congestion_info.update(deepcopy(dict(shelter_congestion_info)))

#     def get_tsunami_precursor_info(self):
#         return self._tsunami_precursor_info

#     def set_tsunami_precursor_info(self, tsunami_precursor_info) -> None:
#         self._tsunami_precursor_info = defaultdict(list)
#         self._tsunami_precursor_info.update(deepcopy(dict(tsunami_precursor_info)))

#     def get_create_time(self):
#         return self._create_time

#     def set_create_time(self, create_time: float) -> None:
#         self._create_time = float(create_time)

#     def get_stopping_time(self):
#         return self._stopping_time

#     def set_stopping_time(self, stopping_time: float) -> None:
#         self._stopping_time = stopping_time

#     def get_evac_start_time(self):
#         return self._evac_start_time

#     def set_evac_start_time(self, evac_start_time: float) -> None:
#         self._evac_start_time = evac_start_time

#     def get_evac_end_time(self):
#         return self._evac_end_time

#     def set_evac_end_time(self, evac_end_time: float) -> None:
#         self._evac_end_time = evac_end_time

#     def get_arrival_flag(self):
#         return self._arrival_flag

#     def set_arrival_flag(self, arrival_flag: bool) -> None:
#         self._arrival_flag = arrival_flag

#     def get_arrival_time(self):
#         return self._arrival_time

#     def set_arrival_time(self, arrival_time: float) -> None:
#         self._arrival_time = arrival_time

#     def get_decline_edge_arrival_flag(self):
#         return self._decline_edge_arrival_flag

#     def set_decline_edge_arrival_flag(
#         self,
#         decline_edge_arrival_flag: bool,
#     ) -> None:
#         self._decline_edge_arrival_flag = decline_edge_arrival_flag

#     def get_parked_flag(self):
#         return self._parked_flag

#     def set_parked_flag(self, parked_flag: bool) -> None:
#         self._parked_flag = parked_flag

#     def get_avg_evac_time_by_route_from_shelter(self):
#         return self._avg_evac_time_by_route_from_shelter

#     def set_avg_evac_time_by_route_from_shelter(
#         self,
#         avg_evac_time_by_route_from_shelter: dict,
#     ) -> None:
#         self._avg_evac_time_by_route_from_shelter = deepcopy(
#             avg_evac_time_by_route_from_shelter
#         )

#     def get_avg_evac_time_by_route_by_recive_time(self):
#         return self._avg_evac_time_by_route_by_recive_time

#     def set_avg_evac_time_by_route_by_recive_time(
#         self,
#         avg_evac_time_by_route_by_recive_time: dict,
#     ) -> None:
#         self._avg_evac_time_by_route_by_recive_time = deepcopy(
#             avg_evac_time_by_route_by_recive_time
#         )

#     def get_stop_flag(self):
#         return self._stop_flag

#     def set_stop_flag(self, stop_flag: bool) -> None:
#         self._stop_flag = stop_flag

#     def get_start_time_measured_flag(self):
#         return self._start_time_measured_flag

#     def set_start_time_measured_flag(
#         self,
#         start_time_measured_flag: bool,
#     ) -> None:
#         self._start_time_measured_flag = start_time_measured_flag

#     def get_agent_changed_flag(self):
#         return self._agent_changed_flag

#     def set_agent_changed_flag(self, agent_changed_flag: bool) -> None:
#         self._agent_changed_flag = agent_changed_flag

#     def get_vehicle_comm_enabled_flag(self):
#         return self._vehicle_comm_enabled_flag

#     def set_vehicle_comm_enabled_flag(
#         self,
#         vehicle_comm_enabled_flag: bool,
#     ) -> None:
#         self._vehicle_comm_enabled_flag = vehicle_comm_enabled_flag

#     def get_approach_edge_dict(self):
#         return self._approach_edge_dict

#     def set_approach_edge_dict(self, approach_edge_dict: dict) -> None:
#         self._approach_edge_dict = deepcopy(approach_edge_dict)

#     def get_edgeIDs_within_junction_to_shelter_dict(self):
#         return self._edgeIDs_within_junction_to_shelter_dict

#     def set_edgeIDs_within_junction_to_shelter_dict(
#         self,
#         edgeIDs_within_junction_to_shelter_dict: dict,
#     ) -> None:
#         self._edgeIDs_within_junction_to_shelter_dict = deepcopy(
#             edgeIDs_within_junction_to_shelter_dict
#         )

#     def print_shelter_congestion_info(self) -> None:
#         print(
#             f"vehID:{self._vehID} has shelter_congestion_info: "
#             f"{self._shelter_congestion_info}"
#         )

#     def print_has_congestion_info(self) -> None:
#         print(
#             f"vehID:{self._vehID} has congestion_info: "
#             f"{self._shelter_congestion_info}"
#         )

#     def print_all_info(self) -> None:
#         print("========== VehicleInfo ==========")
#         for name, value in vars(self).items():
#             if name == "_information_records":
#                 printable = {
#                     kind: {
#                         key: record.detached()
#                         for key, record in records.items()
#                     }
#                     for kind, records in value.items()
#                 }
#                 print(f"{name}: {printable}")
#             elif isinstance(value, defaultdict):
#                 print(f"{name}: {dict(value)}")
#             else:
#                 print(f"{name}: {value}")
#         print("=================================")


from numpy import double
from collections import defaultdict

# 車両が持つ情報を保存
class VehicleInfo():
    '''
    list_around_vehID -> key: vehID, value[1]: speed, value[2]: time_stamp
    '''
    multi_map = defaultdict(list)
    def __init__(self, vehID:str, target_shelter:str, edgeID_connect_target_shelter:str, create_time: double):
        self._vehID = vehID # 車両ID
        self._target_shelter = target_shelter # 車両が向かう避難所ID
        self._edgeID_connect_target_shelter = edgeID_connect_target_shelter # 車両が向かう避難所edgeID
        self._multiDict_around_vehInfos:defaultdict = defaultdict(list) # key: vehID, value: [speed, time_stamp]
        self._shelter_congestion_info:defaultdict = defaultdict(list) # key: vehID, value: [shelter, congestion]
        self._tsunami_precursor_info:defaultdict = defaultdict(list) # key: vehID, value: [tsunami_precursor_flag, time_stamp]
        self._create_time = create_time
        self._stopping_time = 0
        self._evac_start_time = 0.0 # 交差点1に進入してからの開始時間
        self._evac_end_time = 0.0 # 交差点1に進入してからの終了時間
        self._arrival_flag = False
        self._arrival_time = 0
        self._decline_edge_arrival_flag = False # 車両が避難所の手前のedgeに到着したかどうかのフラグ
        self._parked_flag = False # 車両が駐車しているかどうかのフラグ
        self._avg_evac_time_by_route_from_shelter={} # key:time ルートごとの避難時間を保存する辞書
        self._avg_evac_time_by_route_by_recive_time ={}
        self._stop_flag = False # 車両が停止しているかどうかのフラグ
        self._start_time_measured_flag = False # 車両の避難開始時間を測定するフラグ
        self._agent_changed_flag = False # 車両の避難所が変更されたかどうかのフラグ
        self._vehicle_comm_enabled_flag = True # 車両の通信が有効かどうかのフラグ
        self._approach_edge_dict = {} # 進入路のedge
        self._edgeIDs_within_junction_to_shelter_dict = {} # 避難所までの交差点内のedge辞書
        # self._avg_evac_time_by_route = {} # key: route value: avg_time

    # 車両周辺の車両情報を更新
    def update_around_vehInfos(self, around_vehID:str, speed:double, time_stamp:double):
        self._multiDict_around_vehInfos[around_vehID] = [speed, time_stamp]

    # 特定の周辺車両の最新車速を取得
    def get_latest_speed(self, around_vehID:str):
        return self._multiDict_around_vehInfos[around_vehID][0]

    # 特定の周辺車両の最新タイムスタンプを取得
    def get_latest_time_stamp(self, around_vehID:str):
        return self._multiDict_around_vehInfos[around_vehID][1]

    # 避難地の混雑情報を更新
    def update_shelter_congestion_info(self, shelterID:str, congestion:int, time_stamp:double):
        self._shelter_congestion_info[shelterID] = [congestion, time_stamp]

    # ルートごとの避難所要時間を更新する
    def v2shelter_update_avg_evac_time_by_route(self, avg_evac_time_by_route:dict):
        for route, info in avg_evac_time_by_route.items():
            self._avg_evac_time_by_route_from_shelter[route] = info
    
    # ルートごとの避難時間を更新する
    def v2v_avg_evac_time_by_route_by_recive_time(self, current_time:double):
        # 初期化
        self._avg_evac_time_by_route_by_recive_time = {}
        self._avg_evac_time_by_route_by_recive_time[current_time] = self.get_avg_evac_time_by_route_from_shelter()
    
    # 津波接近予兆情報を更新する
    def update_tsunami_precursor_info(self, vehID:str,  tsunami_precursor_flag:int, current_time:double):
        self._tsunami_precursor_info[vehID] = [tsunami_precursor_flag, current_time]
    
    def has_tsunami_precursor_info(self) -> bool:
        """
        info_dict: defaultdict(list) or dict
        e.g. {0: [], 1: [], 'init_ShelterB_1_35': [True, 234.0]}
        Returns True only if any value list starts with True.
        """
        for key, value in self.get_tsunami_precursor_info().items():
            # 値がリストで、かつ最初の要素が True なら
            if isinstance(value, list) and len(value) > 0 and value[0] is True:
                return True
        return False

    # 特定の避難所の混雑情報の最新タイムスタンプを取得
    def get_latest_time_stamp_of_shelter(self, shelter:str):
        try:
            return self._shelter_congestion_info[shelter][1]
        except Exception as e:
            return 0

    # 特定の周辺車両の最新避難地情報を取得
    def get_congestion_level_by_shelter(self, shelter:str):
        try:
            return self._shelter_congestion_info[shelter][0]
        except Exception as e:
            return 0
    
    # 特定の避難所の混雑情報があるかどうかを取得
    def has_shelter_full_info(self, shelterID: str, threshold: float = 0.98) -> bool:
        try:
            congestion, time_stamp = self._shelter_congestion_info[shelterID]
            # print(f"_shelter_congestion_info {self.get_shelter_congestion_info()} congestion: {congestion}, time_stamp: {time_stamp}")
            # if congestion > 0.97:
            #     print(f"vehID: {self.get_vehID()} has shelter_congestion_info for shelterID: {shelterID} congestion: {congestion}, time_stamp: {time_stamp}")
            return congestion >= threshold
        except Exception:
            return False

    def init_set_congestion_level_by_shelter(self, shelter:str, congestion:double, time_stamp:double):
        self._shelter_congestion_info[shelter] = [congestion, time_stamp]
    
    def init_set_avg_evac_time_by_route_by_recive_time(self):
        self._avg_evac_time_by_route_by_recive_time[0.0] = {}
    
    def init_set_tsunami_precursor_info(self):
        self._tsunami_precursor_info[self.get_vehID()] = [False, 0.0]
    
    def update_stopping_time(self):
        self._stopping_time += 1
    def clear_stopping_time(self):
        self._stopping_time = 0

    # 車両IDのgetter/setter
    def get_vehID(self):
        return self._vehID
    def set_vehID(self, vehID:str):
        self._vehID = vehID

    # 車両が向かう避難所のgetter/setter
    def get_target_shelter(self):
        return self._target_shelter
    def set_target_shelter(self, target_shelter:str):
        self._target_shelter = target_shelter

    # 車両が向かう避難所接続するedgeIDのgetter/setter
    def get_edgeID_connect_target_shelter(self):
        return self._edgeID_connect_target_shelter
    def set_edgeID_connect_target_shelter(self, edgeID_connect_target_shelter :str):
        self._edgeID_connect_target_shelter = edgeID_connect_target_shelter

    # 車両周辺の車両情報のgetter/setter
    def get_multiDict_around_vehIDs(self):
        return self._multiDict_around_vehInfos
    def set_multiDict_around_vehIDs(self, around_vehInfo:defaultdict):
        self._multiDict_around_vehInfos = around_vehInfo

    # 避難地の混雑情報のgetter/setter
    def get_shelter_congestion_info(self):
        return self._shelter_congestion_info
    def set_shelter_congestion_info(self, shelter_congestion_info:defaultdict):
        self._shelter_congestion_info = shelter_congestion_info

    # 津波前兆情報のgetter/setter
    def get_tsunami_precursor_info(self):
        return self._tsunami_precursor_info
    def set_tsunami_precursor_info(self, tsunami_precursor_info:defaultdict):
        self._tsunami_precursor_info = tsunami_precursor_info

    # 車両の生成時間のgetter/setter
    def get_create_time(self):
        return self._create_time
    def set_create_time(self, create_time: double):
        self._create_time = create_time
    
    # 車両の停止時間のgetter/setter
    def get_stopping_time(self):
        return self._stopping_time
    def set_stopping_time(self, stopping_time: double):
        self._stopping_time = stopping_time
    
    def get_evac_start_time(self):
        return self._evac_start_time
    def set_evac_start_time(self, evac_start_time: double):
        self._evac_start_time = evac_start_time

    def get_evac_end_time(self):
        return self._evac_end_time
    def set_evac_end_time(self, evac_end_time: double):
        self._evac_end_time = evac_end_time
    
    # 車両の到着フラグのgetter/setter
    def get_arrival_flag(self):
        return self._arrival_flag
    def set_arrival_flag(self, arrival_flag: bool):
        self._arrival_flag = arrival_flag
    
    # 車両の到着時間のgetter/setter
    def get_arrival_time(self):
        return self._arrival_time
    def set_arrival_time(self, arrival_time: double):
        self._arrival_time = arrival_time
    
    # 車両の避難所手前のedgeに到着したかどうかのフラグのgetter/setter
    def get_decline_edge_arrival_flag(self):
        return self._decline_edge_arrival_flag
    def set_decline_edge_arrival_flag(self, decline_edge_arrival_flag: bool):
        self._decline_edge_arrival_flag = decline_edge_arrival_flag
    
    # 車両の駐車しているかどうかのフラグのgetter/setter
    def get_parked_flag(self):
        return self._parked_flag
    def set_parked_flag(self, parked_flag: bool):
        self._parked_flag = parked_flag

    # 避難地から受け取る車両のルートごとの避難時間のgetter/setter
    def get_avg_evac_time_by_route_from_shelter(self):
        return self._avg_evac_time_by_route_from_shelter
    def set_avg_evac_time_by_route_from_shelter(self, avg_evac_time_by_route_from_shelter:dict):
        self._avg_evac_time_by_route_from_shelter = avg_evac_time_by_route_from_shelter
    
    # 現時点で、車両が持つ避難所のルートごとの避難時間のgetter/setter
    def get_avg_evac_time_by_route_by_recive_time(self):
        return self._avg_evac_time_by_route_by_recive_time
    def set_avg_evac_time_by_route_by_recive_time(self, avg_evac_time_by_route_by_recive_time:dict):
        self._avg_evac_time_by_route_by_recive_time = avg_evac_time_by_route_by_recive_time
    
    # 車両の停止フラグのgetter/setter
    def get_stop_flag(self):
        return self._stop_flag
    def set_stop_flag(self, stop_flag: bool):
        self._stop_flag = stop_flag

    # 車両の避難開始時間を測定するフラグのgetter/setter
    def get_start_time_measured_flag(self):
        return self._start_time_measured_flag
    def set_start_time_measured_flag(self, start_time_measured_flag: bool):
        self._start_time_measured_flag = start_time_measured_flag

    # 避難地を変更したか否かのフラグのgetter/setter
    def get_agent_changed_flag(self):
        return self._agent_changed_flag
    def set_agent_changed_flag(self, agent_changed_flag: bool):
        self._agent_changed_flag = agent_changed_flag
    
    # 車両の通信が有効かどうかのフラグのgetter/setter
    def get_vehicle_comm_enabled_flag(self):
        return self._vehicle_comm_enabled_flag
    def set_vehicle_comm_enabled_flag(self, vehicle_comm_enabled_flag: bool):
        self._vehicle_comm_enabled_flag = vehicle_comm_enabled_flag
    
    # 進入路のedgeリストのgetter/setter
    def get_approach_edge_dict(self):
        return self._approach_edge_dict
    def set_approach_edge_dict(self, approach_edge_dict: dict):
        self._approach_edge_dict = approach_edge_dict

    # 避難所までの交差点内のedge辞書のgetter/setter
    def get_edgeIDs_within_junction_to_shelter_dict(self):
        return self._edgeIDs_within_junction_to_shelter_dict
    def set_edgeIDs_within_junction_to_shelter_dict(self, edgeIDs_within_junction_to_shelter_dict: dict):
        self._edgeIDs_within_junction_to_shelter_dict = edgeIDs_within_junction_to_shelter_dict

    def print_shelter_congestion_info(self):
        print(f"vehID:{self._vehID} has shelter_congestion_info: {self._shelter_congestion_info}")

    def print_has_congestion_info(self):
        print(f"vehID:{self._vehID} has congestion_info: {self._shelter_congestion_info}")

    def print_all_info(self):
        print("========== VehicleInfo ==========")
        print(f"_vehID: {self._vehID}")
        print(f"_target_shelter: {self._target_shelter}")
        print(f"_edgeID_connect_target_shelter: {self._edgeID_connect_target_shelter}")
        print(f"_multiDict_around_vehInfos: {dict(self._multiDict_around_vehInfos)}")
        print(f"_shelter_congestion_info: {dict(self._shelter_congestion_info)}")
        print(f"_tsunami_precursor_info: {dict(self._tsunami_precursor_info)}")
        print(f"_create_time: {self._create_time}")
        print(f"_stopping_time: {self._stopping_time}")
        print(f"_evac_start_time: {self._evac_start_time}")
        print(f"_evac_end_time: {self._evac_end_time}")
        print(f"_arrival_flag: {self._arrival_flag}")
        print(f"_arrival_time: {self._arrival_time}")
        print(f"_decline_edge_arrival_flag: {self._decline_edge_arrival_flag}")
        print(f"_parked_flag: {self._parked_flag}")
        print(f"_avg_evac_time_by_route_from_shelter: {self._avg_evac_time_by_route_from_shelter}")
        print(f"_avg_evac_time_by_route_by_recive_time: {self._avg_evac_time_by_route_by_recive_time}")
        print(f"_stop_flag: {self._stop_flag}")
        print(f"_start_time_measured_flag: {self._start_time_measured_flag}")
        print(f"_agent_changed_flag: {self._agent_changed_flag}")
        print(f"_vehicle_comm_enabled_flag: {self._vehicle_comm_enabled_flag}")
        print(f"_approach_edge_dict: {self._approach_edge_dict}")
        print(f"_edgeIDs_within_junction_to_shelter_dict: {self._edgeIDs_within_junction_to_shelter_dict}")
        print("=================================")