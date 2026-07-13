# =========================
# Standard library
# =========================
import random
import sys
from collections import Counter, defaultdict
from pathlib import Path
import argparse
import math
from dataclasses import dataclass
from enum import Enum
from typing import Literal
import secrets

# =========================
# Third-party libraries
# =========================
from evacsim.sim.sumo_env import ensure_sumo_tools_on_path

ensure_sumo_tools_on_path()

from sumolib import checkBinary  # noqa: E402
import traci  # noqa: E402
import numpy as np
from numpy import double

# =========================
# Local / intra-package
# =========================
from evacsim.agents.Agent import Agent
from evacsim.agents.Shelter import Shelter
from evacsim.agents.VehicleInfo import VehicleInfo

from evacsim.sim.traci_cache import (
    create_step_cache,
    get_vehicle_road_id_cached,
)

from evacsim.utils.lookup import (
    find_shelter_by_edgeID_connect_target_shelter,
    find_shelterID_by_edgeID_by_shelterID,
)

from evacsim.sim.arrival import handle_arrival

from evacsim.maps.edge_utils import (
    is_pre_edgeID_near_shelter,
    get_vehicle_start_edges,
    get_vehicle_end_edges,
)

from evacsim.sim.congestion import get_local_density

from evacsim.sim.traffic_state import (
    apply_gap_density_speed_control,
)

from evacsim.sim.neighbors import (
    get_around_vehIDs,
)

from evacsim.sim.communication import (
    CommunicationMode,
    execute_v2v_communication_round_traci,
    merge_route_info_within_shelters,
    record_direct_tsunami_precursor_info,
    v2shelter_communication,
)

from evacsim.metrics.evacuation_time import (
    calculate_avg_evac_time_by_route,
    merge_arrival_vehs_of_shelter,
)

from evacsim.sim.routing import (
    find_alternative_route_calculated_time,
    get_route_time_difference_exceeding_threshold,
    find_uturn_shortest_route_to_current_shelter_group,
)


from evacsim.maps.map_loader import (
    init_shelter,
    init_custome_edge,
)

from evacsim.maps.route_map import (
    get_vehicle_end_list_by_start_edge_dict,
)

from evacsim.io.json_io import (
    import_connected_edges_from_json,
    import_start_end_edgeIDs_from_json,
)

from evacsim.utils.random_utils import choose_edge_by_probability

from evacsim.sim.vehicle_generation import (
    generate_simple_init_vehID,
    generate_new_veh_based_on_route_time,
)

from evacsim.sim.initialization import (
    init_vehicleInfo_list_base,
    init_agent_list,
)

from evacsim.sim.neighbors import (
    count_rc_around_vehicles,
)

# =========================
# Runtime config / seeds
# =========================
DEFAULT_RANDOM_SEED = 0
random.seed(DEFAULT_RANDOM_SEED)
np.random.seed(DEFAULT_RANDOM_SEED)

# runner_base.py のファイル位置から data ディレクトリを解決
HERE = Path(__file__).resolve()
MAP_ONE_DIR = HERE.parent.parent          # .../scenarios/its102/map_one
DATA_DIR = MAP_ONE_DIR / "data"
SUMO_CFG = DATA_DIR / "ishinomaki_one_two_one.sumocfg"

# =========================
# 1) シミュレーション基本設定・時間評価
# =========================
END_SIMULATION_TIME = 3500
DECISION_INTERVAL = 10.0
COMMUNICATION_INTERVAL = 10.0
THRESHOLD_SPEED = 2.00  # 7.2 km/h
STOPPING_TIME_IN_SHELTER = 10000000
SLOW_DURATION = 15.0
SLOW_SPEED = 3.0

# =========================
# Route-change decision model
# =========================
P_FOLLOW = 0.2
SHELTER_FULL_INFO_THRESHOLD = 0.98
ROUTE_CHANGE_COLOR = (60, 180, 120)

# =========================
# 2) 車両生成・発進関連
# =========================
VEHICLE_NUM = 0
DEPART_TIME: double = 0.0
ROUTE_NUM = 0


# =========================
# 8) 心理モデル：初期タイプ別しきい（早期/遅延）
# =========================
EARLY_AGENT_THRESHOLD_LIST = [60, 90, 100, 130] # 早期決断者の閾値
LATE_AGENT_THRESHOLD_LIST = [180, 220, 300, 350] # 遅延決断者の閾値

# =========================
# 10) 実行時カウンタ（毎試行でリセットされる動的値）
# =========================
ROUTE_CHANGED_VEHICLE_COUNT = 0
ROUTE_CHANGE_EVENT_COUNT = 0
NORMALCY_BIAS_ROUTE_CHANGE_COUNT = 0
MAJORITY_BIAS_ROUTE_CHANGE_COUNT = 0
LANE_CHANGED_VEHICLE_COUNT = 0
WRONG_WAY_SUCCESS_COUNT = 0
OBTAIN_INFO_LANE_CHANGE_COUNT = 0
ELAPSED_TIME_LANE_CHANGE_COUNT = 0
POSITIVE_MAJORITY_BIAS_COUNT = 0
PEDESTRIAN_COUNT = 0
NEW_VEHICLE_COUNT = 0
ROUTE_CHANGE_COUNT_AFTER_SHELTER_CAPACITY_FULL = 0
VEHICLE_ABANDONMENT_SUCCESS_COUNT = 0


# リストの初期化
custome_edge_list: list = []
shelter_list: list[Shelter] = []
vehInfo_list = []
vehID_list = []
arrival_time_by_vehID_dict = {}
arrival_time_list = []
elapsed_time_list = []
vehicle_abandant_time_by_pedestrianID_dict = {}
walking_distance_by_pedestrianID_dict = {}
pedstrianID_list = []
agent_by_vehID_dict = {}
vehInfo_by_vehID_dict = {}
route_edges_by_routeID_dict = {}
route_change_time_by_vehID_dict = {}

# main()で設定される実行時設定。関数から安全に参照できるよう初期値を持たせる。
COMM_RANGE = 0.0
COMMUNICATION_MODE = CommunicationMode.V2V_ONLY
edgeID_by_shelterID: dict[str, str] = {}
LAST_COMMUNICATION_ROUND_RESULT = None


# =========================
# Route-change state / event accounting
# =========================
RouteChangeReason = Literal["normalcy", "majority", "shelter_full"]
RouteChangeType = Literal["route_change", "uturn"]


class RouteChangeState(str, Enum):
    NOT_EVALUATED = "not_evaluated"
    EVALUATED_NO_CHANGE = "evaluated_no_change"
    DECIDED = "decided"
    EXECUTING = "executing"
    COMPLETED = "completed"
    FAILED = "failed"


@dataclass(frozen=True)
class RouteChangeEvent:
    event_id: str
    logical_vehicle_id: str
    sumo_vehicle_id_before: str
    sumo_vehicle_id_after: str | None
    decision_time: float
    execution_time: float | None
    reason: RouteChangeReason
    change_type: RouteChangeType
    from_edge_id: str
    target_shelter_id: str
    route_before: tuple[str, ...]
    route_after: tuple[str, ...]
    success: bool
    failure_reason: str | None = None
    observed_event_ids: tuple[str, ...] = tuple()
    follow_random_value: float | None = None


# 原則として1台1回だけ経路変更する現行モデル。
# 複数回を許可する場合も、イベント数とユニーク車両数は別集計される。
ALLOW_MULTIPLE_ROUTE_CHANGES_PER_LOGICAL_VEHICLE = False
ROUTE_CHANGE_PROHIBITED_EDGES = {"E13", "E150", "E16", "E12", "E5"}

route_change_events: list[RouteChangeEvent] = []
route_change_state_by_logical_vehicle_id: dict[str, RouteChangeState] = {}
route_change_reason_by_logical_vehicle_id: dict[str, RouteChangeReason] = {}
logical_vehicle_id_by_sumo_vehicle_id: dict[str, str] = {}
route_changed_logical_vehicle_ids: set[str] = set()

# 同調性バイアス:
# 同じ経路変更イベントは一度だけ評価するが、新しいイベントを観測した場合は再評価する。
MAJORITY_OBSERVATION_WINDOW = 60.0
MAJORITY_EVENT_WEIGHTS: dict[str, float] = {
    "route_change": 1.0,
    "uturn": 1.0,
}

latest_successful_route_change_event_by_logical_vehicle_id: dict[
    str,
    RouteChangeEvent,
] = {}
observed_route_change_event_ids_by_logical_vehicle_id: dict[
    str,
    set[str],
] = defaultdict(set)

majority_bias_evaluation_count_by_logical_vehicle_id: dict[str, int] = defaultdict(int)
majority_bias_rejection_count_by_logical_vehicle_id: dict[str, int] = defaultdict(int)


def get_avg_time_by_route(
    route_info_with_receive_time: dict,
    target_route,
) -> float | None:
    """
    指定した経路に対応する平均所要時間を取得する。

    Args:
        route_info_with_receive_time:
            {
                receive_time: {
                    route_tuple: {
                        "avg_time": float,
                        "vehicles": int,
                    }
                }
            }

        target_route:
            検索対象の経路。listまたはtuple。

    Returns:
        対応するavg_time。
        経路が存在しない場合はNone。
    """
    if not route_info_with_receive_time:
        return None

    latest_time = max(route_info_with_receive_time)
    route_info_dict = route_info_with_receive_time.get(
        latest_time,
        {},
    )

    target_route = tuple(target_route)
    route_info = route_info_dict.get(target_route)

    if route_info is None:
        return None

    avg_time = route_info.get("avg_time")

    if avg_time is None:
        return None

    return float(avg_time)

def _is_interval_step(current_time: float, interval: float) -> bool:
    """浮動小数点誤差を許容して、interval 秒境界かを判定する。"""
    if interval <= 0:
        raise ValueError(f"interval must be positive: {interval}")
    quotient = current_time / interval
    return math.isclose(quotient, round(quotient), rel_tol=0.0, abs_tol=1e-9)


def _get_logical_vehicle_id(sumo_vehicle_id: str) -> str:
    """SUMO IDが変わっても不変の論理車両IDを返す。"""
    return logical_vehicle_id_by_sumo_vehicle_id.setdefault(
        sumo_vehicle_id,
        sumo_vehicle_id,
    )


def _safe_get_route(vehID: str) -> tuple[str, ...]:
    try:
        return tuple(traci.vehicle.getRoute(vehID))
    except Exception:
        return tuple()


def _refresh_route_change_counters() -> None:
    """成功イベントから全カウンタを再計算し、直接加算の不整合を防ぐ。"""
    global ROUTE_CHANGED_VEHICLE_COUNT
    global ROUTE_CHANGE_EVENT_COUNT
    global NORMALCY_BIAS_ROUTE_CHANGE_COUNT
    global MAJORITY_BIAS_ROUTE_CHANGE_COUNT
    global ROUTE_CHANGE_COUNT_AFTER_SHELTER_CAPACITY_FULL

    successful_events = [event for event in route_change_events if event.success]
    unique_vehicle_ids = {
        event.logical_vehicle_id
        for event in successful_events
    }

    ROUTE_CHANGE_EVENT_COUNT = len(successful_events)
    ROUTE_CHANGED_VEHICLE_COUNT = len(unique_vehicle_ids)
    NORMALCY_BIAS_ROUTE_CHANGE_COUNT = sum(
        event.reason == "normalcy" for event in successful_events
    )
    MAJORITY_BIAS_ROUTE_CHANGE_COUNT = sum(
        event.reason == "majority" for event in successful_events
    )
    ROUTE_CHANGE_COUNT_AFTER_SHELTER_CAPACITY_FULL = sum(
        event.reason == "shelter_full" for event in successful_events
    )


def _new_route_change_event_id(
    logical_vehicle_id: str,
    *,
    success: bool,
) -> str:
    """試行内で一意な経路変更イベントIDを生成する。"""
    status = "success" if success else "failure"
    return f"{logical_vehicle_id}:{status}:{len(route_change_events)}"


def _record_route_change_event(event: RouteChangeEvent) -> None:
    route_change_events.append(event)

    if event.success:
        latest_successful_route_change_event_by_logical_vehicle_id[
            event.logical_vehicle_id
        ] = event

    _refresh_route_change_counters()

    # print(
    #     "[ROUTE_CHANGE]",
    #     f"logical_vehicle_id={event.logical_vehicle_id}",
    #     f"sumo_vehicle_id_before={event.sumo_vehicle_id_before}",
    #     f"sumo_vehicle_id_after={event.sumo_vehicle_id_after}",
    #     f"decision_time={event.decision_time}",
    #     f"execution_time={event.execution_time}",
    #     f"reason={event.reason}",
    #     f"state={route_change_state_by_logical_vehicle_id.get(event.logical_vehicle_id)}",
    #     f"from_edge={event.from_edge_id}",
    #     f"new_shelter={event.target_shelter_id}",
    #     f"success={event.success}",
    #     f"failure_reason={event.failure_reason}",
    #     f"observed_event_ids={event.observed_event_ids}",
    #     f"follow_random_value={event.follow_random_value}",
    #     f"route_before={event.route_before}",
    #     f"route_after={event.route_after}",
    #     f"route_changed_vehicle_count={ROUTE_CHANGED_VEHICLE_COUNT}",
    #     f"route_change_event_count={ROUTE_CHANGE_EVENT_COUNT}",
    # )


def _route_change_already_completed(logical_vehicle_id: str) -> bool:
    return (
        not ALLOW_MULTIPLE_ROUTE_CHANGES_PER_LOGICAL_VEHICLE
        and logical_vehicle_id in route_changed_logical_vehicle_ids
    )


def _get_shelter_id_by_destination_edge(
    destination_edge_id: str,
) -> str | None:
    """目的地エッジに対応する避難所IDを返す。"""
    matches = [
        shelter_id
        for shelter_id, edge_id in edgeID_by_shelterID.items()
        if edge_id == destination_edge_id
    ]
    if not matches:
        return None
    if len(matches) > 1:
        raise ValueError(
            "Multiple shelters share the same destination edge: "
            f"edge={destination_edge_id}, shelters={matches}"
        )
    return matches[0]


def _get_shelter_id_from_route(
    route_edges: tuple[str, ...] | list[str],
) -> str | None:
    if not route_edges:
        return None
    return _get_shelter_id_by_destination_edge(route_edges[-1])


def _synchronize_vehicle_destination(
    sumo_vehicle_id: str,
    shelter_id: str,
) -> None:
    """AgentとVehicleInfoの目的避難所を同時に更新する。"""
    if not shelter_id:
        return

    target_edge_id = edgeID_by_shelterID.get(shelter_id)
    if target_edge_id is None:
        raise KeyError(
            f"Unknown shelter_id={shelter_id!r}; "
            f"known={sorted(edgeID_by_shelterID)}"
        )

    target_agent = agent_by_vehID_dict.get(sumo_vehicle_id)
    if target_agent is not None:
        set_target_shelter = getattr(
            target_agent,
            "set_target_shelter",
            None,
        )
        if callable(set_target_shelter):
            set_target_shelter(shelter_id)

    target_vehicle_info = vehInfo_by_vehID_dict.get(sumo_vehicle_id)
    if target_vehicle_info is not None:
        target_vehicle_info.set_target_shelter(shelter_id)
        target_vehicle_info.set_edgeID_connect_target_shelter(
            target_edge_id
        )


def _mark_successful_route_change(
    *,
    logical_vehicle_id: str,
    sumo_vehicle_id_before: str,
    sumo_vehicle_id_after: str | None,
    agent: Agent,
    decision_time: float,
    execution_time: float,
    reason: RouteChangeReason,
    from_edge_id: str,
    target_shelter_id: str,
    route_before: tuple[str, ...],
    route_after: tuple[str, ...],
    change_type: RouteChangeType = "route_change",
    observed_event_ids: tuple[str, ...] = tuple(),
    follow_random_value: float | None = None,
) -> None:
    """経路変更が実際に成功した後だけ、状態・イベント・カウンタを更新する。"""
    if _route_change_already_completed(logical_vehicle_id):
        raise AssertionError(
            "Duplicate successful route change was about to be recorded: "
            f"logical_vehicle_id={logical_vehicle_id}, reason={reason}"
        )

    agent.set_evacuation_route_changed_flg(True)
    agent.set_agent_action_name("rc")

    # 同一IDの経路差し替えでも、AgentだけでなくVehicleInfoも更新する。
    destination_vehicle_id = (
        sumo_vehicle_id_after or sumo_vehicle_id_before
    )
    _synchronize_vehicle_destination(
        destination_vehicle_id,
        target_shelter_id,
    )

    if change_type == "uturn":
        set_uturn_flag = getattr(agent, "set_route_change_uturn_flg", None)
        if callable(set_uturn_flag):
            set_uturn_flag(True)

    route_change_state_by_logical_vehicle_id[logical_vehicle_id] = RouteChangeState.COMPLETED
    route_change_reason_by_logical_vehicle_id[logical_vehicle_id] = reason
    route_changed_logical_vehicle_ids.add(logical_vehicle_id)
    route_change_time_by_vehID_dict.setdefault(logical_vehicle_id, execution_time)

    if sumo_vehicle_id_after:
        logical_vehicle_id_by_sumo_vehicle_id[sumo_vehicle_id_after] = logical_vehicle_id
        new_agent = agent_by_vehID_dict.get(sumo_vehicle_id_after)
        if new_agent is not None:
            new_agent.set_evacuation_route_changed_flg(True)
            new_agent.set_agent_action_name("rc")
            if change_type == "uturn":
                set_new_uturn_flag = getattr(
                    new_agent,
                    "set_route_change_uturn_flg",
                    None,
                )
                if callable(set_new_uturn_flag):
                    set_new_uturn_flag(True)

    event_id = _new_route_change_event_id(
        logical_vehicle_id,
        success=True,
    )
    _record_route_change_event(
        RouteChangeEvent(
            event_id=event_id,
            logical_vehicle_id=logical_vehicle_id,
            sumo_vehicle_id_before=sumo_vehicle_id_before,
            sumo_vehicle_id_after=sumo_vehicle_id_after,
            decision_time=decision_time,
            execution_time=execution_time,
            reason=reason,
            change_type=change_type,
            from_edge_id=from_edge_id,
            target_shelter_id=target_shelter_id,
            route_before=route_before,
            route_after=route_after,
            success=True,
            observed_event_ids=observed_event_ids,
            follow_random_value=follow_random_value,
        )
    )

def _mark_failed_route_change(
    *,
    logical_vehicle_id: str,
    sumo_vehicle_id: str,
    decision_time: float,
    reason: RouteChangeReason,
    from_edge_id: str,
    target_shelter_id: str,
    route_before: tuple[str, ...],
    failure_reason: str,
    change_type: RouteChangeType = "route_change",
    observed_event_ids: tuple[str, ...] = tuple(),
    follow_random_value: float | None = None,
) -> None:
    """失敗はイベントとして残すが、成功カウンタ・変更済みフラグは更新しない。"""
    route_change_state_by_logical_vehicle_id[logical_vehicle_id] = RouteChangeState.FAILED
    event_id = _new_route_change_event_id(
        logical_vehicle_id,
        success=False,
    )
    _record_route_change_event(
        RouteChangeEvent(
            event_id=event_id,
            logical_vehicle_id=logical_vehicle_id,
            sumo_vehicle_id_before=sumo_vehicle_id,
            sumo_vehicle_id_after=None,
            decision_time=decision_time,
            execution_time=None,
            reason=reason,
            change_type=change_type,
            from_edge_id=from_edge_id,
            target_shelter_id=target_shelter_id,
            route_before=route_before,
            route_after=route_before,
            success=False,
            failure_reason=failure_reason,
            observed_event_ids=observed_event_ids,
            follow_random_value=follow_random_value,
        )
    )

def _execute_set_route_id(
    vehID: str,
    routeID: str,
) -> tuple[bool, tuple[str, ...], tuple[str, ...], str | None]:
    route_before = _safe_get_route(vehID)
    try:
        traci.vehicle.setRouteID(vehID, routeID)
    except traci.exceptions.TraCIException as exc:
        return False, route_before, route_before, str(exc)

    route_after = _safe_get_route(vehID)
    if route_before and route_after == route_before:
        return False, route_before, route_after, "route did not change"

    try:
        traci.vehicle.setColor(vehID, ROUTE_CHANGE_COLOR)
    except traci.exceptions.TraCIException as exc:
        # 色変更は可視化のみ。経路変更成功を取り消さない。
        print(f"[ROUTE_CHANGE_COLOR_WARNING] vehID={vehID} error={exc}")

    return True, route_before, route_after, None



def _execute_set_route_edges(
    *,
    vehID: str,
    current_edgeID: str,
    route_edges_for_sumo: tuple[str, ...] | list[str],
) -> tuple[bool, tuple[str, ...], tuple[str, ...], str | None]:
    """候補経路が現在エッジから始まる場合、車両再生成なしで残り経路を差し替える。"""
    route_before = _safe_get_route(vehID)
    route_edges = list(route_edges_for_sumo)
    if not route_edges:
        return False, route_before, route_before, "route_edges_for_sumo is empty"
    if current_edgeID not in route_edges:
        return (
            False,
            route_before,
            route_before,
            f"current edge {current_edgeID} is not in candidate route",
        )

    remaining_route = route_edges[route_edges.index(current_edgeID):]
    if not remaining_route or remaining_route[0] != current_edgeID:
        return False, route_before, route_before, "candidate route does not start at current edge"
    shelterID = _get_shelter_id_by_destination_edge(
        remaining_route[-1]
    )
    if shelterID is None:
        return (
            False,
            route_before,
            route_before,
            "candidate route does not end at a known shelter",
        )
    try:
        traci.vehicle.setRoute(vehID, remaining_route)
        traci.vehicle.setParkingAreaStop(
                vehID=vehID,
                stopID=shelterID,
                duration=100000,
        )
    except traci.exceptions.TraCIException as exc:
        return False, route_before, route_before, str(exc)

    route_after = _safe_get_route(vehID)
    if route_before and route_after == route_before:
        return False, route_before, route_after, "route did not change"

    try:
        traci.vehicle.setColor(vehID, ROUTE_CHANGE_COLOR)
    except traci.exceptions.TraCIException as exc:
        print(f"[ROUTE_CHANGE_COLOR_WARNING] vehID={vehID} error={exc}")

    return True, route_before, route_after, None

def _execute_generated_vehicle_route_change(
    *,
    target_vehID: str,
    current_new_vehicle_count: int,
    vehInfo_by_target_vehID: VehicleInfo,
    agent_by_target_vehID: Agent,
    from_edgeID: str,
    new_shelterID: str,
    to_edgeID: str,
) -> tuple[int, bool, str | None, tuple[str, ...], tuple[str, ...], str | None]:
    """
    車両再生成型の経路変更を実行する。

    Returns:
        updated_count, success, new_vehID, route_before, route_after, error
    """
    if not (from_edgeID and new_shelterID and to_edgeID):
        return (
            current_new_vehicle_count,
            False,
            None,
            _safe_get_route(target_vehID),
            tuple(),
            "empty route-change candidate",
        )

    route_before = _safe_get_route(target_vehID)
    agent_ids_before = set(agent_by_vehID_dict)
    vehinfo_ids_before = set(vehInfo_by_vehID_dict)

    try:
        updated_count = generate_new_veh_based_on_route_time(
            target_vehID=target_vehID,
            NEW_VEHICLE_COUNT=current_new_vehicle_count,
            agent_list=agent_list,
            vehInfo_list=vehInfo_list,
            vehInfo_by_target_vehID=vehInfo_by_target_vehID,
            agent_by_target_vehID=agent_by_target_vehID,
            from_edgeID=from_edgeID,
            new_shelterID=new_shelterID,
            to_edgeID=to_edgeID,
            color_mode="",
            agent_by_vehID_dict=agent_by_vehID_dict,
            vehInfo_by_vehID_dict=vehInfo_by_vehID_dict,
        )
    except Exception as exc:
        return (
            current_new_vehicle_count,
            False,
            None,
            route_before,
            tuple(),
            str(exc),
        )

    new_agent_ids = set(agent_by_vehID_dict) - agent_ids_before
    new_vehinfo_ids = set(vehInfo_by_vehID_dict) - vehinfo_ids_before
    new_vehicle_ids = sorted(new_agent_ids & new_vehinfo_ids)

    if updated_count <= current_new_vehicle_count:
        return updated_count, False, None, route_before, tuple(), "new vehicle count did not increase"
    if len(new_vehicle_ids) != 1:
        return (
            updated_count,
            False,
            None,
            route_before,
            tuple(),
            f"expected one generated vehicle, got {new_vehicle_ids}",
        )

    new_vehID = new_vehicle_ids[0]
    route_after = _safe_get_route(new_vehID)
    return updated_count, True, new_vehID, route_before, route_after, None


def _assert_route_change_consistency() -> None:
    successful_events = [event for event in route_change_events if event.success]
    assert ROUTE_CHANGE_EVENT_COUNT == len(successful_events)
    assert ROUTE_CHANGED_VEHICLE_COUNT == len(
        {event.logical_vehicle_id for event in successful_events}
    )
    assert (
        NORMALCY_BIAS_ROUTE_CHANGE_COUNT
        + MAJORITY_BIAS_ROUTE_CHANGE_COUNT
        + ROUTE_CHANGE_COUNT_AFTER_SHELTER_CAPACITY_FULL
        == ROUTE_CHANGE_EVENT_COUNT
    )

def run() -> None:
    """SUMOシミュレーションを実行し、終了時にTraCI接続を確実に閉じる。"""
    try:
        while traci.simulation.getTime() < END_SIMULATION_TIME:
            traci.simulationStep()
            control_vehicles()
    finally:
        try:
            traci.close()
        except Exception:
            pass
        sys.stdout.flush()


def _build_candidate_neighbors(
    active_vehicle_ids: tuple[str, ...],
    *,
    step_cache,
) -> dict[str, tuple[str, ...]]:
    """通信ラウンド開始時点の近傍候補を全車両分まとめて取得する。"""
    active_vehicle_id_set = set(active_vehicle_ids)
    candidates: dict[str, tuple[str, ...]] = {}

    for vehicle_id in active_vehicle_ids:
        try:
            around_vehicle_ids = get_around_vehIDs(
                target_vehID=vehicle_id,
                custome_edge_list=custome_edge_list,
                step_cache=step_cache,
            )
        except Exception as exc:
            print(
                "[COMMUNICATION_NEIGHBOR_WARNING]",
                f"vehicle_id={vehicle_id}",
                f"error={exc}",
            )
            around_vehicle_ids = ()

        candidates[vehicle_id] = tuple(
            sorted(
                other_vehicle_id
                for other_vehicle_id in around_vehicle_ids
                if other_vehicle_id in active_vehicle_id_set
                and other_vehicle_id != vehicle_id
            )
        )

    return candidates


def process_communication_round(
    *,
    active_vehicle_ids: tuple[str, ...],
    current_time: float,
    step_cache,
    tsunami_detected_vehicle_ids: tuple[str, ...] = (),
):
    """
    1通信ラウンドを順序非依存で処理する。

    1. 全車両のV2Shelter直接取得情報を確定
    2. 必要なら津波前兆の直接検知情報を確定
    3. 通信開始時点のスナップショットからV2V共有結果を一括計算
    """
    if COMMUNICATION_INTERVAL <= 0:
        raise ValueError(
            "COMMUNICATION_INTERVAL must be positive: "
            f"{COMMUNICATION_INTERVAL}"
        )

    communication_round = int(
        round(current_time / COMMUNICATION_INTERVAL)
    )

    shelter_id_by_vehicle_id: dict[str, str] = {}
    eligible_vehicle_ids: list[str] = []

    for vehicle_id in active_vehicle_ids:
        vehicle_info = vehInfo_by_vehID_dict.get(vehicle_id)
        if vehicle_info is None or vehicle_id not in agent_by_vehID_dict:
            continue
        if not vehicle_info.get_vehicle_comm_enabled_flag():
            continue
        if vehicle_info.get_arrival_flag():
            continue

        try:
            current_edge_id = get_vehicle_road_id_cached(
                vehicle_id,
                step_cache=step_cache,
            )
        except Exception:
            continue
        if current_edge_id in set(edgeID_by_shelterID.values()):
            # 到着処理より前に通信を実行しているため、到着エッジ上の車両は除外する。
            continue

        shelter_id = vehicle_info.get_target_shelter()
        if not shelter_id:
            continue

        eligible_vehicle_ids.append(vehicle_id)
        shelter_id_by_vehicle_id[vehicle_id] = shelter_id

    eligible_vehicle_ids_tuple = tuple(sorted(eligible_vehicle_ids))

    # Phase 1: 直接取得情報を先に全車両分確定する。
    for vehicle_id in eligible_vehicle_ids_tuple:
        vehicle_info = vehInfo_by_vehID_dict[vehicle_id]
        v2shelter_communication(
            target_vehID=vehicle_id,
            shelterID=shelter_id_by_vehicle_id[vehicle_id],
            vehInfo_list=vehInfo_list,
            shelter_list=shelter_list,
            COMMUNICATION_RANGE=COMM_RANGE,
            target_vehInfo=vehicle_info,
            vehInfo_by_vehID_dict=vehInfo_by_vehID_dict,
            step_cache=step_cache,
            current_time=current_time,
            communication_round=communication_round,
        )

    eligible_vehicle_id_set = set(eligible_vehicle_ids_tuple)
    for vehicle_id in tsunami_detected_vehicle_ids:
        if vehicle_id not in eligible_vehicle_id_set:
            continue
        record_direct_tsunami_precursor_info(
            vehInfo_by_vehID_dict[vehicle_id],
            detected_time=current_time,
            communication_round=communication_round,
            source_id=vehicle_id,
        )

    candidate_neighbors_by_vehicle_id = _build_candidate_neighbors(
        eligible_vehicle_ids_tuple,
        step_cache=step_cache,
    )

    # Phase 2/3: スナップショット計算と一括反映。
    return execute_v2v_communication_round_traci(
        active_vehicle_ids=eligible_vehicle_ids_tuple,
        vehicle_info_by_id=vehInfo_by_vehID_dict,
        communication_range=COMM_RANGE,
        communication_mode=COMMUNICATION_MODE,
        communication_round=communication_round,
        current_time=current_time,
        candidate_neighbors_by_vehicle_id=(
            candidate_neighbors_by_vehicle_id
        ),
        step_cache=step_cache,
    )


def control_vehicles() -> None:
    global NEW_VEHICLE_COUNT
    global LAST_COMMUNICATION_ROUND_RESULT

    vehIDs = tuple(traci.vehicle.getIDList())
    current_time = float(traci.simulation.getTime())
    is_decision_step = _is_interval_step(
        current_time,
        DECISION_INTERVAL,
    )
    is_communication_step = _is_interval_step(
        current_time,
        COMMUNICATION_INTERVAL,
    )
    step_cache = create_step_cache(current_time=current_time)

    # 通信は車両ループ内ではなく、ラウンドごとに1回だけ実行する。
    if is_communication_step:
        LAST_COMMUNICATION_ROUND_RESULT = process_communication_round(
            active_vehicle_ids=vehIDs,
            current_time=current_time,
            step_cache=step_cache,
        )

    for current_vehID in vehIDs:
        vehInfo_by_current_vehID: VehicleInfo | None = vehInfo_by_vehID_dict.get(current_vehID)
        agent_by_current_vehID: Agent | None = agent_by_vehID_dict.get(current_vehID)

        if vehInfo_by_current_vehID is None or agent_by_current_vehID is None:
            continue

        logical_vehicle_id = _get_logical_vehicle_id(current_vehID)
        route_change_state_by_logical_vehicle_id.setdefault(
            logical_vehicle_id,
            RouteChangeState.NOT_EVALUATED,
        )

        shelter_for_current_vehID: Shelter = find_shelter_by_edgeID_connect_target_shelter(
            vehInfo_by_current_vehID.get_edgeID_connect_target_shelter(),
            shelter_list,
        )
        current_edgeID: str = get_vehicle_road_id_cached(
            current_vehID,
            step_cache=step_cache,
        )
        if not agent_by_current_vehID.get_created_time_flg():
            agent_by_current_vehID.set_created_time(current_time)
            agent_by_current_vehID.set_created_time_flg(True)

        # 到着処理
        if (
            current_edgeID in set(edgeID_by_shelterID.values())
            and not vehInfo_by_current_vehID.get_arrival_flag()
        ):
            handle_arrival(
                current_vehID=current_vehID,
                vehInfo_by_current_vehID=vehInfo_by_current_vehID,
                agent_by_current_vehID=agent_by_current_vehID,
                shelter_for_current_vehID=shelter_for_current_vehID,
                shelter_list=shelter_list,
                arrival_time_list=arrival_time_list,
                arrival_time_by_vehID_dict=arrival_time_by_vehID_dict,
                elapsed_time_list=elapsed_time_list,
            )
            continue

        # 避難所近傍の速度制御
        if not vehInfo_by_current_vehID.get_decline_edge_arrival_flag():
            traci.vehicle.setLaneChangeMode(current_vehID, 1024)
            pre_edgeID_near_shelter_flag = is_pre_edgeID_near_shelter(
                current_edgeID=current_edgeID,
                edgeID_near_shelter=vehInfo_by_current_vehID.get_edgeID_connect_target_shelter(),
                custome_edge_list=custome_edge_list,
            )
            if pre_edgeID_near_shelter_flag:
                local_density = get_local_density(vehID=current_vehID, radius=50.0)
                apply_gap_density_speed_control(
                    vehID=current_vehID,
                    local_density=local_density,
                    v_free=6.0,
                    v_min=2.5,
                    gap_min=7.0,
                    tau=1.8,
                    alpha=0.5,
                    slow_time=1.0,
                )

        # V2V/V2Shelter通信は、この車両ループより前にラウンド単位で完了済み。

        if vehInfo_by_current_vehID.get_arrival_flag():
            continue

        # 現行モデルでは、論理車両単位で経路変更は一度だけ。
        if _route_change_already_completed(logical_vehicle_id):
            assert agent_by_current_vehID.get_evacuation_route_changed_flg(), (
                "completed logical vehicle must keep route-changed flag: "
                f"logical_vehicle_id={logical_vehicle_id}, sumo_vehicle_id={current_vehID}"
            )
            continue

        if not is_decision_step:
            continue

        if current_edgeID in {"E1"}:

            route_info_with_receive_time = (
                vehInfo_by_current_vehID
                .get_avg_evac_time_by_route_by_recive_time()
            )

            # 経路情報がない場合
            if not route_info_with_receive_time:
                continue

            # 最新時刻の経路情報を取得
            latest_receive_time = max(route_info_with_receive_time)
            latest_route_info = route_info_with_receive_time.get(
                latest_receive_time,
                {},
            )

            # 現在経路と代替経路の2経路分が必要
            if len(latest_route_info) < 2:
                continue

            # 分岐前のE1でのみ、受信した平均所要時間を使って判断
            if current_edgeID == "E1":
                current_route = tuple(
                    traci.vehicle.getRoute(current_vehID)
                )

                target_shelterID = (
                    agent_by_current_vehID.get_target_shelter()
                )

                if target_shelterID == "ShelterA_1":
                    alternative_route_id = "E0_E13_0"
                    alternative_shelter_id = "ShelterA_2"
                elif target_shelterID == "ShelterA_2":
                    alternative_route_id = "E0_E16_0"
                    alternative_shelter_id = "ShelterA_1"
                else:
                    continue

                alternative_route_edges = tuple(
                    traci.route.getEdges(
                        alternative_route_id
                    )
                )

                current_route_avg_time = get_avg_time_by_route(
                    route_info_with_receive_time=(
                        route_info_with_receive_time
                    ),
                    target_route=current_route,
                )

                alternative_route_avg_time = get_avg_time_by_route(
                    route_info_with_receive_time=(
                        route_info_with_receive_time
                    ),
                    target_route=alternative_route_edges,
                )

                # どちらかの平均所要時間を取得できない場合
                if (
                    current_route_avg_time is None
                    or alternative_route_avg_time is None
                ):
                    continue

                time_gap = (
                    current_route_avg_time
                    - alternative_route_avg_time
                )

                route_change_threshold = float(
                    agent_by_current_vehID
                    .get_route_change_threshold()
                )

                # 代替経路の時間短縮量が閾値以下なら変更しない
                if time_gap <= route_change_threshold:
                    continue

                # 経路変更を決意
                route_change_state_by_logical_vehicle_id[
                    logical_vehicle_id
                ] = RouteChangeState.DECIDED

                # 代替経路に現在エッジが含まれない場合
                if current_edgeID not in alternative_route_edges:
                    _mark_failed_route_change(
                        logical_vehicle_id=logical_vehicle_id,
                        sumo_vehicle_id=current_vehID,
                        decision_time=current_time,
                        reason="normalcy",
                        change_type="route_change",
                        from_edge_id=current_edgeID,
                        target_shelter_id=alternative_shelter_id,
                        route_before=current_route,
                        failure_reason=(
                            "current edge is not included "
                            "in alternative route"
                        ),
                    )
                    continue

                current_edge_index = (
                    alternative_route_edges.index(
                        current_edgeID
                    )
                )

                route_edges_for_sumo = (
                    alternative_route_edges[
                        current_edge_index:
                    ]
                )

                if not route_edges_for_sumo:
                    _mark_failed_route_change(
                        logical_vehicle_id=logical_vehicle_id,
                        sumo_vehicle_id=current_vehID,
                        decision_time=current_time,
                        reason="normalcy",
                        change_type="route_change",
                        from_edge_id=current_edgeID,
                        target_shelter_id=alternative_shelter_id,
                        route_before=current_route,
                        failure_reason=(
                            "route_edges_for_sumo is empty"
                        ),
                    )
                    continue

                route_change_state_by_logical_vehicle_id[
                    logical_vehicle_id
                ] = RouteChangeState.EXECUTING

                # 既存の共通関数を使って経路変更
                (
                    success,
                    route_before,
                    route_after,
                    error,
                ) = _execute_set_route_edges(
                    vehID=current_vehID,
                    current_edgeID=current_edgeID,
                    route_edges_for_sumo=route_edges_for_sumo,
                )

                if success:
                    # 成功イベントとして記録。AgentとVehicleInfoの目的地も同期する。
                    # この中でカウンタが自動更新される
                    _mark_successful_route_change(
                        logical_vehicle_id=logical_vehicle_id,
                        sumo_vehicle_id_before=current_vehID,
                        sumo_vehicle_id_after=current_vehID,
                        agent=agent_by_current_vehID,
                        decision_time=current_time,
                        execution_time=current_time,
                        reason="normalcy",
                        change_type="route_change",
                        from_edge_id=current_edgeID,
                        target_shelter_id=(
                            alternative_shelter_id
                        ),
                        route_before=route_before,
                        route_after=route_after,
                    )

                else:
                    _mark_failed_route_change(
                        logical_vehicle_id=logical_vehicle_id,
                        sumo_vehicle_id=current_vehID,
                        decision_time=current_time,
                        reason="normalcy",
                        change_type="route_change",
                        from_edge_id=current_edgeID,
                        target_shelter_id=(
                            alternative_shelter_id
                        ),
                        route_before=route_before,
                        failure_reason=(
                            error or "setRoute failed"
                        ),
                    )

                # 同じステップで正常性・同調性を再評価しない
                continue

        if not agent_by_current_vehID.get_encounted_congestion_flg():
            agent_by_current_vehID.set_encounted_congestion_time(current_time)
            agent_by_current_vehID.set_encounted_congestion_flg(True)
        if current_edgeID in ["E1", "E20"]:
            continue
        # ============================================================
        # 2. 正常性バイアス
        # ============================================================
        try:
            (
                from_edgeID,
                shelterID,
                to_edge_list,
                time_gain,
                normalcy_threshold_exceeded,
                route_edges_for_sumo,
            ) = get_route_time_difference_exceeding_threshold(
                current_edgeID=current_edgeID,
                agent_by_target_vehID=agent_by_current_vehID,
                shelter=shelter_for_current_vehID,
                vehInfo_by_target_vehID=vehInfo_by_current_vehID,
                shelter_list=shelter_list,
                custome_edge_list=custome_edge_list,
            )
        except Exception as exc:
            _mark_failed_route_change(
                logical_vehicle_id=logical_vehicle_id,
                sumo_vehicle_id=current_vehID,
                decision_time=current_time,
                reason="normalcy",
                from_edge_id=current_edgeID,
                target_shelter_id="",
                route_before=_safe_get_route(current_vehID),
                failure_reason=f"route evaluation failed: {exc}",
            )
            continue
        
        # 正常性・同調性は現在の実装方針どおり、渋滞中のみ評価する。
        # if not is_vehID_in_congested_edge(
        #     vehID=current_vehID,
        #     threshold_speed=THRESHOLD_SPEED,
        # ):
        #     continue

        # 「情報を取得した」と「閾値を超えた」を分離する。
        agent_by_current_vehID.set_route_congestion_info_obtained_time(current_time)
        agent_by_current_vehID.set_route_congestion_info_obtained_flg(True)

        if normalcy_threshold_exceeded:
            route_change_state_by_logical_vehicle_id[logical_vehicle_id] = RouteChangeState.DECIDED

            if current_edgeID in ROUTE_CHANGE_PROHIBITED_EDGES:
                _mark_failed_route_change(
                    logical_vehicle_id=logical_vehicle_id,
                    sumo_vehicle_id=current_vehID,
                    decision_time=current_time,
                    reason="normalcy",
                    from_edge_id=current_edgeID,
                    target_shelter_id=shelterID or "",
                    route_before=_safe_get_route(current_vehID),
                    failure_reason="route change prohibited on current edge",
                )
                continue

            if not (from_edgeID and shelterID and to_edge_list):
                _mark_failed_route_change(
                    logical_vehicle_id=logical_vehicle_id,
                    sumo_vehicle_id=current_vehID,
                    decision_time=current_time,
                    reason="normalcy",
                    from_edge_id=current_edgeID,
                    target_shelter_id=shelterID or "",
                    route_before=_safe_get_route(current_vehID),
                    failure_reason=f"invalid candidate; time_gain={time_gain}",
                )
                continue

            # 候補経路が現在エッジから始まる場合は、SUMO IDを変更せず経路だけ差し替える。
            if from_edgeID == current_edgeID and route_edges_for_sumo:
                route_change_state_by_logical_vehicle_id[logical_vehicle_id] = RouteChangeState.EXECUTING
                success, route_before, route_after, error = _execute_set_route_edges(
                    vehID=current_vehID,
                    current_edgeID=current_edgeID,
                    route_edges_for_sumo=route_edges_for_sumo,
                )
                if success:
                    _mark_successful_route_change(
                        logical_vehicle_id=logical_vehicle_id,
                        sumo_vehicle_id_before=current_vehID,
                        sumo_vehicle_id_after=current_vehID,
                        agent=agent_by_current_vehID,
                        decision_time=current_time,
                        execution_time=current_time,
                        reason="normalcy",
                        from_edge_id=current_edgeID,
                        target_shelter_id=shelterID,
                        route_before=route_before,
                        route_after=route_after,
                    )
                else:
                    _mark_failed_route_change(
                        logical_vehicle_id=logical_vehicle_id,
                        sumo_vehicle_id=current_vehID,
                        decision_time=current_time,
                        reason="normalcy",
                        from_edge_id=current_edgeID,
                        target_shelter_id=shelterID,
                        route_before=route_before,
                        failure_reason=error or "setRoute failed",
                    )
                continue

            route_change_state_by_logical_vehicle_id[logical_vehicle_id] = RouteChangeState.EXECUTING
            (
                NEW_VEHICLE_COUNT,
                success,
                new_vehID,
                route_before,
                route_after,
                error,
            ) = _execute_generated_vehicle_route_change(
                target_vehID=current_vehID,
                current_new_vehicle_count=NEW_VEHICLE_COUNT,
                vehInfo_by_target_vehID=vehInfo_by_current_vehID,
                agent_by_target_vehID=agent_by_current_vehID,
                from_edgeID=from_edgeID,
                new_shelterID=shelterID,
                to_edgeID=to_edge_list[0],
            )
            if success:
                _mark_successful_route_change(
                    logical_vehicle_id=logical_vehicle_id,
                    sumo_vehicle_id_before=current_vehID,
                    sumo_vehicle_id_after=new_vehID,
                    agent=agent_by_current_vehID,
                    decision_time=current_time,
                    execution_time=current_time,
                    reason="normalcy",
                    change_type="uturn",
                    from_edge_id=current_edgeID,
                    target_shelter_id=shelterID,
                    route_before=route_before,
                    route_after=route_after,
                )
            else:
                _mark_failed_route_change(
                    logical_vehicle_id=logical_vehicle_id,
                    sumo_vehicle_id=current_vehID,
                    decision_time=current_time,
                    reason="normalcy",
                    from_edge_id=current_edgeID,
                    target_shelter_id=shelterID,
                    route_before=route_before,
                    change_type="uturn",
                    failure_reason=error or "vehicle regeneration failed",
                )
            continue

        route_change_state_by_logical_vehicle_id[logical_vehicle_id] = RouteChangeState.EVALUATED_NO_CHANGE

        # ============================================================
        # 3. 同調性バイアス
        #    正常性の閾値を満たさなかった場合のみ。
        #
        # 同じ経路変更イベントは一度だけ評価する。
        # ただし、別の車両による新しい経路変更イベントを観測した場合は、
        # 過去に追従しなかった車両でも再び意思決定できる。
        # ============================================================
        if current_edgeID in ROUTE_CHANGE_PROHIBITED_EDGES:
            continue

        already_observed_event_ids = (
            observed_route_change_event_ids_by_logical_vehicle_id[
                logical_vehicle_id
            ]
        )

        observation = count_rc_around_vehicles(
            agent=agent_by_current_vehID,
            vehInfo=vehInfo_by_current_vehID,
            agent_list=agent_list,
            candidate_action="rc",
            agent_by_vehID_dict=agent_by_vehID_dict,
            custome_edge_list=custome_edge_list,
            current_edgeID=current_edgeID,
            current_time=current_time,
            p_follow=P_FOLLOW,
            logical_vehicle_id_by_sumo_vehicle_id=(
                logical_vehicle_id_by_sumo_vehicle_id
            ),
            latest_successful_route_change_event_by_logical_vehicle_id=(
                latest_successful_route_change_event_by_logical_vehicle_id
            ),
            already_observed_event_ids=already_observed_event_ids,
            event_weights=MAJORITY_EVENT_WEIGHTS,
            observation_window=MAJORITY_OBSERVATION_WINDOW,
            distance_threshold=50.0,
            step_cache=step_cache,
        )

        if not observation.new_events:
            continue

        new_event_ids = observation.new_event_ids

        # 今回追従しなかった場合でも、同じイベントで10秒ごとに
        # 再抽選しないよう、意思決定前に観測済みへ登録する。
        already_observed_event_ids.update(new_event_ids)

        majority_bias_evaluation_count_by_logical_vehicle_id[
            logical_vehicle_id
        ] += 1

        follow_random_value = random.random()
        should_follow = (
            follow_random_value
            < observation.effective_probability
        )

        # if observation.uturn_change_count > 0:
        #     print(
        #         "[MAJORITY_BIAS_EVALUATION]",
        #         f"logical_vehicle_id={logical_vehicle_id}",
        #         f"sumo_vehicle_id={current_vehID}",
        #         f"time={current_time}",
        #         f"rc_change_count={observation.rc_change_count}",
        #         f"uturn_change_count={observation.uturn_change_count}",
        #         f"new_event_ids={new_event_ids}",
        #         f"new_event_count={len(new_event_ids)}",
        #         f"effective_observation_count="
        #         f"{observation.effective_observation_count:.3f}",
        #         f"p_follow={P_FOLLOW:.3f}",
        #         f"effective_probability="
        #         f"{observation.effective_probability:.6f}",
        #         f"random_value={follow_random_value:.6f}",
        #         f"should_follow={should_follow}",
        #     )

        if not should_follow:
            majority_bias_rejection_count_by_logical_vehicle_id[
                logical_vehicle_id
            ] += 1
            continue

        route_change_state_by_logical_vehicle_id[
            logical_vehicle_id
        ] = RouteChangeState.DECIDED

        # 周囲で観測した行動が通常変更かU-turnかにかかわらず、
        # 自車が現在位置で実行可能な変更方法を選ぶ。
        can_follow_by_route_id = current_edgeID in {"E1", "E20"}

        if can_follow_by_route_id:
            routeID = find_alternative_route_calculated_time(
                current_edgeID=current_edgeID,
                vehInfo=vehInfo_by_current_vehID,
                agent=agent_by_current_vehID,
                shelter_list=shelter_list,
                custome_edge_list=custome_edge_list,
            )
            if routeID is None:
                _mark_failed_route_change(
                    logical_vehicle_id=logical_vehicle_id,
                    sumo_vehicle_id=current_vehID,
                    decision_time=current_time,
                    reason="majority",
                    from_edge_id=current_edgeID,
                    target_shelter_id=(
                        agent_by_current_vehID.get_target_shelter()
                    ),
                    route_before=_safe_get_route(current_vehID),
                    failure_reason=(
                        "no valid routeID for majority follow"
                    ),
                    observed_event_ids=new_event_ids,
                    follow_random_value=follow_random_value,
                )
                continue

            try:
                majority_route_edges = tuple(
                    traci.route.getEdges(routeID)
                )
            except traci.exceptions.TraCIException as exc:
                _mark_failed_route_change(
                    logical_vehicle_id=logical_vehicle_id,
                    sumo_vehicle_id=current_vehID,
                    decision_time=current_time,
                    reason="majority",
                    change_type="route_change",
                    from_edge_id=current_edgeID,
                    target_shelter_id="",
                    route_before=_safe_get_route(current_vehID),
                    failure_reason=f"failed to read routeID={routeID}: {exc}",
                    observed_event_ids=new_event_ids,
                    follow_random_value=follow_random_value,
                )
                continue

            majority_target_shelter_id = _get_shelter_id_from_route(
                majority_route_edges
            )
            if majority_target_shelter_id is None:
                _mark_failed_route_change(
                    logical_vehicle_id=logical_vehicle_id,
                    sumo_vehicle_id=current_vehID,
                    decision_time=current_time,
                    reason="majority",
                    change_type="route_change",
                    from_edge_id=current_edgeID,
                    target_shelter_id="",
                    route_before=_safe_get_route(current_vehID),
                    failure_reason=(
                        f"routeID={routeID} ends at an unknown shelter edge"
                    ),
                    observed_event_ids=new_event_ids,
                    follow_random_value=follow_random_value,
                )
                continue

            route_change_state_by_logical_vehicle_id[
                logical_vehicle_id
            ] = RouteChangeState.EXECUTING

            success, route_before, route_after, error = (
                _execute_set_route_id(
                    current_vehID,
                    routeID,
                )
            )
            if success:
                _mark_successful_route_change(
                    logical_vehicle_id=logical_vehicle_id,
                    sumo_vehicle_id_before=current_vehID,
                    sumo_vehicle_id_after=current_vehID,
                    agent=agent_by_current_vehID,
                    decision_time=current_time,
                    execution_time=current_time,
                    reason="majority",
                    change_type="route_change",
                    from_edge_id=current_edgeID,
                    target_shelter_id=majority_target_shelter_id,
                    route_before=route_before,
                    route_after=route_after,
                    observed_event_ids=new_event_ids,
                    follow_random_value=follow_random_value,
                )
            else:
                _mark_failed_route_change(
                    logical_vehicle_id=logical_vehicle_id,
                    sumo_vehicle_id=current_vehID,
                    decision_time=current_time,
                    reason="majority",
                    change_type="route_change",
                    from_edge_id=current_edgeID,
                    target_shelter_id=majority_target_shelter_id,
                    route_before=route_before,
                    failure_reason=error or "setRouteID failed",
                    observed_event_ids=new_event_ids,
                    follow_random_value=follow_random_value,
                )
            continue

        majority_from_edgeID, majority_shelterID, majority_to_edge_list = (
            find_uturn_shortest_route_to_current_shelter_group(
                current_edgeID=current_edgeID,
                vehID=current_vehID,
                vehInfo=vehInfo_by_current_vehID,
                agent=agent_by_current_vehID,
                shelter_list=shelter_list,
            )
        )

        if not (
            majority_from_edgeID
            and majority_shelterID
            and majority_to_edge_list
        ):
            _mark_failed_route_change(
                logical_vehicle_id=logical_vehicle_id,
                sumo_vehicle_id=current_vehID,
                decision_time=current_time,
                reason="majority",
                change_type="uturn",
                from_edge_id=current_edgeID,
                target_shelter_id=majority_shelterID or "",
                route_before=_safe_get_route(current_vehID),
                failure_reason=(
                    "no valid U-turn route for majority follow"
                ),
                observed_event_ids=new_event_ids,
                follow_random_value=follow_random_value,
            )
            continue

        route_change_state_by_logical_vehicle_id[
            logical_vehicle_id
        ] = RouteChangeState.EXECUTING

        (
            NEW_VEHICLE_COUNT,
            success,
            new_vehID,
            route_before,
            route_after,
            error,
        ) = _execute_generated_vehicle_route_change(
            target_vehID=current_vehID,
            current_new_vehicle_count=NEW_VEHICLE_COUNT,
            vehInfo_by_target_vehID=vehInfo_by_current_vehID,
            agent_by_target_vehID=agent_by_current_vehID,
            from_edgeID=majority_from_edgeID,
            new_shelterID=majority_shelterID,
            to_edgeID=majority_to_edge_list[0],
        )

        if success:
            _mark_successful_route_change(
                logical_vehicle_id=logical_vehicle_id,
                sumo_vehicle_id_before=current_vehID,
                sumo_vehicle_id_after=new_vehID,
                agent=agent_by_current_vehID,
                decision_time=current_time,
                execution_time=current_time,
                reason="majority",
                change_type="uturn",
                from_edge_id=current_edgeID,
                target_shelter_id=majority_shelterID,
                route_before=route_before,
                route_after=route_after,
                observed_event_ids=new_event_ids,
                follow_random_value=follow_random_value,
            )
        else:
            _mark_failed_route_change(
                logical_vehicle_id=logical_vehicle_id,
                sumo_vehicle_id=current_vehID,
                decision_time=current_time,
                reason="majority",
                change_type="uturn",
                from_edge_id=current_edgeID,
                target_shelter_id=majority_shelterID,
                route_before=route_before,
                failure_reason=error or "vehicle regeneration failed",
                observed_event_ids=new_event_ids,
                follow_random_value=follow_random_value,
            )

    # SHELTER: 避難地の混雑率を計算する
    calculate_avg_evac_time_by_route(shelter_list=shelter_list)
    if len(shelter_list) >= 2:
        merge_route_info_within_shelters(shelter_list[0], shelter_list[1])
    merge_arrival_vehs_of_shelter(shelter_list=shelter_list)
    for shelter in shelter_list:
        shelter.update_congestion_rate()

    _assert_route_change_consistency()

def get_base_shelter_id(shelterID: str) -> str:
    """
    ShelterA_1 -> ShelterA
    ShelterA_2 -> ShelterA
    ShelterB_1 -> ShelterB
    ShelterA   -> ShelterA
    """
    parts = shelterID.rsplit("_", 1)

    if len(parts) == 2 and parts[1].isdigit():
        return parts[0]

    return shelterID


def has_multiple_shelters(edgeID_by_shelterID: dict[str, str]) -> bool:
    """
    避難地が2種類以上ある場合 True を返す。
    ShelterA_1 と ShelterA_2 は同じ ShelterA とみなす。
    """
    base_shelter_ids = set()

    for shelterID in edgeID_by_shelterID.keys():
        base_shelter_ids.add(get_base_shelter_id(shelterID))

    return len(base_shelter_ids) >= 2

def extract_category(vehID):
    if "ShelterA_1" in vehID:
        return "A1"
    elif "ShelterA_2" in vehID:
        return "A2"
    elif "ShelterB_1" in vehID:
        return "B1"
    elif "ShelterB_2" in vehID:
        return "B2"
    else:
        return "UNKNOWN"

if sys.version_info >= (3, 11):
    import tomllib as _toml_loader
else:
    try:
        import tomli as _toml_loader
    except ImportError as exc:
        raise ImportError(
            "Python 3.10以下では `pip install tomli` が必要です。"
        ) from exc


def load_toml(path: Path) -> dict:
    if not path.is_file():
        raise FileNotFoundError(f"TOML config was not found: {path}")
    with path.open("rb") as file:
        return _toml_loader.load(file)


def _req(cfg: dict, key: str, typ=float):
    if key not in cfg:
        raise KeyError(f"Config missing required key: {key!r}")
    try:
        return typ(cfg[key])
    except (TypeError, ValueError) as exc:
        raise ValueError(
            f"Invalid config value: key={key!r}, value={cfg[key]!r}"
        ) from exc


def _validate_rate(name: str, value: float) -> float:
    value = float(value)
    if not 0.0 <= value <= 1.0:
        raise ValueError(f"{name} must be in [0.0, 1.0]: {value}")
    return value


def parse_runner_args(argv=None):
    parser = argparse.ArgumentParser(
        description=(
            "Run the SUMO evacuation simulator with selectable "
            "V2V-only or V2V+DTN communication."
        )
    )
    parser.add_argument(
        "config",
        type=Path,
        help="Path to the TOML configuration file.",
    )
    parser.add_argument(
        "early_rate",
        type=float,
        help="Rate of early/active drivers in [0.0, 1.0].",
    )
    parser.add_argument(
        "v2v_capable_vehicle_rate",
        type=float,
        help="Rate of communication-capable vehicles in [0.0, 1.0].",
    )
    gui_group = parser.add_mutually_exclusive_group()
    gui_group.add_argument(
        "--nogui",
        dest="nogui",
        action="store_true",
        help="Use the command-line SUMO binary (default).",
    )
    gui_group.add_argument(
        "--gui",
        dest="nogui",
        action="store_false",
        help="Use the SUMO GUI binary.",
    )
    parser.set_defaults(nogui=True)
    parser.add_argument(
        "--communication-mode",
        choices=[mode.value for mode in CommunicationMode],
        default=None,
        help=(
            "Communication mode. When omitted, read communication_mode "
            "from TOML; otherwise default to v2v_only."
        ),
    )
    parser.add_argument(
        "--seed",
        type=int,
        default=None,
        help=(
            "32-bit random seed. When omitted, a seed is generated and "
            "printed so that the run can be reproduced."
        ),
    )
    return parser.parse_args(argv)


if __name__ == "__main__":
    # Example:
    # python3 -m scenarios.JIP.1-1-4.simulation.runner_simulator \
    #   --nogui --communication-mode v2v_only \
    #   scenarios/JIP/configs/config_scenario_1.toml 1.0 1.0
    args = parse_runner_args()
    toml_path = args.config.resolve()
    early_rate = _validate_rate("early_rate", args.early_rate)
    v2v_capable_vehicle_rate = _validate_rate(
        "v2v_capable_vehicle_rate",
        args.v2v_capable_vehicle_rate,
    )

    cfg = load_toml(toml_path)

    configured_mode = args.communication_mode or cfg.get(
        "communication_mode",
        CommunicationMode.V2V_ONLY.value,
    )
    try:
        COMMUNICATION_MODE = CommunicationMode(str(configured_mode))
    except ValueError as exc:
        valid_modes = ", ".join(mode.value for mode in CommunicationMode)
        raise ValueError(
            f"Invalid communication_mode={configured_mode!r}; "
            f"expected one of: {valid_modes}"
        ) from exc

    SUMO_SEED_MAX = (2**31) - 1

    random_seed = (
        secrets.randbelow(SUMO_SEED_MAX + 1)
        if args.seed is None
        else int(args.seed)
    )

    if not 0 <= random_seed <= SUMO_SEED_MAX:
        raise ValueError(
            "seed must be within the range supported by SUMO: "
            f"0 <= seed <= {SUMO_SEED_MAX}, received={random_seed}"
        )
    random.seed(random_seed)
    np.random.seed(random_seed)

    COMMUNICATION_INTERVAL = float(
        cfg.get("communication_interval", COMMUNICATION_INTERVAL)
    )
    DECISION_INTERVAL = float(
        cfg.get("decision_interval", DECISION_INTERVAL)
    )
    END_SIMULATION_TIME = float(
        cfg.get("end_simulation_time", END_SIMULATION_TIME)
    )
    if COMMUNICATION_INTERVAL <= 0:
        raise ValueError("communication_interval must be positive")
    if DECISION_INTERVAL <= 0:
        raise ValueError("decision_interval must be positive")
    if END_SIMULATION_TIME <= 0:
        raise ValueError("end_simulation_time must be positive")

    sumoBinary = checkBinary("sumo" if args.nogui else "sumo-gui")

    print(
        "[RUN_CONFIG]",
        f"config={toml_path}",
        f"seed={random_seed}",
        f"communication_mode={COMMUNICATION_MODE.value}",
        f"communication_interval={COMMUNICATION_INTERVAL}",
        f"decision_interval={DECISION_INTERVAL}",
        f"early_rate={early_rate}",
        f"v2v_capable_vehicle_rate={v2v_capable_vehicle_rate}",
    )

    COMM_RANGE: float = _req(cfg, "comm_range", float)
    NUM_VEHICLES: int = _req(cfg, "num_vehicles", int)
    VEHICLE_INTERVAL: float = _req(cfg, "vehicle_interval", float)
    CHOICE_SHORTEST_ROUTE_RATE: float = _req(cfg, "choice_shortest_route_rate", float)
    ACTIVE_ROUTE_CHANGE_THRESHOLD_CENTER: float = _req(cfg, "active_route_change_threshold_center", float)
    ACTIVE_ROUTE_CHANGE_THRESHOLD_SPREAD: float = _req(cfg, "active_route_change_threshold_spread", float)
    CAUTIOUS_ROUTE_CHANGE_THRESHOLD_CENTER: float = _req(cfg, "cautious_route_change_threshold_center", float)
    CAUTIOUS_ROUTE_CHANGE_THRESHOLD_SPREAD: float = _req(cfg, "cautious_route_change_threshold_spread", float)
    NORMALCY_VALUE_ABOUT_ROUTE_CHANGE_CENTER: float = _req(cfg, "normalcy_value_about_route_change_center", float)
    NORMALCY_VALUE_ABOUT_ROUTE_CHANGE_SPREAD: float = _req(cfg, "normalcy_value_about_route_change_spread", float)
    ACTIVE_WRONG_WAY_DRIVING_THRESHOLD_CENTER: float = _req(cfg, "active_wrong_way_driving_threshold_center", float)
    ACTIVE_WRONG_WAY_DRIVING_THRESHOLD_SPREAD: float = _req(cfg, "active_wrong_way_driving_threshold_spread", float)
    CAUTIOUS_WRONG_WAY_DRIVING_THRESHOLD_CENTER: float = _req(cfg, "cautious_wrong_way_driving_threshold_center", float)
    CAUTIOUS_WRONG_WAY_DRIVING_THRESHOLD_SPREAD: float = _req(cfg, "cautious_wrong_way_driving_threshold_spread", float)
    ACTIVE_VEHICLE_ABANDANTMENT_THRESHOLD_CENTER: float = _req(cfg, "active_vehicle_abandantment_threshold_center", float)
    ACTIVE_VEHICLE_ABANDANTMENT_TTHRESHOLD_SPREAD: float = _req(cfg, "active_vehicle_abandantment_threshold_spread", float)
    CAUTIOUS_VEHICLE_ABANDANTMENT_TTHRESHOLD_CENTER: float = _req(cfg, "cautious_vehicle_abandantment_threshold_center", float)
    CAUTIOUS_VEHICLE_ABANDANTMENT_TTHRESHOLD_SPREAD: float = _req(cfg, "cautious_vehicle_abandantment_threshold_spread", float)

    MIN_MOTIVATION_START: float = _req(cfg, "min_motivation_start", float)
    MIN_MOTIVATION_END: float = _req(cfg, "min_motivation_end", float)
    ACTIVE_NORMALCY_VALUE_ABOUT_TSUNAMI_PRECURSOR_INFO_CENTER: float = _req(cfg, "active_normalcy_value_about_tsunami_precursor_info_center", float)
    ACTIVE_NORMALCY_VALUE_ABOUT_TSUNAMI_PRECURSOR_INFO_SPREAD: float = _req(cfg, "active_normalcy_value_about_tsunami_precursor_info_spread", float)
    CAUTIOUS_NORMALCY_VALUE_ABOUT_TSUNAMI_PRECURSOR_INFO_CENTER: float = _req(cfg, "cautious_normalcy_value_about_tsunami_precursor_info_center", float)
    CAUTIOUS_NORMALCY_VALUE_ABOUT_TSUNAMI_PRECURSOR_INFO_SPREAD: float = _req(cfg, "cautious_normalcy_value_about_tsunami_precursor_info_spread", float)
    ACTIVE_NORMALCY_VALUE_ABOUT_ROUTE_CONGESTION_INFO_CENTER: float = _req(cfg, "active_normalcy_value_about_route_congestion_info_center", float)
    ACTIVE_NORMALCY_VALUE_ABOUT_ROUTE_CONGESTION_INFO_SPREAD: float = _req(cfg, "active_normalcy_value_about_route_congestion_info_spread", float)
    CAUTIOUS_NORMALCY_VALUE_ABOUT_ROUTE_CONGESTION_INFO_CENTER: float = _req(cfg, "cautious_normalcy_value_about_route_congestion_info_center", float)
    CAUTIOUS_NORMALCY_VALUE_ABOUT_ROUTE_CONGESTION_INFO_SPREAD: float = _req(cfg, "cautious_normalcy_value_about_route_congestion_info_spread", float)
    ACTIVE_NORMALCY_VALUE_ABOUT_SHELTER_FULL_INFO_CENTER: float = _req(cfg, "active_normalcy_value_about_shelter_full_info_center", float)
    ACTIVE_NORMALCY_VALUE_ABOUT_SHELTER_FULL_INFO_SPREAD: float = _req(cfg, "active_normalcy_value_about_shelter_full_info_spread", float)
    CAUTIOUS_NORMALCY_VALUE_ABOUT_SHELTER_FULL_INFO_CENTER: float = _req(cfg, "cautious_normalcy_value_about_shelter_full_info_center", float)
    CAUTIOUS_NORMALCY_VALUE_ABOUT_SHELTER_FULL_INFO_SPREAD: float = _req(cfg, "cautious_normalcy_value_about_shelter_full_info_spread", float)
    ACTIVE_MAJORITY_INCREASE_VALUE_CENTER: float = _req(cfg, "active_majority_increase_value_center", float)
    ACTIVE_MAJORITY_INCREASE_VALUE_SPREAD: float = _req(cfg, "active_majority_increase_value_spread", float)
    CAUTIOUS_MAJORITY_INCREASE_VALUE_CENTER: float = _req(cfg, "cautious_majority_increase_value_center", float)
    CAUTIOUS_MAJORITY_INCREASE_VALUE_SPREAD: float = _req(cfg, "cautious_majority_increase_value_spread", float)
    ACTIVE_MAJORITY_DECREASE_VALUE_CENTER: float = _req(cfg, "active_majority_decrease_value_center", float)
    ACTIVE_MAJORITY_DECREASE_VALUE_SPREAD: float = _req(cfg, "active_majority_decrease_value_spread", float)
    CAUTIOUS_MAJORITY_DECREASE_VALUE_CENTER: float = _req(cfg, "cautious_majority_decrease_value_center", float)
    CAUTIOUS_MAJORITY_DECREASE_VALUE_SPREAD: float = _req(cfg, "cautious_majority_decrease_value_spread", float)
    ACTIVE_SHELTER_OCCUPANCY_RATE_THRESHOLD_START: float = _req(cfg, "active_shelter_occupancy_rate_threshold_start", float)
    ACTIVE_SHELTER_OCCUPANCY_RATE_THRESHOLD_END: float = _req(cfg, "active_shelter_occupancy_rate_threshold_end", float)
    CAUTIOUS_SHELTER_OCCUPANCY_RATE_THRESHOLD_START: float = _req(cfg, "cautious_shelter_occupancy_rate_threshold_start", float)
    CAUTIOUS_SHELTER_OCCUPANCY_RATE_THRESHOLD_END: float = _req(cfg, "cautious_shelter_occupancy_rate_threshold_end", float)
    SHELTER_CAPACITY_THRESHOLD: float = _req(cfg, "shelter_capacity_threshold", float)
    TSUNAMI_SIGN_START_TIME: float = _req(cfg, "tsunami_sign_start_time", float)
    TSUNAMI_SIGN_END_TIME: float = _req(cfg, "tsunami_sign_end_time", float)
    DRIVER_VISIBILITY_DISTANCE: float = _req(cfg, "driver_visibility_distance", float)
    ALPHA: float = _req(cfg, "alpha", float)
    P_FOLLOW: float = _validate_rate(
        "p_follow",
        _req(cfg, "p_follow", float),
    )

    if COMM_RANGE < 0:
        raise ValueError(f"comm_range must be non-negative: {COMM_RANGE}")
    if NUM_VEHICLES < 0:
        raise ValueError(f"num_vehicles must be non-negative: {NUM_VEHICLES}")
    CHOICE_SHORTEST_ROUTE_RATE = _validate_rate(
        "choice_shortest_route_rate",
        CHOICE_SHORTEST_ROUTE_RATE,
    )
    if VEHICLE_INTERVAL <= 0:
        raise ValueError(
            f"vehicle_interval must be positive: {VEHICLE_INTERVAL}"
        )

    traci.start(
                [sumoBinary,
                    "-c", str(SUMO_CFG),
                    "--tripinfo-output", "tripinfo.xml",
                    # "--tls.all-off", "true" ,
                    "--time-to-teleport", "1000",

                    "--seed", str(random_seed),
                ],
                # traceFile="traci_log.txt",
                # traceGetters=False,

                )

    # 避難地の情報をもとに、Shelter一覧を生成
    shelter_capacity_by_ID:dict = {"ShelterA_1": 200, "ShelterA_2": 200}
    edgeID_by_shelterID:dict = {"ShelterA_1": 'E16', "ShelterA_2": 'E13'}
    for shelterID, near_edgeID in edgeID_by_shelterID.items():
        shelter_list:list[Shelter] = init_shelter(
                                                    shelterID=shelterID, 
                                                    shelter_capacity_by_ID=shelter_capacity_by_ID, 
                                                    near_edgeID=near_edgeID, 
                                                    shelter_list=shelter_list
                                                    )
        
    custome_edge_list:list = init_custome_edge()
    vehicle_start_edges:list = get_vehicle_start_edges(custome_edge_list=custome_edge_list)
    vehicle_end_edges:list = get_vehicle_end_edges(custome_edge_list=custome_edge_list)
    # 車両の開始エッジと終了エッジの組み合わせを辞書にする
    get_vehicle_end_list_by_start_edge_dict(vehicle_start_edges=vehicle_start_edges, vehicle_end_edges=vehicle_end_edges)
    # 全経路で総当たりをし、通行可能経路を取得しておく。
    import_connected_edges_from_json(file_path=str(DATA_DIR / "all_edgeIDs.json"))
    nearest_end_edgeID_by_start_edgeID_dict:dict = import_start_end_edgeIDs_from_json(file_path=str(DATA_DIR / "start_end_edgeIDs_ishinomaki_one_two_one.json"))
    mapping = nearest_end_edgeID_by_start_edgeID_dict

    # 開始エッジごとの目的地確率
    # mapping["E0"] = ["E16", "E13"] の順番に対応
    probabilities_by_start_edge = {
        "E0": [1-CHOICE_SHORTEST_ROUTE_RATE, CHOICE_SHORTEST_ROUTE_RATE],
    }

    # 開始エッジごとの生成台数
    vehicle_count_by_start_edge = {
        "E0": NUM_VEHICLES,
    }

    vehID_list = []

    start_interval = float(
        cfg.get("vehicle_interval_start", VEHICLE_INTERVAL)
    )
    end_interval = float(
        cfg.get("vehicle_interval_end", VEHICLE_INTERVAL)
    )
    if start_interval <= 0 or end_interval <= 0:
        raise ValueError(
            "vehicle generation intervals must be positive: "
            f"start={start_interval}, end={end_interval}"
        )

    for start_edgeID, end_edgeID_list in mapping.items():
        assigned_vehicle_num = vehicle_count_by_start_edge.get(start_edgeID, 0)

        if assigned_vehicle_num <= 0:
            print(f"No vehicles generated from start_edgeID={start_edgeID}")
            continue
        if len(end_edgeID_list) == 0:
            raise ValueError(f"end_edgeID_list is empty for start_edgeID={start_edgeID}")
        probabilities = probabilities_by_start_edge.get(
            start_edgeID,
            [1.0 / len(end_edgeID_list)] * len(end_edgeID_list)
        )

        if len(probabilities) != len(end_edgeID_list):
            raise ValueError(
                f"Probability length mismatch for {start_edgeID}: "
                f"len(probabilities)={len(probabilities)} vs "
                f"len(end_edgeID_list)={len(end_edgeID_list)}. "
                f"end_edgeID_list={end_edgeID_list}, probabilities={probabilities}"
            )

        prob_sum = sum(probabilities)
        if prob_sum <= 0:
            raise ValueError(
                f"Invalid probabilities for start_edgeID={start_edgeID}: "
                f"{probabilities}"
            )

        probabilities = [p / prob_sum for p in probabilities]

        vehicle_intervals = np.linspace(
            start_interval,
            end_interval,
            num=int(assigned_vehicle_num)
        )

        DEPART_TIME = 0.0

        for vehicle_index in range(int(assigned_vehicle_num)):
            selected_end_edgeID = choose_edge_by_probability(
                edgeID_list=end_edgeID_list,
                probabilities=probabilities
            )

            target_shelterID = find_shelterID_by_edgeID_by_shelterID(
                edgeID=selected_end_edgeID,
                edgeID_by_shelterID=edgeID_by_shelterID
            )

            if target_shelterID is None:
                raise ValueError(
                    f"No shelter found for selected_end_edgeID={selected_end_edgeID}. "
                    f"edgeID_by_shelterID={edgeID_by_shelterID}"
                )
            VEHICLE_NUM, ROUTE_NUM, vehID_list_by_shelter, DEPART_TIME = \
                generate_simple_init_vehID(
                    from_edgeID=start_edgeID,
                    to_edgeID=selected_end_edgeID,
                    shelterID=target_shelterID,
                    generate_interval=float(vehicle_intervals[vehicle_index]),
                    generate_route_count=ROUTE_NUM,
                    generate_veh_count=VEHICLE_NUM,
                    depart_time=DEPART_TIME
                )

            vehID_list.extend(vehID_list_by_shelter)

    # カテゴリのカウント
    counter = Counter(extract_category(v) for v in vehID_list)

    # 合計と割合を表示
    total = sum(counter.values())
    print("出現数:", dict(counter))
    print("割合:")
    print(f"  Total: {total}")
    for cat in ["A1", "A2", "B1", "B2"]:
        count = counter.get(cat, 0)
        ratio = count / total if total else 0.0
        print(f"  {cat}: {count} ({ratio:.2%})")
    # 車両情報の初期化
    # ここで調整がいる
    vehInfo_list:list[VehicleInfo] = init_vehicleInfo_list_base(vehIDs=vehID_list, shelter_list=shelter_list, v2v_capable_vehicle_rate=v2v_capable_vehicle_rate) 

    # Agentの初期化
    # 乗り捨てに関する初期化
    agent_list: list[Agent] = init_agent_list(
        vehIDs=vehID_list,
        edgeID_by_shelterID=edgeID_by_shelterID,
        ATTR_RATE=early_rate,

        ACTIVE_ROUTE_CHANGE_THRESHOLD_CENTER=ACTIVE_ROUTE_CHANGE_THRESHOLD_CENTER,
        ACTIVE_ROUTE_CHANGE_THRESHOLD_SPREAD=ACTIVE_ROUTE_CHANGE_THRESHOLD_SPREAD,
        CAUTIOUS_ROUTE_CHANGE_THRESHOLD_CENTER=CAUTIOUS_ROUTE_CHANGE_THRESHOLD_CENTER,
        CAUTIOUS_ROUTE_CHANGE_THRESHOLD_SPREAD=CAUTIOUS_ROUTE_CHANGE_THRESHOLD_SPREAD,

        NORMALCY_VALUE_ABOUT_ROUTE_CHANGE_CENTER=NORMALCY_VALUE_ABOUT_ROUTE_CHANGE_CENTER,
        NORMALCY_VALUE_ABOUT_ROUTE_CHANGE_SPREAD=NORMALCY_VALUE_ABOUT_ROUTE_CHANGE_SPREAD,

        ACTIVE_WRONG_WAY_DRIVING_THRESHOLD_CENTER=ACTIVE_WRONG_WAY_DRIVING_THRESHOLD_CENTER,
        ACTIVE_WRONG_WAY_DRIVING_THRESHOLD_SPREAD=ACTIVE_WRONG_WAY_DRIVING_THRESHOLD_SPREAD,
        CAUTIOUS_WRONG_WAY_DRIVING_THRESHOLD_CENTER=CAUTIOUS_WRONG_WAY_DRIVING_THRESHOLD_CENTER,
        CAUTIOUS_WRONG_WAY_DRIVING_THRESHOLD_SPREAD=CAUTIOUS_WRONG_WAY_DRIVING_THRESHOLD_SPREAD,

        ACTIVE_VEHICLE_ABANDANTMENT_THRESHOLD_CENTER=ACTIVE_VEHICLE_ABANDANTMENT_THRESHOLD_CENTER,
        ACTIVE_VEHICLE_ABANDANTMENT_TTHRESHOLD_SPREAD=ACTIVE_VEHICLE_ABANDANTMENT_TTHRESHOLD_SPREAD,
        CAUTIOUS_VEHICLE_ABANDANTMENT_TTHRESHOLD_CENTER=CAUTIOUS_VEHICLE_ABANDANTMENT_TTHRESHOLD_CENTER,
        CAUTIOUS_VEHICLE_ABANDANTMENT_TTHRESHOLD_SPREAD=CAUTIOUS_VEHICLE_ABANDANTMENT_TTHRESHOLD_SPREAD,

        MIN_MOTIVATION_START=MIN_MOTIVATION_START,
        MIN_MOTIVATION_END=MIN_MOTIVATION_END,

        ACTIVE_NORMALCY_VALUE_ABOUT_TSUNAMI_PRECURSOR_INFO_CENTER=ACTIVE_NORMALCY_VALUE_ABOUT_TSUNAMI_PRECURSOR_INFO_CENTER,
        ACTIVE_NORMALCY_VALUE_ABOUT_TSUNAMI_PRECURSOR_INFO_SPREAD=ACTIVE_NORMALCY_VALUE_ABOUT_TSUNAMI_PRECURSOR_INFO_SPREAD,
        CAUTIOUS_NORMALCY_VALUE_ABOUT_TSUNAMI_PRECURSOR_INFO_CENTER=CAUTIOUS_NORMALCY_VALUE_ABOUT_TSUNAMI_PRECURSOR_INFO_CENTER,
        CAUTIOUS_NORMALCY_VALUE_ABOUT_TSUNAMI_PRECURSOR_INFO_SPREAD=CAUTIOUS_NORMALCY_VALUE_ABOUT_TSUNAMI_PRECURSOR_INFO_SPREAD,

        ACTIVE_NORMALCY_VALUE_ABOUT_ROUTE_CONGESTION_INFO_CENTER=ACTIVE_NORMALCY_VALUE_ABOUT_ROUTE_CONGESTION_INFO_CENTER,
        ACTIVE_NORMALCY_VALUE_ABOUT_ROUTE_CONGESTION_INFO_SPREAD=ACTIVE_NORMALCY_VALUE_ABOUT_ROUTE_CONGESTION_INFO_SPREAD,
        CAUTIOUS_NORMALCY_VALUE_ABOUT_ROUTE_CONGESTION_INFO_CENTER=CAUTIOUS_NORMALCY_VALUE_ABOUT_ROUTE_CONGESTION_INFO_CENTER,
        CAUTIOUS_NORMALCY_VALUE_ABOUT_ROUTE_CONGESTION_INFO_SPREAD=CAUTIOUS_NORMALCY_VALUE_ABOUT_ROUTE_CONGESTION_INFO_SPREAD,

        ACTIVE_NORMALCY_VALUE_ABOUT_SHELTER_FULL_INFO_CENTER=ACTIVE_NORMALCY_VALUE_ABOUT_SHELTER_FULL_INFO_CENTER,
        ACTIVE_NORMALCY_VALUE_ABOUT_SHELTER_FULL_INFO_SPREAD=ACTIVE_NORMALCY_VALUE_ABOUT_SHELTER_FULL_INFO_SPREAD,
        CAUTIOUS_NORMALCY_VALUE_ABOUT_SHELTER_FULL_INFO_CENTER=CAUTIOUS_NORMALCY_VALUE_ABOUT_SHELTER_FULL_INFO_CENTER,
        CAUTIOUS_NORMALCY_VALUE_ABOUT_SHELTER_FULL_INFO_SPREAD=CAUTIOUS_NORMALCY_VALUE_ABOUT_SHELTER_FULL_INFO_SPREAD,

        ACTIVE_MAJORITY_INCREASE_VALUE_CENTER=ACTIVE_MAJORITY_INCREASE_VALUE_CENTER,
        ACTIVE_MAJORITY_INCREASE_VALUE_SPREAD=ACTIVE_MAJORITY_INCREASE_VALUE_SPREAD,
        CAUTIOUS_MAJORITY_INCREASE_VALUE_CENTER=CAUTIOUS_MAJORITY_INCREASE_VALUE_CENTER,
        CAUTIOUS_MAJORITY_INCREASE_VALUE_SPREAD=CAUTIOUS_MAJORITY_INCREASE_VALUE_SPREAD,

        ACTIVE_MAJORITY_DECREASE_VALUE_CENTER=ACTIVE_MAJORITY_DECREASE_VALUE_CENTER,
        ACTIVE_MAJORITY_DECREASE_VALUE_SPREAD=ACTIVE_MAJORITY_DECREASE_VALUE_SPREAD,
        CAUTIOUS_MAJORITY_DECREASE_VALUE_CENTER=CAUTIOUS_MAJORITY_DECREASE_VALUE_CENTER,
        CAUTIOUS_MAJORITY_DECREASE_VALUE_SPREAD=CAUTIOUS_MAJORITY_DECREASE_VALUE_SPREAD,

        ACTIVE_SHELTER_OCCUPANCY_RATE_THRESHOLD_START=ACTIVE_SHELTER_OCCUPANCY_RATE_THRESHOLD_START,
        ACTIVE_SHELTER_OCCUPANCY_RATE_THRESHOLD_END=ACTIVE_SHELTER_OCCUPANCY_RATE_THRESHOLD_END,
        CAUTIOUS_SHELTER_OCCUPANCY_RATE_THRESHOLD_START=CAUTIOUS_SHELTER_OCCUPANCY_RATE_THRESHOLD_START,
        CAUTIOUS_SHELTER_OCCUPANCY_RATE_THRESHOLD_END=CAUTIOUS_SHELTER_OCCUPANCY_RATE_THRESHOLD_END,
    )

    vehInfo_by_vehID_dict = {vehInfo.get_vehID(): vehInfo for vehInfo in vehInfo_list}
    agent_by_vehID_dict = {agent.get_vehID(): agent for agent in agent_list}
    run()

    mean_elapsed_time = (
        float(np.mean(elapsed_time_list))
        if elapsed_time_list
        else 0.0
    )
    print(f"mean elapsed_time: {mean_elapsed_time:.2f}")

    arrived_vehicle_count = len(arrival_time_by_vehID_dict)
    if arrived_vehicle_count == NUM_VEHICLES:
        print("OK all vehicles arrived")
    else:
        print(
            "NG not all vehicles arrived "
            f"{arrived_vehicle_count} / {NUM_VEHICLES}"
        )

    check_vehicle_abandonment_count = 0
    total_congestion_duration = 0.0
    for agent in agent_list:
        encountered_time = agent.get_encounted_congestion_time()
        agent_arrival_time = agent.get_arrival_time()
        if encountered_time is not None and agent_arrival_time is not None:
            total_congestion_duration += max(
                0.0,
                agent_arrival_time - encountered_time,
            )

        if agent.get_vehicle_abandoned_flg():
            check_vehicle_abandonment_count += 1

    avg_congestion_duration = (
        total_congestion_duration / len(agent_list)
        if agent_list
        else 0.0
    )
    print(f"平均渋滞継続時間: {avg_congestion_duration:.2f}秒")

    if check_vehicle_abandonment_count == PEDESTRIAN_COUNT:
        print("OK all vehicle abandonments were detected")
    else:
        print(
            "NG not all vehicle abandonments were detected "
            f"{check_vehicle_abandonment_count} / {PEDESTRIAN_COUNT}"
        )

    for shelter in shelter_list:
        shelter_id_getter = getattr(shelter, "get_shelterID", None)
        shelter_id = (
            shelter_id_getter()
            if callable(shelter_id_getter)
            else repr(shelter)
        )
        print(
            f"shelter_arrival_count[{shelter_id}]:"
            f"{len(shelter.get_arrival_vehID_list())}"
        )

    vehicle_abandonment_rate = (
        VEHICLE_ABANDONMENT_SUCCESS_COUNT / VEHICLE_NUM * 100.0
        if VEHICLE_NUM > 0
        else 0.0
    )

    print("===== Simulation Result Summary =====")
    print(f"random_seed:{random_seed}")
    print(f"communication_mode:{COMMUNICATION_MODE.value}")
    print(f"avg_congestion_duration:{avg_congestion_duration:.2f}")
    print(f"arrival_time_by_vehID_dict:{arrival_time_by_vehID_dict}")
    print(
        "vehicle_abandant_time_by_pedestrianID_dict:"
        f"{vehicle_abandant_time_by_pedestrianID_dict}"
    )
    print(
        "walking_distance_by_pedestrianID_dict:"
        f"{walking_distance_by_pedestrianID_dict}"
    )
    print(f"route_change_time_by_vehID_dict:{route_change_time_by_vehID_dict}")
    print(f"pedestrian_count:{PEDESTRIAN_COUNT}")
    print(f"route_changed_vehicle_count:{ROUTE_CHANGED_VEHICLE_COUNT}")
    print(f"route_change_event_count:{ROUTE_CHANGE_EVENT_COUNT}")
    print(f"rate_vehicle_abandonment:{vehicle_abandonment_rate:.2f}%")
    print(f"wrong_way_driving_count:{WRONG_WAY_SUCCESS_COUNT}")
    print(
        "vehicle_abandonment_count:"
        f"{VEHICLE_ABANDONMENT_SUCCESS_COUNT}"
    )
    print(
        "normalcy_bias_route_change_count:"
        f"{NORMALCY_BIAS_ROUTE_CHANGE_COUNT}"
    )
    print(
        "majority_bias_route_change_count:"
        f"{MAJORITY_BIAS_ROUTE_CHANGE_COUNT}"
    )
    print(f"lane_changed_vehicle_count:{LANE_CHANGED_VEHICLE_COUNT}")
    print(
        "info_obtained_lanechange_count:"
        f"{OBTAIN_INFO_LANE_CHANGE_COUNT}"
    )
    print(
        "elapsed_time_lanechange_count:"
        f"{ELAPSED_TIME_LANE_CHANGE_COUNT}"
    )
    print(
        "majority_bias_lanechange_count:"
        f"{POSITIVE_MAJORITY_BIAS_COUNT}"
    )
