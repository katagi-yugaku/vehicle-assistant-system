from __future__ import annotations

from dataclasses import dataclass
from typing import Mapping, Protocol

import traci

from evacsim.agents.Agent import Agent
from evacsim.agents.VehicleInfo import VehicleInfo
from evacsim.maps.edge_utils import get_opposite_edgeID_by_edgeID
# get_around_vehIDs は、このコードを追加する既存 neighbors.py 内の関数を使用する。
from evacsim.sim.traci_cache import get_vehicle_position_cached
from evacsim.utils.lookup import find_agent_by_vehID
from evacsim.sim.routing_distance import distance_each_vehIDs
from evacsim.maps.edge_utils import (
    get_custome_edge_by_edgeID,
    remove_junction_from_edgeID,
)
from evacsim.utils.lookup import (
    find_shelter_by_edgeID_connect_target_shelter,
    find_shelterID_by_edgeID_by_shelterID,
    find_agent_by_vehID
)
from evacsim.sim.traci_cache import(
    get_edge_vehicle_ids_cached,
    get_vehicle_position_cached,
)
# distance_each_vehIDs は既存 neighbors.py の import / 定義を利用する。
# このファイルを単独モジュールとして使う場合に備えて、未定義時の代替も用意する。
try:
    distance_each_vehIDs
except NameError:
    def distance_each_vehIDs(
        position_a: tuple[float, float],
        position_b: tuple[float, float],
    ) -> float:
        dx = position_a[0] - position_b[0]
        dy = position_a[1] - position_b[1]
        return (dx * dx + dy * dy) ** 0.5

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

class RouteChangeEventLike(Protocol):
    """runner側の RouteChangeEvent が満たす最小インターフェース。"""

    event_id: str
    logical_vehicle_id: str
    sumo_vehicle_id_after: str | None
    execution_time: float | None
    success: bool
    change_type: str


@dataclass(frozen=True)
class NearbyRouteChangeEvent:
    """対象車両が今回新しく観測した周辺経路変更イベント。"""

    event_id: str
    logical_vehicle_id: str
    sumo_vehicle_id: str
    execution_time: float
    change_type: str
    distance: float
    weight: float


@dataclass(frozen=True)
class RouteChangeObservationResult:
    """
    周辺経路変更の観測結果。

    rc_change_count / uturn_change_count:
        観測有効時間内かつ距離条件内にいる経路変更済み車両数。

    new_events:
        対象車両がまだ一度も評価していない経路変更イベント。

    effective_observation_count:
        new_events の重み付き件数。

    effective_probability:
        1 - (1 - p_follow) ** effective_observation_count
    """

    rc_change_count: int
    uturn_change_count: int
    new_events: tuple[NearbyRouteChangeEvent, ...]
    effective_observation_count: float
    effective_probability: float

    @property
    def new_event_ids(self) -> tuple[str, ...]:
        return tuple(event.event_id for event in self.new_events)

    def __iter__(self):
        """
        既存の
            rc_count, uturn_count = count_rc_around_vehicles(...)
        という呼び出しとの最低限の互換性を残す。
        """
        yield self.rc_change_count
        yield self.uturn_change_count


def count_rc_around_vehicles(
    agent: Agent,
    vehInfo: VehicleInfo,
    agent_list: list[Agent],
    candidate_action: str,
    agent_by_vehID_dict: dict[str, Agent],
    custome_edge_list: list,
    current_edgeID: str,
    *,
    current_time: float,
    p_follow: float,
    logical_vehicle_id_by_sumo_vehicle_id: Mapping[str, str],
    latest_successful_route_change_event_by_logical_vehicle_id: Mapping[
        str,
        RouteChangeEventLike,
    ],
    already_observed_event_ids: set[str],
    event_weights: Mapping[str, float] | None = None,
    observation_window: float = 30.0,
    distance_threshold: float = 10.0,
    step_cache=None,
) -> RouteChangeObservationResult:
    """
    周囲にいる最近の経路変更済み車両を調べ、新規イベントに基づく
    同調性バイアスの実効追従確率を返す。

    重要な仕様:
    - 同じ車両を複数ステップ見続けても、同じ event_id では再抽選しない。
    - 別の車両が新しく経路変更した場合は、再び同調性を評価できる。
    - 同じ車両が将来もう一度正当に経路変更した場合も、
      event_id が異なれば新規イベントとして扱える。
    - 通常経路変更と U-turn は event_weights で異なる影響度を設定できる。
    - 確率は新規イベントだけから計算する。

    Notes:
        candidate_action と vehInfo は既存呼び出しとの互換性のために残している。
        この関数は already_observed_event_ids を変更しない。
        呼び出し側は意思決定を行う直前に new_event_ids を観測済みへ追加する。
    """
    del candidate_action, vehInfo

    if not 0.0 <= p_follow <= 1.0:
        raise ValueError(f"p_follow must be in [0, 1]: {p_follow}")
    if observation_window < 0.0:
        raise ValueError(
            f"observation_window must be non-negative: {observation_window}"
        )
    if distance_threshold < 0.0:
        raise ValueError(
            f"distance_threshold must be non-negative: {distance_threshold}"
        )

    weights = {
        "route_change": 1.0,
        "uturn": 1.0,
    }
    if event_weights is not None:
        weights.update(event_weights)

    for event_type, weight in weights.items():
        if weight < 0.0:
            raise ValueError(
                f"event weight must be non-negative: "
                f"type={event_type}, weight={weight}"
            )

    target_vehID = agent.get_vehID()
    target_logical_vehicle_id = (
        logical_vehicle_id_by_sumo_vehicle_id.get(
            target_vehID,
            target_vehID,
        )
    )

    try:
        target_position = get_vehicle_position_cached(
            target_vehID,
            step_cache=step_cache,
        )
    except Exception as exc:
        print(
            "[MAJORITY_OBSERVATION_WARNING]",
            f"failed_to_get_target_position vehID={target_vehID}",
            f"error={exc}",
        )
        return RouteChangeObservationResult(0, 0, tuple(), 0.0, 0.0)

    opposite_moving_vehIDs: list[str] = []
    try:
        opposite_edgeID = get_opposite_edgeID_by_edgeID(current_edgeID)
        if opposite_edgeID:
            opposite_moving_vehIDs = list(
                traci.edge.getLastStepVehicleIDs(opposite_edgeID)
            )
    except Exception as exc:
        print(
            "[MAJORITY_OBSERVATION_WARNING]",
            f"failed_to_get_opposite_vehicles vehID={target_vehID}",
            f"current_edgeID={current_edgeID}",
            f"error={exc}",
        )

    try:
        around_vehIDs = get_around_vehIDs(
            target_vehID=target_vehID,
            custome_edge_list=custome_edge_list,
            step_cache=step_cache,
        )
    except Exception as exc:
        print(
            "[MAJORITY_OBSERVATION_WARNING]",
            f"failed_to_get_around_vehicles vehID={target_vehID}",
            f"error={exc}",
        )
        around_vehIDs = []

    candidate_vehIDs = set(opposite_moving_vehIDs) | set(around_vehIDs)

    # 同じ論理車両の旧SUMO IDと新SUMO IDが同時に候補へ入っても、
    # 同じ event_id は一度だけ扱う。
    visible_events_by_event_id: dict[str, NearbyRouteChangeEvent] = {}

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
            other_position = get_vehicle_position_cached(
                other_vehID,
                step_cache=step_cache,
            )
        except Exception:
            continue

        distance = distance_each_vehIDs(
            target_position,
            other_position,
        )
        if distance > distance_threshold:
            continue

        other_logical_vehicle_id = (
            logical_vehicle_id_by_sumo_vehicle_id.get(
                other_vehID,
                other_vehID,
            )
        )

        # 再生成前後のSUMO IDが異なっていても、
        # 自分自身の論理車両イベントは同調対象から除外する。
        if other_logical_vehicle_id == target_logical_vehicle_id:
            continue

        event = latest_successful_route_change_event_by_logical_vehicle_id.get(
            other_logical_vehicle_id
        )
        if event is None or not event.success:
            continue
        if event.execution_time is None:
            continue

        event_age = current_time - event.execution_time
        if event_age < 0.0 or event_age > observation_window:
            continue

        # イベント記録を正本にする。
        # Agentのフラグが引き継がれていなくても分類を失わない。
        change_type = event.change_type
        weight = weights.get(change_type, 1.0)

        visible_events_by_event_id[event.event_id] = NearbyRouteChangeEvent(
            event_id=event.event_id,
            logical_vehicle_id=event.logical_vehicle_id,
            sumo_vehicle_id=other_vehID,
            execution_time=event.execution_time,
            change_type=change_type,
            distance=distance,
            weight=weight,
        )

    visible_events = tuple(visible_events_by_event_id.values())

    rc_change_count = sum(
        event.change_type == "route_change"
        for event in visible_events
    )
    uturn_change_count = sum(
        event.change_type == "uturn"
        for event in visible_events
    )

    new_events = tuple(
        sorted(
            (
                event
                for event in visible_events
                if event.event_id not in already_observed_event_ids
            ),
            key=lambda event: (
                event.execution_time,
                event.event_id,
            ),
        )
    )

    effective_observation_count = sum(
        event.weight
        for event in new_events
    )

    if effective_observation_count <= 0.0:
        effective_probability = 0.0
    else:
        effective_probability = (
            1.0
            - (1.0 - p_follow) ** effective_observation_count
        )

    return RouteChangeObservationResult(
        rc_change_count=rc_change_count,
        uturn_change_count=uturn_change_count,
        new_events=new_events,
        effective_observation_count=effective_observation_count,
        effective_probability=effective_probability,
    )


# # =========================
# # Neighbor vehicle helpers
# # =========================
# #
# # 注意:
# # このモジュールは TraCI に依存する。
# # 対象車両の周辺 edge / 周辺車両 ID の取得を扱う。

# from __future__ import annotations

# from typing import TYPE_CHECKING

# import numpy as np
# import traci

# if TYPE_CHECKING:
#     from evacsim.agents.Agent import Agent
#     from evacsim.agents.VehicleInfo import VehicleInfo

# from evacsim.sim.sumo_env import ensure_sumo_tools_on_path

# ensure_sumo_tools_on_path()

# from evacsim.maps.edge_utils import (
#     get_custome_edge_by_edgeID,
#     remove_junction_from_edgeID,
# )
# from evacsim.utils.lookup import (
#     find_shelter_by_edgeID_connect_target_shelter,
#     find_shelterID_by_edgeID_by_shelterID,
#     find_agent_by_vehID
# )

# from evacsim.maps.edge_utils import (
#     is_pre_edgeID_near_shelter,
#     get_vehicle_start_edges,
#     get_vehicle_end_edges,
#     get_opposite_edgeID_by_edgeID,
# )
# from evacsim.sim.traci_cache import(
#     get_edge_vehicle_ids_cached,
#     get_vehicle_position_cached,
# )

# from evacsim.sim.routing_distance import distance_each_vehIDs


# def get_around_edgeIDs(
#     target_vehID: str,
#     custome_edge_list: list,
# ):
#     """
#     車両IDを元に、周辺 edgeID を取得する。

#     既存 utilities.py の get_around_edgeIDs の挙動を維持する。
#     """
#     # 車両がいる edgeID を取得
#     current_edgeID = traci.vehicle.getRoadID(target_vehID)

#     # 車両がいる edgeID をもとに、周辺 edgeID を取得する
#     current_edge = get_custome_edge_by_edgeID(
#         current_edgeID,
#         custome_edge_list,
#     )

#     around_edgeIDs_with_junction = remove_junction_from_edgeID(
#         current_edge.around_edgeIDs()
#     )

#     return around_edgeIDs_with_junction


# def get_around_vehIDs(
#     target_vehID: str,
#     custome_edge_list: list,
#     step_cache=None,
# ):
#     """
#     車両IDを元に周辺 edgeID を取得し、さらに周辺車両IDを取得する。

#     get_edge_vehicle_ids_cached を使うため、
#     step_cache が渡された場合は TraCI 呼び出しをキャッシュする。
#     """
#     around_edgeIDs_for_target_vehID = get_around_edgeIDs(
#         target_vehID,
#         custome_edge_list,
#     )

#     around_vehIDs = []

#     for around_edgeID in around_edgeIDs_for_target_vehID:
#         tmp_vehIDs = get_edge_vehicle_ids_cached(
#             around_edgeID,
#             step_cache=step_cache,
#         )
#         around_vehIDs.extend(tmp_vehIDs)

#     return around_vehIDs

# def count_rc_around_vehicles(
#     agent: Agent,
#     vehInfo: VehicleInfo,
#     agent_list: list[Agent],
#     candidate_action: str,
#     agent_by_vehID_dict: dict,
#     custome_edge_list: list,
#     current_edgeID: str,
#     distance_threshold: float = 10.0,
# ) -> tuple[int, int]:
#     """
#     周囲にいる経路変更済み車両数と U-turn 経路変更済み車両数を数える。

#     Returns:
#         (rc_change_count, uturn_change_count)

#         rc_change_count:
#             通常の経路変更済み車両数。

#         uturn_change_count:
#             U-turn 系の経路変更済み車両数。

#     Notes:
#         candidate_action と vehInfo は既存呼び出しとの互換性のために残している。
#     """
#     step_cache = None
#     target_vehID = agent.get_vehID()
#     rc_change_count = 0
#     uturn_change_count = 0

#     try:
#         target_position = get_vehicle_position_cached(
#             target_vehID,
#             step_cache=step_cache,
#         )
#     except Exception as e:
#         print(
#             f"Failed to get target vehicle position: "
#             f"vehID={target_vehID}, error={e}"
#         )
#         return 0, 0

#     opposite_moving_vehIDs = []
#     try:
#         opposite_edgeID = get_opposite_edgeID_by_edgeID(current_edgeID)

#         if opposite_edgeID:
#             opposite_moving_vehIDs = list(
#                 traci.edge.getLastStepVehicleIDs(opposite_edgeID)
#             )

#     except Exception as e:
#         print(
#             f"Failed to get opposite moving vehicles: "
#             f"vehID={target_vehID}, current_edgeID={current_edgeID}, error={e}"
#         )
#         opposite_moving_vehIDs = []

#     try:
#         around_vehIDs = get_around_vehIDs(
#             target_vehID=target_vehID,
#             custome_edge_list=custome_edge_list,
#             step_cache=step_cache,
#         )
#     except Exception as e:
#         print(
#             f"Failed to get around vehicles: "
#             f"vehID={target_vehID}, error={e}"
#         )
#         around_vehIDs = []

#     # opposite edge 上の車両と周囲車両を統合する。
#     # set にすることで、同じ車両の重複カウントを防ぐ。
#     candidate_vehIDs = set(opposite_moving_vehIDs) | set(around_vehIDs)

#     for other_vehID in candidate_vehIDs:
#         if other_vehID == target_vehID:
#             continue

#         other_agent = find_agent_by_vehID(
#             vehID=other_vehID,
#             agent_list=agent_list,
#             agent_by_vehID_dict=agent_by_vehID_dict,
#         )
#         if other_agent is None:
#             continue

#         try:
#             other_agent_position = get_vehicle_position_cached(
#                 other_agent.get_vehID(),
#                 step_cache=step_cache,
#             )
#         except Exception:
#             continue

#         distance = distance_each_vehIDs(
#             target_position,
#             other_agent_position,
#         )

#         if distance > distance_threshold:
#             continue

#         # U-turn 車両が evacuation_route_changed_flg も持つ可能性があるため、
#         # U-turn を優先して分類する。
#         if other_agent.get_route_change_uturn_flg():
#             uturn_change_count += 1
#         elif other_agent.get_evacuation_route_changed_flg():
#             rc_change_count += 1

#     return rc_change_count, uturn_change_count