from __future__ import annotations

from dataclasses import dataclass, field
from datetime import datetime


# =========================
# 1) Parent: simulation_settings
# =========================
@dataclass(frozen=True)
class SimulationSettings:
    """
    Table: simulation_settings
    """
    num_simulations: int # シミュレーション回数
    map_comm_id: int # 外部キー: map_comm_params
    demand_id: int # 外部キー: demand_params
    shelter_config_id: int # 外部キー: shelter_config
    driver_common_id: int # 外部キー: driver_common_params
    active_driver_id: int # 外部キー: active_driver_params
    cautious_driver_id: int # 外部キー: cautious_driver_params
    reverse_behavior_id: int # 外部キー: reverse_behavior_params
    is_executed: bool = False # シミュレーション実行済みフラグ
    executed_at: datetime | None = None # シミュレーション実行日時
    is_deleted: bool = False # 論理削除フラグ
    scenario_id: int | None = None # 主キー


# =========================
# 2) Communication & Map
# =========================
@dataclass(frozen=True)
class MapCommParams:
    """
    Table: map_comm_params
    """
    comm_range: int # 通信範囲(m)
    is_deleted: bool = False # 論理削除フラグ
    map_comm_id: int | None = None # 主キー


# =========================
# 3) Demand (generation)
# =========================
@dataclass(frozen=True)
class DemandParams:
    """
    Table: demand_params
    """
    num_vehicles: int # 総車両数
    vehicle_interval: int # 車両発生間隔(秒)
    is_deleted: bool = False # 論理削除フラグ
    demand_id: int | None = None # 主キー


# =========================
# 4) Shelter config & shelters
# =========================
@dataclass(frozen=True)
class ShelterConfig:
    """
    Table: shelter_config
    """
    shelter_capacity_threshold: float # 避難所混雑閾値
    is_deleted: bool = False # 論理削除フラグ
    shelter_config_id: int | None = None # 主キー


@dataclass(frozen=True)
class ShelterParams:
    """
    Table: shelter_params
    """
    shelter_id: int # 避難所ID
    shelter_config_id: int # 外部キー: shelter_config
    capacity: int # 避難所収容台数
    is_deleted: bool = False # 論理削除フラグ


@dataclass(frozen=True)
class InitialDemand:
    """
    Table: initial_demand
    """
    demand_id: int # 外部キー: demand_params
    shelter_id: int # 外部キー: shelter_params
    vehicles_to_shelter: int # 避難所への初期割り当て台数
    is_deleted: bool = False # 論理削除フラグ


# =========================
# 5) Driver common
# =========================
@dataclass(frozen=True)
class DriverCommonParams:
    """
    Table: driver_common_params
    """
    driver_visibility_distance: int
    following_rate: float
    is_deleted: bool = False
    driver_common_id: int | None = None


# =========================
# 6) Active / Cautious
# =========================
@dataclass(frozen=True)
class ActiveDriverParams:
    """
    Table: active_driver_params
    """
    active_route_change_mean: float
    active_route_change_var: float
    positive_lanechange_start: int
    positive_lanechange_end: int
    active_shelter_occupancy_rate_threshold_start: float
    active_shelter_occupancy_rate_threshold_end: float
    is_deleted: bool = False
    active_driver_id: int | None = None


@dataclass(frozen=True)
class CautiousDriverParams:
    """
    Table: cautious_driver_params
    """
    cautious_route_change_mean: float
    cautious_route_change_var: float
    negative_lanechange_start: int
    negative_lanechange_end: int
    cautious_shelter_occupancy_rate_threshold_start: float
    cautious_shelter_occupancy_rate_threshold_end: float
    is_deleted: bool = False
    cautious_driver_id: int | None = None


# =========================
# 7) Reverse behavior & event params
# =========================
@dataclass(frozen=True)
class ReverseBehaviorParams:
    """
    Table: reverse_behavior_params
    """
    tsunami_sign_start_time: int
    tsunami_sign_end_time: int
    motivation_threshold_start: int
    motivation_threshold_end: int
    min_motivation_start: int
    min_motivation_end: int
    positive_majority_bias: float
    negative_majority_bias: float
    is_deleted: bool = False
    reverse_behavior_id: int | None = None


# =========================
# 8) Simulation Results
# =========================
@dataclass(frozen=True)
class SimulationResult:
    """
    Table: simulation_results
    """
    scenario_id: int
    executed_at: datetime = field(default_factory=datetime.now)
    is_deleted: bool = False
    result_id: int | None = None


@dataclass(frozen=True)
class ResultMetric:
    """
    Table: result_metrics
    """
    result_id: int
    scenario_id: int
    v2v_rate: float
    early_rate: float
    route_changed_vehicle_count: int | None = None
    normalcy_bias_count: int | None = None
    majority_bias_count: int | None = None
    shelter_congestion_count: int | None = None
    shelter_capacity_full_count: int | None = None
    is_deleted: bool = False
    metric_id: int | None = None


@dataclass(frozen=True)
class ArrivalTimeSeries:
    """
    Table: arrival_times_series
    各車両の到着時刻
    """
    result_id: int
    scenario_id: int
    veh_id: str
    arrival_time: float
    is_deleted: bool = False
    created_time: datetime = field(default_factory=datetime.now)
    updated_time: datetime = field(default_factory=datetime.now)
    series_id: int | None = None