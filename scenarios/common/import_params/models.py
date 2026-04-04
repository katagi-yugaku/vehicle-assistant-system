from __future__ import annotations

from dataclasses import dataclass
from datetime import datetime


# =========================
# 1) Parent: simulation_settings
# =========================
@dataclass(frozen=True)
class SimulationSettings:
    """
    Table: simulation_settings
    """
    num_simulations: int
    map_comm_id: int
    demand_id: int
    shelter_config_id: int
    driver_common_id: int
    active_driver_id: int
    cautious_driver_id: int
    reverse_behavior_id: int
    is_executed: bool = False
    executed_at: datetime | None = None
    is_deleted: bool = False
    scenario_id: int | None = None


# =========================
# 2) Communication & Map
# =========================
@dataclass(frozen=True)
class MapCommParams:
    """
    Table: map_comm_params
    """
    comm_range: int
    is_deleted: bool = False
    map_comm_id: int | None = None


# =========================
# 3) Demand (generation)
# =========================
@dataclass(frozen=True)
class DemandParams:
    """
    Table: demand_params
    """
    num_vehicles: int
    vehicle_interval: int
    is_deleted: bool = False
    demand_id: int | None = None


# =========================
# 4) Shelter config & shelters
# =========================
@dataclass(frozen=True)
class ShelterConfig:
    """
    Table: shelter_config
    """
    shelter_capacity_threshold: float
    is_deleted: bool = False
    shelter_config_id: int | None = None


@dataclass(frozen=True)
class ShelterParams:
    """
    Table: shelter_params
    """
    shelter_id: int
    shelter_config_id: int
    capacity: int
    is_deleted: bool = False


@dataclass(frozen=True)
class InitialDemand:
    """
    Table: initial_demand
    """
    demand_id: int
    shelter_id: int
    vehicles_to_shelter: int
    is_deleted: bool = False


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
    system_mode: bool
    executed_at: datetime = datetime.now()
    is_deleted: bool = False
    result_id: int | None = None


@dataclass(frozen=True)
class ResultMetric:
    """
    Table: result_metrics
    """
    result_id: int
    early_rate: float
    route_changed_vehicle_count: float | None = None
    normalcy_bias_count: float | None = None
    majority_bias_count: float | None = None
    shelter_congestion_count: float | None = None
    shelter_capacity_full_count: float | None = None
    is_deleted: bool = False
    metric_id: int | None = None


@dataclass(frozen=True)
class ArrivalTimeSeries:
    """
    Table: arrival_times_series
    """
    metric_id: int
    order_index: int
    arrival_time: float
    is_deleted: bool = False
    series_id: int | None = None