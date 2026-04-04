# models.py
from dataclasses import dataclass


@dataclass(frozen=True)
class SimulationSettings:
    scenario_id: int
    num_simulations: int
    map_comm_id: int
    demand_id: int
    shelter_config_id: int
    driver_common_id: int
    active_driver_id: int
    cautious_driver_id: int
    reverse_behavior_id: int

