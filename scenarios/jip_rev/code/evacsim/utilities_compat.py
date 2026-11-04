# =========================
# Backward compatibility layer for evacsim.utilities
# =========================
#
# 新しいコードでは、このファイルではなく以下から直接 import する:
#
#   evacsim.core.*
#   evacsim.sim.*
#   evacsim.maps.*
#   evacsim.metrics.*
#   evacsim.io.*
#   evacsim.viz.*
#   evacsim.utils.*
#
# 既存 runner / scenarios を壊さないため、
# 旧 evacsim.utilities の互換レイヤとして使う。

from evacsim.sim.sumo_env import ensure_sumo_tools_on_path

ensure_sumo_tools_on_path()

# =========================
# Agents / classes
# =========================
from evacsim.agents.Agent import Agent
from evacsim.agents.CustomeEdge import CustomeEdge, ConnectedEdges
from evacsim.agents.Shelter import Shelter
from evacsim.agents.VehicleInfo import VehicleInfo

# =========================
# Core
# =========================
from evacsim.core.decision import (
    is_driver_vehicle_abandant,
    is_again_driver_vehicle_abandant,
    is_candidate_shelter,
)

from evacsim.core.motivation import (
    two_stage_sigmoid,
    generate_motivation_curve,
    sigmoid,
    generate_info_activation_dict,
    re_calculate_motivation_value,
    get_value_from_time_dict,
    calculate_motivation_for_evacuation_action,
)

# =========================
# Sim
# =========================
from evacsim.sim.traci_cache import (
    create_step_cache,
    _step_cache_get,
    get_vehicle_position_cached,
    get_vehicle_road_id_cached,
    get_vehicle_route_id_cached,
    get_vehicle_route_edges_cached,
    get_vehicle_lane_index_cached,
    get_vehicle_lane_position_cached,
    get_vehicle_speed_cached,
    get_vehicle_leader_cached,
    get_edge_vehicle_ids_cached,
    get_person_position_cached,
    get_lane_length_cached,
)

from evacsim.sim.neighbors import (
    get_around_edgeIDs,
    get_around_vehIDs,
)

from evacsim.sim.routing_distance import (
    _get_shelter_group,
    _get_full_edge_length,
    _sum_route_distance_from_second_edge,
    calculate_remaining_route_distance,
    calculate_reroute_distance,
    calculate_distance_between_edgeIDs,
    distance_each_vehIDs,
)

from evacsim.sim.routing import (
    get_new_shelterID_and_near_edgeID_by_vehID,
    get_new_shelterID_and_near_edgeID_by_vehID_based_on_distance,
    find_route_name_by_edge,
    extract_current_and_best_alternative_time,
    find_alternative_route_calculated_time,
    find_alternative_route_better,
    find_alternative_better_choice_fixed,
    is_route_time_difference_exceeding_threshold,
    find_alternative_better_choice,
    find_alternative_shelter_choice,
    find_better_route,
)

from evacsim.sim.congestion import (
    is_candidate_shelter_full,
    is_vehIDs_changed_evaciation,
    is_vehIDs_another_lane,
    is_vehIDs_changed_evaciation_with_random_true,
    get_local_density,
    is_vehID_in_congested_edge,
    can_escape_to_right_lane,
)

from evacsim.sim.abandonment import (
    count_near_abandoned_vehicle_in_right_lane,
    is_vehicle_abandant_this_position,
    vehicle_abandant_behavior,
    vehicle_abandant_behavior_with_vehicle_remove,
    has_abandoned_vehicle_within_front_n,
    is_vehID_blocked_by_abandoned_vehicle,
)

from evacsim.sim.arrival import (
    handle_arrival,
    handle_arrival_for_pedestrian,
    extract_vehicle_id,
)

from evacsim.sim.communication import (
    merge_route_info_within_shelters,
    v2shelter_communication,
    v2v_communication,
    v2v_communication_about_tsunami_info,
)

from evacsim.sim.lane_change import (
    lane_change_by_vehID,
    update_agent_lane_change_motivation,
)

from evacsim.sim.initialization import (
    init_driver_behavior,
    init_vehicleInfo_list_base,
    init_agent_list,
    init_vehicleInfo_list,
)

from evacsim.sim.vehicle_generation import (
    generate_init_vehID,
    generate_simple_init_vehID,
    generate_init_vehID_with_route_edges,
    generate_new_veh,
    generate_new_veh_based_on_route_time,
)

from evacsim.sim.traffic_state import (
    calculate_wait_time,
    calculate_speed,
    density_to_coeff,
    speed_from_gap,
    smooth_speed_command,
    apply_gap_density_speed_control,
)

# =========================
# Maps
# =========================
from evacsim.maps.edge_utils import (
    get_custome_edge_by_edgeID,
    find_customedge_by_edgeID,
    get_edgeIDs_without_junction,
    remove_junction_from_edgeID,
    get_vehicle_start_edges,
    get_vehicle_end_edges,
    get_opposite_edgeID_by_edgeID,
    is_has_middle_edge,
    is_near_shelterID_on_opposite_edges,
    is_pre_edgeID_near_shelter,
    is_route_exist,
)

from evacsim.maps.route_map import (
    get_vehicle_end_list_by_start_edge_dict,
    get_nearest_end_edgeID_by_start_edgeID,
)

from evacsim.maps.map_loader import (
    init_custome_edge,
    init_shelter,
    init_connected_edges_list,
)

# =========================
# Metrics
# =========================
from evacsim.metrics.cdf import (
    calculate_cdf,
    convert_to_cdf,
)

from evacsim.metrics.evacuation_time import (
    create_arrival_time_list,
    calculate_avg_evac_time_by_route,
    merge_arrival_vehs_of_shelter,
)

# =========================
# I/O
# =========================
from evacsim.io.route_file import (
    generate_initial_vehIDs_for_row_xml,
    write_initial_vehIDs_for_row_xml,
    clean_vehIDs_for_row_xml,
    convert_routefile_to_routes_by_id,
)

from evacsim.io.json_io import (
    export_start_end_edgeIDs_to_json,
    import_start_end_edgeIDs_from_json,
    export_connected_edges_to_json,
    import_connected_edges_from_json,
)

# =========================
# Viz
# =========================
from evacsim.viz.cdf_plot import (
    plot_cdf,
)

from evacsim.viz.agent_plot import (
    plot_dot,
)

# =========================
# Utils
# =========================
from evacsim.utils.lookup import (
    find_agent_by_vehID,
    find_vehInfo_by_vehID,
    find_shelter_by_edgeID_connect_target_shelter,
    find_shelterID_by_edgeID_by_shelterID,
)

from evacsim.utils.random_utils import (
    generate_route_prob_list,
    random_true,
    choose_edge_by_probability,
    choose_route_edges_by_probability,
)

from evacsim.utils.route_utils import (
    get_next_edge,
    get_prev_edge,
    get_remaining_edges,
)

# =========================
# Constants
# =========================
VEHICLE_SHELTER_DURATION_TIME = 10000
FREE_FLOW_SPEED = 13.0
