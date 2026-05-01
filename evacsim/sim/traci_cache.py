# =========================
# SUMO / TraCI cache helpers
# =========================

from .sumo_env import ensure_sumo_tools_on_path

ensure_sumo_tools_on_path()

import traci


def create_step_cache(current_time=None):
    return {
        "current_time": current_time,
        "vehicle_position": {},
        "vehicle_road_id": {},
        "vehicle_route_id": {},
        "vehicle_route_edges": {},
        "vehicle_lane_index": {},
        "vehicle_lane_position": {},
        "vehicle_speed": {},
        "vehicle_leader": {},
        "edge_vehicle_ids": {},
        "person_position": {},
        "lane_length": {},
    }


def _step_cache_get(step_cache, bucket_name, key, loader):
    if step_cache is None:
        return loader()

    bucket = step_cache.setdefault(bucket_name, {})

    if key not in bucket:
        bucket[key] = loader()

    return bucket[key]


def get_vehicle_position_cached(vehID: str, step_cache=None):
    return _step_cache_get(
        step_cache,
        "vehicle_position",
        vehID,
        lambda: traci.vehicle.getPosition(vehID),
    )


def get_vehicle_road_id_cached(vehID: str, step_cache=None):
    return _step_cache_get(
        step_cache,
        "vehicle_road_id",
        vehID,
        lambda: traci.vehicle.getRoadID(vehID),
    )


def get_vehicle_route_id_cached(vehID: str, step_cache=None):
    return _step_cache_get(
        step_cache,
        "vehicle_route_id",
        vehID,
        lambda: traci.vehicle.getRouteID(vehID),
    )


def get_vehicle_route_edges_cached(vehID: str, step_cache=None):
    return _step_cache_get(
        step_cache,
        "vehicle_route_edges",
        vehID,
        lambda: traci.route.getEdges(
            get_vehicle_route_id_cached(vehID, step_cache=step_cache)
        ),
    )


def get_vehicle_lane_index_cached(vehID: str, step_cache=None):
    return _step_cache_get(
        step_cache,
        "vehicle_lane_index",
        vehID,
        lambda: traci.vehicle.getLaneIndex(vehID),
    )


def get_vehicle_lane_position_cached(vehID: str, step_cache=None):
    return _step_cache_get(
        step_cache,
        "vehicle_lane_position",
        vehID,
        lambda: traci.vehicle.getLanePosition(vehID),
    )


def get_vehicle_speed_cached(vehID: str, step_cache=None):
    return _step_cache_get(
        step_cache,
        "vehicle_speed",
        vehID,
        lambda: traci.vehicle.getSpeed(vehID),
    )


def get_vehicle_leader_cached(vehID: str, dist=None, step_cache=None):
    key = (vehID, dist)

    if dist is None:
        loader = lambda: traci.vehicle.getLeader(vehID)
    else:
        loader = lambda: traci.vehicle.getLeader(vehID, dist=dist)

    return _step_cache_get(
        step_cache,
        "vehicle_leader",
        key,
        loader,
    )


def get_edge_vehicle_ids_cached(edgeID: str, step_cache=None):
    if edgeID is None:
        return ()

    return _step_cache_get(
        step_cache,
        "edge_vehicle_ids",
        edgeID,
        lambda: tuple(traci.edge.getLastStepVehicleIDs(edgeID)),
    )


def get_person_position_cached(personID: str, step_cache=None):
    return _step_cache_get(
        step_cache,
        "person_position",
        personID,
        lambda: traci.person.getPosition(personID),
    )


def get_lane_length_cached(laneID: str, step_cache=None):
    if laneID is None:
        return 0.0

    return _step_cache_get(
        step_cache,
        "lane_length",
        laneID,
        lambda: traci.lane.getLength(laneID),
    )