# results_save_data.py
from __future__ import annotations
import sys
import ast
import re
from pathlib import Path
from evacsim.repo.db import Database
from evacsim.repo.repositories import SimulationResultRepository, ScenarioRepository

def parse_simulation_out_file(out_path: str | Path) -> dict:
    path = Path(out_path)
    text = path.read_text(encoding="utf-8")

    def extract(pattern: str, cast=str, default=None):
        m = re.search(pattern, text, re.MULTILINE)
        if not m:
            return default
        return cast(m.group(1))

    scenario_id = extract(r"^scenario\s*:\s*(\d+)", int)
    early_rate = extract(r"^early_rate:\s*([0-9.]+)", float)
    v2v_rate = extract(r"^v2v_rate\s*:\s*([0-9.]+)", float)
    run_id = extract(r"^run_id\s*:\s*(\d+)", int)

    route_changed_vehicle_count = extract(r"^route_changed_vehicle_count:(\d+)", int, 0)
    normalcy_bias_route_change_count = extract(r"^normalcy_bias_route_change_count:(\d+)", int, 0)
    majority_bias_route_change_count = extract(r"^majority_bias_route_change_count:(\d+)", int, 0)
    shelter_congestion_count = extract(r"^shelter_congestion_count:(\d+)", int, 0)
    shelter_capacity_full_count = extract(r"^shelter_capacity_full_count:(\d+)", int, 0)

    arrival_dict_str = extract(
        r"^arrival_time_by_vehID_dict:(\{.*\})",
        str,
        "{}",
    )
    arrival_time_by_vehID_dict = ast.literal_eval(arrival_dict_str)

    return {
        "scenario_id": scenario_id,
        "early_rate": early_rate,
        "v2v_rate": v2v_rate,
        "run_id": run_id,
        "arrival_time_by_vehID_dict": arrival_time_by_vehID_dict,
        "route_changed_vehicle_count": route_changed_vehicle_count,
        "normalcy_bias_route_change_count": normalcy_bias_route_change_count,
        "majority_bias_route_change_count": majority_bias_route_change_count,
        "shelter_congestion_count": shelter_congestion_count,
        "shelter_capacity_full_count": shelter_capacity_full_count,
    }

def save_parsed_result_to_db(parsed: dict) -> None:
    db = Database.from_env()

    scenario_id = parsed["scenario_id"]
    arrival_time_by_vehID_dict = parsed["arrival_time_by_vehID_dict"]

    with db.transaction() as conn:
        results_repo = SimulationResultRepository(conn)
        result_id, metric_id = results_repo.save_simulation_result_with_metrics(
            scenario_id=scenario_id,
            v2v_rate=parsed["v2v_rate"],
            early_rate=parsed["early_rate"],
            route_changed_vehicle_count=parsed["route_changed_vehicle_count"],
            normalcy_bias_count=parsed["normalcy_bias_route_change_count"],
            majority_bias_count=parsed["majority_bias_route_change_count"],
            shelter_congestion_count=parsed["shelter_congestion_count"],
            shelter_capacity_full_count=parsed["shelter_capacity_full_count"],
            )

        results_repo.insert_arrival_time_series_bulk(
            result_id=result_id,
            scenario_id=scenario_id,
            arrival_time_by_vehID_dict=arrival_time_by_vehID_dict,
            )

        scenario_repo = ScenarioRepository(conn)
        scenario_repo.mark_simulation_executed(scenario_id)

def main():
    if len(sys.argv) != 2:
        print("Usage: python results_save_data.py <out_file>")
        sys.exit(1)

    out_file = sys.argv[1]
    parsed = parse_simulation_out_file(out_file)
    save_parsed_result_to_db(parsed)
    print(f"saved: {out_file}")

if __name__ == "__main__":
    main()