# import_scenarios.py
from __future__ import annotations
from psycopg import Connection

import csv
import sys
import os

sys.path.append(os.path.dirname(os.path.abspath(__file__)))

from db import Database 
from repositories import ScenarioRepository
from models import (
    SimulationSettings,
    MapCommParams,
    DemandParams,
    ShelterConfig,
    ShelterParams,
    InitialDemand,
    DriverCommonParams,
    ActiveDriverParams,
    CautiousDriverParams,
    ReverseBehaviorParams,
)


def main() -> None:
    if len(sys.argv) != 2:
        raise SystemExit("Usage: python -m scenarios.common.import_params.import_scenarios /path/to/simulation_params.csv")

    csv_path = sys.argv[1]
    with open(csv_path, newline="", encoding="utf-8") as f:
        rows = list(csv.DictReader(f))
    if not rows:
        raise SystemExit("CSV is empty")
    db = Database.from_env()
    with db.transaction() as conn:
        repo = ScenarioRepository(conn)
        toml_string = repo.get_scenario_as_toml(scenario_id=1)
        
        with open("config_scenario_1.toml", "w") as f:
            f.write(toml_string)

    # r0 = rows[0]
    # db = Database.from_env()
    # with db.transaction() as conn:
    #     repo = ScenarioRepository(conn=conn)
    #     count = repo.generate_all_simulation_combinations(num_simulations=50)
    #     print(f"新たに {count} 件のシミュレーション設定を生成しました。")

    # with db.transaction() as conn:
    #     repo = ScenarioRepository(conn=conn)
    #     for row in rows:
    #         print(f"Processing row: {row}")
    #         params = MapCommParams(comm_range=int(row["comm_range"]))
    #         new_id = repo.insert_map_comm(p=params)
            
    #         print(f"挿入完了: 新しいID={new_id}, 範囲={row['comm_range']}")
                

    # db = Database.from_env()
    # # python3 -m scenarios.common.import_params.import_scenarios scenarios/common/params_csv/initial_demand_202512311758.csv
    # # トランザクション開始し、リポジトリ経由でデータを挿入
    # with db.transaction() as conn:
    #     repo = ScenarioRepository(conn)
    #     # test_record = repo.get_map_comm(map_comm_id=1)
    #     # print(test_record)
    #     # print(test_record.comm_range if test_record else "No record")
    #     # 例: 共通マスタ（必要ならr0から作る）
    #     print(r0)
    #     repo.insert_map_comm(p=MapCommParams(map_comm_id=0, comm_range=int(r0["comm_range"])))
        # repo.upsert_demand(DemandParams(demand_id=1, num_vehicles=int(r0["num_vehicles"]), vehicle_interval=int(r0["vehicle_interval"])))
        # repo.upsert_shelter_config(ShelterConfig(shelter_config_id=1, shelter_capacity_threshold=float(r0["shelter_capacity_threshold"])))

        # # shelter_id '1','2'（A,B）
        # repo.upsert_shelter(ShelterParams(shelter_id="1", shelter_config_id=1, capacity=int(r0["shelter_A_capacity"])))
        # repo.upsert_shelter(ShelterParams(shelter_id="2", shelter_config_id=1, capacity=int(r0["shelter_B_capacity"])))
        # repo.upsert_initial_demand(InitialDemand(demand_id=1, shelter_id="1", vehicles_to_shelter=int(r0["vehicles_to_shelter_A"])))
        # repo.upsert_initial_demand(InitialDemand(demand_id=1, shelter_id="2", vehicles_to_shelter=int(r0["vehicles_to_shelter_B"])))

        # repo.upsert_driver_common(DriverCommonParams(driver_common_id=1, driver_visibility_distance=int(r0["driver_visibility_distance"]), following_rate=float(r0["following_rate"])))
        # repo.upsert_reverse_behavior(ReverseBehaviorParams(
        #     reverse_behavior_id=1,
        #     tsunami_sign_start_time=int(r0["tsunami_sign_start_time"]),
        #     tsunami_sign_end_time=int(r0["tsunami_sign_end_time"]),
        #     motivation_threshold_start=int(r0["motivation_threshold_start"]),
        #     motivation_threshold_end=int(r0["motivation_threshold_end"]),
        #     min_motivation_start=int(r0["min_motivation_start"]),
        #     min_motivation_end=int(r0["min_motivation_end"]),
        #     positive_majority_bias=float(r0["positive_majority_bias"]),
        #     negative_majority_bias=float(r0["negative_majority_bias"]),
        # ))

        # ここに active/cautious の dedup & ID 採番を追加して scenario_settings を入れる
        # （必要ならあなたのCSV仕様に合わせて完成版出します）

    print("Done")


if __name__ == "__main__":
    main()
