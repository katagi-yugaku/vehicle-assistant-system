# repositories.py
from __future__ import annotations

from psycopg import Connection
import toml

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


class ScenarioRepository:
    def __init__(self, conn: Connection):
        self.conn = conn

    def get_scenario_all_params(self, scenario_id: int) -> str:
        """指定したscenario_idの全パラメータを結合し、TOML形式の文字列で返す"""
        with self.conn.cursor() as cur:
            cur.execute(
                """
                SELECT 
                    s.scenario_id, s.num_simulations,
                    m.comm_range,
                    d.num_vehicles, d.vehicle_interval,
                    sc.shelter_capacity_threshold,
                    dc.driver_visibility_distance, dc.following_rate,
                    ad.active_route_change_mean, ad.active_route_change_var,
                    ad.positive_lanechange_start, ad.positive_lanechange_end,
                    ad.active_shelter_occupancy_rate_threshold_start,
                    ad.active_shelter_occupancy_rate_threshold_end,
                    cd.cautious_route_change_mean, cd.cautious_route_change_var,
                    cd.negative_lanechange_start, cd.negative_lanechange_end,
                    cd.cautious_shelter_occupancy_rate_threshold_start,
                    cd.cautious_shelter_occupancy_rate_threshold_end,
                    rb.tsunami_sign_start_time, rb.tsunami_sign_end_time,
                    rb.motivation_threshold_start, rb.motivation_threshold_end,
                    rb.min_motivation_start, rb.min_motivation_end,
                    rb.positive_majority_bias, rb.negative_majority_bias
                FROM simulation_settings s
                JOIN map_comm_params m ON s.map_comm_id = m.map_comm_id
                JOIN demand_params d ON s.demand_id = d.demand_id
                JOIN shelter_config sc ON s.shelter_config_id = sc.shelter_config_id
                JOIN driver_common_params dc ON s.driver_common_id = dc.driver_common_id
                JOIN active_driver_params ad ON s.active_driver_id = ad.active_driver_id
                JOIN cautious_driver_params cd ON s.cautious_driver_id = cd.cautious_driver_id
                JOIN reverse_behavior_params rb ON s.reverse_behavior_id = rb.reverse_behavior_id
                WHERE s.scenario_id = %s;
                """,
                (scenario_id,)
            )
            row = cur.fetchone()
            if not row:
                return ""

            # psycopgの辞書形式(dict_row)を使っている前提でのマッピング
            # タプルの場合は row[0], row[1]... で辞書を作る
            data = dict(row) if isinstance(row, dict) else {
                "scenario_id": row[0],
                "num_simulations": row[1],
                "comm_range": row[2],
                "num_vehicles": row[3],
                "vehicle_interval": row[4],
                "shelter_capacity_threshold": row[5],
                "driver_visibility_distance": row[6],
                "following_rate": row[7],
                "active_route_change_mean": row[8],
                "active_route_change_var": row[9],
                "positive_lanechange_start": row[10],
                "positive_lanechange_end": row[11],
                "active_shelter_occupancy_rate_threshold_start": row[12],
                "active_shelter_occupancy_rate_threshold_end": row[13],
                "cautious_route_change_mean": row[14],
                "cautious_route_change_var": row[15],
                "negative_lanechange_start": row[16],
                "negative_lanechange_end": row[17],
                "cautious_shelter_occupancy_rate_threshold_start": row[18],
                "cautious_shelter_occupancy_rate_threshold_end": row[19],
                "tsunami_sign_start_time": row[20],
                "tsunami_sign_end_time": row[21],
                "motivation_threshold_start": row[22],
                "motivation_threshold_end": row[23],
                "min_motivation_start": row[24],
                "min_motivation_end": row[25],
                "positive_majority_bias": row[26],
                "negative_majority_bias": row[27],
            }

            # 避難所と初期需要は「1対多」なので別途取得して追加する例
            cur.execute("""
                SELECT sp.shelter_id, sp.capacity, id.vehicles_to_shelter
                FROM shelter_params sp
                JOIN initial_demand id ON sp.shelter_id = id.shelter_id
                WHERE id.demand_id = (SELECT demand_id FROM simulation_settings WHERE scenario_id = %s)
                AND sp.shelter_config_id = (SELECT shelter_config_id FROM simulation_settings WHERE scenario_id = %s);
            """, (scenario_id, scenario_id))
            
            shelters = cur.fetchall()
            for s_row in shelters:
                # 1=A, 2=B のような命名規則で辞書に追加
                s_id = "A" if s_row["shelter_id"] == 1 else "B"
                data[f"shelter_{s_id}_capacity"] = s_row["capacity"]
                data[f"vehicles_to_shelter_{s_id}"] = s_row["vehicles_to_shelter"]
            return data

    def get_scenario_as_toml(self, scenario_id: int) -> str:
        """指定したscenario_idの全パラメータを結合し、TOML形式の文字列で返す"""
        with self.conn.cursor() as cur:
            cur.execute(
                """
                SELECT 
                    s.scenario_id, s.num_simulations,
                    m.comm_range,
                    d.num_vehicles, d.vehicle_interval,
                    sc.shelter_capacity_threshold,
                    dc.driver_visibility_distance, dc.following_rate,
                    ad.active_route_change_mean, ad.active_route_change_var,
                    ad.positive_lanechange_start, ad.positive_lanechange_end,
                    ad.active_shelter_occupancy_rate_threshold_start,
                    ad.active_shelter_occupancy_rate_threshold_end,
                    cd.cautious_route_change_mean, cd.cautious_route_change_var,
                    cd.negative_lanechange_start, cd.negative_lanechange_end,
                    cd.cautious_shelter_occupancy_rate_threshold_start,
                    cd.cautious_shelter_occupancy_rate_threshold_end,
                    rb.tsunami_sign_start_time, rb.tsunami_sign_end_time,
                    rb.motivation_threshold_start, rb.motivation_threshold_end,
                    rb.min_motivation_start, rb.min_motivation_end,
                    rb.positive_majority_bias, rb.negative_majority_bias
                FROM simulation_settings s
                JOIN map_comm_params m ON s.map_comm_id = m.map_comm_id
                JOIN demand_params d ON s.demand_id = d.demand_id
                JOIN shelter_config sc ON s.shelter_config_id = sc.shelter_config_id
                JOIN driver_common_params dc ON s.driver_common_id = dc.driver_common_id
                JOIN active_driver_params ad ON s.active_driver_id = ad.active_driver_id
                JOIN cautious_driver_params cd ON s.cautious_driver_id = cd.cautious_driver_id
                JOIN reverse_behavior_params rb ON s.reverse_behavior_id = rb.reverse_behavior_id
                WHERE s.scenario_id = %s;
                """,
                (scenario_id,)
            )
            row = cur.fetchone()
            if not row:
                return ""

            # psycopgの辞書形式(dict_row)を使っている前提でのマッピング
            # タプルの場合は row[0], row[1]... で辞書を作る
            data = dict(row) if isinstance(row, dict) else {
                "scenario_id": row[0],
                "num_simulations": row[1],
                "comm_range": row[2],
                "num_vehicles": row[3],
                "vehicle_interval": row[4],
                "shelter_capacity_threshold": row[5],
                "driver_visibility_distance": row[6],
                "following_rate": row[7],
                "active_route_change_mean": row[8],
                "active_route_change_var": row[9],
                "positive_lanechange_start": row[10],
                "positive_lanechange_end": row[11],
                "active_shelter_occupancy_rate_threshold_start": row[12],
                "active_shelter_occupancy_rate_threshold_end": row[13],
                "cautious_route_change_mean": row[14],
                "cautious_route_change_var": row[15],
                "negative_lanechange_start": row[16],
                "negative_lanechange_end": row[17],
                "cautious_shelter_occupancy_rate_threshold_start": row[18],
                "cautious_shelter_occupancy_rate_threshold_end": row[19],
                "tsunami_sign_start_time": row[20],
                "tsunami_sign_end_time": row[21],
                "motivation_threshold_start": row[22],
                "motivation_threshold_end": row[23],
                "min_motivation_start": row[24],
                "min_motivation_end": row[25],
                "positive_majority_bias": row[26],
                "negative_majority_bias": row[27],
            }

            # 避難所と初期需要は「1対多」なので別途取得して追加する例
            cur.execute("""
                SELECT sp.shelter_id, sp.capacity, id.vehicles_to_shelter
                FROM shelter_params sp
                JOIN initial_demand id ON sp.shelter_id = id.shelter_id
                WHERE id.demand_id = (SELECT demand_id FROM simulation_settings WHERE scenario_id = %s)
                AND sp.shelter_config_id = (SELECT shelter_config_id FROM simulation_settings WHERE scenario_id = %s);
            """, (scenario_id, scenario_id))
            
            shelters = cur.fetchall()
            for s_row in shelters:
                # 1=A, 2=B のような命名規則で辞書に追加
                s_id = "A" if s_row["shelter_id"] == 1 else "B"
                data[f"shelter_{s_id}_capacity"] = s_row["capacity"]
                data[f"vehicles_to_shelter_{s_id}"] = s_row["vehicles_to_shelter"]
            return toml.dumps(data)
    
    def generate_all_simulation_combinations(self, num_simulations: int) -> int:
        """全パラメータテーブルの組み合わせを生成して simulation_settings に挿入する"""
        with self.conn.cursor() as cur:
            cur.execute(
                """
                INSERT INTO simulation_settings (
                    num_simulations, 
                    map_comm_id, 
                    demand_id, 
                    shelter_config_id, 
                    driver_common_id, 
                    active_driver_id, 
                    cautious_driver_id, 
                    reverse_behavior_id
                )
                SELECT 
                    %s, -- num_simulations
                    m.map_comm_id, 
                    d.demand_id, 
                    s.shelter_config_id, 
                    dc.driver_common_id, 
                    ad.active_driver_id, 
                    cd.cautious_driver_id, 
                    rb.reverse_behavior_id
                FROM map_comm_params m
                CROSS JOIN demand_params d
                CROSS JOIN shelter_config s
                CROSS JOIN driver_common_params dc
                CROSS JOIN active_driver_params ad
                CROSS JOIN cautious_driver_params cd
                CROSS JOIN reverse_behavior_params rb
                ON CONFLICT (
                    num_simulations, map_comm_id, demand_id, shelter_config_id, 
                    driver_common_id, active_driver_id, cautious_driver_id, reverse_behavior_id
                ) DO NOTHING;
                """,
                (num_simulations,)
            )
            return cur.rowcount  # 新しく挿入された件数を返す
    
    def insert_map_comm(self, p: MapCommParams) -> int:
        with self.conn.cursor() as cur:
            # IDは指定せず、comm_rangeだけを挿入する
            cur.execute(
                """
                INSERT INTO map_comm_params (comm_range)
                VALUES (%s)
                ON CONFLICT (comm_range) DO NOTHING
                RETURNING map_comm_id;
                """,
                (p.comm_range,)  # p.map_comm_id は使わない
            )
            row = cur.fetchone()
            
            # 新しく挿入された場合はそのIDを返す
            if row:
                return row["map_comm_id"] if isinstance(row, dict) else row[0]

            # 重複して挿入されなかった場合は、既存のIDを取得して返す
            cur.execute(
                "SELECT map_comm_id FROM map_comm_params WHERE comm_range = %s",
                (p.comm_range,)
            )
            row = cur.fetchone()
            return row["map_comm_id"] if isinstance(row, dict) else row[0]

    def update_map_comm(self, p: MapCommParams) -> None:
        with self.conn.cursor() as cur:
            cur.execute(
                """
                INSERT INTO map_comm_params (map_comm_id, comm_range)
                VALUES (%s, %s)
                ON CONFLICT (map_comm_id)
                DO UPDATE SET
                comm_range = EXCLUDED.comm_range
                """,
                (p.map_comm_id, p.comm_range),
            )

    def upsert_demand(self, p: DemandParams) -> None:
        with self.conn.cursor() as cur:
            cur.execute(
                """
                INSERT INTO demand_params (demand_id, num_vehicles, vehicle_interval)
                VALUES (%s, %s, %s)
                ON CONFLICT (demand_id) DO NOTHING
                """,
                (p.demand_id, p.num_vehicles, p.vehicle_interval),
            )

    def upsert_shelter_config(self, p: ShelterConfig) -> None:
        with self.conn.cursor() as cur:
            cur.execute(
                """
                INSERT INTO shelter_config (shelter_config_id, shelter_capacity_threshold)
                VALUES (%s, %s)
                ON CONFLICT (shelter_config_id) DO NOTHING
                """,
                (p.shelter_config_id, p.shelter_capacity_threshold),
            )

    def upsert_shelter(self, p: ShelterParams) -> None:
        with self.conn.cursor() as cur:
            cur.execute(
                """
                INSERT INTO shelter_params (shelter_id, shelter_config_id, capacity)
                VALUES (%s, %s, %s)
                ON CONFLICT (shelter_id) DO NOTHING
                """,
                (p.shelter_id, p.shelter_config_id, p.capacity),
            )

    def upsert_initial_demand(self, p: InitialDemand) -> None:
        with self.conn.cursor() as cur:
            cur.execute(
                """
                INSERT INTO initial_demand (demand_id, shelter_id, vehicles_to_shelter)
                VALUES (%s, %s, %s)
                ON CONFLICT (demand_id, shelter_id) DO NOTHING
                """,
                (p.demand_id, p.shelter_id, p.vehicles_to_shelter),
            )

    def upsert_driver_common(self, p: DriverCommonParams) -> None:
        with self.conn.cursor() as cur:
            cur.execute(
                """
                INSERT INTO driver_common_params (driver_common_id, driver_visibility_distance, following_rate)
                VALUES (%s, %s, %s)
                ON CONFLICT (driver_common_id) DO NOTHING
                """,
                (p.driver_common_id, p.driver_visibility_distance, p.following_rate),
            )

    def upsert_active_driver(self, p: ActiveDriverParams) -> None:
        with self.conn.cursor() as cur:
            cur.execute(
                """
                INSERT INTO active_driver_params (
                  active_driver_id,
                  active_route_change_mean, active_route_change_var,
                  positive_lanechange_start, positive_lanechange_end,
                  active_shelter_occupancy_rate_threshold_start,
                  active_shelter_occupancy_rate_threshold_end
                ) VALUES (%s,%s,%s,%s,%s,%s,%s)
                ON CONFLICT (active_driver_id) DO NOTHING
                """,
                (
                    p.active_driver_id,
                    p.active_route_change_mean,
                    p.active_route_change_var,
                    p.positive_lanechange_start,
                    p.positive_lanechange_end,
                    p.active_shelter_occupancy_rate_threshold_start,
                    p.active_shelter_occupancy_rate_threshold_end,
                ),
            )

    def upsert_cautious_driver(self, p: CautiousDriverParams) -> None:
        with self.conn.cursor() as cur:
            cur.execute(
                """
                INSERT INTO cautious_driver_params (
                  cautious_driver_id,
                  cautious_route_change_mean, cautious_route_change_var,
                  negative_lanechange_start, negative_lanechange_end,
                  cautious_shelter_occupancy_rate_threshold_start,
                  cautious_shelter_occupancy_rate_threshold_end
                ) VALUES (%s,%s,%s,%s,%s,%s,%s)
                ON CONFLICT (cautious_driver_id) DO NOTHING
                """,
                (
                    p.cautious_driver_id,
                    p.cautious_route_change_mean,
                    p.cautious_route_change_var,
                    p.negative_lanechange_start,
                    p.negative_lanechange_end,
                    p.cautious_shelter_occupancy_rate_threshold_start,
                    p.cautious_shelter_occupancy_rate_threshold_end,
                ),
            )

    def upsert_reverse_behavior(self, p: ReverseBehaviorParams) -> None:
        with self.conn.cursor() as cur:
            cur.execute(
                """
                INSERT INTO reverse_behavior_params (
                  reverse_behavior_id,
                  tsunami_sign_start_time, tsunami_sign_end_time,
                  motivation_threshold_start, motivation_threshold_end,
                  min_motivation_start, min_motivation_end,
                  positive_majority_bias, negative_majority_bias
                ) VALUES (%s,%s,%s,%s,%s,%s,%s,%s,%s)
                ON CONFLICT (reverse_behavior_id) DO NOTHING
                """,
                (
                    p.reverse_behavior_id,
                    p.tsunami_sign_start_time,
                    p.tsunami_sign_end_time,
                    p.motivation_threshold_start,
                    p.motivation_threshold_end,
                    p.min_motivation_start,
                    p.min_motivation_end,
                    p.positive_majority_bias,
                    p.negative_majority_bias,
                ),
            )

    # --- Parent table ---

    def upsert_simulation_settings(self, s: SimulationSettings) -> None:
        with self.conn.cursor() as cur:
            cur.execute(
                """
                INSERT INTO simulation_settings (
                  scenario_id, num_simulations,
                  map_comm_id, demand_id, shelter_config_id,
                  driver_common_id, active_driver_id, cautious_driver_id, reverse_behavior_id
                ) VALUES (%s,%s,%s,%s,%s,%s,%s,%s,%s)
                ON CONFLICT (scenario_id) DO NOTHING
                """,
                (
                    s.scenario_id,
                    s.num_simulations,
                    s.map_comm_id,
                    s.demand_id,
                    s.shelter_config_id,
                    s.driver_common_id,
                    s.active_driver_id,
                    s.cautious_driver_id,
                    s.reverse_behavior_id,
                ),
            )
