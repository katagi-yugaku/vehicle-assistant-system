# =========================
# Standard library
# =========================
import copy
import os
import random
import sys
from collections import Counter, defaultdict
from math import sqrt
from pathlib import Path
import optparse
import sys
import argparse
# sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '../../../..')))
# =========================
# Third-party libraries
# =========================
# SUMO tools need SUMO_HOME on sys.path *before* importing sumolib/traci
if "SUMO_HOME" in os.environ:
    sys.path.append(os.path.join(os.environ["SUMO_HOME"], "tools"))

from sumolib import checkBinary  # noqa: E402
import traci  # noqa: E402
import matplotlib.pyplot as plt
import numpy as np
from numpy import double
from typing import List

# =========================
# Local / intra-package
# =========================
from evacsim import utilities
from evacsim.agents.Agent import Agent
from evacsim.agents.CustomeEdge import CustomeEdge, ConnectedEdges  # ファイル名の綴りに注意
from evacsim.agents.Shelter import Shelter
from evacsim.agents.VehicleInfo import VehicleInfo
import datetime

# =========================
# Runtime config / seeds
# =========================
# random.seed(318)  # 乱数シードを固定（再現性）
random.seed()
random.seed(os.getpid() + int(datetime.datetime.now().timestamp()))
np.random.seed(os.getpid() + int(datetime.datetime.now().timestamp()))

# runner_base.py のファイル位置から data ディレクトリを解決
HERE = Path(__file__).resolve()
MAP_ONE_DIR = HERE.parent.parent          # .../scenarios/its102/map_one
DATA_DIR = MAP_ONE_DIR / "data"
SUMO_CFG = DATA_DIR / "one_shelter_one_departure.sumocfg"

# =========================
# 1) シミュレーション基本設定・時間評価
# =========================
END_SIMULATION_TIME = 2000
DECISION_EVALUATION_INTERVAL = 10.0
THRESHOLD_SPEED = 2.77 # 10km/h
STOPPING_TIME_IN_SHELTER = 15
SPEED_ARRANGE = 1
CONGESTION_RATE = 0.3
SHOW_DEBUG_COUNT = 0

# =========================
# 2) 車両生成・発進関連
# =========================
VEHICLE_NUM = 0
DEPART_TIME: double = 0.0
ROUTE_NUM = 0

# # ======================================================================================================================================================
# # 3) 通信・知覚範囲,  4) ルート変更分布パラメータ・収容率, 5) 津波予兆（取得タイミング）, 6) 心理モデル：動機付け関連パラメータ, 7) 心理モデル：同調行動関連パラメータ 
# # ======================================================================================================================================================
# TOTAL_VEHNUM = 300
# DRIVER_VISIBILITY_DISTANCE: float = 30.0
# COMMUNICATION_RANGE = 100
# ACTIVE_ROUTE_CHANGE_MEAN = 60.0
# ACTIVE_ROUTE_CHANGE_VAR = 30.0
# CAUTIOUS_ROUTE_CHANGE_MEAN = -60.0
# CAUTIOUS_ROUTE_CHANGE_VAR = 30.0
# SHELTER_CAPACITY_THRESHOLD = 0.5
# TSUNAMI_SIGN_START_TIME = 300
# TSUNAMI_SIGN_END_TIME = 400
# TSUNAMI_PRECURSOR_INFO_OBTAIN_TIME = 0
# MOTIVATION_THRESHOLD_START = 800.0
# MOTIVATION_THRESHOLD_END = 1200.0
# MIN_MOTIVATION_START = 500.0
# MIN_MOTIVATION_END = 700.0
# POSITIVE_LANECHANGE_START = 900.0
# POSITIVE_LANECHANGE_END = 1100.0
# NEGATIVE_LANECHANGE_START = 500.0
# NEGATIVE_LANECHANGE_END = 700.0
# POSITIVE_MAJORITY_BIAS = 100.0
# NEGATIVE_MAJORITY_BIAS = 100.0

# =========================
# 8) 心理モデル：初期タイプ別しきい（早期/遅延）
# =========================
EARLY_AGENT_THRESHOLD_LIST = [60, 90, 100, 130] # 早期決断者の閾値
LATE_AGENT_THRESHOLD_LIST = [180, 220, 300, 350] # 遅延決断者の閾値

# =========================
# 10) 実行時カウンタ（毎試行でリセットされる動的値）
# =========================
ROUTE_CHANGED_VEHICLE_COUNT = 0
NORMALCY_BIAS_ROUTE_CHANGE_COUNT = 0
MAJORITY_BIAS_ROUTE_CHANGE_COUNT = 0
LANE_CHANGED_VEHICLE_COUNT = 0
OBTAIN_INFO_LANE_CHANGE_COUNT = 0
ELAPSED_TIME_LANE_CHANGE_COUNT = 0
NORMALCY_BIAS_COUNT = 0
POSITIVE_MAJORITY_BIAS_COUNT = 0
NEGATIVE_MAJORITY_BIAS_COUNT = 0
NEW_VEHICLE_COUNT = 0

# リストの初期化
custome_edge_list: list = []
shelter_list :List[Shelter] = []
vehInfo_list = []
vehID_list = []
veh_written_list = []
connected_edges_list = []
shelterA_arrival_time_list = []
shelterB_arrival_time_list = []
arrival_time_by_vehID_dict = {}
arrival_time_list = []
elapsed_time_list = []
# dictの初期化
current_route_dict = {}

import argparse



def run():
    while traci.simulation.getTime() < END_SIMULATION_TIME:
        traci.simulationStep()
        control_vehicles()
    traci.close()
    sys.stdout.flush()


def control_vehicles():
    vehIDs = traci.vehicle.getIDList()
    global NEW_VEHICLE_COUNT
    global LANE_CHANGED_VEHICLE_COUNT
    global POSITIVE_MAJORITY_BIAS_COUNT
    global NEGATIVE_MAJORITY_BIAS_COUNT
    global NORMALCY_BIAS_COUNT
    global OBTAIN_INFO_LANE_CHANGE_COUNT
    global ELAPSED_TIME_LANE_CHANGE_COUNT
    # 現在の存在するrouteを確認する
    for routeID in traci.route.getIDList():
        # 現在のrouteを取得
        current_route_dict[routeID] = traci.route.getEdges(routeID)

    for current_vehID in vehIDs:
        vehInfo_by_current_vehID: VehicleInfo = utilities.find_vehInfo_by_vehID(current_vehID, vehInfo_list)
        shelter_for_current_vehID: Shelter = utilities.find_shelter_by_edgeID_connect_target_shelter(vehInfo_by_current_vehID.get_edgeID_connect_target_shelter(), shelter_list)
        agent_by_current_vehID: Agent = utilities.find_agent_by_vehID(current_vehID, agent_list)
        current_edgeID: str = traci.vehicle.getRoadID(current_vehID)
        if not agent_by_current_vehID.get_created_time_flg():
            agent_by_current_vehID.set_created_time(traci.simulation.getTime())
            agent_by_current_vehID.set_created_time_flg(True)

        # edgeについた瞬間に到着とみなす
        if current_edgeID == "E17" or current_edgeID == "E12" or current_edgeID == "E37":
            vehInfo_by_current_vehID.set_arrival_flag(True)
            vehInfo_by_current_vehID.set_decline_edge_arrival_flag(True)

        # 到着処理 # 到着によってparked_flagがTrue
        if traci.vehicle.isStoppedParking(current_vehID) and not vehInfo_by_current_vehID.get_parked_flag():
            handle_arrival(
                            current_vehID=current_vehID,
                            vehInfo_by_current_vehID=vehInfo_by_current_vehID,
                            agent_by_current_vehID=agent_by_current_vehID,
                            shelter_for_current_vehID=shelter_for_current_vehID,
                            shelter_list=shelter_list,
                            arrival_time_list=arrival_time_list,
                            arrival_time_by_vehID_dict=arrival_time_by_vehID_dict
                            )
        
        if not vehInfo_by_current_vehID.get_decline_edge_arrival_flag():  # 減速処理を行う
            #避難地に接続する(直前の)道路にいる場合、減速する
            pre_edgeID_near_shelter_flag = \
                    utilities.is_pre_edgeID_near_shelter(
                                                            current_edgeID=current_edgeID, 
                                                            edgeID_near_shelter=vehInfo_by_current_vehID.get_edgeID_connect_target_shelter(),
                                                            custome_edge_list=custome_edge_list
                                                            )
            if pre_edgeID_near_shelter_flag and not vehInfo_by_current_vehID.get_decline_edge_arrival_flag():
                # 避難地直前のエッジに入った車両だけ減速制御
                local_density = utilities.get_local_density(vehID=current_vehID, radius=50.0)
                utilities.apply_gap_density_speed_control(
                                                            vehID=current_vehID,
                                                            local_density=local_density,
                                                            v_free=6.0,     # 自由流速度
                                                            v_min=2.0,      # 最低速度
                                                            gap_min=7.0,    # 強い減速を始めるギャップ
                                                            tau=1.8,        # ギャップ→速度変換の傾き
                                                            alpha=0.5,      # 平滑化
                                                            slow_time=1.0   # 速度変更時間
                                                            )
            else:
                # それ以外は自由流走行
                traci.vehicle.slowDown(current_vehID, 8.0, 1.0)

        if not vehInfo_by_current_vehID.get_arrival_flag(): # 未到着の車両に対して処理を実行
            # 通信可能範囲内にいる車両と通信を行う　通信可能範囲は100m設定になる
            if traci.simulation.getTime() % 10 == 0:
                around_vehIDs: list = utilities.get_around_vehIDs(target_vehID=current_vehID, custome_edge_list=custome_edge_list)
                utilities.v2v_communication(
                                            target_vehID=current_vehID, 
                                            target_vehInfo=vehInfo_by_current_vehID, 
                                            around_vehIDs=around_vehIDs,
                                            agent_list=agent_list,
                                            vehInfo_list=vehInfo_list,
                                            COMMUNICATION_RANGE=COMM_RANGE
                                            )

                utilities.v2shelter_communication(
                                                    target_vehID=current_vehID, 
                                                    shelterID=vehInfo_by_current_vehID.get_target_shelter(),
                                                    vehInfo_list=vehInfo_list,
                                                    shelter_list=shelter_list,
                                                    COMMUNICATION_RANGE=COMM_RANGE
                                                    )

            # ドライバーの心理による行動決定
            # 正常性バイアスによる経路変更の実装
            if not vehInfo_by_current_vehID.get_arrival_flag() and utilities.is_vehID_in_congested_edge(vehID=current_vehID, THRESHOLD_SPEED=THRESHOLD_SPEED):
                traci.vehicle.setColor(current_vehID, (180, 0, 0, 255))
                if traci.simulation.getTime() % 5 == 0:
                    # ここで経路を変更するかを判定それとも避難地を変更する
                    from_edgeID, shelterID, to_edgeID = utilities.find_alternative_better_choice(
                                                                    current_edgeID=current_edgeID,
                                                                    vehInfo=vehInfo_by_current_vehID,
                                                                    agent=agent_by_current_vehID,
                                                                    shelter_list=shelter_list,
                                                                    custome_edge_list=custome_edge_list
                                                                    )
                    if from_edgeID != "" and shelterID != "" and to_edgeID != "":
                        # print(f"from_edgeID: {from_edgeID} shelterID: {shelterID} to_edgeID: {to_edgeID}")
                        NEW_VEHICLE_COUNT = utilities.generate_new_veh_based_on_route_time(
                                                                            target_vehID=current_vehID, 
                                                                            NEW_VEHICLE_COUNT= NEW_VEHICLE_COUNT, 
                                                                            agent_list=agent_list, 
                                                                            vehInfo_list=vehInfo_list,
                                                                            vehInfo_by_target_vehID=vehInfo_by_current_vehID, 
                                                                            agent_by_target_vehID=agent_by_current_vehID, 
                                                                            from_edgeID=from_edgeID,
                                                                            new_shelterID=shelterID,
                                                                            to_edgeID=to_edgeID
                                                                            )

            # 同調性バイアスによる経路変更の実装
            # if (not vehInfo_by_current_vehID.get_arrival_flag() 
            #     and utilities.is_vehIDs_changed_evaciation(target_vehID=current_vehID, vehInfo_list=vehInfo_list) 
            #     and utilities.random_true(0.25)):

                # if traci.simulation.getTime() % 5 == 0:
                #     from_edgeID, shelterID, to_edgeID = utilities.find_alternative_route_better(
                #                                                     current_edgeID=current_edgeID,
                #                                                     vehInfo=vehInfo_by_current_vehID,
                #                                                     agent=agent_by_current_vehID,
                #                                                     shelter_list=shelter_list,
                #                                                     custome_edge_list=custome_edge_list,
                #                                                     )
                #     NEW_VEHICLE_COUNT = utilities.generate_new_veh(
                #                                                 target_vehID=current_vehID, 
                #                                                 NEW_VEHICLE_COUNT= NEW_VEHICLE_COUNT, 
                #                                                 agent_list=agent_list, 
                #                                                 vehInfo_list=vehInfo_list,
                #                                                 vehInfo_by_target_vehID=vehInfo_by_current_vehID, 
                #                                                 agent_by_target_vehID=agent_by_current_vehID, 
                #                                                 shelter_list=shelter_list,
                #                                                 connected_edges_list=connected_edges_list,
                #                                                 LATE_AGENT_THRESHOLD_LIST=LATE_AGENT_THRESHOLD_LIST
                #                                                 )
    
    # ルートごとの避難所要時間を計算する
    utilities.calculate_avg_evac_time_by_route(shelter_list=shelter_list)
    # shelterごとで情報を共有する
    utilities.merge_route_info_within_shelters(shelter_list[0], shelter_list[1])
    # utilities.merge_route_info_within_shelters(shelter_list[2], shelter_list[3])
    # shelterごとに到着した車両の数を共有する
    utilities.merge_arrival_vehs_of_shelter(shelter_list=shelter_list)
    for shelter in shelter_list:
        shelter.update_congestion_rate()

def handle_arrival(current_vehID, vehInfo_by_current_vehID:VehicleInfo, agent_by_current_vehID:Agent,
                    shelter_for_current_vehID:Shelter, shelter_list,
                    arrival_time_list, arrival_time_by_vehID_dict):
    """
    避難地到着時の処理
    """
    arrival_time_list.append(traci.simulation.getTime())
    arrival_time_by_vehID_dict[f"{current_vehID}"] = traci.simulation.getTime()
    traci.vehicle.setSpeed(current_vehID, 9.0)
    # 避難地オブジェクトに登録
    shelter_for_current_vehID.add_arrival_vehID(current_vehID)
    vehInfo_by_current_vehID.set_evac_end_time(traci.simulation.getTime())
    # 近傍エッジから避難地オブジェクトを取得して避難時間を更新
    shelter: Shelter = utilities.find_shelter_by_edgeID_connect_target_shelter(
        agent_by_current_vehID.get_near_edgeID_by_target_shelter(), shelter_list
    )
    departure_time = traci.vehicle.getDeparture(current_vehID) + 100
    shelter.update_evac_time_default_dict(
        vehID=current_vehID,
        route=traci.vehicle.getRoute(current_vehID),
        evac_time=vehInfo_by_current_vehID.get_evac_end_time() - departure_time
    )
    # 到着フラグと駐車フラグを更新
    vehInfo_by_current_vehID.set_parked_flag(True)
    agent_by_current_vehID.set_arrival_time(traci.simulation.getTime())
    elapsed_time_list.append(traci.simulation.getTime() - agent_by_current_vehID.get_created_time())

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

def get_options():
    optParser = optparse.OptionParser()
    optParser.add_option(
                            "--nogui", action="store_true",
                            default=False, 
                            help="run the commandline version of sumo"
                            )
    options, args = optParser.parse_args()
    return options


import sys as _sys
if _sys.version_info >= (3, 11):
    import tomllib as _toml_loader
else:
    try:
        import tomli as _toml_loader
    except ImportError as e:
        raise ImportError("Python 3.10以下では `pip install tomli` が必要です。") from e

def load_toml(path: Path) -> dict:
    with path.open("rb") as f:
        return _toml_loader.load(f)

def _req(cfg: dict, key: str, typ=float):
    if key not in cfg:
        raise KeyError(f"Config missing required key: '{key}'")
    return typ(cfg[key])

def parse_runner_args(argv=None):
    parser = argparse.ArgumentParser(
        description="runner_simulator using TOML config",
        add_help=False  # get_options() と競合しないように独自help無効化
    )
    parser.add_argument("--config", required=True, help="Path to TOML config file")
    parser.add_argument("--nogui", action="store_true", help="Run SUMO without GUI")
    parser.add_argument("--early-rate", type=float, help="Override early_rate value from TOML")

    # argvを明示的に渡す（Noneならsys.argv[1:]）
    args, remaining = parser.parse_known_args(argv)
    return args, remaining

if __name__ == "__main__":
    # python3 -m scenarios.pervehicle.map_one.simulation.runner_simulator --nogui scenarios/pervehicle/configs/1.toml 0.5 
    toml_path = sys.argv[2]
    early_rate:float= float(sys.argv[3]) 

    options = get_options()
    if options.nogui:
        sumoBinary = checkBinary('sumo')
    else:
        sumoBinary = checkBinary('sumo-gui')
    cfg = load_toml(Path(toml_path))
    print(f"Loaded config: {cfg}")

    COMM_RANGE: float = _req(cfg, "comm_range", float)
    NUM_VEHICLES: int = _req(cfg, "num_vehicles", int)
    VEHICLE_INTERVAL: float = _req(cfg, "vehicle_interval", float)
    ACTIVE_ROUTE_CHANGE_MEAN: float = _req(cfg, "active_route_change_mean", float)
    ACTIVE_ROUTE_CHANGE_VAR: float = _req(cfg, "active_route_change_var", float)
    CAUTIOUS_ROUTE_CHANGE_MEAN: float = _req(cfg, "cautious_route_change_mean", float)
    CAUTIOUS_ROUTE_CHANGE_VAR: float = _req(cfg, "cautious_route_change_var", float)
    SHELTER_CAPACITY_THRESHOLD: float = _req(cfg, "shelter_capacity_threshold", float)
    TSUNAMI_SIGN_START_TIME: float = _req(cfg, "tsunami_sign_start_time", float)
    TSUNAMI_SIGN_END_TIME: float = _req(cfg, "tsunami_sign_end_time", float)
    MOTIVATION_THRESHOLD_START: float = _req(cfg, "motivation_threshold_start", float)
    MOTIVATION_THRESHOLD_END: float = _req(cfg, "motivation_threshold_end", float)
    MIN_MOTIVATION_START: float = _req(cfg, "min_motivation_start", float)
    MIN_MOTIVATION_END: float = _req(cfg, "min_motivation_end", float)
    POSITIVE_LANECHANGE_START: float = _req(cfg, "positive_lanechange_start", float)
    POSITIVE_LANECHANGE_END: float = _req(cfg, "positive_lanechange_end", float)
    NEGATIVE_LANECHANGE_START: float = _req(cfg, "negative_lanechange_start", float)
    NEGATIVE_LANECHANGE_END: float = _req(cfg, "negative_lanechange_end", float)
    DRIVER_VISIBILITY_DISTANCE: float = _req(cfg, "driver_visibility_distance", float)
    POSITIVE_MAJORITY_BIAS: float = _req(cfg, "positive_majority_bias", float)
    NEGATIVE_MAJORITY_BIAS: float = _req(cfg, "negative_majority_bias", float)

    # シミュレーションの初期化
    # 避難地の情報をもとに、Shelter一覧を生成
    shelter_capacity_by_ID:dict = {"ShelterA_1": 150, "ShelterA_2": 150, "ShelterB_1": 150}
    edgeID_by_shelterID:dict = {"ShelterA_1": 'E17', "ShelterA_2": 'E37', "ShelterB_1": 'E12'}
    for shelterID, near_edgeID in edgeID_by_shelterID.items():
        shelter_list:list = utilities.init_shelter(
                                                    shelterID=shelterID, 
                                                    shelter_capacity_by_ID=shelter_capacity_by_ID, 
                                                    near_edgeID=near_edgeID, 
                                                    shelter_list=shelter_list
                                                    )
    traci.start(
                [sumoBinary,
                 "-c", str(SUMO_CFG),
                 "--tripinfo-output", "tripinfo.xml"],
                traceFile="traci_log.txt",
                traceGetters=False
                )
    custome_edge_list:list = utilities.init_custom_edge()
    vehicle_start_edges:list = utilities.get_vehicle_start_edges(custome_edge_list=custome_edge_list)
    vehicle_end_edges:list = utilities.get_vehicle_end_edges(custome_edge_list=custome_edge_list)
    # 車両の開始エッジと終了エッジの組み合わせを辞書にする
    vehicle_end_list_by_start_edge_dict:dict = utilities.get_vehicle_end_list_by_start_edge_dict(vehicle_start_edges=vehicle_start_edges, vehicle_end_edges=vehicle_end_edges)
    # 全経路で総当たりをし、通行可能経路を取得しておく。
    connected_edges_list:list = utilities.import_connected_edges_from_json(file_path=str(DATA_DIR / "all_edgeIDs.json"))
    nearest_end_edgeID_by_start_edgeID_dict:dict = utilities.import_start_end_edgeIDs_from_json(file_path=str(DATA_DIR / "start_end_edgeIDs.json"))

    mapping = nearest_end_edgeID_by_start_edgeID_dict
    # 開始エッジごとの目的地確率（対応するend_edge数と一致させる）
    probabilities_by_start_edge = {
        "E13": [1.0],  # for shelter B
        "E21": [0.8, 0.2],       # for shelter A
    }
    vehID_list = []
    start_interval = 5.0
    end_interval = 4.0
    base_vehicle_num_list = []
    extra_vehicle_count = 0
    base_vehicle_num_list.append(100); base_vehicle_num_list.append(200)
    for edge_index, (start_edgeID, end_edgeID_list) in enumerate(mapping.items()):
        # 各開始エッジに割り当てる車両数
        assigned_vehicle_num = base_vehicle_num_list[edge_index] + (1 if edge_index < extra_vehicle_count else 0)
        vehicle_intervals = [start_interval - (start_interval - end_interval) * (i / (assigned_vehicle_num - 1))
                            for i in range(assigned_vehicle_num)]
        # 確率リストを取得（未指定なら一様分布）
        probabilities = probabilities_by_start_edge.get(
            start_edgeID,
            [1.0 / len(end_edgeID_list)] * len(end_edgeID_list)
        )
        # 安全チェック：確率とend_edgeリストの長さ一致を確認
        if len(probabilities) != len(end_edgeID_list):
            raise ValueError(
                f"Probability length mismatch for {start_edgeID}: "
                f"len(probabilities)={len(probabilities)} vs len(end_edgeID_list)={len(end_edgeID_list)}."
            )
        # 出発時刻リセット
        DEPART_TIME = 0.0
        # 車両生成ループ
        for vehicle_index in range(assigned_vehicle_num):
            selected_end_edgeID = utilities.choose_edge_by_probability(edgeID_list=end_edgeID_list, probabilities=probabilities)
            target_shelterID = utilities.find_shelterID_by_edgeID_by_shelterID(edgeID=selected_end_edgeID, edgeID_by_shelterID=edgeID_by_shelterID)

            VEHICLE_NUM, ROUTE_NUM, vehID_list_by_shelter, DEPART_TIME = utilities.generate_simple_init_vehID(
                                                                                                                from_edgeID=start_edgeID,
                                                                                                                to_edgeID=selected_end_edgeID,
                                                                                                                shelterID=target_shelterID,
                                                                                                                generate_interval=vehicle_intervals[vehicle_index],
                                                                                                                generate_route_count=ROUTE_NUM,
                                                                                                                generate_veh_count=VEHICLE_NUM,
                                                                                                                depart_time=DEPART_TIME
                                                                                                            )
            vehID_list.extend(vehID_list_by_shelter)

    # カテゴリのカウント
    categories = [extract_category(v) for v in vehID_list]
    counter = Counter(categories)

    # 合計と割合を表示
    total = sum(counter.values())
    print("出現数:", dict(counter))
    print("割合:")
    for cat in ["A1", "A2", "B1", "B2"]:
        count = counter.get(cat, 0)
        print(f"  {cat}: {count} ({count / total:.2%})")
    # 車両情報の初期化
    vehInfo_list:list = utilities.init_vehicleInfo_list(vehIDs=vehID_list, shelter_list=shelter_list)
    # Agentの初期化
    agent_list:list = utilities.init_agent_list(
                                                vehIDs=vehID_list, 
                                                edgeID_by_shelterID=edgeID_by_shelterID, 
                                                EARLY_AGENT_THRESHOLD_LIST=EARLY_AGENT_THRESHOLD_LIST, 
                                                LATE_AGENT_THRESHOLD_LIST=LATE_AGENT_THRESHOLD_LIST, 
                                                ATTR_RATE=early_rate,
                                                MOTIVATION_THRESHOLD_START=MOTIVATION_THRESHOLD_START,
                                                MOTIVATION_THRESHOLD_END=MOTIVATION_THRESHOLD_END,
                                                MIN_MOTIVATION_START=MIN_MOTIVATION_START,
                                                MIN_MOTIVATION_END=MIN_MOTIVATION_END,
                                                ACTIVE_ROUTE_CHANGE_MEAN=ACTIVE_ROUTE_CHANGE_MEAN,
                                                ACTIVE_ROUTE_CHANGE_VAR=ACTIVE_ROUTE_CHANGE_VAR,
                                                CAUTIOUS_ROUTE_CHANGE_MEAN=CAUTIOUS_ROUTE_CHANGE_MEAN,
                                                CAUTIOUS_ROUTE_CHANGE_VAR=CAUTIOUS_ROUTE_CHANGE_VAR,
                                                POSITIVE_LANECHANGE_START=POSITIVE_LANECHANGE_START,
                                                POSITIVE_LANECHANGE_END=POSITIVE_LANECHANGE_END,
                                                NEGATIVE_LANECHANGE_START=NEGATIVE_LANECHANGE_START,
                                                NEGATIVE_LANECHANGE_END=NEGATIVE_LANECHANGE_END,
                                                POSITIVE_MAJORITY_BIAS=POSITIVE_MAJORITY_BIAS,
                                                NEGATIVE_MAJORITY_BIAS=NEGATIVE_MAJORITY_BIAS
                                                )

    for vehID in traci.vehicle.getIDList():
        traci.vehicle.setMaxSpeed(vehID, 9.0)
    run()
    print(f"mean elapsed_time: {np.mean(elapsed_time_list)}")
    if len(arrival_time_list) == NUM_VEHICLES:
        print("OK all vehs arrived ")
    else:
        print(f"NG all vehs not arrived {len(arrival_time_list)}")
    print("===== Simlation Result Summary =====")
    print(f"arrival_time_by_vehID_dict:{arrival_time_by_vehID_dict}")
    print(f"route_changed_vehicle_count:{ROUTE_CHANGED_VEHICLE_COUNT}")
    print(f"normalcy_bias_route_change_count:{NORMALCY_BIAS_ROUTE_CHANGE_COUNT}")
    print(f"majority_bias_route_change_count:{MAJORITY_BIAS_ROUTE_CHANGE_COUNT}")
    print(f"lane_changed_vehicle_count:{LANE_CHANGED_VEHICLE_COUNT}")
    print(f"info_obtained_lanechange_count:{OBTAIN_INFO_LANE_CHANGE_COUNT}")
    print(f"elapsed_time_lanechange_count:{ELAPSED_TIME_LANE_CHANGE_COUNT}")
    print(f"majority_bias_lanechange_count:{POSITIVE_MAJORITY_BIAS_COUNT}")




