import os
import sys
import optparse
if 'SUMO_HOME' in os.environ:
    sys.path.append(os.path.join(os.environ['SUMO_HOME'], 'tools'))
from sumolib import checkBinary  # noqa
import traci

from ...agents.Agent import Agent
from ...agents.CustomeEdge import CustomeEdge, ConnectedEdges
from ...agents.VehicleInfo import VehicleInfo
from ...agents.Shelter import Shelter

from numpy import double
from math import sqrt
import random
random.seed(314) # 乱数シードを314に設定
from its102 import utilities
import numpy as np
import matplotlib.pyplot as plt
import copy
# 標準エラー出力を無効化
# sys.stderr = open(os.devnull, 'w')

'''
    すれ違う車両と車両間通信を行う
    正しく通信されるのかのテスト
'''
COMMUNICATION_RANGE = 100
END_SIMULATION_TIME = 1800
VEHICLE_NUM = 0
NEW_VEHICLE_COUNT = 0
ROUTE_NUM = 0
SPEED_ARRANGE = 1
CONGESTION_RATE = 0.3
INSIGHT_RANGE = 15 
SHOW_DEBUG_COUNT = 0
THRESHOLD_SPEED = 5.0
STOPPING_TIME_IN_SHELTER = 15
EARLY_AGENT_THRESHOLD_LIST = [60, 90, 100, 130] # 早期決断者の閾値
LATE_AGENT_THRESHOLD_LIST = [180, 220, 300, 350] # 遅延決断者の閾値
VEH_START_TIME_BY_SHELTERID = {"ShelterA": 0, "ShelterB": 0}
VEHNUM_TO_SHELTER_BY_SHELTERID = {"ShelterA": 40, "ShelterB": 60}
full_time_for_shelterA = 0
count = 0

# リストの初期化
custome_edge_list:list = []
shelter_list = []
vehInfo_list = []
vehID_list = []
veh_written_list = []
connected_edges_list = []
shelterA_arrival_time_list = []
shelterB_arrival_time_list = []

def get_options():
    optParser = optparse.OptionParser()
    optParser.add_option("--nogui", action="store_true",
                         default=False, help="run the commandline version of sumo")
    options, args = optParser.parse_args()
    return options

# this is the main entry point of this script
if __name__ == "__main__":
    options = get_options()
    file_path = "data/two_route_one_evac.rou.xml"
    if options.nogui:
        sumoBinary = checkBinary('sumo')
    else:
        sumoBinary = checkBinary('sumo-gui')

    # 避難地の情報をもとに、Shelter一覧を生成
    shelter_capacity_by_ID:dict = {"ShelterA": 40, "ShelterB": 60}
    shelter_edge_by_IDs:dict = {"ShelterA": 'E26', "ShelterB": 'E28'}   
    for shelterID, near_edgeID in shelter_edge_by_IDs.items():
        shelter_list:list = utilities.init_shelter(shelterID=shelterID, shelter_capacity_by_ID=shelter_capacity_by_ID, near_edgeID=near_edgeID, shelter_list=shelter_list)
    # 現在のedge情報をもとに、CustomEdge一覧を生成
    traci.start([sumoBinary, "-c", "/opt/homebrew/Cellar/sumo/1.20.0/vehicle-evacuation-system/its102/map_one/data/one_shelter_one_departure.sumocfg",
                             "--tripinfo-output", "tripinfo.xml"], traceFile="traci_log.txt", traceGetters=False)
    # 開始エッジ, 終了エッジ一覧を作成
    custome_edge_list:list = utilities.init_custom_edge()
    vehicle_start_edges:list = utilities.get_vehicle_start_edges(custome_edge_list=custome_edge_list)
    vehicle_end_edges:list = utilities.get_vehicle_end_edges(custome_edge_list=custome_edge_list)
    # 車両の開始エッジと終了エッジの組み合わせを辞書にする
    vehicle_end_list_by_start_edge_dict:dict = utilities.get_vehicle_end_list_by_start_edge_dict(vehicle_start_edges=vehicle_start_edges, vehicle_end_edges=vehicle_end_edges)
    # ここが空
    print(f'length of vehicle_end_list_by_start_edge_dict: {len(vehicle_end_list_by_start_edge_dict)}')
    # 全経路で総当たりをし、通行可能経路を取得しておく。
    connected_edges_list:list = utilities.init_connected_edges_list(custome_edge_list=custome_edge_list)
    print(f"connected_edges_list: {connected_edges_list}")
    # 取得した総当たり情報をjson形式で保存する
    utilities.export_connected_edges_to_json(connected_edges_list=connected_edges_list,
                                                file_path="/opt/homebrew/Cellar/sumo/1.20.0/vehicle-evacuation-system/its102/map_one/data/all_edgeIDs.json")
    
    print(f'length of connected_edges_list: {len(connected_edges_list)}')
    # ここまで空
    nearest_end_edgeID_by_start_edgeID_dict:dict = \
        utilities.get_nearest_end_edgeID_by_start_edgeID(vehicle_end_list_by_start_edge_dict=vehicle_end_list_by_start_edge_dict)
    print(f'nearest_end_edgeID_by_start_edgeID_dict: {nearest_end_edgeID_by_start_edgeID_dict}')
    # 取得した開始地点と終了地点の情報をjson形式で保存する
    utilities.export_start_end_edgeIDs_to_json(start_end_edgeIDs=nearest_end_edgeID_by_start_edgeID_dict,
                                                file_path="/opt/homebrew/Cellar/sumo/1.20.0/vehicle-evacuation-system/its102/map_one/data/start_end_edgeIDs.json")
    print(f'find route complete')



