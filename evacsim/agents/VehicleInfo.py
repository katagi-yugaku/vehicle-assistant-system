from numpy import double
from collections import defaultdict

# 車両が持つ情報を保存
class VehicleInfo():
    '''
    list_around_vehID -> key: vehID, value[1]: speed, value[2]: time_stamp
    '''
    multi_map = defaultdict(list)
    def __init__(self, vehID:str, target_shelter:str, edgeID_connect_target_shelter:str, create_time: double):
        self._vehID = vehID # 車両ID
        self._target_shelter = target_shelter # 車両が向かう避難所ID
        self._edgeID_connect_target_shelter = edgeID_connect_target_shelter # 車両が向かう避難所edgeID
        self._multiDict_around_vehInfos:defaultdict = defaultdict(list) # key: vehID, value: [speed, time_stamp]
        self._shelter_congestion_info:defaultdict = defaultdict(list) # key: vehID, value: [shelter, congestion]
        self._tsunami_precursor_info:defaultdict = defaultdict(list) # key: vehID, value: [tsunami_precursor_flag, time_stamp]
        self._create_time = create_time
        self._stopping_time = 0
        self._evac_start_time = 0.0 # 交差点1に進入してからの開始時間
        self._evac_end_time = 0.0 # 交差点1に進入してからの終了時間
        self._arrival_flag = False
        self._arrival_time = 0
        self._decline_edge_arrival_flag = False # 車両が避難所の手前のedgeに到着したかどうかのフラグ
        self._parked_flag = False # 車両が駐車しているかどうかのフラグ
        self._avg_evac_time_by_route_from_shelter={} # key:time ルートごとの避難時間を保存する辞書
        self._avg_evac_time_by_route_by_recive_time ={}
        self._stop_flag = False # 車両が停止しているかどうかのフラグ
        self._start_time_measured_flag = False # 車両の避難開始時間を測定するフラグ
        self._agent_changed_flag = False # 車両の避難所が変更されたかどうかのフラグ
        self._vehicle_comm_enabled_flag = True # 車両の通信が有効かどうかのフラグ
        self._approach_edge_dict = {} # 進入路のedge
        self._edgeIDs_within_junction_to_shelter_dict = {} # 避難所までの交差点内のedge辞書
        # self._avg_evac_time_by_route = {} # key: route value: avg_time

    # 車両周辺の車両情報を更新
    def update_around_vehInfos(self, around_vehID:str, speed:double, time_stamp:double):
        self._multiDict_around_vehInfos[around_vehID] = [speed, time_stamp]

    # 特定の周辺車両の最新車速を取得
    def get_latest_speed(self, around_vehID:str):
        return self._multiDict_around_vehInfos[around_vehID][0]

    # 特定の周辺車両の最新タイムスタンプを取得
    def get_latest_time_stamp(self, around_vehID:str):
        return self._multiDict_around_vehInfos[around_vehID][1]

    # 避難地の混雑情報を更新
    def update_shelter_congestion_info(self, shelterID:str, congestion:int, time_stamp:double):
        self._shelter_congestion_info[shelterID] = [congestion, time_stamp]

    # ルートごとの避難所要時間を更新する
    def v2shelter_update_avg_evac_time_by_route(self, avg_evac_time_by_route:dict):
        for route, info in avg_evac_time_by_route.items():
            self._avg_evac_time_by_route_from_shelter[route] = info
    
    # ルートごとの避難時間を更新する
    def v2v_avg_evac_time_by_route_by_recive_time(self, current_time:double):
        # 初期化
        self._avg_evac_time_by_route_by_recive_time = {}
        self._avg_evac_time_by_route_by_recive_time[current_time] = self.get_avg_evac_time_by_route_from_shelter()
    
    # 津波接近予兆情報を更新する
    def update_tsunami_precursor_info(self, vehID:str,  tsunami_precursor_flag:int, current_time:double):
        self._tsunami_precursor_info[vehID] = [tsunami_precursor_flag, current_time]
    
    def has_tsunami_precursor_info(self) -> bool:
        """
        info_dict: defaultdict(list) or dict
        e.g. {0: [], 1: [], 'init_ShelterB_1_35': [True, 234.0]}
        Returns True only if any value list starts with True.
        """
        for key, value in self.get_tsunami_precursor_info().items():
            # 値がリストで、かつ最初の要素が True なら
            if isinstance(value, list) and len(value) > 0 and value[0] is True:
                return True
        return False

    # 特定の避難所の混雑情報の最新タイムスタンプを取得
    def get_latest_time_stamp_of_shelter(self, shelter:str):
        try:
            return self._shelter_congestion_info[shelter][1]
        except Exception as e:
            return 0

    # 特定の周辺車両の最新避難地情報を取得
    def get_congestion_level_by_shelter(self, shelter:str):
        try:
            return self._shelter_congestion_info[shelter][0]
        except Exception as e:
            return 0

    def init_set_congestion_level_by_shelter(self, shelter:str, congestion:double, time_stamp:double):
        self._shelter_congestion_info[shelter] = [congestion, time_stamp]
    
    def init_set_avg_evac_time_by_route_by_recive_time(self):
        self._avg_evac_time_by_route_by_recive_time[0.0] = {}
    
    def init_set_tsunami_precursor_info(self):
        self._tsunami_precursor_info[self.get_vehID()] = [False, 0.0]
    
    def update_stopping_time(self):
        self._stopping_time += 1
    def clear_stopping_time(self):
        self._stopping_time = 0

    # 車両IDのgetter/setter
    def get_vehID(self):
        return self._vehID
    def set_vehID(self, vehID:str):
        self._vehID = vehID

    # 車両が向かう避難所のgetter/setter
    def get_target_shelter(self):
        return self._target_shelter
    def set_target_shelter(self, target_shelter:str):
        self._target_shelter = target_shelter

    # 車両が向かう避難所接続するedgeIDのgetter/setter
    def get_edgeID_connect_target_shelter(self):
        return self._edgeID_connect_target_shelter
    def set_edgeID_connect_target_shelter(self, edgeID_connect_target_shelter :str):
        self._edgeID_connect_target_shelter = edgeID_connect_target_shelter

    # 車両周辺の車両情報のgetter/setter
    def get_multiDict_around_vehIDs(self):
        return self._multiDict_around_vehInfos
    def set_multiDict_around_vehIDs(self, around_vehInfo:defaultdict):
        self._multiDict_around_vehInfos = around_vehInfo

    # 避難地の混雑情報のgetter/setter
    def get_shelter_congestion_info(self):
        return self._shelter_congestion_info
    def set_shelter_congestion_info(self, shelter_congestion_info:defaultdict):
        self._shelter_congestion_info = shelter_congestion_info

    # 津波前兆情報のgetter/setter
    def get_tsunami_precursor_info(self):
        return self._tsunami_precursor_info
    def set_tsunami_precursor_info(self, tsunami_precursor_info:defaultdict):
        self._tsunami_precursor_info = tsunami_precursor_info

    # 車両の生成時間のgetter/setter
    def get_create_time(self):
        return self._create_time
    def set_create_time(self, create_time: double):
        self._create_time = create_time
    
    # 車両の停止時間のgetter/setter
    def get_stopping_time(self):
        return self._stopping_time
    def set_stopping_time(self, stopping_time: double):
        self._stopping_time = stopping_time
    
    def get_evac_start_time(self):
        return self._evac_start_time
    def set_evac_start_time(self, evac_start_time: double):
        self._evac_start_time = evac_start_time

    def get_evac_end_time(self):
        return self._evac_end_time
    def set_evac_end_time(self, evac_end_time: double):
        self._evac_end_time = evac_end_time
    
    # 車両の到着フラグのgetter/setter
    def get_arrival_flag(self):
        return self._arrival_flag
    def set_arrival_flag(self, arrival_flag: bool):
        self._arrival_flag = arrival_flag
    
    # 車両の到着時間のgetter/setter
    def get_arrival_time(self):
        return self._arrival_time
    def set_arrival_time(self, arrival_time: double):
        self._arrival_time = arrival_time
    
    # 車両の避難所手前のedgeに到着したかどうかのフラグのgetter/setter
    def get_decline_edge_arrival_flag(self):
        return self._decline_edge_arrival_flag
    def set_decline_edge_arrival_flag(self, decline_edge_arrival_flag: bool):
        self._decline_edge_arrival_flag = decline_edge_arrival_flag
    
    # 車両の駐車しているかどうかのフラグのgetter/setter
    def get_parked_flag(self):
        return self._parked_flag
    def set_parked_flag(self, parked_flag: bool):
        self._parked_flag = parked_flag

    # 避難地から受け取る車両のルートごとの避難時間のgetter/setter
    def get_avg_evac_time_by_route_from_shelter(self):
        return self._avg_evac_time_by_route_from_shelter
    def set_avg_evac_time_by_route_from_shelter(self, avg_evac_time_by_route_from_shelter:dict):
        self._avg_evac_time_by_route_from_shelter = avg_evac_time_by_route_from_shelter
    
    # 現時点で、車両が持つ避難所のルートごとの避難時間のgetter/setter
    def get_avg_evac_time_by_route_by_recive_time(self):
        return self._avg_evac_time_by_route_by_recive_time
    def set_avg_evac_time_by_route_by_recive_time(self, avg_evac_time_by_route_by_recive_time:dict):
        self._avg_evac_time_by_route_by_recive_time = avg_evac_time_by_route_by_recive_time
    
    # 車両の停止フラグのgetter/setter
    def get_stop_flag(self):
        return self._stop_flag
    def set_stop_flag(self, stop_flag: bool):
        self._stop_flag = stop_flag

    # 車両の避難開始時間を測定するフラグのgetter/setter
    def get_start_time_measured_flag(self):
        return self._start_time_measured_flag
    def set_start_time_measured_flag(self, start_time_measured_flag: bool):
        self._start_time_measured_flag = start_time_measured_flag

    # 避難地を変更したか否かのフラグのgetter/setter
    def get_agent_changed_flag(self):
        return self._agent_changed_flag
    def set_agent_changed_flag(self, agent_changed_flag: bool):
        self._agent_changed_flag = agent_changed_flag
    
    # 車両の通信が有効かどうかのフラグのgetter/setter
    def get_vehicle_comm_enabled_flag(self):
        return self._vehicle_comm_enabled_flag
    def set_vehicle_comm_enabled_flag(self, vehicle_comm_enabled_flag: bool):
        self._vehicle_comm_enabled_flag = vehicle_comm_enabled_flag
    
    # 進入路のedgeリストのgetter/setter
    def get_approach_edge_dict(self):
        return self._approach_edge_dict
    def set_approach_edge_dict(self, approach_edge_dict: dict):
        self._approach_edge_dict = approach_edge_dict

    # 避難所までの交差点内のedge辞書のgetter/setter
    def get_edgeIDs_within_junction_to_shelter_dict(self):
        return self._edgeIDs_within_junction_to_shelter_dict
    def set_edgeIDs_within_junction_to_shelter_dict(self, edgeIDs_within_junction_to_shelter_dict: dict):
        self._edgeIDs_within_junction_to_shelter_dict = edgeIDs_within_junction_to_shelter_dict

    def print_shelter_congestion_info(self):
        print(f"vehID:{self._vehID} has shelter_congestion_info: {self._shelter_congestion_info}")

    def print_has_congestion_info(self):
        print(f"vehID:{self._vehID} has congestion_info: {self._shelter_congestion_info}")