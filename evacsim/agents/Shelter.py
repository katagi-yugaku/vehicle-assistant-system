from numpy import double
from collections import defaultdict

# 避難地の情報をセットするクラス
class Shelter:
    def __init__(self, shelterID:str, capacity:int, near_edgeID:str):
        self._shelterID = shelterID
        self._capacity = capacity
        self.near_edgeID = near_edgeID
        self.arrival_vehID_list = [] # 避難所に到着した車両のIDリスト
        self.congestion_rate = 0.0 # 避難所の混雑度
        self.positoin = (0.0, 0.0)
        self.position_flag = False # 避難所の位置情報を取得したかどうかのフラグ
        self.evacuation_time_from_junction_multidict:defaultdict = defaultdict(list) # 避難にかかった時間　key: vehID, value: [routeID, evac_time]
        self.avg_evac_time_by_route = {}
        self.total_arrival_vehIDs = 0 # 避難所に到着した車両の総数
    
    # 避難所に到着した車両IDを追加する
    def add_arrival_vehID(self, vehID:str):
        self.arrival_vehID_list.append(vehID)
    
    # 避難地の混雑度を更新する
    def update_congestion_rate(self):
        # ガード節
        if self.total_arrival_vehIDs > self._capacity:
            self.congestion_rate = 1.0
        else:
            self.congestion_rate = self.total_arrival_vehIDs / self._capacity
    
    # 車両周辺の車両情報を更新
    def update_evac_time_default_dict(self, vehID:str, route:list, evac_time:double):
        self.evacuation_time_from_junction_multidict[vehID] = [route, evac_time]

    # 避難所IDのgetter/setter
    def get_shelterID(self):
        return self._shelterID
    def set_shelterID(self, shelterID:str):
        self._shelterID = shelterID
    
    # 避難所の収容人数のgetter/setter
    def get_capacity(self):
        return self._capacity
    def set_capacity(self, capacity:int):
        self._capacity = capacity

    # 避難所の最寄りエッジIDのgetter/setter
    def get_near_edgeID(self):
        return self.near_edgeID
    def set_near_edgeID(self, near_edgeID:str):
        self.near_edgeID = near_edgeID
    
    # 避難所に到着した車両のIDリストのgetter/setter
    def get_arrival_vehID_list(self):
        return self.arrival_vehID_list
    def set_arrival_vehID_list(self, arrival_vehID_list:list):
        self.arrival_vehID_list = arrival_vehID_list

    # 避難所の混雑度のgetter/setter
    def get_congestion_rate(self):
        return self.congestion_rate
    def set_congestion_rate(self, congestion_rate:float):
        self.congestion_rate = congestion_rate
    
    # 避難所の位置情報のgetter/setter
    def get_position(self):
        return self.positoin
    def set_position(self, position:tuple):
        self.positoin = position
    
    # 避難所の位置情報のフラグのgetter/setter
    def get_position_flag(self):
        return self.position_flag
    def set_position_flag(self, position_flag:bool):
        self.position_flag = position_flag
    
    # 交差点から避難地までにかかった時間のgetter/setter
    def get_evacuation_time_from_junction_multidict(self):
        return self.evacuation_time_from_junction_multidict
    def set_evacuation_time_from_junction_multidict(self, evacuation_time_from_junction_multidict:defaultdict):
        self.evacuation_time_from_junction_multidict = evacuation_time_from_junction_multidict
    
    # routeごとの避難時間のgetter/setter
    def get_avg_evac_time_by_route(self):
        return self.avg_evac_time_by_route
    def set_avg_evac_time_by_route(self, avg_evac_time_by_route:dict):
        self.avg_evac_time_by_route = avg_evac_time_by_route
    
    # 避難所に到着した車両の総数のgetter/setter
    def init_total_arrival_vehIDs(self):
        self.total_arrival_vehIDs = 0

    def get_total_arrival_vehIDs(self):
        return self.total_arrival_vehIDs
    def set_total_arrival_vehIDs(self, total_arrival_vehIDs:int):
        self.total_arrival_vehIDs = total_arrival_vehIDs

    def print_shelter_info(self):
        print(f'shelterID: {self._shelterID}, capacity: {self._capacity}, near_edgeID: {self.near_edgeID}')
