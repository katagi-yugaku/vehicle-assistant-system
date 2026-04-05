from .VehicleInfo import VehicleInfo

class Agent():
    def __init__(self, vehID:str, target_shelter:str, tunning_threshold:int, 
                 route_change_threshold:float, lane_change_init_threshold:float, 
                 normalcy_motivation_increase:float, motivation_decrease_due_to_inactive_neighbors:float,
                 motivation_increase_due_to_following_neighbors:float, lane_minimum_motivation_value:float,
                 shelter_occupancy_rate_threshold:float, vehicle_abandoned_threshold:float, 
                 normalcy_value_about_vehicle_abandonment:float, majority_value_about_vehicle_abandonment:float):
        self.vehID = vehID #　車両ID
        self.target_shelter = target_shelter #　車両が向かう避難所
        self.near_edgeID_by_target_shelter = "" #　車両が向かう避難所に接続するedgeID
        self.target_abandoned_vehID = ""
        self.candidate_edge_by_shelterID = {} #　車両が向かう避難所の候補地
        self.congestion_duration = 0 #　渋滞継続時間
        self.tunning_threshold = tunning_threshold # Agentの行動を変更するstress->tunningへの耐久時間
        self.route_change_threshold = route_change_threshold # Agentの行動を変更するtunnig->change渋滞継続時間の耐久時間
        self.route_change_threhold_list = [] #　耐久時間リスト
        self.lane_change_decision_threshold = lane_change_init_threshold # 車線変更動機付け閾値
        self.minimum_motivation_value = lane_minimum_motivation_value
        self.obtain_info_time = 0.0 # 情報取得時間
        self.motivation_increase_from_info_receive = normalcy_motivation_increase # 情報受信による動機付け増加量
        self.motivation_decrease_due_to_inactive_neighbors = motivation_decrease_due_to_inactive_neighbors # 非活動的な近隣車両による動機付け減少量
        self.motivation_increase_due_to_following_neighbors = motivation_increase_due_to_following_neighbors  # 追従する近隣車両による動機付け増加量
        self.calculated_motivation_value = 0.0
        self.x_elapsed_time_for_lane_change_list = []
        self.y_motivation_value_for_lane_change_list = []
        self.lane_change_xy_dict = {}
        self.created_time = 0.0 # エージェント作成時間
        self.arrival_time = 0.0 # 避難地到着時間
        self.lane_change_time = 0.0 # 車線変更時間
        self.reach_lane_minimum_motivation_time = 0.0 # 車線変更動機付けの最小値に到達した時間
        self.vehicle_abandoned_threshold = vehicle_abandoned_threshold # 車両放棄閾値
        self.normalcy_value_about_vehicle_abandonment = normalcy_value_about_vehicle_abandonment # 車両放棄に対する正常性バイアス
        self.majority_value_about_vehicle_abandonment = majority_value_about_vehicle_abandonment # 車両放棄に対する同調性バイアス
        self.tsunami_info_obtaiend_time = 0.0 # 津波情報取得時間
        self.vehicle_abandoned_time = 0.0
        self.shelter_occupancy_rate_threshold = shelter_occupancy_rate_threshold # 避難所の混雑率
        self.created_time_flg = False # エージェント作成時間設定フラグ
        self.shelter_flg = False #　駐車フラグ
        self.shelter_changed_flg = False #　避難地変更フラグ
        self.shelter_full_flg = False # 避難所満杯フラグ
        self.evacuation_route_changed_flg = False # 避難ルート変更フラグ
        self.normalcy_lane_change_motivation_flg = False # 通常時の車線変更動機付けフラグ
        self.acceleration_flag = False # 加速フラグ
        self.lane_minimum_motivation_value_flg = False # 車線変更動機付けの最小値
        self.vehicle_abandoned_flg = False # 車両放棄フラグ
        self.encounted_congestion_flg = False # 渋滞に遭遇したフラグ
        self.avoiding_abandoned_vehicle_flg = False # 車両放棄回避フラグ
        self.arrival_shelter_flg = False # 避難所到着フラグ
        self.tsunami_info_obtaiend_flg = False # 津波情報取得フラグ

    # ドライバーの現在の車線変更動機付け値の更新
    def update_calculated_motivation_value(self, current_time:float):
        # TODO ここが間違ってる
        current_motivation_value = self.get_lane_change_xy_dict().get(current_time)
        # print(f"current_time: {current_time}, current_motivation_value: {current_motivation_value}") 
        self.set_calculated_motivation_value(current_motivation_value)

    #　渋滞継続時間の更新
    def update_congestion_duration(self):
        self.congestion_duration += 1

    #　渋滞継続時間のリセット
    def reset_congestion_duration(self):
        self.congestion_duration = 0

    # 候補避難地の初期設定
    def init_set_candidate_near_shelter(self, shelter_edge_by_IDs:dict):
        self.set_candidate_shelter(shelter_edge_by_IDs)
    
    # 避難地のの混雑度の初期設定
    def init_set_shelter_occupancy_rate_dict(self):
        shelterIDs = self.get_candidate_edge_by_shelterID().keys()
        for shelterID in shelterIDs:
            self.shelter_occupancy_rate_dict[shelterID] = 0.0
    
    def update_shelter_occupancy_rate_dict(self, shelterID:str, occupancy_rate:float):
        self.shelter_occupancy_rate_dict[shelterID] = occupancy_rate

    # 避難地の候補地を更新
    def update_candidate_edge_by_shelterID(self, vehInfo:VehicleInfo):
        if not self.get_candidate_edge_by_shelterID() == {}:
            print(f'shelterID: {shelterID}, near_edge: {near_edge}  vehInfo.get_congestion_level_by_shelter(shelterID): {vehInfo.get_congestion_level_by_shelter(shelterID)}')
        # 混雑情報を取得
        new_candidate_edge_by_shelterID = {}
        for shelterID, near_edge in self.get_candidate_edge_by_shelterID().items():
            # 候補地ごとの避難地の混雑情報を取得し、満杯でない避難地のみを残す
            if not vehInfo.get_congestion_level_by_shelter(shelterID) > 0.99:
                new_candidate_edge_by_shelterID[shelterID] = near_edge
        # 候補地が0になってしまう場合は、現在の避難地を残すように変更
        self.set_candidate_shelter(new_candidate_edge_by_shelterID)

    #　車両IDの取得・設定
    def get_vehID(self):
        return self.vehID
    def set_vehID(self, vehID:str):
        self.vehID = vehID

    #　車両が向かう避難所の取得・設定
    def get_target_shelter(self):
        return self.target_shelter
    def set_target_shelter(self, target_shelter:str):
        self.target_shelter = target_shelter

    #　車両が向かう避難所に接続するedgeIDの取得・設定
    def get_near_edgeID_by_target_shelter(self):
        return self.near_edgeID_by_target_shelter
    def set_near_edgeID_by_target_shelter(self, near_edgeID_by_target_shelter:str):
        self.near_edgeID_by_target_shelter = near_edgeID_by_target_shelter

    # 車両が向かう避難所に接続するedgeIDの取得・設定
    def get_target_abandoned_vehID(self):
        return self.target_abandoned_vehID
    def set_target_abandoned_vehID(self, target_abandoned_vehID:str):
        self.target_abandoned_vehID = target_abandoned_vehID

    # 車両が向かう避難所の候補地の取得・設定
    def get_candidate_edge_by_shelterID(self):
        return self.candidate_edge_by_shelterID
    def set_candidate_edge_by_shelterID(self, candidate_edge_by_shelterID:dict):
        self.candidate_edge_by_shelterID = candidate_edge_by_shelterID

    # 車両が向かう避難所の候補の取得・設定
    def get_candidate_shelter(self):
        return self.candidate_shelter
    def set_candidate_shelter(self, candidate_shelter:list):
        self.candidate_shelter = candidate_shelter

    #　渋滞継続時間の取得・設定
    def get_congestion_duration(self):
        return self.congestion_duration
    def set_congestion_duration(self, congestion_duration:str):
        self.congestion_duration = congestion_duration

    # Agentの行動を変更するstress->tunningへの耐久時間の取得・設定
    def get_tunning_threshold(self):
        return self.tunning_threshold
    def set_tunning_threshold(self, tunning_threshold:int):
        self.tunning_threshold = tunning_threshold

    # Agentの行動を変更するtunnig->change渋滞継続時間の耐久時間の取得・設定
    def get_route_change_threshold(self):
        return self.route_change_threshold
    def set_route_change_threshold(self, route_change_threshold:int):
        self.route_change_threshold = route_change_threshold

    # 現在の車線変更動機付けの取得・設定
    def get_lane_change_decision_threshold(self):
        return self.lane_change_decision_threshold
    def set_lane_change_decision_threshold(self, lane_change_decision_threshold:float):
        self.lane_change_decision_threshold = lane_change_decision_threshold

    # 最小車線変更動機付け値の取得・設定
    def get_minimum_motivation_value(self):
        return self.minimum_motivation_value
    def set_minimum_motivation_value(self, minimum_motivation_value:float):
        self.minimum_motivation_value = minimum_motivation_value

    # 情報取得時間
    def get_obtain_info_time(self):
        return self.obtain_info_time
    def set_obtain_info_time(self, obtain_info_time:float):
        self.obtain_info_time = obtain_info_time

    # 情報受信による動機付け増加量の取得・設定
    def get_motivation_increase_from_info_receive(self):
        return self.motivation_increase_from_info_receive
    def set_motivation_increase_from_info_receive(self, motivation_increase_from_info_receive:float):
        self.motivation_increase_from_info_receive = motivation_increase_from_info_receive

    # 非活動的な近隣車両による動機付け減少量の取得・設定
    def get_motivation_decrease_due_to_inactive_neighbors(self):
        return self.motivation_decrease_due_to_inactive_neighbors
    def set_motivation_decrease_due_to_inactive_neighbors(self, motivation_decrease_due_to_inactive_neighbors:float):
        self.motivation_decrease_due_to_inactive_neighbors = motivation_decrease_due_to_inactive_neighbors

    # 追従する近隣車両による動機付け増加量の取得・設定
    def get_motivation_increase_due_to_following_neighbors(self):
        return self.motivation_increase_due_to_following_neighbors
    def set_motivation_increase_due_to_following_neighbors(self, motivation_increase_due_to_following_neighbors:float):
        self.motivation_increase_due_to_following_neighbors = motivation_increase_due_to_following_neighbors

    # 現在の動機付け値の取得・設定
    def get_calculated_motivation_value(self):
        return self.calculated_motivation_value
    def set_calculated_motivation_value(self, calculated_motivation_value:float):
        self.calculated_motivation_value = calculated_motivation_value

    # 車線変更x座標の取得・設定
    def get_x_elapsed_time_for_lane_change_list(self):
        return self.x_elapsed_time_for_lane_change_list
    def set_x_elapsed_time_for_lane_change_list(self, x_elapsed_time_for_lane_change_list:list):
        self.x_elapsed_time_for_lane_change_list = x_elapsed_time_for_lane_change_list
    def append_x_elapsed_time_for_lane_change_list(self, value:float):
        self.x_elapsed_time_for_lane_change_list.append(value)
    def pop_x_elapsed_time_for_lane_change_list(self):
        return self.x_elapsed_time_for_lane_change_list.pop()

    # 車線変更y座標の取得・設定
    def get_y_motivation_value_for_lane_change_list(self):
        return self.y_motivation_value_for_lane_change_list
    def set_y_motivation_value_for_lane_change_list(self, y_motivation_value_for_lane_change_list:list):
        self.y_motivation_value_for_lane_change_list = y_motivation_value_for_lane_change_list
    def append_y_motivation_value_for_lane_change_list(self, value:float):
        self.y_motivation_value_for_lane_change_list.append(value)
    def pop_y_motivation_value_for_lane_change_list(self):
        return self.y_motivation_value_for_lane_change_list.pop()

    # 車線変更xy座標辞書の取得・設定
    def get_lane_change_xy_dict(self):
        return self.lane_change_xy_dict
    def set_lane_change_xy_dict(self, lane_change_xy_dict:dict):
        self.lane_change_xy_dict = lane_change_xy_dict

    # エージェント作成時間の取得・設定
    def get_created_time(self):
        return self.created_time
    def set_created_time(self, created_time:int):
        self.created_time = created_time

    # 避難地到着時間の取得・設定
    def get_arrival_time(self):
        return self.arrival_time
    def set_arrival_time(self, arrival_time:int):
        self.arrival_time = arrival_time
    
    # 避難所の混雑率
    def get_shelter_occupancy_rate_threshold(self):
        return self.shelter_occupancy_rate_threshold
    def set_shelter_occupancy_rate_threshold(self, shelter_occupancy_rate_threshold:float):
        self.shelter_occupancy_rate_threshold = shelter_occupancy_rate_threshold

    # エージェント作成時間設定フラグの取得・設定
    def get_created_time_flg(self):
        return self.created_time_flg
    def set_created_time_flg(self, created_time_flg:bool):
        self.created_time_flg = created_time_flg

    # 車線変更時間の取得・設定
    def get_lane_change_time(self):
        return self.lane_change_time
    def set_lane_change_time(self, lane_change_time:float):
        self.lane_change_time = lane_change_time

    # 車線変更動機付けの最小値に到達した時間の取得・設定
    def get_reach_lane_minimum_motivation_time(self):
        return self.reach_lane_minimum_motivation_time
    def set_reach_lane_minimum_motivation_time(self, reach_lane_minimum_motivation_time:float):
        self.reach_lane_minimum_motivation_time = reach_lane_minimum_motivation_time
    
    # 車両放棄閾値の取得・設定
    def get_vehicle_abandoned_threshold(self):
        return self.vehicle_abandoned_threshold
    def set_vehicle_abandoned_threshold(self, vehicle_abandoned_threshold:float):
        self.vehicle_abandoned_threshold = vehicle_abandoned_threshold

    # 車両乗り捨てに関する正常性バイアスの取得・設定
    def get_normalcy_value_about_vehicle_abandonment(self):
        return self.normalcy_value_about_vehicle_abandonment
    def set_normalcy_value_about_vehicle_abandonment(self, normalcy_value_about_vehicle_abandonment:float):
        self.normalcy_value_about_vehicle_abandonment = normalcy_value_about_vehicle_abandonment
    
    # 車両乗り捨てに関する同調性バイアスの取得・設定
    def get_majority_value_about_vehicle_abandonment(self):
        return self.majority_value_about_vehicle_abandonment
    def set_majority_value_about_vehicle_abandonment(self, majority_value_about_vehicle_abandonment:float):
        self.majority_value_about_vehicle_abandonment = majority_value_about_vehicle_abandonment
    
    # 津波情報取得時間の取得・設定
    def get_tsunami_info_obtaiend_time(self):
        return self.tsunami_info_obtaiend_time
    def set_tsunami_info_obtaiend_time(self, tsunami_info_obtaiend_time:float):
        self.tsunami_info_obtaiend_time = tsunami_info_obtaiend_time
    
    # 車両乗り捨て時刻
    def get_vehicle_abandoned_time(self):
        return self.vehicle_abandoned_time
    def set_vehicle_abandoned_time(self, vehicle_abandoned_time:float):
        self.vehicle_abandoned_time = vehicle_abandoned_time

    #　駐車フラグの取得・設定
    def get_shelter_flg(self):
        return self.shelter_flg
    def set_shelter_flg(self, shelter_flg:bool):
        self.shelter_flg = shelter_flg

    #　避難地変更フラグの取得・設定
    def get_shelter_changed_flg(self):
        return self.shelter_changed_flg
    def set_shelter_changed_flg(self, shelter_changed_flg:bool):
        self.shelter_changed_flg = shelter_changed_flg
    
    #　避難所満杯フラグの取得・設定
    def get_shelter_full_flg(self):
        return self.shelter_full_flg
    def set_shelter_full_flg(self, shelter_full_flg:bool):
        self.shelter_full_flg = shelter_full_flg
    
    # 避難ルート変更フラグの取得・設定
    def get_evacuation_route_changed_flg(self):
        return self.evacuation_route_changed_flg
    def set_evacuation_route_changed_flg(self, evacuation_route_changed_flg:bool):
        self.evacuation_route_changed_flg = evacuation_route_changed_flg
    
    # 通常時の車線変更動機付けフラグの取得・設定
    def get_normalcy_lane_change_motivation_flg(self):
        return self.normalcy_lane_change_motivation_flg
    def set_normalcy_lane_change_motivation_flg(self, normalcy_lane_change_motivation_flg:bool):
        self.normalcy_lane_change_motivation_flg = normalcy_lane_change_motivation_flg
    
    # 加速フラグの取得・設定
    def get_acceleration_flag(self):
        return self.acceleration_flag
    def set_acceleration_flag(self, acceleration_flag:bool):
        self.acceleration_flag = acceleration_flag
    
    # 車線変更動機付けの最小値フラグの取得・設定
    def get_lane_minimum_motivation_value_flg(self):
        return self.lane_minimum_motivation_value_flg
    def set_lane_minimum_motivation_value_flg(self, lane_minimum_motivation_value_flg:bool):
        self.lane_minimum_motivation_value_flg = lane_minimum_motivation_value_flg

    # 車両放棄フラグの取得・設定
    def get_vehicle_abandoned_flg(self):
        return self.vehicle_abandoned_flg
    def set_vehicle_abandoned_flg(self, vehicle_abandoned_flg:bool):
        self.vehicle_abandoned_flg = vehicle_abandoned_flg

    # 渋滞に遭遇したフラグの取得・設定
    def get_encounted_congestion_flg(self):
        return self.encounted_congestion_flg
    def set_encounted_congestion_flg(self, encounted_congestion_flg:bool):
        self.encounted_congestion_flg = encounted_congestion_flg

    # 車両放棄回避フラグの取得・設定
    def get_avoiding_abandoned_vehicle_flg(self):
        return self.avoiding_abandoned_vehicle_flg
    def set_avoiding_abandoned_vehicle_flg(self, avoiding_abandoned_vehicle_flg:bool):
        self.avoiding_abandoned_vehicle_flg = avoiding_abandoned_vehicle_flg
    
    # 避難所到着フラグの取得・設定
    def get_arrival_shelter_flg(self):
        return self.arrival_shelter_flg
    def set_arrival_shelter_flg(self, arrival_shelter_flg:bool):
        self.arrival_shelter_flg = arrival_shelter_flg

    # 津波情報取得フラグの取得・設定
    def get_tsunami_info_obtaiend_flg(self):
        return self.tsunami_info_obtaiend_flg
    def set_tsunami_info_obtaiend_flg(self, tsunami_info_obtaiend_flg:bool):
        self.tsunami_info_obtaiend_flg = tsunami_info_obtaiend_flg

    #　エージェントの情報を表示
    def print_info(self):
        print(f'vehID: {self.vehID}, target_shelter: {self.target_shelter}, near_edgeID_by_target_shelter: {self.near_edgeID_by_target_shelter}, candidate_shelter: {self.candidate_shelter}, congestion_duration: {self.congestion_duration}, tunning_threshold: {self.tunning_threshold}, route_change_threshold: {self.route_change_threshold}, following_threshold: {self.following_threshold}, shelter_flg: {self.shelter_flg}, shelter_changed_flg: {self.shelter_changed_flg}')
