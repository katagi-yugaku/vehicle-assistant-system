from .VehicleInfo import VehicleInfo

class Agent():
    def __init__(self, vehID:str, target_shelter:str, tunning_threshold:int, 
                 route_change_threshold:float, lane_change_init_threshold:float, 
                 normalcy_motivation_increase:float, motivation_decrease_due_to_inactive_neighbors:float,
                 motivation_increase_due_to_following_neighbors:float, lane_minimum_motivation_value:float):
        self.vehID = vehID #　車両ID
        self.target_shelter = target_shelter #　車両が向かう避難所
        self.near_edgeID_by_target_shelter = "" #　車両が向かう避難所に接続するedgeID
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
        self.created_time_flg = False # エージェント作成時間設定フラグ
        self.shelter_flg = False #　駐車フラグ
        self.shelter_changed_flg = False #　避難地変更フラグ
        self.evacuation_route_changed_flg = False # 避難ルート変更フラグ
        self.normalcy_lane_change_motivation_flg = False # 通常時の車線変更動機付けフラグ
        self.acceleration_flag = False # 加速フラグ
        self.lane_minimum_motivation_value_flg = False # 車線変更動機付けの最小値

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

    #　エージェントの情報を表示
    def print_info(self):
        print(f'vehID: {self.vehID}, target_shelter: {self.target_shelter}, near_edgeID_by_target_shelter: {self.near_edgeID_by_target_shelter}, candidate_shelter: {self.candidate_shelter}, congestion_duration: {self.congestion_duration}, tunning_threshold: {self.tunning_threshold}, route_change_threshold: {self.route_change_threshold}, following_threshold: {self.following_threshold}, shelter_flg: {self.shelter_flg}, shelter_changed_flg: {self.shelter_changed_flg}')
