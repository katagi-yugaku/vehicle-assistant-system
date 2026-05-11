from .VehicleInfo import VehicleInfo

from typing import Any


class Agent:
    def __init__(
    self,
    vehID: str,
    target_shelter: str,
    route_change_threshold: float,
    wrong_way_driving_threshold: float,
    wrong_way_driving_min_motivation_value: float,
    vehicle_abandoned_threshold: float,
    normalcy_value_about_tsunami_precursor_info: float,
    normalcy_value_about_route_congestion_info: float,
    normalcy_value_about_shelter_full_info: float,
    majority_value_increase: float,
    majority_value_decrease: float,
    shelter_occupancy_rate_threshold: float,
    ):
        # ==================================================
        # 1. 基本情報
        # ==================================================
        self.vehID: str = vehID
        self.target_shelter: str = target_shelter
        self.near_edgeID_by_target_shelter: str = ""
        self.created_time: float = 0.0
        self.arrival_time: float = 0.0

        # ==================================================
        # 2. 避難所・経路に関する情報
        # ==================================================
        self.candidate_edge_by_shelterID: dict[str, Any] = {}
        self.candidate_shelter: dict[str, Any] = {}
        self.shelter_occupancy_rate_dict: dict[str, float] = {}

        self.shelter_occupancy_rate_threshold: float = (
            shelter_occupancy_rate_threshold
        )

        self.target_abandoned_vehID: str = ""
        self.reserved_vehicle_abandonment_edgeID: str = ""

        # ==================================================
        # 3. 渋滞・経路変更に関するパラメータ
        # ==================================================
        self.congestion_duration: float = 0.0
        self.encounted_congestion_time: float = 0.0
        self.route_change_threshold: float = route_change_threshold

        # ==================================================
        # 4. 逆走行為に関するパラメータ・状態
        # ==================================================
        self.wrong_way_driving_threshold: float = wrong_way_driving_threshold
        self.wrong_way_driving_min_motivation_value: float = (
            wrong_way_driving_min_motivation_value
        )
        self.wrong_way_driving_encounted_other_vehicle_time: float = -1.0

        self.lane_change_xy_dict: dict[float, Any] = {}
        self.x_elapsed_time_for_lane_change_list: list[float] = []
        self.x_elapsed_time: list[float] = []

        # ==================================================
        # 5. 情報取得時刻
        # ==================================================
        self.obtain_info_time: float = 0.0

        self.tsunami_info_obtained_time: float = 1_000_000.0
        self.route_congestion_info_obtained_time: float = 0.0
        self.shelter_full_info_obtained_time: float = 0.0

        # ==================================================
        # 6. 正常性バイアス
        # ==================================================
        # 津波接近情報に対する正常性バイアス
        self.normalcy_value_about_tsunami_precursor_info: float = (
            normalcy_value_about_tsunami_precursor_info
        )
        # 経路変更情報に対する正常性バイアス
        self.normalcy_value_about_route_congestion_info: float = (
            normalcy_value_about_route_congestion_info
        )
        # 満杯情報に対する正常性バイアス
        self.normalcy_value_about_shelter_full_info: float = (
            normalcy_value_about_shelter_full_info
        )


        # ==================================================
        # 7. 同調性バイアス
        # ==================================================
        self.majority_value_increase: float = majority_value_increase
        self.majority_value_decrease: float = majority_value_decrease

        # ==================================================
        # 8. 動機付け値・履歴
        # ==================================================
        self.calculated_motivation_value: float = 0.0

        self.base_motivation_value_by_elapsed_time_dict: dict[float, float] = {}

        self.tsunami_precursor_normalcy_value_by_elapsed_time_dict: dict[
            float, float
        ] = {}
        self.route_congestion_normalcy_value_by_elapsed_time_dict: dict[
            float, float
        ] = {}
        self.shelter_full_normalcy_value_by_elapsed_time_dict: dict[
            float, float
        ] = {}

        # ==================================================
        # 9. 車両乗り捨てに関するパラメータ・状態
        # ==================================================
        self.vehicle_abandoned_threshold: float = vehicle_abandoned_threshold
        self.vehicle_abandoned_time: float = 0.0

        # ==================================================
        # 10. 各種フラグ
        # ==================================================

        # --- エージェント・避難所関連 ---
        self.created_time_flg: bool = False
        self.shelter_flg: bool = False
        self.shelter_changed_flg: bool = False
        self.shelter_full_flg: bool = False
        self.arrival_shelter_flg: bool = False
        self.agent_action_name: str = ""

        # --- 経路・逆走関連 ---
        self.evacuation_route_changed_flg: bool = False
        self.wrong_way_driving_flg: bool = False
        self.normalcy_lane_change_motivation_flg: bool = False
        self.lane_minimum_motivation_value_flg: bool = False
        self.acceleration_flag: bool = False

        # --- 車両乗り捨て関連 ---
        self.vehicle_abandoned_flg: bool = False
        self.avoiding_abandoned_vehicle_flg: bool = False
        self.failed_vehicle_abandonment_flg: bool = False

        # --- 渋滞関連 ---
        self.encounted_congestion_flg: bool = False

        # --- 情報取得関連 ---
        self.tsunami_info_obtained_flg: bool = False
        self.route_congestion_info_obtained_flg: bool = False
        self.shelter_full_info_obtained_flg: bool = False

    # ドライバーの現在の車線変更動機付け値の更新
    def update_calculated_motivation_value(self, current_time:float):
        # TODO ここが間違ってる
        current_motivation_value = self.get_lane_change_xy_dict().get(current_time)
        # print(f"current_time: {current_time}, current_motivation_value: {current_motivation_value}") 
        self.set_calculated_motivation_value(current_motivation_value)

    #　渋滞継続時間の更新
    def update_congestion_duration(self, time_increment:float):
        self.congestion_duration += time_increment
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

    #　エージェントの情報を表示
    def print_info(self):
        print(f'vehID: {self.vehID}, target_shelter: {self.target_shelter}, near_edgeID_by_target_shelter: {self.near_edgeID_by_target_shelter}, candidate_shelter: {self.candidate_shelter}, congestion_duration: {self.congestion_duration}, tunning_threshold: {self.tunning_threshold}, route_change_threshold: {self.route_change_threshold}, following_threshold: {self.following_threshold}, shelter_flg: {self.shelter_flg}, shelter_changed_flg: {self.shelter_changed_flg}')



    # ==================================================
    # 1. 基本情報
    # ==================================================
    #　車両IDの取得・設定
    def get_vehID(self):
        return self.vehID
    def set_vehID(self, vehID:str):
        self.vehID = vehID
    
    # 避難地の取得・設定
    def get_target_shelter(self):
        return self.target_shelter
    def set_target_shelter(self, target_shelter:str):
        self.target_shelter = target_shelter
    
    # 近くのエッジIDの取得・設定
    def get_near_edgeID_by_target_shelter(self):
        return self.near_edgeID_by_target_shelter
    def set_near_edgeID_by_target_shelter(self, near_edgeID_by_target_shelter:str):
        self.near_edgeID_by_target_shelter = near_edgeID_by_target_shelter
    
    # 作成時間の取得・設定
    def get_created_time(self):
        return self.created_time
    def set_created_time(self, created_time:float):
        self.created_time = created_time
    
    # 到着時間の取得・設定
    def get_arrival_time(self):
        return self.arrival_time
    def set_arrival_time(self, arrival_time:float):
        self.arrival_time = arrival_time
    
    # ==================================================
    # 2. 避難所・経路に関する情報
    # ==================================================
    # 避難所IDごとの候補エッジの取得・設定
    def get_candidate_edge_by_shelterID(self):
        return self.candidate_edge_by_shelterID
    def set_candidate_edge_by_shelterID(self, candidate_edge_by_shelterID:dict[str, Any]):
        self.candidate_edge_by_shelterID = candidate_edge_by_shelterID
    
    # 候補避難所の取得・設定
    def get_candidate_shelter(self):
        return self.candidate_shelter
    def set_candidate_shelter(self, candidate_shelter:dict[str, Any]):
        self.candidate_shelter = candidate_shelter
    
    # 避難所占有率の取得・設定
    def get_shelter_occupancy_rate_dict(self):
        return self.shelter_occupancy_rate_dict
    def set_shelter_occupancy_rate_dict(self, shelter_occupancy_rate_dict:dict[str, float]):
        self.shelter_occupancy_rate_dict = shelter_occupancy_rate_dict
    
    # 避難所占有率閾値の取得・設定
    def get_shelter_occupancy_rate_threshold(self):
        return self.shelter_occupancy_rate_threshold
    def set_shelter_occupancy_rate_threshold(self, shelter_occupancy_rate_threshold:float):
        self.shelter_occupancy_rate_threshold = shelter_occupancy_rate_threshold
    
    # 目標乗り捨て車両IDの取得・設定
    def get_target_abandoned_vehID(self):
        return self.target_abandoned_vehID
    def set_target_abandoned_vehID(self, target_abandoned_vehID:str):
        self.target_abandoned_vehID = target_abandoned_vehID
    
    # 予約乗り捨てエッジIDの取得・設定
    def get_reserved_vehicle_abandonment_edgeID(self):
        return self.reserved_vehicle_abandonment_edgeID
    def set_reserved_vehicle_abandonment_edgeID(self, reserved_vehicle_abandonment_edgeID:str):
        self.reserved_vehicle_abandonment_edgeID = reserved_vehicle_abandonment_edgeID
    
    # ==================================================
    # 3. 渋滞・経路変更に関するパラメータ
    # ==================================================
    # 渋滞継続時間の取得・設定 
    def get_congestion_duration(self):
        return self.congestion_duration
    def set_congestion_duration(self, congestion_duration:float):
        self.congestion_duration = congestion_duration
    
    # 遭遇渋滞時間の取得・設定
    def get_encounted_congestion_time(self):
        return self.encounted_congestion_time
    def set_encounted_congestion_time(self, encounted_congestion_time:float):
        self.encounted_congestion_time = encounted_congestion_time
    def updated_encounted_congestion_time(self, time_increment:float):
        self.encounted_congestion_time += time_increment

    
    # 経路変更閾値の取得・設定
    def get_route_change_threshold(self):
        return self.route_change_threshold
    def set_route_change_threshold(self, route_change_threshold:float):
        self.route_change_threshold = route_change_threshold
    
    # ==================================================
    # 4. 逆走行為に関するパラメータ・状態
    # ==================================================
    # 逆走行為閾値の取得・設定
    def get_wrong_way_driving_threshold(self):
        return self.wrong_way_driving_threshold
    def set_wrong_way_driving_threshold(self, wrong_way_driving_threshold:float):
        self.wrong_way_driving_threshold = wrong_way_driving_threshold
    
    # 逆走行為最小動機付け値の取得・設定
    def get_wrong_way_driving_min_motivation_value(self):
        return self.wrong_way_driving_min_motivation_value
    def set_wrong_way_driving_min_motivation_value(self, wrong_way_driving_min_motivation_value:float):
        self.wrong_way_driving_min_motivation_value = wrong_way_driving_min_motivation_value
    
    # 逆走行為で他車両に遭遇した時間の取得・設定
    def get_wrong_way_driving_encounted_other_vehicle_time(self):
        return self.wrong_way_driving_encounted_other_vehicle_time
    def set_wrong_way_driving_encounted_other_vehicle_time(self, wrong_way_driving_encounted_other_vehicle_time:float):
        self.wrong_way_driving_encounted_other_vehicle_time = wrong_way_driving_encounted_other_vehicle_time

    # 車線変更のxy座標辞書の取得・設定
    def get_lane_change_xy_dict(self):
        return self.lane_change_xy_dict
    def set_lane_change_xy_dict(self, lane_change_xy_dict:dict[float, Any]):
        self.lane_change_xy_dict = lane_change_xy_dict
    
    # 車線変更の経過時間リストの取得・設定
    def get_x_elapsed_time_for_lane_change_list(self):
        return self.x_elapsed_time_for_lane_change_list
    def set_x_elapsed_time_for_lane_change_list(self, x_elapsed_time_for_lane_change_list:list[float]):
        self.x_elapsed_time_for_lane_change_list = x_elapsed_time_for_lane_change_list
    
    # 経過時間リストの取得・設定
    def get_x_elapsed_time_list(self):
        return self.x_elapsed_time_list
    def set_x_elapsed_time_list(self, x_elapsed_time_list:list[float]):
        self.x_elapsed_time_list = x_elapsed_time_list
    
    # ==================================================
    # 5. 情報取得時刻
    # ==================================================
    # 情報取得時刻の取得・設定
    def get_obtain_info_time(self):
        return self.obtain_info_time
    def set_obtain_info_time(self, obtain_info_time:float):
        self.obtain_info_time = obtain_info_time
    
    # 津波情報取得時刻の取得・設定
    def get_tsunami_info_obtained_time(self):
        return self.tsunami_info_obtained_time
    def set_tsunami_info_obtained_time(self, tsunami_info_obtained_time:float):
        self.tsunami_info_obtained_time = tsunami_info_obtained_time
    def update_tsunami_info_obtained_time(self, time_increment:float):
        self.tsunami_info_obtaiend_time += time_increment

    # 経路渋滞情報取得時刻の取得・設定
    def get_route_congestion_info_obtained_time(self):
        return self.route_congestion_info_obtained_time
    def set_route_congestion_info_obtained_time(self, route_congestion_info_obtained_time:float):
        self.route_congestion_info_obtained_time = route_congestion_info_obtained_time
    def update_route_congestion_info_obtained_time(self, time_increment:float):
        self.route_congestion_info_obtained_time += time_increment
    
    # 避難所満杯情報取得時刻の取得・設定
    def get_shelter_full_info_obtained_time(self):
        return self.shelter_full_info_obtained_time
    def set_shelter_full_info_obtained_time(self, shelter_full_info_obtained_time:float):
        self.shelter_full_info_obtained_time = shelter_full_info_obtained_time
    def update_shelter_full_info_obtained_time(self, time_increment:float):
        self.shelter_full_info_obtained_time += time_increment
    
    # ==================================================
    # 6. 正常性バイアス
    # ==================================================
    # 津波接近情報に対する正常性バイアス値の取得・設定
    def get_normalcy_value_about_tsunami_precursor_info(self):
        return self.normalcy_value_about_tsunami_precursor_info
    def set_normalcy_value_about_tsunami_precursor_info(self, normalcy_value_about_tsunami_precursor_info:float):
        self.normalcy_value_about_tsunami_precursor_info = normalcy_value_about_tsunami_precursor_info
    
    # 経路変更に対する正常性バイアス値の取得・設定
    def get_normalcy_value_about_route_congestion_info(self):
        return self.normalcy_value_about_route_congestion_info
    def set_normalcy_value_about_route_congestion_info(self, normalcy_value_about_route_congestion_info:float):
        self.normalcy_value_about_route_congestion_info = normalcy_value_about_route_congestion_info

    # 避難地満杯情報に対する正常性バイアス値の取得・設定
    def get_normalcy_value_about_shelter_full_info(self):
        return self.normalcy_value_about_shelter_full_info
    def set_normalcy_value_about_shelter_full_infot(self, normalcy_value_about_shelter_full_info:float):
        self.normalcy_value_about_shelter_full_info = normalcy_value_about_shelter_full_info

    # ==================================================
    # 7. 同調性バイアス
    # ==================================================
    # 同調性バイアス増加値の取得・設定
    def get_majority_value_increase(self):
        return self.majority_value_increase
    def set_majority_value_increase(self, majority_value_increase:float):
        self.majority_value_increase = majority_value_increase
    
    # 同調性バイアス減少値の取得・設定
    def get_majority_value_decrease(self):
        return self.majority_value_decrease
    def set_majority_value_decrease(self, majority_value_decrease:float):
        self.majority_value_decrease = majority_value_decrease
    
    # ==================================================
    # 8. 動機付け値・履歴
    # ==================================================
    # 計算された動機付け値の取得・設定
    def get_calculated_motivation_value(self):
        return self.calculated_motivation_value
    def set_calculated_motivation_value(self, calculated_motivation_value:float):
        self.calculated_motivation_value = calculated_motivation_value
    
    # 経過時間ごとの基本動機付け値の取得・設定
    def get_base_motivation_value_by_elapsed_time_dict(self):
        return self.base_motivation_value_by_elapsed_time_dict
    def set_base_motivation_value_by_elapsed_time_dict(self, base_motivation_value_by_elapsed_time_dict:dict[float, float]):
        self.base_motivation_value_by_elapsed_time_dict = base_motivation_value_by_elapsed_time_dict
    
    # 経過時間ごとの津波接近情報に対する正常性バイアス値の取得・設定
    def get_tsunami_precursor_normalcy_value_by_elapsed_time_dict(self):
        return self.tsunami_precursor_normalcy_value_by_elapsed_time_dict
    def set_tsunami_precursor_normalcy_value_by_elapsed_time_dict(self, tsunami_precursor_normalcy_value_by_elapsed_time_dict:dict[float, float]):
        self.tsunami_precursor_normalcy_value_by_elapsed_time_dict = tsunami_precursor_normalcy_value_by_elapsed_time_dict
    
    # 経過時間ごとの経路渋滞情報に対する正常性バイアス値の取得・設定
    def get_route_congestion_normalcy_value_by_elapsed_time_dict(self):
        return self.route_congestion_normalcy_value_by_elapsed_time_dict
    def set_route_congestion_normalcy_value_by_elapsed_time_dict(self, route_congestion_normalcy_value_by_elapsed_time_dict:dict[float, float]):
        self.route_congestion_normalcy_value_by_elapsed_time_dict = route_congestion_normalcy_value_by_elapsed_time_dict
    
    # 経過時間ごとの避難所満杯情報に対する正常性バイアス値の取得・設定
    def get_shelter_full_normalcy_value_by_elapsed_time_dict(self):
        return self.shelter_full_normalcy_value_by_elapsed_time_dict
    def set_shelter_full_normalcy_value_by_elapsed_time_dict(self, shelter_full_normalcy_value_by_elapsed_time_dict:dict[float, float]):
        self.shelter_full_normalcy_value_by_elapsed_time_dict = shelter_full_normalcy_value_by_elapsed_time_dict
    
    # ==================================================
    # 9. 車両乗り捨てに関するパラメータ・状態
    # ==================================================
    # 車両乗り捨て閾値の取得・設定  
    def get_vehicle_abandoned_threshold(self):
        return self.vehicle_abandoned_threshold
    def set_vehicle_abandoned_threshold(self, vehicle_abandoned_threshold:float):
        self.vehicle_abandoned_threshold = vehicle_abandoned_threshold
    
    # 車両乗り捨て時間の取得・設定
    def get_vehicle_abandoned_time(self):
        return self.vehicle_abandoned_time
    def set_vehicle_abandoned_time(self, vehicle_abandoned_time:float):
        self.vehicle_abandoned_time = vehicle_abandoned_time
    
    # ==================================================
    # 10. 各種フラグ
    # ==================================================
    # --- エージェント・避難所関連 ---
    def get_created_time_flg(self):
        return self.created_time_flg
    def set_created_time_flg(self, created_time_flg:bool):
        self.created_time_flg = created_time_flg
    def get_shelter_flg(self):
        return self.shelter_flg
    def set_shelter_flg(self, shelter_flg:bool):
        self.shelter_flg = shelter_flg
    def get_shelter_changed_flg(self):
        return self.shelter_changed_flg
    def set_shelter_changed_flg(self, shelter_changed_flg:bool):
        self.shelter_changed_flg = shelter_changed_flg
    def get_shelter_full_flg(self):
        return self.shelter_full_flg
    def set_shelter_full_flg(self, shelter_full_flg:bool):
        self.shelter_full_flg = shelter_full_flg
    def get_arrival_shelter_flg(self):
        return self.arrival_shelter_flg
    def set_arrival_shelter_flg(self, arrival_shelter_flg:bool):
        self.arrival_shelter_flg = arrival_shelter_flg
    def get_agent_action_name(self):
        return self.agent_action_name
    def set_agent_action_name(self, agent_action_name:str):
        self.agent_action_name = agent_action_name
    
    # --- 経路・逆走関連 ---
    def get_evacuation_route_changed_flg(self):
        return self.evacuation_route_changed_flg
    def set_evacuation_route_changed_flg(self, evacuation_route_changed_flg:bool):
        self.evacuation_route_changed_flg = evacuation_route_changed_flg
    def get_wrong_way_driving_flg(self):
        return self.wrong_way_driving_flg
    def set_wrong_way_driving_flg(self, wrong_way_driving_flg:bool):
        self.wrong_way_driving_flg = wrong_way_driving_flg
    def get_normalcy_lane_change_motivation_flg(self):
        return self.normalcy_lane_change_motivation_flg
    def set_normalcy_lane_change_motivation_flg(self, normalcy_lane_change_motivation_flg:bool):
        self.normalcy_lane_change_motivation_flg = normalcy_lane_change_motivation_flg
    def get_lane_minimum_motivation_value_flg(self):
        return self.lane_minimum_motivation_value_flg
    def set_lane_minimum_motivation_value_flg(self, lane_minimum_motivation_value_flg:bool):
        self.lane_minimum_motivation_value_flg = lane_minimum_motivation_value_flg
    def get_acceleration_flag(self):
        return self.acceleration_flag
    def set_acceleration_flag(self, acceleration_flag:bool):
        self.acceleration_flag = acceleration_flag
    
    # --- 車両乗り捨て関連 ---
    def get_vehicle_abandoned_flg(self):
        return self.vehicle_abandoned_flg
    def set_vehicle_abandoned_flg(self, vehicle_abandoned_flg:bool):
        self.vehicle_abandoned_flg = vehicle_abandoned_flg
    def get_avoiding_abandoned_vehicle_flg(self):
        return self.avoiding_abandoned_vehicle_flg
    def set_avoiding_abandoned_vehicle_flg(self, avoiding_abandoned_vehicle_flg:bool):
        self.avoiding_abandoned_vehicle_flg = avoiding_abandoned_vehicle_flg
    def get_failed_vehicle_abandonment_flg(self):
        return self.failed_vehicle_abandonment_flg
    def set_failed_vehicle_abandonment_flg(self, failed_vehicle_abandonment_flg:bool):
        self.failed_vehicle_abandonment_flg = failed_vehicle_abandonment_flg
    
    # --- 渋滞関連 ---
    def get_encounted_congestion_flg(self):
        return self.encounted_congestion_flg
    def set_encounted_congestion_flg(self, encounted_congestion_flg:bool):
        self.encounted_congestion_flg = encounted_congestion_flg
    
    # --- 情報取得関連 ---
    def get_tsunami_info_obtained_flg(self):
        return self.tsunami_info_obtained_flg
    def set_tsunami_info_obtained_flg(self, tsunami_info_obtained_flg:bool):
        self.tsunami_info_obtained_flg = tsunami_info_obtained_flg
    def get_route_congestion_info_obtained_flg(self):
        return self.route_congestion_info_obtained_flg
    def set_route_congestion_info_obtained_flg(self, route_congestion_info_obtained_flg:bool):
        self.route_congestion_info_obtained_flg = route_congestion_info_obtained_flg
    def get_shelter_full_info_obtained_flg(self):
        return self.shelter_full_info_obtained_flg
    def set_shelter_full_info_obtained_flg(self, shelter_full_info_obtained_flg:bool):
        self.shelter_full_info_obtained_flg = shelter_full_info_obtained_flg


