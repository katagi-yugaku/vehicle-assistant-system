import traci

class CustomeEdge():
    def __init__(self, current_edgeID:str):
        self.current_edgeID = current_edgeID
        self.opposite_edgeID  = ""
        self.start_junction = ""  # 開始の交差点
        self.end_junction = "" # 終了の交差点
        self.start_edge_flag = False # このedgeが開始エッジ（incomエッジが存在しない）
        self.end_edge_flag = False #　このedgeが終了エッジ（outgoエッジが存在しない）

    # 初期の反対edgeの設定
    def setting_init_opposite_edgeID(self, edgeIDs:list):
        if self.get_current_edgeID().startswith("-"):
            opposite_edgeID:str = self.get_current_edgeID().lstrip('-')
        else:
            opposite_edgeID:str = "-" + self.get_current_edgeID()
        for edgeID in edgeIDs:
            if edgeID == opposite_edgeID:
                self.set_opposite_edgeID(opposite_edgeID)

    # 初期の開始交差点の設定
    def setting_init_start_end_junctions(self):
        # currentEdgeが-から始まる場合、開始交差点はFromJunction、終了交差点はToJunction
        # TODO　確認 左側通行なので　ToJunctionが開始交差点、FromJunctionが終了交差点
        # print("current_edgeID: ", self.get_current_edgeID())
        self.set_start_junction(traci.edge.getToJunction(self.get_current_edgeID()))
        self.set_end_junction(traci.edge.getFromJunction(self.get_current_edgeID()))

    # start_junctionによってincoming方向に接続する隣接するedgeIDsの取得
    # current_edgeIDが-から始まる場合、左側車線　start_junctionが開始交差点　end_junctionが終了交差点
    def obtain_neighbour_incom_edgeIDs_with_junc_by_start_junc(self):
        return tuple(item for item in traci.junction.getOutgoingEdges(self.get_start_junction()) if item != self.get_opposite_edgeID())

    # end_junctionによってoutgoing方向に接続する隣接edgeIDsの取得
    def obtain_neighbour_outgo_edgeIDs_with_junc_by_start_junc(self):
        return tuple(item for item in traci.junction.getIncomingEdges(self.get_start_junction()) if item != self.get_current_edgeID())

    # start_junctionによってoutgoing方向に接続する隣接edgeIDsの取得
    def obtain_neighbour_incom_edgeIDs_with_junc_by_end_junc(self):
        return tuple(item for item in traci.junction.getOutgoingEdges(self.get_end_junction()) if item != self.get_opposite_edgeID())

    # end_junctionによってincoming方向に接続する隣接edgeIDsの取得
    def obtain_neighbour_outgo_edgeIDs_with_junc_by_end_junc(self):
        return tuple(item for item in traci.junction.getIncomingEdges(self.get_end_junction()) if item != self.get_current_edgeID())

    def custom_edge_info_print(self):
        print(f'[{self.get_start_junction()}]====={self.get_current_edgeID()}=====[{self.get_end_junction()}]')
        print(f'    ====={self.get_opposite_edgeID()}=====  ')

    def connect_edge_print(self):
        print(f'[{self.obtain_neighbour_incom_edgeIDs_with_junc_by_start_junc()}]-->[{self.get_start_junction()}]====={self.get_current_edgeID()}=====[{self.get_end_junction()}] --> [{self.obtain_neighbour_outgo_edgeIDs_with_junc_by_end_junc()}]')

    def custom_edge_info_list_print(self):
        print("current_edgeID: ", self.get_current_edgeID())
        print("opposite_edgeID: ", self.get_opposite_edgeID())
        print("start_junction: ", self.get_start_junction())
        print("end_junction: ", self.get_end_junction())
    
    def around_edgeIDs(self):
        return self.obtain_neighbour_incom_edgeIDs_with_junc_by_start_junc() + self.obtain_neighbour_outgo_edgeIDs_with_junc_by_start_junc()\
            + self.obtain_neighbour_incom_edgeIDs_with_junc_by_end_junc() + self.obtain_neighbour_outgo_edgeIDs_with_junc_by_end_junc()

    # このエッジが車両生成の開始エッジか否かを判定
    def is_current_edgeID_start_edge(self):
        start_edgeIDs:tuple = self.obtain_neighbour_outgo_edgeIDs_with_junc_by_end_junc()
        if not start_edgeIDs:
            # print(f'{self.get_current_edgeID()}は開始エッジです  around_edgeIDs: {start_edgeIDs}')
            return True
    
    # このエッジが車両生成の終了edgeか否かを判定
    def is_current_edgeID_end_edge(self):
        end_edgeIDs:tuple = self.obtain_neighbour_incom_edgeIDs_with_junc_by_start_junc()
        if not end_edgeIDs:
            # print(f'{self.get_current_edgeID()}は終了エッジです around_edgeIDs: {end_edgeIDs}')
            return True

    # 現在のエッジIDの取得と設定
    def get_current_edgeID(self):
        return self.current_edgeID
    def set_current_edgeID(self, current_edgeID:str):
        self.current_edgeID = current_edgeID

    # 反対のエッジIDの取得と設定
    def get_opposite_edgeID(self):
        return self.opposite_edgeID
    def set_opposite_edgeID(self, opposite_edgeID:str):
        self.opposite_edgeID = opposite_edgeID

    # 開始の交差点の取得と設定
    def get_start_junction(self):
        return self.start_junction
    def set_start_junction(self, start_junction:str):
        self.start_junction = start_junction

    # 終了の交差点の取得と設定
    def get_end_junction(self):
        return self.end_junction
    def set_end_junction(self, end_junction:str):
        self.end_junction = end_junction

    # 開始エッジフラグの取得と設定
    def get_start_edge_flag(self):
        return self.start_edge_flag
    def set_start_edge_flag(self, start_edge_flag:bool):
        self.start_edge_flag = start_edge_flag

    # 終了エッジフラグの取得と設定
    def get_end_edge_flag(self):
        return self.end_edge_flag
    def set_end_edge_flag(self, end_edge_flag:bool):
        self.end_edge_flag = end_edge_flag

class ConnectedEdges():
    def __init__(self, start_edgeID:str, end_edgeID:str, via_edgeIDs:list):
        self.start_edgeID = start_edgeID
        self.end_edgeID = end_edgeID
        self.via_edgeIDs = via_edgeIDs
    
    def get_start_edgeID(self):
        return self.start_edgeID
    def set_start_edgeID(self, start_edgeID:str):
        self.start_edgeID = start_edgeID

    def get_end_edgeID(self):
        return self.end_edgeID
    def set_end_edgeID(self, end_edgeID:str):
        self.end_edgeID = end_edgeID
    
    def get_via_edgeIDs(self):
        return self.via_edgeIDs
    def set_via_edgeIDs(self, via_edgeIDs:str):
        self.via_edgeIDs = via_edgeIDs