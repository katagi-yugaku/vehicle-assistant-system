# =========================
# Lookup helpers
# =========================


def find_agent_by_vehID(
    vehID: str,
    agent_list: list = None,
    agent_by_vehID_dict: dict = None,
):
    """
    vehID から Agent を取得する。

    優先順位:
      1. agent_by_vehID_dict が渡されていれば dict 検索
      2. agent_list が dict の場合も dict 検索
      3. list の場合は get_vehID() で線形探索

    既存 utilities.py の挙動を維持する。
    """
    if agent_by_vehID_dict is not None:
        return agent_by_vehID_dict.get(vehID)

    if isinstance(agent_list, dict):
        return agent_list.get(vehID)

    for agent in agent_list:
        if agent.get_vehID() == vehID:
            return agent


def find_vehInfo_by_vehID(
    vehID: str,
    vehInfo_list: list = None,
    vehInfo_by_vehID_dict: dict = None,
):
    """
    vehID から VehicleInfo を取得する。

    優先順位:
      1. vehInfo_by_vehID_dict が渡されていれば dict 検索
      2. vehInfo_list が dict の場合も dict 検索
      3. list の場合は get_vehID() で線形探索

    既存 utilities.py の挙動を維持する。
    """
    if vehInfo_by_vehID_dict is not None:
        return vehInfo_by_vehID_dict.get(vehID)

    if isinstance(vehInfo_list, dict):
        return vehInfo_list.get(vehID)

    for vehInfo in vehInfo_list:
        if vehInfo.get_vehID() == vehID:
            return vehInfo


def find_shelter_by_edgeID_connect_target_shelter(
    edgeID: str,
    shelter_list: list,
):
    """
    避難地に接続する edgeID から Shelter を取得する。
    """
    for shelter in shelter_list:
        if shelter.get_near_edgeID() == edgeID:
            return shelter


def find_shelterID_by_edgeID_by_shelterID(
    edgeID: str,
    edgeID_by_shelterID: dict,
):
    """
    edgeID_by_shelterID から、指定 edgeID に対応する shelterID を取得する。
    見つからない場合は None を返す。
    """
    for shelterID, edgeID_near_shelterID in edgeID_by_shelterID.items():
        if edgeID_near_shelterID == edgeID:
            return shelterID

    # TODO: 必要であれば例外処理を追加する
    return None
