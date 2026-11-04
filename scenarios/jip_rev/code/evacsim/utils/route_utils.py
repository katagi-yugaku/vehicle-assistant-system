# =========================
# Route / edge list utilities
# =========================


def get_next_edge(edgeIDs: tuple, current_edgeID: str) -> str | None:
    """
    指定したエッジ current_edgeID の次のエッジを返す。

    Args:
        edgeIDs: エッジの順序付きタプル
        current_edgeID: 現在のエッジID

    Returns:
        次のエッジID。存在しない場合は None。
    """
    if current_edgeID in edgeIDs:
        idx = edgeIDs.index(current_edgeID)
        if idx + 1 < len(edgeIDs):
            return edgeIDs[idx + 1]

    return None


def get_prev_edge(edgeIDs: tuple, current_edgeID: str):
    """
    指定したエッジ current_edgeID の一つ前のエッジを返す。

    Args:
        edgeIDs: エッジの順序付きタプル
        current_edgeID: 現在のエッジID

    Returns:
        一つ前のエッジID。存在しない場合は None。
    """
    if current_edgeID in edgeIDs:
        idx = edgeIDs.index(current_edgeID)
        if idx > 0:
            return edgeIDs[idx - 1]

    return None


def get_remaining_edges(route_edges, current_edgeID):
    """
    現在エッジを含めて、残りの route edges を返す。

    Args:
        route_edges: 経路エッジ列
        current_edgeID: 現在のエッジID

    Returns:
        current_edgeID 以降の edge list。
        current_edgeID が見つからない場合は空リスト。
    """
    if current_edgeID not in route_edges:
        return []

    idx = route_edges.index(current_edgeID)
    return route_edges[idx:]


def remove_junction_from_edgeID(edgeIDs: list):
    """
    junction edge を除外した edgeID list を返す。

    SUMO の internal edge は ':' で始まるため、それを除外する。
    """
    return [edgeID for edgeID in edgeIDs if not edgeID.startswith(":")]
