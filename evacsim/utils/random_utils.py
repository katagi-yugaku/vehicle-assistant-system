# =========================
# Random utility helpers
# =========================
#
# 注意:
# このモジュールでは random.seed(...) を設定しない。
# seed はライブラリ import 時ではなく、runner / 実験設定側で明示的に設定する。


import random


def generate_route_prob_list(route_num):
    """
    route_num に基づき、一様な確率リストを生成する。

    既存 utilities.py の挙動を維持する。
    route_num + 1 個の候補に対して、同じ確率を割り当てる。
    """
    n = route_num + 1
    prob = 1.0 / n
    return [prob] * n


def random_true(probablity: float) -> bool:
    """
    指定確率で True を返す。

    NOTE:
    引数名 probablity は既存コードの typo を維持している。
    既存コードで named argument を使っている場合に壊さないため。
    """
    return random.random() < probablity


def choose_edge_by_probability(edgeID_list: list, probabilities: list) -> str:
    """
    edgeID_list から probabilities に従って edgeID を1つ選択する。

    Args:
        edgeID_list:
            エッジIDのリスト。例: ['E26', 'E28']
        probabilities:
            各エッジに対応する確率。例: [0.7, 0.3]

    Returns:
        選ばれた edgeID。
    """
    if len(edgeID_list) != len(probabilities):
        raise ValueError("edge_listとprobabilitiesの長さが一致しません。")

    if not abs(sum(probabilities) - 1.0) < 1e-6:
        raise ValueError("probabilitiesの合計は1.0である必要があります。")

    return random.choices(edgeID_list, weights=probabilities, k=1)[0]


def choose_route_edges_by_probability(
    route_edges_list_by_start_end_index: dict,
    routeID_list,
    probabilities: list,
) -> list:
    """
    routeID_list から probabilities に従って routeID を1つ選び、
    対応する route_edges を返す。

    既存 utilities.py の挙動を維持する。
    """
    routeID = random.choices(routeID_list, weights=probabilities, k=1)[0]
    return route_edges_list_by_start_end_index[routeID]
