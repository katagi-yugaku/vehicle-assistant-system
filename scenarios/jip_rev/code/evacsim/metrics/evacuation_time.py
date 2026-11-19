# =========================
# Evacuation time metrics
# =========================
#
# 注意:
# このモジュールでは TraCI を import しない。
# 避難時間・到着台数などの評価指標だけを扱う。

from __future__ import annotations

from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from evacsim.agents.Shelter import Shelter
    from evacsim.agents.VehicleInfo import VehicleInfo


def create_arrival_time_list(vehInfo_list: list):
    """
    VehicleInfo のリストから arrival_time を取り出して昇順に並べる。

    既存 utilities.py の create_arrival_time_list の挙動を維持する。
    """
    arrival_time_list = []

    for vehInfo in vehInfo_list:
        arrival_time_list.append(vehInfo.get_arrival_time())

    return sorted(arrival_time_list)


# 改訂(時間窓): None なら元の累積平均。runner が revision_options から設定する。
AVG_WINDOW_S = None
AVG_WINDOW_MIN_SAMPLES = 3


def windowed_route_average(records, current_time, window_s, min_samples):
    """
    1 つの経路の記録 [(evac_time, arrival_time), ...] から、時間窓 + 最低台数の平均を返す。

    S = 到着時刻が (current_time - window_s, current_time] の記録。
    |S| < min_samples なら、到着時刻が新しい順に min_samples 件(全体がそれ未満なら全件)。
    戻り値: (平均, 使った台数, 使った記録の最新の到着時刻)。記録が空なら None。
    """
    recs = [(float(e), float(a)) for e, a in records if a is not None and a <= current_time + 1e-9]
    if not recs:
        return None
    in_window = [r for r in recs if r[1] > current_time - window_s]
    if len(in_window) < min_samples:
        in_window = sorted(recs, key=lambda r: r[1])[-min_samples:]
    avg = sum(e for e, _ in in_window) / len(in_window)
    return avg, len(in_window), max(a for _, a in in_window)


def calculate_avg_evac_time_by_route(shelter_list: list, current_time=None):
    """
    shelter_list 内の各避難所について、
    各ルートの平均避難時間と通過車両数を計算して各避難所に設定する。

    既存 utilities.py の calculate_avg_evac_time_by_route の挙動を維持する。
    改訂: AVG_WINDOW_S が設定され current_time が渡されたときだけ、時間窓 + 最低台数で計算する。
    """
    if AVG_WINDOW_S is not None and current_time is not None:
        for shelter in shelter_list:
            by_route: dict = {}
            for rec in shelter.get_evacuation_time_from_junction_multidict().values():
                arrival_time = rec[2] if len(rec) > 2 else None
                by_route.setdefault(rec[0], []).append((rec[1], arrival_time))
            final = {}
            for route, records in by_route.items():
                res = windowed_route_average(records, float(current_time),
                                             float(AVG_WINDOW_S), int(AVG_WINDOW_MIN_SAMPLES))
                if res is None:
                    continue
                avg, n, latest = res
                final[route] = {"avg_time": avg, "vehicles": n, "latest_arrival": latest}
            shelter.set_avg_evac_time_by_route(final)
        return
    # TODO: shelter 同士で近いなら情報を共有するように変更する
    for shelter in shelter_list:
        avg_evac_time_by_route = {}

        # 避難時間とルートごとのデータを取得
        evac_time_with_route_by_vehID = (
            shelter.get_evacuation_time_from_junction_multidict()
        )

        for vehID, route_with_evac_time in evac_time_with_route_by_vehID.items():
            route_by_vehID = route_with_evac_time[0]
            evac_time = route_with_evac_time[1]

            if route_by_vehID in avg_evac_time_by_route:
                avg_evac_time_by_route[route_by_vehID][0] += evac_time
                avg_evac_time_by_route[route_by_vehID][1] += 1
                avg_evac_time_by_route[route_by_vehID][2] += 1
            else:
                avg_evac_time_by_route[route_by_vehID] = [evac_time, 1, 1]

        # 平均を計算
        final_avg_evac_time_by_route = {}

        for route, (total_time, count, vehicles) in avg_evac_time_by_route.items():
            avg_time = total_time / count
            final_avg_evac_time_by_route[route] = {
                "avg_time": avg_time,
                "vehicles": vehicles,
            }

        # ルートごとの平均避難時間と台数をセット
        shelter.set_avg_evac_time_by_route(final_avg_evac_time_by_route)


def merge_arrival_vehs_of_shelter(shelter_list: list):
    """
    shelterID の先頭タイプが一致する避難所の到着車両数を集計し、
    total_arrival_vehIDs にセットする。

    例:
        A_1, A_2 は同じ A 系として集計する。
    """
    for shelter in shelter_list:
        shelter.init_total_arrival_vehIDs()

        shelter_type = shelter.get_shelterID().split("_")[0]
        total = len(shelter.get_arrival_vehID_list())

        for other_shelter in shelter_list:
            other_type = other_shelter.get_shelterID().split("_")[0]

            if shelter is not other_shelter and shelter_type == other_type:
                total += len(other_shelter.get_arrival_vehID_list())

        shelter.set_total_arrival_vehIDs(total)
