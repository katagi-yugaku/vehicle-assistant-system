# =========================
# Motivation model helpers
# =========================
#
# 注意:
# このモジュールでは TraCI を import しない。
# 運転者の心理モデル・情報活性化・モチベーション計算だけを扱う。

from __future__ import annotations

from typing import TYPE_CHECKING

import numpy as np

if TYPE_CHECKING:
    from evacsim.agents.Agent import Agent

from evacsim.sim.routing_distance import distance_each_vehIDs

def two_stage_sigmoid(value: float):
    """
    二段階上昇型シグモイド（ストレス上昇モデル向け）

    - 第1段階: x=150 を中心にゆるやかに上昇（予兆・軽い緊張）
    - 第2段階: x=350 を中心に急激に上昇（危険察知による急上昇）

    既存 utilities.py の挙動を維持する。
    """
    k1 = 0.015   # 緩やかな傾き
    x01 = 200    # 軽い上昇の中心

    k2 = 0.045   # 急な立ち上がり
    x02 = 330    # 急上昇の中心

    ymin = 0
    ymax = 1000

    s1 = 1 / (1 + np.exp(-k1 * (value - x01)))
    s2 = 1 / (1 + np.exp(-k2 * (value - x02)))

    y_norm = 0.5 * (s1 + s2)
    return ymin + (ymax - ymin) * y_norm


def generate_motivation_curve(
    max_time: int = 450,
    step: int = 1,
) -> dict[float, float]:
    """
    経過時間を key，モチベーション値を value とする辞書を生成する。

    既存 utilities.py の挙動を維持する。
    """
    dencimals = 3
    x_values = np.arange(0.0, max_time, step)
    y_values = [
        round(float(two_stage_sigmoid(x)), dencimals)
        for x in x_values
    ]
    x_list = x_values.tolist()
    xy_dict = dict(zip(x_list, y_values))

    return xy_dict


def sigmoid(x: np.ndarray) -> np.ndarray:
    """
    シグモイド関数。

    exp のオーバーフローを避けるために clip している。
    """
    x = np.clip(x, -500, 500)
    return 1.0 / (1.0 + np.exp(-x))


def generate_info_activation_dict(
    rho: float,
    delta: float,
    max_time: int = 450,
    step: int = 1,
) -> dict[float, float]:
    """
    情報取得後の経過時間を key，活性化度 phi を value とする dict を生成する。

    Returns
    -------
    activation_dict:
        {
            情報取得後の経過時間: 活性化度 phi
        }
    """
    base = sigmoid(np.array([-rho * delta]))[0]
    denominator = 1.0 - base

    x_values = np.arange(0, max_time, step)

    activation_dict = {}
    decimals = 3

    for x in x_values:
        numerator = sigmoid(rho * (x - delta)) - base
        phi = numerator / denominator
        phi = float(np.clip(phi, 0.0, 1.0))
        phi = round(phi, decimals)
        activation_dict[float(x)] = phi

    return activation_dict


def re_calculate_motivation_value(
    info_activation_dict: dict[float, float],
    elapsed_time: float,
) -> float:
    """
    既存 utilities.py の未完成実装をそのまま残す。

    NOTE:
    現状では値を返していないため、挙動変更を避ける目的でそのままにしている。
    今後、使用箇所を確認したうえで整理する。
    """
    for time, value in info_activation_dict.items():
        time += elapsed_time

    # print(f"elapsed_time: {elapsed_time}")


def get_value_from_time_dict(
    value_by_time_dict: dict,
    elapsed_time: float,
    default: float = 0.0,
) -> float:
    """
    時刻を key とする辞書から，elapsed_time 以下の最大 key に対応する値を取得する。

    - 辞書が空の場合は default を返す
    - elapsed_time 以下の最大 key を使う
    - elapsed_time が最小 key より小さい場合は，最小 key の値を返す
    - elapsed_time が最大 key より大きい場合は，最大 key の値を返す
    - key は int / float を想定する
    """
    if not value_by_time_dict:
        return float(default)

    numeric_keys = [
        key for key in value_by_time_dict.keys()
        if isinstance(key, (int, float))
    ]

    if not numeric_keys:
        return float(default)

    elapsed_time = float(elapsed_time)

    selected_key = None

    for key in numeric_keys:
        if key <= elapsed_time:
            if selected_key is None or key > selected_key:
                selected_key = key

    if selected_key is None:
        selected_key = min(numeric_keys)

    return float(value_by_time_dict[selected_key])


def calculate_motivation_for_evacuation_action(
    agent: Agent,
    agent_list: list[Agent],
    current_time: float,
    action: str,
    debug: bool = False,
) -> float:
    """
    指定された運転者 agent について，現時点の避難行動モチベーションを計算して返す。

    今回は最小実装として，車両乗り捨て va のみを対象とする。
    この関数内では agent の状態更新は行わない。
    """
    base_motivation_value_by_elapsed_time: dict = (
        agent.get_base_motivation_value_by_elapsed_time_dict()
    )

    congestion_duration = agent.get_congestion_duration()
    base_value = get_value_from_time_dict(
        value_by_time_dict=base_motivation_value_by_elapsed_time,
        elapsed_time=congestion_duration,
        default=0.0,
    )
    # 情報取得による増加
    info_motivation_value = calculate_motivation_for_obtained_info(agent=agent, current_time=current_time, action=action)
    # 同調性バイアスによる増減
    majority_motivation_value = calculate_motivation_for_majority_tunning_bias(agent=agent, vehInfo=0.0, agent_list=agent_list, candidate_action=action)


    motivation = base_value + info_motivation_value + majority_motivation_value

    if debug:
        print(
            "[calculate_motivation_for_evacuation_action] "
            f"vehID={agent.get_vehID()}, "
            f"action={action}, "
            f"current_time={current_time}, "
            f"congestion_duration={congestion_duration}, "
            f"base={base_value}, "
            f"motivation={motivation}"
        )

    return float(motivation)


def calculate_motivation_for_obtained_info(
    agent: Agent,
    current_time: float,
    action: str,
) -> float:
    info_motivation_value = 0.0
    # 満杯情報に対するモチベーション増加のシグモイド関数
    shelter_full_activation_value_by_elapsed_time: dict = (
        agent.get_shelter_full_normalcy_value_by_elapsed_time_dict()
    )
    # その情報の取得時間
    shelter_full_info_obtained_time = agent.get_shelter_full_info_obtained_time()

    # 津波接近情報に対するモチベーション増加のシグモイド関数
    tsunami_precursor_normalcy_value_by_elapsed_time: dict = (
        agent.get_tsunami_precursor_normalcy_value_by_elapsed_time_dict()
    )
    # その情報の取得時間
    tsunami_precursor_info_obtained_time = agent.get_tsunami_info_obtained_time()

    # 経路の混雑情報に対するモチベーション増加のシグモイド関数
    route_congestion_normalcy_value_by_elapsed_time: dict = (
        agent.get_route_congestion_normalcy_value_by_elapsed_time_dict()
    )
    # その情報の取得時間
    route_congestion_info_obtained_time = agent.get_route_congestion_info_obtained_time()

    if action == "rc":
        shelter_full_activation_value = (
                                            agent.get_normalcy_value_about_shelter_full_info()
                                            * get_value_from_time_dict(
                                                value_by_time_dict=shelter_full_activation_value_by_elapsed_time,
                                                elapsed_time=current_time - shelter_full_info_obtained_time,
                                                default=0.0,
                                            )
                                        )
        route_congestion_activation_value = (
            agent.get_normalcy_value_about_route_congestion_info()
            * get_value_from_time_dict(
                value_by_time_dict=route_congestion_normalcy_value_by_elapsed_time,
                elapsed_time=current_time - route_congestion_info_obtained_time,
                default=0.0,
            )
        )
        Info_motivation_vale = shelter_full_activation_value + route_congestion_activation_value

    elif action == "ww":
        route_congestion_activation_value = (
            agent.get_normalcy_value_about_route_congestion_info()
            * get_value_from_time_dict(
                value_by_time_dict=route_congestion_normalcy_value_by_elapsed_time,
                elapsed_time=current_time - route_congestion_info_obtained_time,
                default=0.0,
            )
        )
        tsunami_precursor_activation_value = (
            agent.get_normalcy_value_about_tsunami_precursor_info()
            * get_value_from_time_dict(
                value_by_time_dict=tsunami_precursor_normalcy_value_by_elapsed_time,
                elapsed_time=current_time - tsunami_precursor_info_obtained_time,
                default=0.0,
            )
        )
        Info_motivation_vale = shelter_full_activation_value + tsunami_precursor_activation_value
    
    elif action == "va":
        shelter_full_activation_value = (
        agent.get_normalcy_value_about_shelter_full_info()
        * get_value_from_time_dict(
            value_by_time_dict=shelter_full_activation_value_by_elapsed_time,
            elapsed_time=shelter_full_info_obtained_time,
            default=0.0,
        )
        )
        tsunami_precursor_activation_value = (
            agent.get_normalcy_value_about_tsunami_precursor_info()
            * get_value_from_time_dict(
                value_by_time_dict=tsunami_precursor_normalcy_value_by_elapsed_time,
                elapsed_time=current_time - tsunami_precursor_info_obtained_time,
                default=0.0,
            )
        )
        Info_motivation_vale = shelter_full_activation_value + tsunami_precursor_activation_value
    return float(Info_motivation_vale)


def calculate_motivation_for_majority_tunning_bias(
    agent: Agent,
    vehInfo: float,
    agent_list: list[Agent],
    candidate_action: str,
) -> float:
    same_action_count, total_count = count_same_action_drivers(agent=agent, vehInfo=vehInfo, agent_list=agent_list, candidate_action=candidate_action)
    if same_action_count == 0 or total_count == 0:
        return float(-1 * agent.get_majority_value_decrease())
    else:
        return same_action_count * agent.get_majority_value_increase()

def count_same_action_drivers(
    agent: Agent,
    vehInfo: float,
    agent_list: list[Agent],
    candidate_action: str,
):
    """
    同調性バイアスの計算

    - agent: 対象の運転者エージェント
    - vehInfo: 周囲の車両情報（例: 近くの車両の避難行動割合）

    Returns
    -------
    
    """
    # agent_listから、周囲の中で、同じ行動をとっている　かつ、周囲の50m以内の車両の数を数える
    same_action_count = 0
    total_count = 0
    for other_agent in agent_list:
        if other_agent == agent:
            continue

        distance = distance_each_vehIDs(agent.get_vehID(), other_agent.get_vehID())
        if distance <= 50:
            total_count += 1
            if other_agent.get_agent_action_name() == candidate_action:
                same_action_count += 1

    return same_action_count, total_count
