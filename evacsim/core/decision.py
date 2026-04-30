# =========================
# Decision model helpers
# =========================
#
# 注意:
# このモジュールでは TraCI を import しない。
# 運転者の意思決定式・閾値判定だけを扱う。
#
# NOTE:
# 既存コード互換のため、関数名中の abandant などの typo は維持する。

from __future__ import annotations

from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from evacsim.agents.Agent import Agent
    from evacsim.agents.VehicleInfo import VehicleInfo


def is_driver_vehicle_abandant(
    agent_by_target_vehID: Agent,
    vehInfo_by_target_vehID: VehicleInfo,
    current_time: float,
    neighbor_vehicle_abandant_nums: int,
    alpha: float,
):
    """
    車両乗り捨てを行うかどうかを判定する。

    既存 utilities.py の is_driver_vehicle_abandant の挙動を維持する。

    NOTE:
    vehInfo_by_target_vehID は既存シグネチャ互換のため残す。
    現在の実装ではこの関数内では使用していない。
    """
    if neighbor_vehicle_abandant_nums > 6:
        neighbor_vehicle_abandant_nums = 6

    current_vehID = agent_by_target_vehID.get_vehID()
    encounted_congestion_time = agent_by_target_vehID.get_congestion_duration()
    tsunami_info_obtain_time = agent_by_target_vehID.get_tsunami_info_obtaiend_time()

    if (
        agent_by_target_vehID.get_created_time()
        > agent_by_target_vehID.get_tsunami_info_obtaiend_time()
    ):
        tsunami_info_obtain_time = agent_by_target_vehID.get_created_time()

    current_vehicle_abandantment_value = (
        alpha * max(0, (current_time - encounted_congestion_time) ** 2)
        + 30.0 * max(0, current_time - tsunami_info_obtain_time)
        - 1.0 * agent_by_target_vehID.get_normalcy_value_about_vehicle_abandonment()
        + neighbor_vehicle_abandant_nums
        * agent_by_target_vehID.get_majority_value_about_vehicle_abandonment()
    )

    if (
        current_vehicle_abandantment_value
        > agent_by_target_vehID.get_vehicle_abandoned_threshold()
    ):
        print(
            f"Check_{current_vehID}_{current_time}:  "
            f"{current_vehicle_abandantment_value} =  "
            f"(jam: {current_time} - {encounted_congestion_time} "
            f"+ info 30.0*{max(0, current_time - agent_by_target_vehID.get_tsunami_info_obtaiend_time())} "
            f"- {1.0 * agent_by_target_vehID.get_normalcy_value_about_vehicle_abandonment()} "
            f"+ {neighbor_vehicle_abandant_nums}*"
            f"{agent_by_target_vehID.get_majority_value_about_vehicle_abandonment()}) "
            f"> {agent_by_target_vehID.get_vehicle_abandoned_threshold()}"
        )
        return True

    return False


def is_again_driver_vehicle_abandant(
    agent_by_target_vehID: Agent,
    vehInfo_by_target_vehID: VehicleInfo,
    current_time: float,
    neighbor_vehicle_abandant_nums: int,
):
    """
    再判定用の車両乗り捨て判定。

    既存 utilities.py の is_again_driver_vehicle_abandant の挙動を維持する。

    NOTE:
    vehInfo_by_target_vehID は既存シグネチャ互換のため残す。
    現在の実装ではこの関数内では使用していない。
    """
    if neighbor_vehicle_abandant_nums > 2:
        neighbor_vehicle_abandant_nums = 2

    current_vehID = agent_by_target_vehID.get_vehID()
    encounted_congestion_time = agent_by_target_vehID.get_congestion_duration()

    current_vehicle_abandantment_value = (
        0.1 * max(0, (current_time - encounted_congestion_time) ** 2)
        + 30.0 * max(
            0,
            current_time - agent_by_target_vehID.get_tsunami_info_obtaiend_time(),
        )
        - 1.0 * agent_by_target_vehID.get_normalcy_value_about_vehicle_abandonment()
        + neighbor_vehicle_abandant_nums
        * agent_by_target_vehID.get_majority_value_about_vehicle_abandonment()
    )

    if (
        current_vehicle_abandantment_value
        > agent_by_target_vehID.get_vehicle_abandoned_threshold()
    ):
        print(
            f"Check_{current_vehID}_{current_time}:  "
            f"{current_vehicle_abandantment_value} =  "
            f"(jam: {current_time} - {encounted_congestion_time} "
            f"+ info 30.0*{max(0, current_time - agent_by_target_vehID.get_tsunami_info_obtaiend_time())} "
            f"- {1.0 * agent_by_target_vehID.get_normalcy_value_about_vehicle_abandonment()} "
            f"+ {neighbor_vehicle_abandant_nums}*"
            f"{agent_by_target_vehID.get_majority_value_about_vehicle_abandonment()}) "
            f"> {agent_by_target_vehID.get_vehicle_abandoned_threshold()}"
        )
        return True

    print(
        f"Failed {current_vehID}_{current_time}:  "
        f"{current_vehicle_abandantment_value} =  "
        f"(jam: {current_time} - {encounted_congestion_time} "
        f"+ info 30.0*{max(0, current_time - agent_by_target_vehID.get_tsunami_info_obtaiend_time())} "
        f"- {1.0 * agent_by_target_vehID.get_normalcy_value_about_vehicle_abandonment()} "
        f"+ {neighbor_vehicle_abandant_nums}*"
        f"{agent_by_target_vehID.get_majority_value_about_vehicle_abandonment()}) "
        f"<= {agent_by_target_vehID.get_vehicle_abandoned_threshold()}"
    )

    return False
