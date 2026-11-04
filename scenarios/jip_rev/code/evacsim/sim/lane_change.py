# =========================
# Lane change helpers
# =========================
#
# 注意:
# このモジュールは TraCI に依存する。
# レーン変更実行処理と、レーン変更モチベーション更新処理を扱う。

from __future__ import annotations

from typing import TYPE_CHECKING

from evacsim.sim.sumo_env import ensure_sumo_tools_on_path

ensure_sumo_tools_on_path()

import traci

from evacsim.core.motivation import two_stage_sigmoid

if TYPE_CHECKING:
    from evacsim.agents.Agent import Agent
    from evacsim.agents.VehicleInfo import VehicleInfo


def _init_driver_behavior(
    vehIDs: list,
    lane_change_mode: int,
):
    """
    車両の laneChangeMode を設定する。

    NOTE:
    元の utilities.py では init_driver_behavior を呼んでいた。
    現時点では evacsim/sim/initialization.py への切り出し前なので、
    挙動維持のため、このモジュール内に最小限の private helper として置く。
    """
    for vehID in vehIDs:
        traci.vehicle.setLaneChangeMode(vehID, lane_change_mode)


def lane_change_by_vehID(
    vehID: str,
    agent: Agent,
    vehInfo: VehicleInfo,
):
    """
    レーン変更の意思決定・実行処理。

    条件:
      - 前方車両が存在する
      - 前方車両との距離が 20m 未満
      - 自車速度が 7.0 m/s 未満

    条件を満たす場合:
      - laneIndex=1 に lane change
      - laneChangeMode=512 を設定
      - ShelterA_2 に parkingAreaStop を設定
      - 速度を 9.0 m/s に設定
      - VehicleInfo の target_shelter を ShelterA_2 に変更
      - Agent の route changed flag と lane change time を更新

    既存 utilities.py の lane_change_by_vehID の挙動を維持する。
    """
    leader_info = traci.vehicle.getLeader(vehID)
    speed = traci.vehicle.getSpeed(vehID)

    # 条件：
    # 前方に車両が存在し（leader_infoがNoneでない）
    # 距離が近い（例: 20m未満）
    # 自車速度が遅い（例: 7.0 m/s 未満）
    if (leader_info is not None and leader_info[1] < 20.0) and (speed < 7.0):
        try:
            # レーン変更（右 or 左 laneIndex は環境に合わせて調整）
            traci.vehicle.changeLane(
                vehID=vehID,
                laneIndex=1,
                duration=1000,
            )

        except Exception as e:
            print(f"Error changing lane for vehicle {vehID}: {e}")
            return False

        # レーン変更後の設定
        _init_driver_behavior(
            vehIDs=[vehID],
            lane_change_mode=512,
        )

        traci.vehicle.setParkingAreaStop(
            vehID=vehID,
            stopID="ShelterA_2",
            duration=100000,
        )

        traci.vehicle.setSpeed(vehID, 9.0)

        vehInfo.set_target_shelter("ShelterA_2")

        # フラグと記録更新
        agent.set_evacuation_route_changed_flg(True)

        elapsed = traci.simulation.getTime() - agent.get_created_time()
        agent.set_lane_change_time(elapsed)

        return True

    return False


def update_agent_lane_change_motivation(
    agent: Agent,
):
    """
    Agent のレーン変更モチベーションを更新する。

    処理:
      - 初回更新時に created_time を現在時刻に設定
      - 現在時刻 - created_time で経過時間を算出
      - two_stage_sigmoid で次のモチベーション値を計算
      - Agent の x/y 履歴に append

    既存 utilities.py の update_agent_lane_change_motivation の挙動を維持する。
    """
    if len(agent.get_x_lane_change()) == 0:
        created_time = traci.simulation.getTime()
        agent.set_created_time(created_time)

    current_simulation_elapsed_time = (
        traci.simulation.getTime()
        - agent.get_created_time()
    )

    next_lane_change_motivation = two_stage_sigmoid(
        value=current_simulation_elapsed_time
    )

    agent.append_x_lane_change(current_simulation_elapsed_time)
    agent.append_y_lane_change(next_lane_change_motivation)
