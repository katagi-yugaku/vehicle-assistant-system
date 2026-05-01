# =========================
# Traffic state / speed control helpers
# =========================
#
# 注意:
# このモジュールは一部 TraCI に依存する。
# 待ち時間・速度・密度係数・gap に基づく速度制御を扱う。

from __future__ import annotations

import math

from evacsim.sim.sumo_env import ensure_sumo_tools_on_path

ensure_sumo_tools_on_path()

import traci


def calculate_wait_time(
    vehicle_count,
    tau=1.2,
    k=0.04,
):
    """
    車両台数から待ち時間を計算する。

    既存 utilities.py の calculate_wait_time の挙動を維持する。
    """
    return tau * math.exp(k * vehicle_count)


def calculate_speed(
    wait_time,
    max_speed=7,
    v_min=0.28,
):
    """
    待ち時間から速度を計算する。

    既存 utilities.py の calculate_speed の挙動を維持する。
    """
    return max(max_speed / (1 + wait_time), v_min)


def density_to_coeff(local_density: float) -> float:
    """
    局所密度に応じて速度係数を返す。

    仕様:
      - 返り値は 0.4〜1.0
      - 密度 0.3 程度までは影響を小さくする
      - それ以降はシグモイド的に低下する

    既存 utilities.py の density_to_coeff の挙動を維持する。
    """
    k = 12.0
    inflection = 0.35
    slow_factor = 1 / (1 + math.exp(k * (local_density - inflection)))
    coeff = 0.4 + 0.6 * slow_factor

    return max(0.4, min(1.0, coeff))


def speed_from_gap(
    gap: float,
    v_free: float,
    v_min: float,
    gap_min: float,
    tau: float,
) -> float:
    """
    前方車との gap から基本速度を計算する。

    - gap が None の場合は v_free
    - gap <= gap_min では v_min
    - gap が大きいほど (gap-gap_min)/tau に比例して速度上昇

    既存 utilities.py の speed_from_gap の挙動を維持する。
    """
    if gap is None:
        return v_free

    if gap <= gap_min:
        return v_min

    v = (gap - gap_min) / max(tau, 1e-3)

    return max(v_min, min(v_free, v))


# --- 平滑化処理（ジャーク抑制） ---
_prev_speed_cmd = {}


def smooth_speed_command(
    vehID: str,
    v_des: float,
    alpha: float = 0.5,
) -> float:
    """
    前回速度との指数移動平均をとることで急加減速を防ぐ。

    alpha を下げるほど平滑になる。
    既存 utilities.py の smooth_speed_command の挙動を維持する。
    """
    v_prev = _prev_speed_cmd.get(vehID, v_des)
    v_cmd = (1 - alpha) * v_prev + alpha * v_des
    _prev_speed_cmd[vehID] = v_cmd

    return v_cmd


def apply_gap_density_speed_control(
    vehID: str,
    local_density: float,
    v_free: float,
    v_min: float,
    gap_min: float,
    tau: float,
    alpha: float,
    slow_time: float,
):
    """
    gap と局所密度を合成して速度を制御する。

    処理:
      1. 前方車までの gap から目標速度 v_gap を算出
      2. 局所密度から係数を掛けて v_des を決定
      3. 平滑化して jerk を抑え、slowDown で適用

    既存 utilities.py の apply_gap_density_speed_control の挙動を維持する。
    """
    lead_info = traci.vehicle.getLeader(vehID)
    gap = lead_info[1] if lead_info is not None else None

    v_gap = speed_from_gap(
        gap,
        v_free=v_free,
        v_min=v_min,
        gap_min=gap_min,
        tau=tau,
    )

    coeff = density_to_coeff(local_density)
    v_des = v_gap * coeff
    v_cmd = smooth_speed_command(vehID, v_des, alpha=alpha)

    traci.vehicle.slowDown(
        vehID,
        v_cmd,
        max(0.1, slow_time),
    )
