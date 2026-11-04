# =========================
# Agent visualization helpers
# =========================

from __future__ import annotations

from typing import TYPE_CHECKING

import matplotlib.pyplot as plt

if TYPE_CHECKING:
    from evacsim.agents.Agent import Agent


def plot_dot(agent: Agent):
    """
    agent のレーン変更モチベーション推移を描画する。

    既存 utilities.py の plot_dot の挙動を維持する。
    """
    fig, ax = plt.subplots(figsize=(6, 5))

    # --- 折れ線グラフ ---
    ax.plot(
        agent.get_x_elapsed_time_for_lane_change_list(),
        agent.get_y_motivation_value_for_lane_change_list(),
        linewidth=2,
        label="Motivation over time",
    )

    print(
        f"agent.get_lane_change_time(){agent.get_lane_change_time()}, "
        f"agent.get_calculated_motivation_value(){agent.get_calculated_motivation_value()}"
    )

    y_thr = agent.get_lane_change_decision_threshold()
    x_reach_min_motivation = agent.get_reach_lane_minimum_motivation_time()

    ax.axhline(
        y=y_thr,
        color="red",
        linestyle="--",
        linewidth=1.5,
        label=f"Threshold = {y_thr:.2f}",
    )

    ax.axvline(
        x=x_reach_min_motivation,
        color="red",
        linestyle="--",
        linewidth=1.5,
        label=f"reach min time = {y_thr:.2f}",
    )

    try:
        y_cur = agent.get_calculated_motivation_value()
        ax.axhline(
            y=y_cur,
            color="green",
            linestyle="--",
            linewidth=1.5,
            label=f"Current Value = {y_cur:.2f}",
        )
    except Exception:
        pass

    # --- グラフ設定 ---
    ax.legend(loc="best")
    ax.set_xlabel("X")
    ax.set_ylabel("Y")
    ax.grid(True, linestyle="--", alpha=0.5)
    plt.tight_layout()
    plt.show()
