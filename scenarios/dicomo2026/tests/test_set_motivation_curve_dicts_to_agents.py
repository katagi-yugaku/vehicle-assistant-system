from pathlib import Path

import matplotlib

matplotlib.use("Agg")

import matplotlib.pyplot as plt
import pytest

from evacsim.core.motivation import (
    generate_info_activation_dict,
    generate_motivation_curve,
    set_motivation_curve_dicts_to_agents,
)


RHO = {
    "tsu": 0.08,
    "jam": 0.02,
    "full": 0.05,
}

DELTA = {
    "tsu": 20.0,
    "jam": 60.0,
    "full": 30.0,
}

TEST_DIR = Path(__file__).resolve().parent
OUTPUT_DIR = TEST_DIR / "test_result_fig" / "set_motivation_curve_dicts_to_agents"


def dict_to_xy(value_by_time_dict: dict[float, float]):
    x = sorted(value_by_time_dict.keys())
    y = [value_by_time_dict[t] for t in x]
    return x, y


def plot_single_agent_motivation_curves(agent, output_dir: Path):
    """
    1体の Agent が保持する motivation 曲線を plot する。

    ここで描画する tsu / jam / full は，
    0〜1 の activation ではなく，
    agent.normalcy_value_about_* を掛けた後の motivation 値である。
    """

    output_dir.mkdir(parents=True, exist_ok=True)

    veh_id = agent.get_vehID()

    base_dict = agent.get_base_motivation_value_by_elapsed_time_dict()
    tsu_dict = agent.get_tsunami_precursor_normalcy_value_by_elapsed_time_dict()
    jam_dict = agent.get_route_congestion_normalcy_value_by_elapsed_time_dict()
    full_dict = agent.get_shelter_full_normalcy_value_by_elapsed_time_dict()

    base_x, base_y = dict_to_xy(base_dict)
    tsu_x, tsu_y = dict_to_xy(tsu_dict)
    jam_x, jam_y = dict_to_xy(jam_dict)
    full_x, full_y = dict_to_xy(full_dict)

    plt.figure(figsize=(9, 5.5))

    plt.plot(base_x, base_y, label="base motivation")
    plt.plot(tsu_x, tsu_y, label="tsunami info motivation")
    plt.plot(jam_x, jam_y, label="route congestion info motivation")
    plt.plot(full_x, full_y, label="shelter full info motivation")

    plt.xlabel("Elapsed time [s]")
    plt.ylabel("Motivation value")
    plt.grid(True)
    plt.legend(fontsize=8)
    plt.tight_layout()

    pdf_path = output_dir / f"{veh_id}_motivation_curves.pdf"
    png_path = output_dir / f"{veh_id}_motivation_curves.png"

    plt.savefig(pdf_path)
    plt.savefig(png_path, dpi=300)
    plt.close()

    return pdf_path, png_path


def plot_all_agents_info_motivation_overlay(agent_list, output_dir: Path):
    """
    全 Agent の tsu / jam / full motivation 曲線を重ねて plot する。
    Agent ごとの normalcy 値の違いにより，曲線の高さが変わることを確認する。
    """

    output_dir.mkdir(parents=True, exist_ok=True)

    plt.figure(figsize=(10, 6))

    for agent in agent_list:
        veh_id = agent.get_vehID()

        tsu_dict = agent.get_tsunami_precursor_normalcy_value_by_elapsed_time_dict()
        jam_dict = agent.get_route_congestion_normalcy_value_by_elapsed_time_dict()
        full_dict = agent.get_shelter_full_normalcy_value_by_elapsed_time_dict()

        tsu_x, tsu_y = dict_to_xy(tsu_dict)
        jam_x, jam_y = dict_to_xy(jam_dict)
        full_x, full_y = dict_to_xy(full_dict)

        plt.plot(tsu_x, tsu_y, label=f"{veh_id} tsu", alpha=0.45)
        plt.plot(jam_x, jam_y, label=f"{veh_id} jam", alpha=0.45)
        plt.plot(full_x, full_y, label=f"{veh_id} full", alpha=0.45)

    plt.xlabel("Elapsed time [s]")
    plt.ylabel("Information-based motivation value")
    plt.grid(True)
    plt.legend(fontsize=6, ncol=2)
    plt.tight_layout()

    pdf_path = output_dir / "all_agents_info_motivation_overlay.pdf"
    png_path = output_dir / "all_agents_info_motivation_overlay.png"

    plt.savefig(pdf_path)
    plt.savefig(png_path, dpi=300)
    plt.close()

    return pdf_path, png_path


def test_set_motivation_curve_dicts_to_agents_sets_dicts(agent_list):
    """
    set_motivation_curve_dicts_to_agents() によって，
    各 Agent に base / tsu / jam / full の dict が設定されることを確認する。
    """

    set_motivation_curve_dicts_to_agents(
        agent_list=agent_list,
        rho=RHO,
        delta=DELTA,
    )

    for agent in agent_list:
        assert len(agent.get_base_motivation_value_by_elapsed_time_dict()) == 450
        assert len(agent.get_tsunami_precursor_normalcy_value_by_elapsed_time_dict()) == 450
        assert len(agent.get_route_congestion_normalcy_value_by_elapsed_time_dict()) == 450
        assert len(agent.get_shelter_full_normalcy_value_by_elapsed_time_dict()) == 450


def test_info_activation_values_are_multiplied_by_agent_normalcy_values(agent_list):
    """
    generate_info_activation_dict() の 0〜1 の値に対して，
    Agent ごとの normalcy_value_about_* が掛けられていることを確認する。
    """

    expected_tsu_activation_dict = generate_info_activation_dict(
        rho=RHO["tsu"],
        delta=DELTA["tsu"],
        max_time=450,
        step=1,
    )

    expected_jam_activation_dict = generate_info_activation_dict(
        rho=RHO["jam"],
        delta=DELTA["jam"],
        max_time=450,
        step=1,
    )

    expected_full_activation_dict = generate_info_activation_dict(
        rho=RHO["full"],
        delta=DELTA["full"],
        max_time=450,
        step=1,
    )

    set_motivation_curve_dicts_to_agents(
        agent_list=agent_list,
        rho=RHO,
        delta=DELTA,
    )

    check_times = [0.0, 20.0, 60.0, 120.0, 300.0, 449.0]

    for agent in agent_list:
        actual_tsu_dict = agent.get_tsunami_precursor_normalcy_value_by_elapsed_time_dict()
        actual_jam_dict = agent.get_route_congestion_normalcy_value_by_elapsed_time_dict()
        actual_full_dict = agent.get_shelter_full_normalcy_value_by_elapsed_time_dict()

        for t in check_times:
            expected_tsu_value = (
                expected_tsu_activation_dict[t]
                * agent.get_normalcy_value_about_tsunami_precursor_info()
            )

            expected_jam_value = (
                expected_jam_activation_dict[t]
                * agent.get_normalcy_value_about_route_congestion_info()
            )

            expected_full_value = (
                expected_full_activation_dict[t]
                * agent.get_normalcy_value_about_shelter_full_info()
            )

            assert actual_tsu_dict[t] == pytest.approx(expected_tsu_value)
            assert actual_jam_dict[t] == pytest.approx(expected_jam_value)
            assert actual_full_dict[t] == pytest.approx(expected_full_value)


def test_plot_set_motivation_curve_dicts_to_agents(agent_list):
    """
    set_motivation_curve_dicts_to_agents() の結果を plot して，
    PDF / PNG として保存する。

    このテストは数値検証だけでなく，
    曲線の形を目視確認するためのテストである。
    """

    set_motivation_curve_dicts_to_agents(
        agent_list=agent_list,
        rho=RHO,
        delta=DELTA,
    )

    saved_files = []

    for agent in agent_list:
        pdf_path, png_path = plot_single_agent_motivation_curves(
            agent=agent,
            output_dir=OUTPUT_DIR / "each_agent",
        )
        saved_files.extend([pdf_path, png_path])

    overlay_pdf_path, overlay_png_path = plot_all_agents_info_motivation_overlay(
        agent_list=agent_list,
        output_dir=OUTPUT_DIR,
    )

    saved_files.extend([overlay_pdf_path, overlay_png_path])

    for path in saved_files:
        assert path.exists(), f"plot file was not created: {path}"
