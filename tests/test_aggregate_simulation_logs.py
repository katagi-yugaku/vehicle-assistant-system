from pathlib import Path
import sys
import pytest

# リポジトリルートに aggregate_simulation_logs.py がある前提
ROOT_DIR = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT_DIR))

import aggregate_simulation_logs as agg


SAMPLE_LOG_TEXT = """
===== Simlation Result Summary =====
avg_congestion_duration: 456.65
arrival_time_by_vehID_dict:{'init_ShelterA_1_0': 128.0, 'init_ShelterA_1_1': 145.0, 'ped_init_ShelterB_1_246_1': 786.0}
vehicle_abandant_time_by_pedestrianID_dict:{}
walking_distance_by_pedestrianID_dict:{}
pedestrian_count:61
route_changed_vehicle_count:174
wrong_way_driving_count:61
vehicle_abandonment_count:61
normalcy_bias_route_change_count:0
majority_bias_route_change_count:0
lane_changed_vehicle_count:0
info_obtained_lanechange_count:0
elapsed_time_lanechange_count:0
majority_bias_lanechange_count:0
"""


def test_parse_out_filename_success():
    path = "/tmp/logs/va_s1_e0.5_v1.0_r12_98765.out"

    file_info = agg.parse_out_filename(path)

    assert file_info is not None
    assert file_info.scenario == 1
    assert file_info.early_rate == pytest.approx(0.5)
    assert file_info.v2v_rate == pytest.approx(1.0)
    assert file_info.run_id == 12
    assert file_info.jobid == 98765
    assert file_info.condition_key == "s1_e0.5_v1.0"


def test_parse_out_filename_invalid_name_is_skipped():
    path = "/tmp/logs/slurm-12345.out"

    file_info = agg.parse_out_filename(path)

    assert file_info is None


def test_normalize_vehicle_id_for_pedestrian_id():
    assert (
        agg.normalize_vehicle_id("ped_init_ShelterB_1_246_1")
        == "init_ShelterB_1_246"
    )

    assert (
        agg.normalize_vehicle_id("ped_init_ShelterA_1_55_3")
        == "init_ShelterA_1_55"
    )


def test_normalize_vehicle_id_for_normal_vehicle_id():
    assert (
        agg.normalize_vehicle_id("init_ShelterA_1_0")
        == "init_ShelterA_1_0"
    )


def test_parse_log_content_extracts_scalar_values():
    count_values, dict_values, messages = agg.parse_log_content(SAMPLE_LOG_TEXT)

    assert count_values["avg_congestion_duration"] == pytest.approx(456.65)
    assert count_values["pedestrian_count"] == pytest.approx(61)
    assert count_values["route_changed_vehicle_count"] == pytest.approx(174)
    assert count_values["wrong_way_driving_count"] == pytest.approx(61)
    assert count_values["vehicle_abandonment_count"] == pytest.approx(61)

    assert count_values["normalcy_bias_route_change_count"] == pytest.approx(0)
    assert count_values["majority_bias_route_change_count"] == pytest.approx(0)
    assert count_values["lane_changed_vehicle_count"] == pytest.approx(0)
    assert count_values["info_obtained_lanechange_count"] == pytest.approx(0)
    assert count_values["elapsed_time_lanechange_count"] == pytest.approx(0)
    assert count_values["majority_bias_lanechange_count"] == pytest.approx(0)

    assert messages == []


def test_parse_log_content_extracts_arrival_time_dict_and_normalizes_ped_id():
    count_values, dict_values, messages = agg.parse_log_content(SAMPLE_LOG_TEXT)

    arrival_dict = dict_values["arrival_time_by_vehID_dict"]

    assert arrival_dict["init_ShelterA_1_0"] == pytest.approx(128.0)
    assert arrival_dict["init_ShelterA_1_1"] == pytest.approx(145.0)

    # ped_init_..._末尾番号 が init_... に正規化されることを確認
    assert arrival_dict["init_ShelterB_1_246"] == pytest.approx(786.0)

    assert "ped_init_ShelterB_1_246_1" not in arrival_dict


def test_parse_log_content_empty_abandonment_dicts():
    count_values, dict_values, messages = agg.parse_log_content(SAMPLE_LOG_TEXT)

    assert dict_values["vehicle_abandant_time_by_pedestrianID_dict"] == {}
    assert dict_values["walking_distance_by_pedestrianID_dict"] == {}


def test_compute_cdf_points():
    values = [300.0, 100.0, 200.0]

    x, y = agg.compute_cdf_points(values)

    assert x == [100.0, 200.0, 300.0]
    assert y == pytest.approx([1 / 3, 2 / 3, 1.0])


def test_build_histogram_from_values():
    values = [810.0, 850.0, 920.0, 1200.0]

    result = agg.build_histogram_from_values(
        values,
        [800, 900, 1000, 1100, 1200, 1300],
    )

    assert result["bin_width"] == 100
    assert result["bin_edges"] == [800, 900, 1000, 1100, 1200, 1300]

    # [800,900): 810, 850
    # [900,1000): 920
    # [1000,1100): none
    # [1100,1200): none
    # [1200,1300]: 1200
    assert result["counts"] == [2, 1, 0, 0, 1]
    assert result["ratios"] == pytest.approx([0.5, 0.25, 0.0, 0.0, 0.25])


def test_aggregate_condition_single_run(tmp_path):
    log_path = tmp_path / "va_s1_e0.5_v1.0_r1_12345.out"
    log_path.write_text(SAMPLE_LOG_TEXT, encoding="utf-8")

    file_info = agg.LogFileInfo(
        path=str(log_path),
        filename=log_path.name,
        scenario=1,
        early_rate=0.5,
        v2v_rate=1.0,
        run_id=1,
        jobid=12345,
    )

    result = agg.aggregate_condition(
        condition_key="s1_e0.5_v1.0",
        files=[file_info],
        output_dir=str(tmp_path),
        expected_max_run_id=3,
    )

    assert result["scenario"] == 1
    assert result["early_rate"] == pytest.approx(0.5)
    assert result["v2v_rate"] == pytest.approx(1.0)

    assert result["run_status"]["expected_run_ids"] == [1, 2, 3]
    assert result["run_status"]["found_run_ids"] == [1]
    assert result["run_status"]["missing_run_ids"] == [2, 3]
    assert result["run_status"]["parsed_run_ids"] == [1]
    assert result["run_status"]["failed_run_ids"] == []

    assert result["count_averages"]["avg_congestion_duration"] == pytest.approx(456.65)
    assert result["count_averages"]["pedestrian_count"] == pytest.approx(61)
    assert result["count_averages"]["route_changed_vehicle_count"] == pytest.approx(174)
    assert result["count_averages"]["wrong_way_driving_count"] == pytest.approx(61)
    assert result["count_averages"]["vehicle_abandonment_count"] == pytest.approx(61)

    vehicle_mean_arrival_time = result["arrival_time"]["vehicle_mean_arrival_time"]

    assert vehicle_mean_arrival_time["init_ShelterA_1_0"] == pytest.approx(128.0)
    assert vehicle_mean_arrival_time["init_ShelterA_1_1"] == pytest.approx(145.0)
    assert vehicle_mean_arrival_time["init_ShelterB_1_246"] == pytest.approx(786.0)

    assert result["arrival_time"]["arrival_time_list"] == [128.0, 145.0, 786.0]

    assert result["abandonment"]["mean_abandon_time"] is None
    assert result["abandonment"]["mean_walking_distance"] is None
    assert result["abandonment"]["all_abandon_time_events"] == []


def test_aggregate_condition_two_runs_averages_values(tmp_path):
    log_text_run1 = SAMPLE_LOG_TEXT

    log_text_run2 = """
===== Simlation Result Summary =====
avg_congestion_duration: 500.35
arrival_time_by_vehID_dict:{'init_ShelterA_1_0': 132.0, 'init_ShelterA_1_1': 155.0, 'ped_init_ShelterB_1_246_2': 800.0}
vehicle_abandant_time_by_pedestrianID_dict:{}
walking_distance_by_pedestrianID_dict:{}
pedestrian_count:66
route_changed_vehicle_count:178
wrong_way_driving_count:55
vehicle_abandonment_count:66
normalcy_bias_route_change_count:0
majority_bias_route_change_count:0
lane_changed_vehicle_count:0
info_obtained_lanechange_count:0
elapsed_time_lanechange_count:0
majority_bias_lanechange_count:0
"""

    log_path1 = tmp_path / "va_s1_e0.5_v1.0_r1_12345.out"
    log_path2 = tmp_path / "va_s1_e0.5_v1.0_r2_12346.out"

    log_path1.write_text(log_text_run1, encoding="utf-8")
    log_path2.write_text(log_text_run2, encoding="utf-8")

    files = [
        agg.LogFileInfo(
            path=str(log_path1),
            filename=log_path1.name,
            scenario=1,
            early_rate=0.5,
            v2v_rate=1.0,
            run_id=1,
            jobid=12345,
        ),
        agg.LogFileInfo(
            path=str(log_path2),
            filename=log_path2.name,
            scenario=1,
            early_rate=0.5,
            v2v_rate=1.0,
            run_id=2,
            jobid=12346,
        ),
    ]

    result = agg.aggregate_condition(
        condition_key="s1_e0.5_v1.0",
        files=files,
        output_dir=str(tmp_path),
        expected_max_run_id=2,
    )

    assert result["run_status"]["missing_run_ids"] == []
    assert result["run_status"]["parsed_run_ids"] == [1, 2]

    assert result["count_averages"]["avg_congestion_duration"] == pytest.approx(
        (456.65 + 500.35) / 2
    )
    assert result["count_averages"]["pedestrian_count"] == pytest.approx(
        (61 + 66) / 2
    )
    assert result["count_averages"]["route_changed_vehicle_count"] == pytest.approx(
        (174 + 178) / 2
    )
    assert result["count_averages"]["wrong_way_driving_count"] == pytest.approx(
        (61 + 55) / 2
    )
    assert result["count_averages"]["vehicle_abandonment_count"] == pytest.approx(
        (61 + 66) / 2
    )

    vehicle_mean_arrival_time = result["arrival_time"]["vehicle_mean_arrival_time"]

    assert vehicle_mean_arrival_time["init_ShelterA_1_0"] == pytest.approx(
        (128.0 + 132.0) / 2
    )
    assert vehicle_mean_arrival_time["init_ShelterA_1_1"] == pytest.approx(
        (145.0 + 155.0) / 2
    )

    # ped_init_ShelterB_1_246_1 と ped_init_ShelterB_1_246_2 は
    # どちらも init_ShelterB_1_246 に正規化される
    assert vehicle_mean_arrival_time["init_ShelterB_1_246"] == pytest.approx(
        (786.0 + 800.0) / 2
    )

    assert result["arrival_time"]["arrival_time_list"] == pytest.approx(
        [130.0, 150.0, 793.0]
    )


def test_get_requested_output_key():
    assert agg.get_requested_output_key(0.1, 1.0) == "0.1"
    assert agg.get_requested_output_key(0.5, 1.0) == "0.5"
    assert agg.get_requested_output_key(0.9, 1.0) == "0.9"
    assert agg.get_requested_output_key(0.5, 0.0) == "nosystem"

    assert agg.get_requested_output_key(0.2, 1.0) is None
    assert agg.get_requested_output_key(0.1, 0.0) is None