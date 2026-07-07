# tests/test_motivation_model.py
# NOTE:
# MODULE_PATH は実際の配置に合わせて修正すること。
#
# 例:
#   MODULE_PATH = "evacsim.core.motivation_model"
#   MODULE_PATH = "evacsim.core.decision"
#   MODULE_PATH = "evacsim.sim.motivation"
#
# または，環境変数で指定して実行できます。
#   MOTIVATION_MODULE_PATH=evacsim.core.motivation_model pytest tests/test_motivation_model.py -v

from __future__ import annotations

import importlib
import os
from dataclasses import dataclass, field
from pathlib import Path

import matplotlib

matplotlib.use("Agg")

import matplotlib.pyplot as plt
import numpy as np
import pytest


# =========================
# Import target module
# =========================

MODULE_PATH = os.environ.get("MOTIVATION_MODULE_PATH")

CANDIDATE_MODULE_PATHS = [
    MODULE_PATH,
    "evacsim.core.motivation_model",
    "evacsim.core.motivation",
    "evacsim.sim.motivation_model",
    "evacsim.sim.motivation",
    "evacsim.utilities",
    "utilities",
]


def import_target_module():
    import_errors: list[str] = []

    for module_path in CANDIDATE_MODULE_PATHS:
        if not module_path:
            continue

        try:
            return importlib.import_module(module_path)
        except Exception as exc:
            import_errors.append(
                f"{module_path}: {type(exc).__name__}: {exc}"
            )

    raise RuntimeError(
        "テスト対象モジュールを import できませんでした。\n"
        "MODULE_PATH または MOTIVATION_MODULE_PATH を実際の配置に合わせて修正してください。\n\n"
        "試行した import:\n"
        + "\n".join(import_errors)
    )


motivation_model = import_target_module()


# =========================
# Output directory for figures
# =========================

TEST_DIR = Path(__file__).resolve().parent
OUTPUT_DIR = TEST_DIR / "test_result_fig"
OUTPUT_DIR.mkdir(parents=True, exist_ok=True)


# =========================
# Fake Agent
# =========================

@dataclass
class FakeAgent:
    veh_id: str
    action_name: str = "none"

    congestion_duration: float = 0.0

    base_motivation_dict: dict[float, float] = field(
        default_factory=lambda: {0.0: 0.0}
    )

    shelter_full_dict: dict[float, float] = field(
        default_factory=lambda: {0.0: 0.0}
    )
    tsunami_dict: dict[float, float] = field(
        default_factory=lambda: {0.0: 0.0}
    )
    route_congestion_dict: dict[float, float] = field(
        default_factory=lambda: {0.0: 0.0}
    )

    shelter_full_info_obtained_time: float = 200.0
    tsunami_info_obtained_time: float = 180.0
    route_congestion_info_obtained_time: float = 150.0

    normalcy_value_about_shelter_full_info: float = 1.0
    normalcy_value_about_tsunami_precursor_info: float = 1.0
    normalcy_value_about_route_congestion_info: float = 1.0

    majority_value_increase: float = 10.0
    majority_value_decrease: float = 5.0

    def get_vehID(self) -> str:
        return self.veh_id

    def get_agent_action_name(self) -> str:
        return self.action_name

    def get_congestion_duration(self) -> float:
        return self.congestion_duration

    def set_congestion_duration(self, value: float) -> None:
        self.congestion_duration = float(value)

    def get_base_motivation_value_by_elapsed_time_dict(self) -> dict[float, float]:
        return self.base_motivation_dict

    def get_shelter_full_normalcy_value_by_elapsed_time_dict(self) -> dict[float, float]:
        return self.shelter_full_dict

    def get_tsunami_precursor_normalcy_value_by_elapsed_time_dict(self) -> dict[float, float]:
        return self.tsunami_dict

    def get_route_congestion_normalcy_value_by_elapsed_time_dict(self) -> dict[float, float]:
        return self.route_congestion_dict

    def get_shelter_full_info_obtained_time(self) -> float:
        return self.shelter_full_info_obtained_time

    def get_tsunami_info_obtained_time(self) -> float:
        return self.tsunami_info_obtained_time

    def get_route_congestion_info_obtained_time(self) -> float:
        return self.route_congestion_info_obtained_time

    def get_normalcy_value_about_shelter_full_info(self) -> float:
        return self.normalcy_value_about_shelter_full_info

    def get_normalcy_value_about_tsunami_precursor_info(self) -> float:
        return self.normalcy_value_about_tsunami_precursor_info

    def get_normalcy_value_about_route_congestion_info(self) -> float:
        return self.normalcy_value_about_route_congestion_info

    def get_majority_value_increase(self) -> float:
        return self.majority_value_increase

    def get_majority_value_decrease(self) -> float:
        return self.majority_value_decrease


# =========================
# Fixtures
# =========================

@pytest.fixture
def agent_for_info_test() -> FakeAgent:
    """
    情報取得時刻を 0 秒にしないテスト用 Agent。

    current_time = 300.0 のとき:
      shelter_full elapsed = 100.0
      tsunami elapsed = 120.0
      route_congestion elapsed = 150.0
    """
    return FakeAgent(
        veh_id="ego",
        action_name="none",
        congestion_duration=10.0,
        base_motivation_dict={
            0.0: 0.0,
            10.0: 100.0,
        },
        shelter_full_dict={
            0.0: 0.0,
            100.0: 30.0,
        },
        tsunami_dict={
            0.0: 0.0,
            120.0: 70.0,
        },
        route_congestion_dict={
            0.0: 0.0,
            150.0: 50.0,
        },
        shelter_full_info_obtained_time=200.0,
        tsunami_info_obtained_time=180.0,
        route_congestion_info_obtained_time=150.0,
        normalcy_value_about_shelter_full_info=2.0,
        majority_value_increase=20.0,
        majority_value_decrease=5.0,
    )


# =========================
# two_stage_sigmoid
# =========================

def test_two_stage_sigmoid_increases_and_stays_in_range() -> None:
    times = [0.0, 100.0, 200.0, 330.0, 450.0]
    values = [motivation_model.two_stage_sigmoid(t) for t in times]

    assert all(isinstance(v, (float, np.floating)) for v in values)
    assert all(0.0 <= float(v) <= 1000.0 for v in values)

    # 入力が大きくなると概ね増加すること
    assert all(
        later >= earlier
        for earlier, later in zip(values, values[1:])
    )


def test_two_stage_sigmoid_returns_float_like_value_for_typical_time() -> None:
    value = motivation_model.two_stage_sigmoid(200.0)

    assert isinstance(value, (float, np.floating))
    assert 0.0 <= float(value) <= 1000.0


# =========================
# generate_motivation_curve
# =========================

def test_generate_motivation_curve_returns_time_value_dict() -> None:
    curve = motivation_model.generate_motivation_curve(
        max_time=10,
        step=1,
    )

    assert isinstance(curve, dict)
    assert len(curve) == 10

    assert all(isinstance(k, float) for k in curve.keys())
    assert all(isinstance(v, float) for v in curve.values())

    assert all(0.0 <= v <= 1000.0 for v in curve.values())

    assert list(curve.keys()) == [
        0.0,
        1.0,
        2.0,
        3.0,
        4.0,
        5.0,
        6.0,
        7.0,
        8.0,
        9.0,
    ]


# =========================
# sigmoid
# =========================

def test_sigmoid_basic_values_and_no_overflow() -> None:
    values = motivation_model.sigmoid(
        np.array([0.0, 1000.0, -1000.0])
    )

    assert values[0] == pytest.approx(0.5)
    assert values[1] > 0.999
    assert values[2] < 0.001

    assert np.all(np.isfinite(values))
    assert np.all(values >= 0.0)
    assert np.all(values <= 1.0)


# =========================
# generate_info_activation_dict
# =========================

def test_generate_info_activation_dict_returns_activation_dict() -> None:
    activation = motivation_model.generate_info_activation_dict(
        rho=0.1,
        delta=3.0,
        max_time=10,
        step=1,
    )

    assert isinstance(activation, dict)
    assert len(activation) == 10

    assert all(isinstance(k, float) for k in activation.keys())
    assert all(isinstance(v, float) for v in activation.values())

    assert all(0.0 <= v <= 1.0 for v in activation.values())

    values = list(activation.values())

    # 時間経過に伴って概ね増加すること
    assert all(
        later >= earlier
        for earlier, later in zip(values, values[1:])
    )


# =========================
# multiply_time_dict
# =========================

def test_multiply_time_dict_multiplies_values_without_mutating_original() -> None:
    original = {
        0.0: 1.0,
        1.0: 2.0,
        2.0: 3.0,
    }
    original_copy = original.copy()

    multiplied = motivation_model.multiply_time_dict(
        value_by_time_dict=original,
        coefficient=10.0,
    )

    assert multiplied == {
        0.0: 10.0,
        1.0: 20.0,
        2.0: 30.0,
    }

    # 元の dict を破壊しないこと
    assert original == original_copy


# =========================
# get_value_from_time_dict
# =========================

def test_get_value_from_time_dict_returns_default_for_empty_dict() -> None:
    value = motivation_model.get_value_from_time_dict(
        value_by_time_dict={},
        elapsed_time=10.0,
        default=123.0,
    )

    assert value == pytest.approx(123.0)


def test_get_value_from_time_dict_returns_value_of_largest_key_under_elapsed_time() -> None:
    value_by_time = {
        0.0: 10.0,
        10.0: 20.0,
        20.0: 30.0,
    }

    value = motivation_model.get_value_from_time_dict(
        value_by_time_dict=value_by_time,
        elapsed_time=15.0,
        default=0.0,
    )

    assert value == pytest.approx(20.0)


def test_get_value_from_time_dict_returns_min_key_value_when_elapsed_time_is_smaller_than_min_key() -> None:
    value_by_time = {
        10.0: 20.0,
        20.0: 30.0,
    }

    value = motivation_model.get_value_from_time_dict(
        value_by_time_dict=value_by_time,
        elapsed_time=5.0,
        default=0.0,
    )

    assert value == pytest.approx(20.0)


def test_get_value_from_time_dict_returns_max_key_value_when_elapsed_time_is_larger_than_max_key() -> None:
    value_by_time = {
        0.0: 10.0,
        10.0: 20.0,
        20.0: 30.0,
    }

    value = motivation_model.get_value_from_time_dict(
        value_by_time_dict=value_by_time,
        elapsed_time=999.0,
        default=0.0,
    )

    assert value == pytest.approx(30.0)


def test_get_value_from_time_dict_ignores_non_numeric_keys() -> None:
    value_by_time = {
        "invalid": 999.0,
        0.0: 10.0,
        10.0: 20.0,
    }

    value = motivation_model.get_value_from_time_dict(
        value_by_time_dict=value_by_time,
        elapsed_time=10.0,
        default=0.0,
    )

    assert value == pytest.approx(20.0)


def test_get_value_from_time_dict_returns_default_when_all_keys_are_non_numeric() -> None:
    value_by_time = {
        "invalid": 999.0,
        "also_invalid": 1000.0,
    }

    value = motivation_model.get_value_from_time_dict(
        value_by_time_dict=value_by_time,
        elapsed_time=10.0,
        default=77.0,
    )

    assert value == pytest.approx(77.0)


# =========================
# calculate_motivation_for_obtained_info
# =========================

@pytest.mark.parametrize(
    ("action", "expected"),
    [
        # 現在の実装では rc の shelter_full 側だけ
        # get_normalcy_value_about_shelter_full_info() がさらに掛けられる。
        # shelter_full: 30.0, normalcy: 2.0, route: 50.0
        # rc = 2.0 * 30.0 + 50.0 = 110.0
        ("rc", 110.0),

        # ww = route_congestion + tsunami
        # ww = 50.0 + 70.0 = 120.0
        ("ww", 120.0),

        # va = shelter_full + tsunami
        # va = 30.0 + 70.0 = 100.0
        ("va", 100.0),
    ],
)
def test_calculate_motivation_for_obtained_info_by_action(
    agent_for_info_test: FakeAgent,
    action: str,
    expected: float,
) -> None:
    value = motivation_model.calculate_motivation_for_obtained_info(
        agent=agent_for_info_test,
        current_time=300.0,
        action=action,
    )

    assert value == pytest.approx(expected)


def test_calculate_motivation_for_obtained_info_raises_value_error_for_invalid_action(
    agent_for_info_test: FakeAgent,
) -> None:
    with pytest.raises(ValueError):
        motivation_model.calculate_motivation_for_obtained_info(
            agent=agent_for_info_test,
            current_time=300.0,
            action="invalid_action",
        )


def test_calculate_motivation_for_obtained_info_changes_before_and_after_information_time(
    agent_for_info_test: FakeAgent,
) -> None:
    before_value = motivation_model.calculate_motivation_for_obtained_info(
        agent=agent_for_info_test,
        current_time=100.0,
        action="va",
    )

    after_value = motivation_model.calculate_motivation_for_obtained_info(
        agent=agent_for_info_test,
        current_time=300.0,
        action="va",
    )

    assert before_value == pytest.approx(0.0)
    assert after_value > before_value


# =========================
# count_same_action_drivers
# =========================

def test_count_same_action_drivers_counts_only_nearby_same_action_and_excludes_self(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    ego = FakeAgent(veh_id="ego", action_name="none")
    near_same = FakeAgent(veh_id="near_same", action_name="va")
    near_other = FakeAgent(veh_id="near_other", action_name="rc")
    far_same = FakeAgent(veh_id="far_same", action_name="va")

    agent_list = [
        ego,
        near_same,
        near_other,
        far_same,
    ]

    distances = {
        frozenset(("ego", "near_same")): 20.0,
        frozenset(("ego", "near_other")): 45.0,
        frozenset(("ego", "far_same")): 55.0,
    }

    def fake_distance_each_vehIDs(veh_id_1: str, veh_id_2: str) -> float:
        assert veh_id_1 != veh_id_2
        return distances[frozenset((veh_id_1, veh_id_2))]

    monkeypatch.setattr(
        motivation_model,
        "distance_each_vehIDs",
        fake_distance_each_vehIDs,
    )

    same_action_count, total_count = motivation_model.count_same_action_drivers(
        agent=ego,
        vehInfo=0.0,
        agent_list=agent_list,
        candidate_action="va",
    )

    # 50m 以内: near_same, near_other の2台
    assert total_count == 2

    # 50m 以内かつ candidate_action == "va": near_same の1台
    assert same_action_count == 1


# =========================
# calculate_motivation_for_majority_tunning_bias
# =========================

def test_calculate_motivation_for_majority_tunning_bias_returns_negative_decrease_when_no_same_action(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    ego = FakeAgent(
        veh_id="ego",
        action_name="none",
        majority_value_increase=20.0,
        majority_value_decrease=7.0,
    )
    near_other = FakeAgent(veh_id="near_other", action_name="rc")

    def fake_distance_each_vehIDs(veh_id_1: str, veh_id_2: str) -> float:
        return 10.0

    monkeypatch.setattr(
        motivation_model,
        "distance_each_vehIDs",
        fake_distance_each_vehIDs,
    )

    value = motivation_model.calculate_motivation_for_majority_tunning_bias(
        agent=ego,
        vehInfo=0.0,
        agent_list=[ego, near_other],
        candidate_action="va",
    )

    assert value == pytest.approx(-7.0)


def test_calculate_motivation_for_majority_tunning_bias_returns_same_action_count_times_increase(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    ego = FakeAgent(
        veh_id="ego",
        action_name="none",
        majority_value_increase=20.0,
        majority_value_decrease=7.0,
    )
    near_same_1 = FakeAgent(veh_id="near_same_1", action_name="va")
    near_same_2 = FakeAgent(veh_id="near_same_2", action_name="va")
    near_other = FakeAgent(veh_id="near_other", action_name="rc")

    def fake_distance_each_vehIDs(veh_id_1: str, veh_id_2: str) -> float:
        return 10.0

    monkeypatch.setattr(
        motivation_model,
        "distance_each_vehIDs",
        fake_distance_each_vehIDs,
    )

    value = motivation_model.calculate_motivation_for_majority_tunning_bias(
        agent=ego,
        vehInfo=0.0,
        agent_list=[
            ego,
            near_same_1,
            near_same_2,
            near_other,
        ],
        candidate_action="va",
    )

    assert value == pytest.approx(2 * 20.0)


# =========================
# calculate_motivation_for_evacuation_action
# =========================

@pytest.mark.parametrize(
    ("action", "expected_info_value"),
    [
        ("rc", 110.0),
        ("ww", 120.0),
        ("va", 100.0),
    ],
)
def test_calculate_motivation_for_evacuation_action_can_calculate_each_action(
    agent_for_info_test: FakeAgent,
    action: str,
    expected_info_value: float,
) -> None:
    # agent_list に自分自身しかいない場合，
    # majority_motivation_value は - majority_value_decrease になる。
    value = motivation_model.calculate_motivation_for_evacuation_action(
        agent=agent_for_info_test,
        agent_list=[agent_for_info_test],
        current_time=300.0,
        action=action,
    )

    expected_base_value = 100.0
    expected_majority_value = -5.0

    assert value == pytest.approx(
        expected_base_value
        + expected_info_value
        + expected_majority_value
    )


def test_calculate_motivation_for_evacuation_action_returns_sum_of_base_info_and_majority(
    agent_for_info_test: FakeAgent,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    near_same_1 = FakeAgent(veh_id="near_same_1", action_name="va")
    near_same_2 = FakeAgent(veh_id="near_same_2", action_name="va")
    near_other = FakeAgent(veh_id="near_other", action_name="rc")

    def fake_distance_each_vehIDs(veh_id_1: str, veh_id_2: str) -> float:
        return 10.0

    monkeypatch.setattr(
        motivation_model,
        "distance_each_vehIDs",
        fake_distance_each_vehIDs,
    )

    value = motivation_model.calculate_motivation_for_evacuation_action(
        agent=agent_for_info_test,
        agent_list=[
            agent_for_info_test,
            near_same_1,
            near_same_2,
            near_other,
        ],
        current_time=300.0,
        action="va",
    )

    expected_base_value = 100.0
    expected_info_value = 100.0
    expected_majority_value = 2 * 20.0

    assert value == pytest.approx(
        expected_base_value
        + expected_info_value
        + expected_majority_value
    )


# =========================
# Graph output test
# =========================

# =========================
# Graph output test
# =========================

def test_calculate_motivation_for_evacuation_action_outputs_pdf_graphs(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    """
    calculate_motivation_for_evacuation_action() の時間変化を PDF に保存する。

    同じ図に以下を描画する:
      - base motivation
      - obtained info motivation
      - majority tuning bias
      - total motivation

    情報取得時刻は 0 秒にしない:
      route_congestion_info_obtained_time = 150.0
      tsunami_info_obtained_time = 180.0
      shelter_full_info_obtained_time = 200.0

    同調性バイアスは，周囲車両の行動を時刻に応じて変えることで，
    グラフ上で変化が見えるようにする。
    """
    route_congestion_info_obtained_time = 150.0
    tsunami_info_obtained_time = 180.0
    shelter_full_info_obtained_time = 200.0

    # 周囲車両が candidate_action と同じ行動を取り始める時刻
    first_neighbor_action_time = 220.0
    second_neighbor_action_time = 320.0

    base_motivation_dict = motivation_model.generate_motivation_curve(
        max_time=500,
        step=1,
    )

    shelter_full_activation = motivation_model.generate_info_activation_dict(
        rho=0.04,
        delta=30.0,
        max_time=500,
        step=1,
    )
    tsunami_activation = motivation_model.generate_info_activation_dict(
        rho=0.04,
        delta=30.0,
        max_time=500,
        step=1,
    )
    route_congestion_activation = motivation_model.generate_info_activation_dict(
        rho=0.04,
        delta=30.0,
        max_time=500,
        step=1,
    )

    # グラフ上で変化が見えやすいように，活性化度 0〜1 を
    # motivation 値にスケーリングする。
    shelter_full_motivation_dict = motivation_model.multiply_time_dict(
        shelter_full_activation,
        coefficient=400.0,
    )
    tsunami_motivation_dict = motivation_model.multiply_time_dict(
        tsunami_activation,
        coefficient=500.0,
    )
    route_congestion_motivation_dict = motivation_model.multiply_time_dict(
        route_congestion_activation,
        coefficient=300.0,
    )

    agent = FakeAgent(
        veh_id="ego",
        action_name="none",
        congestion_duration=0.0,
        base_motivation_dict=base_motivation_dict,
        shelter_full_dict=shelter_full_motivation_dict,
        tsunami_dict=tsunami_motivation_dict,
        route_congestion_dict=route_congestion_motivation_dict,
        shelter_full_info_obtained_time=shelter_full_info_obtained_time,
        tsunami_info_obtained_time=tsunami_info_obtained_time,
        route_congestion_info_obtained_time=route_congestion_info_obtained_time,
        normalcy_value_about_shelter_full_info=1.0,

        # 同調性バイアスの線が見えるように，少し大きめの値にする。
        majority_value_increase=120.0,
        majority_value_decrease=80.0,
    )

    near_same_1 = FakeAgent(
        veh_id="near_same_1",
        action_name="none",
    )
    near_same_2 = FakeAgent(
        veh_id="near_same_2",
        action_name="none",
    )
    near_other = FakeAgent(
        veh_id="near_other",
        action_name="other",
    )
    far_same = FakeAgent(
        veh_id="far_same",
        action_name="none",
    )

    agent_list = [
        agent,
        near_same_1,
        near_same_2,
        near_other,
        far_same,
    ]

    def fake_distance_each_vehIDs(veh_id_1: str, veh_id_2: str) -> float:
        """
        ego から far_same だけは 50m より遠い。
        それ以外は 50m 以内にいるものとして扱う。
        """
        pair = frozenset((veh_id_1, veh_id_2))

        if pair == frozenset(("ego", "far_same")):
            return 100.0

        return 10.0

    monkeypatch.setattr(
        motivation_model,
        "distance_each_vehIDs",
        fake_distance_each_vehIDs,
    )

    def update_neighbor_actions(
        current_time: float,
        candidate_action: str,
    ) -> None:
        """
        同調性バイアスの時間変化を可視化するために，
        周囲車両の行動を時刻に応じて変える。

        - 220 秒未満:
            candidate_action と同じ行動の車両は 0 台
            majority = - majority_value_decrease

        - 220 秒以上 320 秒未満:
            candidate_action と同じ行動の車両は 1 台
            majority = 1 * majority_value_increase

        - 320 秒以上:
            candidate_action と同じ行動の車両は 2 台
            majority = 2 * majority_value_increase
        """
        near_same_1.action_name = "none"
        near_same_2.action_name = "none"
        near_other.action_name = "other"
        far_same.action_name = candidate_action

        if current_time >= first_neighbor_action_time:
            near_same_1.action_name = candidate_action

        if current_time >= second_neighbor_action_time:
            near_same_2.action_name = candidate_action

    current_times = np.arange(0.0, 451.0, 1.0)

    output_files = []

    for action in ["va", "rc", "ww"]:
        base_values: list[float] = []
        info_values: list[float] = []
        majority_values: list[float] = []
        total_values: list[float] = []

        for current_time in current_times:
            # 添付コードでは base_value が current_time ではなく
            # agent.get_congestion_duration() から計算される。
            # 時間変化を見るため，テスト用 FakeAgent の congestion_duration を更新する。
            agent.set_congestion_duration(current_time)

            update_neighbor_actions(
                current_time=float(current_time),
                candidate_action=action,
            )

            base_value = motivation_model.get_value_from_time_dict(
                value_by_time_dict=agent.get_base_motivation_value_by_elapsed_time_dict(),
                elapsed_time=agent.get_congestion_duration(),
                default=0.0,
            )

            info_value = motivation_model.calculate_motivation_for_obtained_info(
                agent=agent,
                current_time=float(current_time),
                action=action,
            )

            majority_value = motivation_model.calculate_motivation_for_majority_tunning_bias(
                agent=agent,
                vehInfo=0.0,
                agent_list=agent_list,
                candidate_action=action,
            )

            total_value = motivation_model.calculate_motivation_for_evacuation_action(
                agent=agent,
                agent_list=agent_list,
                current_time=float(current_time),
                action=action,
            )

            assert total_value == pytest.approx(
                base_value + info_value + majority_value
            )

            base_values.append(base_value)
            info_values.append(info_value)
            majority_values.append(majority_value)
            total_values.append(total_value)

        fig, ax = plt.subplots(figsize=(9, 5))

        ax.plot(
            current_times,
            base_values,
            label="base motivation",
        )
        ax.plot(
            current_times,
            info_values,
            label="obtained info motivation",
        )
        ax.plot(
            current_times,
            majority_values,
            label="majority tuning bias",
        )
        ax.plot(
            current_times,
            total_values,
            linewidth=2.0,
            label=f"total motivation: {action}",
        )

        ax.axvline(
            route_congestion_info_obtained_time,
            linestyle="--",
            label="route congestion info",
        )
        ax.axvline(
            tsunami_info_obtained_time,
            linestyle="--",
            label="tsunami info",
        )
        ax.axvline(
            shelter_full_info_obtained_time,
            linestyle="--",
            label="shelter full info",
        )

        ax.axvline(
            first_neighbor_action_time,
            linestyle=":",
            label="1st neighbor same action",
        )
        ax.axvline(
            second_neighbor_action_time,
            linestyle=":",
            label="2nd neighbor same action",
        )

        ax.set_xlabel("current_time [s]")
        ax.set_ylabel("motivation value")
        ax.set_title(f"Motivation components for evacuation action: {action}")
        ax.grid(True)
        ax.legend()
        fig.tight_layout()

        output_path = OUTPUT_DIR / f"motivation_for_evacuation_action_{action}.pdf"
        fig.savefig(output_path)
        plt.close(fig)

        output_files.append(output_path)

    expected_file_names = {
        "motivation_for_evacuation_action_va.pdf",
        "motivation_for_evacuation_action_rc.pdf",
        "motivation_for_evacuation_action_ww.pdf",
    }

    assert {path.name for path in output_files} == expected_file_names

    for output_path in output_files:
        assert output_path.exists()
        assert output_path.suffix == ".pdf"
        assert output_path.parent == OUTPUT_DIR
        assert output_path.stat().st_size > 0
    