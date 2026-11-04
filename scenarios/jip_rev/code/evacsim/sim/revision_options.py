# =========================
# Revision (JIP 26-S015f) switches
# =========================
#
# このモジュールは改訂作業で追加した修正をフラグで切り替えるためのもの。
# すべてのフラグの既定値は「元のコードと同じ動作」になるように決めてある。
#
#   fix_origin_edge         (B1) E0 を E1 と同じ「分岐前の区間」として扱う
#   fix_pre_x_alternatives  (B3) 4 経路構成の E1/E0 で代替経路を共有平均から選ぶ
#   seed                    (B4) random と numpy の乱数シード(未指定なら元の動作)
#   save_raw                (B5) 1 回ごとの生データを JSON で保存する
#   log_sections            (M1/R2-3) 30 s ごとの道路区間ごとの車両数と平均速度
#   travel_time_offset      (B6) 旅行時間から差し引く値(元は 100 s)
#   communication_mode      (B2) legacy / v2v_dtn / v2v_only
#
# 注意:
#   - TOML の設定キーで受け取る。--seed と --raw-output はコマンドラインでも
#     受け取り、TOML より優先する(runner の位置引数を崩さないよう、
#     bootstrap() で sys.argv から取り除く)。
#   - communication_mode が v2v_dtn / v2v_only のときは、jip/v2v-only (eda485c)
#     から取り込んだ VehicleInfo_round / communication_round を
#     evacsim.agents.VehicleInfo / evacsim.sim.communication として登録する。
#     そのため bootstrap() は runner が evacsim の他のモジュールを import する
#     前に呼ぶ必要がある。
#   - このモジュール自身は traci も VehicleInfo も import しない。

from __future__ import annotations

import json
import math
import os
import sys
from dataclasses import asdict, dataclass, field, is_dataclass
from pathlib import Path
from typing import Any

if sys.version_info >= (3, 11):
    import tomllib as _toml_loader
else:  # pragma: no cover - Python 3.10
    import tomli as _toml_loader


LEGACY = "legacy"
V2V_DTN = "v2v_dtn"
V2V_ONLY = "v2v_only"
COMMUNICATION_MODES = (LEGACY, V2V_DTN, V2V_ONLY)
ROUND_BASED_MODES = (V2V_DTN, V2V_ONLY)

_BOOL_TRUE = {"1", "true", "yes", "on"}
_BOOL_FALSE = {"0", "false", "no", "off"}


@dataclass
class RevisionOptions:
    fix_origin_edge: bool = False
    fix_pre_x_alternatives: bool = False
    seed: int | None = None
    save_raw: bool = False
    raw_output: str = "raw.json"
    log_sections: bool = False
    section_interval: float = 30.0
    travel_time_offset: float = 100.0
    communication_mode: str = LEGACY
    end_simulation_time: float | None = None
    config_path: str | None = None

    @property
    def round_based_communication(self) -> bool:
        return self.communication_mode in ROUND_BASED_MODES

    @property
    def any_output(self) -> bool:
        return self.save_raw or self.log_sections

    def as_dict(self) -> dict[str, Any]:
        return asdict(self)


# 実行中に参照する唯一のインスタンス。bootstrap() で置き換える。
OPTIONS = RevisionOptions()


def _to_bool(key: str, value: Any) -> bool:
    if isinstance(value, bool):
        return value
    text = str(value).strip().lower()
    if text in _BOOL_TRUE:
        return True
    if text in _BOOL_FALSE:
        return False
    raise ValueError(f"{key} must be a boolean: {value!r}")


def options_from_config(cfg: dict) -> RevisionOptions:
    opts = RevisionOptions()
    if "fix_origin_edge" in cfg:
        opts.fix_origin_edge = _to_bool("fix_origin_edge", cfg["fix_origin_edge"])
    if "fix_pre_x_alternatives" in cfg:
        opts.fix_pre_x_alternatives = _to_bool(
            "fix_pre_x_alternatives", cfg["fix_pre_x_alternatives"]
        )
    if cfg.get("seed") is not None:
        opts.seed = int(cfg["seed"])
    if "save_raw" in cfg:
        opts.save_raw = _to_bool("save_raw", cfg["save_raw"])
    if cfg.get("raw_output"):
        opts.raw_output = str(cfg["raw_output"])
    if "log_sections" in cfg:
        opts.log_sections = _to_bool("log_sections", cfg["log_sections"])
    if "section_interval" in cfg:
        opts.section_interval = float(cfg["section_interval"])
    if "travel_time_offset" in cfg:
        opts.travel_time_offset = float(cfg["travel_time_offset"])
    if "communication_mode" in cfg:
        opts.communication_mode = str(cfg["communication_mode"]).strip().lower()
    if cfg.get("end_simulation_time") is not None:
        opts.end_simulation_time = float(cfg["end_simulation_time"])
    validate(opts)
    return opts


def validate(opts: RevisionOptions) -> None:
    if opts.communication_mode not in COMMUNICATION_MODES:
        raise ValueError(
            f"communication_mode must be one of {COMMUNICATION_MODES}: "
            f"{opts.communication_mode!r}"
        )
    if opts.seed is not None and not 0 <= opts.seed <= 0xFFFFFFFF:
        raise ValueError(f"seed must be a 32-bit unsigned integer: {opts.seed}")
    if opts.section_interval <= 0:
        raise ValueError("section_interval must be positive")
    if not math.isfinite(opts.travel_time_offset):
        raise ValueError("travel_time_offset must be finite")
    if opts.end_simulation_time is not None and opts.end_simulation_time <= 0:
        raise ValueError("end_simulation_time must be positive")


def _pop_option(argv: list[str], name: str) -> str | None:
    """argv から ``--name VALUE`` / ``--name=VALUE`` を取り除いて値を返す。"""
    value = None
    i = 0
    while i < len(argv):
        arg = argv[i]
        if arg == name:
            if i + 1 >= len(argv):
                raise ValueError(f"{name} requires a value")
            value = argv[i + 1]
            del argv[i:i + 2]
            continue
        if arg.startswith(name + "="):
            value = arg.split("=", 1)[1]
            del argv[i]
            continue
        i += 1
    return value


def _find_config_path(argv: list[str]) -> str | None:
    for arg in argv[1:]:
        if arg.endswith(".toml"):
            return arg
    return None


def bootstrap(
    argv: list[str] | None = None,
    *,
    supported_modes: tuple[str, ...] = COMMUNICATION_MODES,
    layout: str = "",
) -> RevisionOptions:
    """
    runner の先頭で呼ぶ。

    1. sys.argv から --seed / --raw-output を取り除く(元の位置引数の解釈を保つ)。
    2. TOML を読んでフラグを決める。
    3. round 単位の通信方式なら、取り込んだ実装をモジュールとして登録する。
    """
    global OPTIONS
    if argv is None:
        argv = sys.argv

    cli_seed = _pop_option(argv, "--seed")
    cli_raw_output = _pop_option(argv, "--raw-output")
    cli_mode = _pop_option(argv, "--communication-mode")

    config_path = _find_config_path(argv)
    cfg: dict = {}
    if config_path is not None and Path(config_path).is_file():
        with open(config_path, "rb") as f:
            cfg = _toml_loader.load(f)

    opts = options_from_config(cfg)
    opts.config_path = config_path
    if cli_seed is not None:
        opts.seed = int(cli_seed)
    if cli_raw_output is not None:
        opts.raw_output = cli_raw_output
    if cli_mode is not None:
        opts.communication_mode = cli_mode.strip().lower()
    validate(opts)

    if opts.communication_mode not in supported_modes:
        raise ValueError(
            f"communication_mode={opts.communication_mode!r} is not supported "
            f"by the {layout or 'this'} runner (supported: {supported_modes})"
        )

    if opts.round_based_communication:
        install_round_based_communication()

    OPTIONS = opts
    return opts


def install_round_based_communication() -> None:
    """
    jip/v2v-only (eda485c) の VehicleInfo と communication を、元のモジュール名で
    参照されるように登録する。元のファイル(legacy 実装)は変更しない。
    """
    import importlib

    already = [
        name
        for name in ("evacsim.agents.VehicleInfo", "evacsim.sim.communication")
        if name in sys.modules
        and not getattr(sys.modules[name], "__ROUND_BASED__", False)
    ]
    if already:
        raise RuntimeError(
            "round-based communication must be installed before these modules "
            f"are imported: {already}"
        )

    vehicle_info_round = importlib.import_module("evacsim.agents.VehicleInfo_round")
    sys.modules["evacsim.agents.VehicleInfo"] = vehicle_info_round
    communication_round = importlib.import_module("evacsim.sim.communication_round")
    sys.modules["evacsim.sim.communication"] = communication_round


# =========================
# B3: 分岐前の代替経路の選択
# =========================
def select_pre_x_alternative(
    *,
    current_route: tuple[str, ...],
    current_edge_id: str,
    latest_route_info: dict,
    edge_id_by_shelter_id: dict[str, str],
) -> tuple[tuple[str, ...], float, str] | None:
    """
    現在の経路と分岐する経路のうち、共有された平均値を持つものから
    平均値が最小のものを返す。

    分岐する経路: 現在の edge までは現在の経路と同じ edge 列で、それ以降が異なる経路。
    戻り値: (経路, 平均値, 行先の避難所 ID)。候補がなければ None。
    """
    current_route = tuple(current_route)
    if current_edge_id not in current_route:
        return None
    prefix = current_route[: current_route.index(current_edge_id) + 1]

    shelter_by_edge = {edge: shelter for shelter, edge in edge_id_by_shelter_id.items()}
    candidates: list[tuple[float, tuple[str, ...], str]] = []
    for route, info in (latest_route_info or {}).items():
        route = tuple(route)
        if route == current_route or route[: len(prefix)] != prefix:
            continue
        if len(route) <= len(prefix):
            continue
        avg = (info or {}).get("avg_time")
        if avg is None:
            continue
        avg = float(avg)
        if not math.isfinite(avg):
            continue
        shelter_id = shelter_by_edge.get(route[-1])
        if shelter_id is None:
            continue
        candidates.append((avg, route, shelter_id))

    if not candidates:
        return None
    candidates.sort(key=lambda item: (item[0], item[1]))
    avg, route, shelter_id = candidates[0]
    return route, avg, shelter_id


# =========================
# B5 / M1: 生データの記録
# =========================
class RawRecorder:
    """1 回分の生データを集める。OPTIONS.any_output が False なら何もしない。"""

    def __init__(self) -> None:
        self.first_seen: dict[str, dict[str, Any]] = {}
        self.arrivals: list[dict[str, Any]] = []
        self.sections: dict[str, dict[str, list[float]]] = {}
        self._next_section_time: float | None = None

    # runner の車両ループから呼ぶ
    def note_vehicle(self, logical_id: str, sumo_id: str, current_time: float, traci) -> None:
        if not OPTIONS.save_raw or logical_id in self.first_seen:
            return
        try:
            depart = float(traci.vehicle.getDeparture(sumo_id))
        except Exception:
            depart = None
        try:
            route = list(traci.vehicle.getRoute(sumo_id))
        except Exception:
            route = []
        self.first_seen[logical_id] = {
            "sumo_id": sumo_id,
            "first_seen_time": float(current_time),
            "depart_time": depart,
            "initial_route": route,
        }

    # evacsim.sim.arrival.handle_arrival から呼ぶ
    def note_arrival(self, sumo_id: str, arrival_time: float, route, departure: float) -> None:
        if not OPTIONS.save_raw:
            return
        self.arrivals.append(
            {
                "sumo_id": sumo_id,
                "arrival_time": float(arrival_time),
                "route": list(route),
                "sumo_depart_time": float(departure),
            }
        )

    # run() の simulationStep の直後に呼ぶ
    def maybe_log_sections(self, current_time: float, traci) -> None:
        if not OPTIONS.log_sections:
            return
        if self._next_section_time is None:
            self._next_section_time = OPTIONS.section_interval
        if current_time + 1e-9 < self._next_section_time:
            return
        self._next_section_time = (
            math.floor(current_time / OPTIONS.section_interval + 1e-9) + 1
        ) * OPTIONS.section_interval
        counts: dict[str, list[float]] = {}
        for edge_id in traci.edge.getIDList():
            if edge_id.startswith(":"):
                continue
            n = traci.edge.getLastStepVehicleNumber(edge_id)
            if n:
                counts[edge_id] = [int(n), round(float(traci.edge.getLastStepMeanSpeed(edge_id)), 3)]
        self.sections[str(int(round(current_time)))] = counts

    def write(
        self,
        *,
        layout: str,
        seed: int | None,
        arrival_time_by_vehID: dict,
        route_change_events: list,
        logical_vehicle_id_by_sumo_vehicle_id: dict,
        counters: dict,
        num_vehicles: int,
        extra: dict | None = None,
    ) -> str | None:
        if not OPTIONS.any_output:
            return None

        def conv(x):
            if is_dataclass(x):
                return {k: conv(v) for k, v in asdict(x).items()}
            if isinstance(x, (list, tuple)):
                return [conv(v) for v in x]
            if isinstance(x, dict):
                return {str(k): conv(v) for k, v in x.items()}
            if hasattr(x, "value") and type(x).__module__ != "builtins":
                return x.value
            return x

        arrival_times = [float(t) for t in arrival_time_by_vehID.values()]
        events = conv(route_change_events)
        successful = [e for e in events if e.get("success")]
        e0_regen = [
            e for e in successful
            if e.get("change_type") == "uturn" and e.get("from_edge_id") == "E0"
        ]

        vehicles: dict[str, dict[str, Any]] = {}
        if OPTIONS.save_raw:
            for logical_id, info in self.first_seen.items():
                vehicles[logical_id] = dict(info)
            for arrival in self.arrivals:
                logical_id = logical_vehicle_id_by_sumo_vehicle_id.get(
                    arrival["sumo_id"], arrival["sumo_id"]
                )
                vehicles.setdefault(logical_id, {}).update(
                    {
                        "arrival_sumo_id": arrival["sumo_id"],
                        "arrival_time": arrival["arrival_time"],
                        "arrival_route": arrival["route"],
                        "arrival_sumo_depart_time": arrival["sumo_depart_time"],
                    }
                )

        payload: dict[str, Any] = {
            "schema": "evacsim-raw/1",
            "layout": layout,
            "seed": seed,
            "flags": OPTIONS.as_dict(),
            "num_vehicles": num_vehicles,
            "arrived_count": len(arrival_times),
            "completion_time": max(arrival_times) if arrival_times else None,
            "mean_arrival_time": (
                sum(arrival_times) / len(arrival_times) if arrival_times else None
            ),
            "counters": counters,
            "route_change_summary": {
                "success_total": len(successful),
                "success_by_reason": _count(successful, "reason"),
                "success_by_type": _count(successful, "change_type"),
                "success_by_from_edge": _count(successful, "from_edge_id"),
                "failed_total": len(events) - len(successful),
                "e0_regeneration_count": len(e0_regen),
            },
        }
        if OPTIONS.save_raw:
            payload["arrival_time_by_sumo_id"] = {
                str(k): float(v) for k, v in arrival_time_by_vehID.items()
            }
            payload["vehicles"] = vehicles
            payload["route_change_events"] = events
        if OPTIONS.log_sections:
            payload["section_interval"] = OPTIONS.section_interval
            payload["sections"] = self.sections
        if extra:
            payload.update(conv(extra))

        out = OPTIONS.raw_output
        out_dir = os.path.dirname(out)
        if out_dir:
            os.makedirs(out_dir, exist_ok=True)
        tmp = out + ".tmp"
        with open(tmp, "w") as f:
            json.dump(payload, f, ensure_ascii=False, sort_keys=True)
        os.replace(tmp, out)
        print(f"[RAW_OUTPUT] path={out}")
        return out


def _count(events: list[dict], key: str) -> dict[str, int]:
    out: dict[str, int] = {}
    for e in events:
        k = str(e.get(key))
        out[k] = out.get(k, 0) + 1
    return dict(sorted(out.items()))


RAW = RawRecorder()
