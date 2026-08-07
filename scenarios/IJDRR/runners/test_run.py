"""Small libsumo-only smoke run for the new IJDRR architecture.

The route file contains route definitions but no vehicles, so this runner adds
a deterministic test fleet through libsumo.  It deliberately does not fall
back to socket TraCI: a missing Python libsumo binding is reported as a setup
error.

Example:
    python scenarios/IJDRR/runners/test_run.py --vehicles 8 --max-time 600
"""

from __future__ import annotations

import argparse
import hashlib
import json
import platform
import subprocess
import sys
from datetime import datetime, timezone
from pathlib import Path


HERE = Path(__file__).resolve()
PROJECT_ROOT = HERE.parents[3]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from ijdrr_evacuations.agents import (  # noqa: E402
    AgentStore,
    DriverState,
    DTNNodeState,
    EvacuationState,
    VehicleState,
)
from ijdrr_evacuations.behavior import (  # noqa: E402
    BehaviorManager,
    EvacuationManager,
)
from ijdrr_evacuations.communication import (  # noqa: E402
    CommunicationManager,
    ContactDetector,
    MessageBuffer,
    MessageKind,
    MessageRegistry,
)
from ijdrr_evacuations.environment import (  # noqa: E402
    EnvironmentState,
    ShelterState,
)
from ijdrr_evacuations.events import (  # noqa: E402
    EventQueue,
    EventType,
    SimulationEvent,
)
from ijdrr_evacuations.metrics import (  # noqa: E402
    EventLogWriter,
    MetricsCollector,
    RunManifest,
)
from ijdrr_evacuations.simulation import (  # noqa: E402
    LibsumoAdapter,
    LibsumoUnavailableError,
    MultiRateScheduler,
    SeedStreams,
    SimulationContext,
    SimulationEngine,
)


DEFAULT_SUMO_HOME = PROJECT_ROOT / "sumo"
DEFAULT_SUMO_CONFIG = (
    PROJECT_ROOT
    / "scenarios"
    / "IJDRR"
    / "1-1-2"
    / "data"
    / "ishinomaki_one_two_one.sumocfg"
)
ROUTES = (
    ("E0_E16_0", "ShelterA_1"),
    ("E0_E13_0", "ShelterA_2"),
)


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--sumo-home", type=Path, default=DEFAULT_SUMO_HOME)
    parser.add_argument("--sumocfg", type=Path, default=DEFAULT_SUMO_CONFIG)
    parser.add_argument("--vehicles", type=int, default=6)
    parser.add_argument("--max-time", type=float, default=600.0)
    parser.add_argument("--step-length", type=float, default=1.0)
    parser.add_argument("--communication-interval", type=float, default=1.0)
    parser.add_argument("--behavior-interval", type=float, default=5.0)
    parser.add_argument("--communication-range", type=float, default=300.0)
    parser.add_argument("--dtn-rate", type=float, default=1.0)
    parser.add_argument("--buffer-capacity", type=int, default=64)
    parser.add_argument("--seed", type=int, default=23_423)
    parser.add_argument("--sumo-seed", type=int)
    parser.add_argument("--contact-backend", choices=("auto", "ckdtree", "spatial_hash"), default="auto")
    parser.add_argument("--output-dir", type=Path)
    args = parser.parse_args(argv)

    if args.vehicles <= 0:
        parser.error("--vehicles must be positive")
    if args.max_time <= 0 or args.step_length <= 0:
        parser.error("--max-time and --step-length must be positive")
    if not 0.0 <= args.dtn_rate <= 1.0:
        parser.error("--dtn-rate must be between 0 and 1")
    if args.buffer_capacity < 0:
        parser.error("--buffer-capacity must be non-negative")
    return args


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as source:
        for chunk in iter(lambda: source.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def git_state() -> tuple[str | None, bool | None]:
    try:
        commit = subprocess.run(
            ["git", "rev-parse", "HEAD"],
            cwd=PROJECT_ROOT,
            check=True,
            capture_output=True,
            text=True,
        ).stdout.strip()
        dirty_result = subprocess.run(
            ["git", "status", "--porcelain"],
            cwd=PROJECT_ROOT,
            check=True,
            capture_output=True,
            text=True,
        )
    except (OSError, subprocess.CalledProcessError):
        return None, None
    return commit, bool(dirty_result.stdout.strip())


def build_output_dir(args: argparse.Namespace) -> Path:
    if args.output_dir is not None:
        output_dir = args.output_dir.expanduser().resolve()
    else:
        run_id = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
        output_dir = (
            PROJECT_ROOT
            / "scenarios"
            / "IJDRR"
            / "results"
            / "libsumo_test"
            / f"seed_{args.seed}_{run_id}"
        )
    output_dir.mkdir(parents=True, exist_ok=False)
    return output_dir


def build_context(
    *,
    args: argparse.Namespace,
    adapter: LibsumoAdapter,
    seed_streams: SeedStreams,
) -> SimulationContext:
    registry = MessageRegistry()
    agent_store = AgentStore()
    dtn_rng = seed_streams.generator("dtn_assignment")
    driver_rng = seed_streams.generator("driver_traits")

    for vehicle_number in range(args.vehicles):
        vehicle_id = f"ijdrr_test_{vehicle_number:04d}"
        _, destination_id = ROUTES[vehicle_number % len(ROUTES)]
        dtn_enabled = dtn_rng.random() < args.dtn_rate
        agent_store.add(
            vehicle=VehicleState(vehicle_id=vehicle_id),
            driver=DriverState(
                normalcy_bias=driver_rng.random(),
                conformity=driver_rng.random(),
                risk_perception=driver_rng.random(),
                information_trust=driver_rng.random(),
            ),
            evacuation=EvacuationState(destination_id=destination_id),
            dtn=(
                DTNNodeState(
                    enabled=True,
                    capacity_messages=args.buffer_capacity,
                )
                if dtn_enabled
                else None
            ),
        )

    environment = EnvironmentState(
        shelters={
            "ShelterA_1": ShelterState("ShelterA_1", 400, "E16"),
            "ShelterA_2": ShelterState("ShelterA_2", 400, "E13"),
        }
    )
    communication_manager = CommunicationManager(
        registry=registry,
        communication_range_m=args.communication_range,
        detector=ContactDetector(args.contact_backend),
        rng=seed_streams.generator("communication_success"),
    )
    scheduler = MultiRateScheduler()
    scheduler.register(
        "communication",
        interval_s=args.communication_interval,
        first_due_s=args.communication_interval,
    )
    scheduler.register(
        "behavior",
        interval_s=args.behavior_interval,
        first_due_s=args.behavior_interval,
    )
    return SimulationContext(
        sumo=adapter,
        agent_store=agent_store,
        environment=environment,
        message_registry=registry,
        communication_manager=communication_manager,
        behavior_manager=BehaviorManager(registry=registry),
        evacuation_manager=EvacuationManager(),
        metrics=MetricsCollector(keep_events=True),
        event_queue=EventQueue(),
        scheduler=scheduler,
    )


def add_test_vehicles(
    *,
    args: argparse.Namespace,
    adapter: LibsumoAdapter,
) -> None:
    for vehicle_number in range(args.vehicles):
        vehicle_id = f"ijdrr_test_{vehicle_number:04d}"
        route_id, _ = ROUTES[vehicle_number % len(ROUTES)]
        adapter.add_vehicle(
            vehicle_id=vehicle_id,
            route_id=route_id,
            depart_s=vehicle_number * max(args.step_length, 1.0),
        )


def seed_demo_message(context: SimulationContext) -> int | None:
    if not context.agent_store.dtn_nodes:
        return None
    source_index = min(context.agent_store.dtn_nodes)
    source_vehicle_id = context.agent_store.vehicle_id_of(source_index)
    message = context.message_registry.create(
        kind=MessageKind.CONGESTION,
        source_vehicle_id=source_vehicle_id,
        subject_id="E20",
        now_s=0.0,
        ttl_s=300.0,
        payload={"mean_speed_mps": 1.0},
        priority=5,
        reliability=1.0,
        hop_limit=16,
    )
    MessageBuffer(context.message_registry).seed(
        node=context.agent_store.dtn_nodes[source_index],
        message=message,
        now_s=0.0,
    )
    context.metrics.record(
        SimulationEvent.create(
            time_s=0.0,
            event_type=EventType.MESSAGE_CREATED,
            vehicle_id=source_vehicle_id,
            subject_id=str(message.message_id),
            data={"kind": message.header.kind.value},
        )
    )
    return message.message_id


def write_results(
    *,
    output_dir: Path,
    run_id: str,
    args: argparse.Namespace,
    sumo_version: str,
    context: SimulationContext,
    seed_streams: SeedStreams,
    started_at: datetime,
) -> None:
    with EventLogWriter(output_dir / "events.jsonl", chunk_size=1_000) as writer:
        writer.append_many(context.metrics.events)

    summaries = [
        row.to_dict()
        for row in context.metrics.vehicle_summaries(
            run_id=run_id,
            agent_store=context.agent_store,
        )
    ]
    with (output_dir / "vehicle_summary.json").open("w", encoding="utf-8") as output:
        json.dump(summaries, output, ensure_ascii=False, indent=2)

    commit, dirty = git_state()
    manifest = RunManifest(
        schema_version="1",
        run_id=run_id,
        scenario_id="IJDRR-libsumo-smoke",
        master_seed=args.seed,
        sumo_seed=args.sumo_seed,
        config_sha256=sha256_file(args.sumocfg),
        git_commit=commit,
        git_dirty=dirty,
        sumo_version=sumo_version,
        python_version=platform.python_version(),
        started_at_iso=started_at.isoformat(),
        stream_seeds=tuple(sorted(seed_streams.seeds.items())),
    )
    with (output_dir / "manifest.json").open("w", encoding="utf-8") as output:
        json.dump(manifest.to_dict(), output, ensure_ascii=False, indent=2)

    result = {
        "final_time_s": context.now_s,
        "vehicle_count": len(context.agent_store),
        "event_counts": {
            event_type.value: count
            for event_type, count in sorted(
                context.metrics.event_counts.items(),
                key=lambda item: item[0].value,
            )
        },
        "message_registry_size": len(context.message_registry),
    }
    with (output_dir / "run_summary.json").open("w", encoding="utf-8") as output:
        json.dump(result, output, ensure_ascii=False, indent=2)


def run(args: argparse.Namespace) -> Path:
    args.sumo_home = args.sumo_home.expanduser().resolve()
    args.sumocfg = args.sumocfg.expanduser().resolve()
    if not args.sumocfg.is_file():
        raise FileNotFoundError(f"SUMO configuration not found: {args.sumocfg}")
    if args.sumo_seed is None:
        args.sumo_seed = args.seed

    adapter = LibsumoAdapter.load(sumo_home=args.sumo_home)
    output_dir = build_output_dir(args)
    run_id = output_dir.name
    started_at = datetime.now(timezone.utc)
    seed_streams = SeedStreams(args.seed)
    context = build_context(
        args=args,
        adapter=adapter,
        seed_streams=seed_streams,
    )

    command = [
        "sumo",
        "-c",
        str(args.sumocfg),
        "--seed",
        str(args.sumo_seed),
        "--step-length",
        str(args.step_length),
        "--no-step-log",
        "true",
        "--tripinfo-output",
        str(output_dir / "tripinfo.xml"),
    ]

    try:
        adapter.start(command)
        sumo_version = adapter.version()
        add_test_vehicles(args=args, adapter=adapter)
        seed_demo_message(context)
        engine = SimulationEngine(context)
        while (
            adapter.min_expected_number() > 0
            and context.now_s < args.max_time
        ):
            engine.step()
    finally:
        adapter.close()

    write_results(
        output_dir=output_dir,
        run_id=run_id,
        args=args,
        sumo_version=sumo_version,
        context=context,
        seed_streams=seed_streams,
        started_at=started_at,
    )
    return output_dir


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    try:
        output_dir = run(args)
    except LibsumoUnavailableError as exc:
        print(f"[libsumo setup error] {exc}", file=sys.stderr)
        return 2
    except Exception as exc:
        print(f"[simulation error] {type(exc).__name__}: {exc}", file=sys.stderr)
        return 1

    print(f"libsumo smoke run completed: {output_dir}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
