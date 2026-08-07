from __future__ import annotations

from ijdrr_evacuations.agents import (
    AgentStore,
    DriverState,
    EvacuationState,
    EvacuationStatus,
    VehicleState,
)
from ijdrr_evacuations.behavior import BehaviorManager, EvacuationManager
from ijdrr_evacuations.communication import (
    CommunicationManager,
    ContactDetector,
    MessageRegistry,
)
from ijdrr_evacuations.environment import EnvironmentState
from ijdrr_evacuations.events import EventQueue, EventType, SimulationEvent
from ijdrr_evacuations.metrics import MetricsCollector
from ijdrr_evacuations.simulation import (
    MultiRateScheduler,
    SeedStreams,
    SimulationContext,
    SimulationEngine,
    VehicleSnapshot,
    derive_seed,
)


def test_scheduler_uses_absolute_next_due_times() -> None:
    scheduler = MultiRateScheduler()
    scheduler.register("communication", interval_s=1.0, first_due_s=1.0)

    assert not scheduler.is_due("communication", 0.9)
    assert scheduler.is_due("communication", 1.0)
    assert not scheduler.is_due("communication", 1.5)
    assert scheduler.is_due("communication", 2.0000000001)
    assert scheduler.next_due("communication") == 3.0


def test_named_random_streams_are_repeatable_and_separate() -> None:
    one = SeedStreams(42)
    two = SeedStreams(42)

    assert one.seeds == two.seeds
    assert derive_seed(42, "behavior_choice") != derive_seed(
        42, "communication_success"
    )
    assert one.generator("behavior_choice").random() == two.generator(
        "behavior_choice"
    ).random()


def test_event_queue_orders_equal_time_by_insertion() -> None:
    queue = EventQueue()
    later = SimulationEvent.create(
        time_s=2.0,
        event_type=EventType.HAZARD_DETECTED,
        subject_id="later",
    )
    first = SimulationEvent.create(
        time_s=1.0,
        event_type=EventType.MESSAGE_CREATED,
        subject_id="first",
    )
    second = SimulationEvent.create(
        time_s=1.0,
        event_type=EventType.MESSAGE_CREATED,
        subject_id="second",
    )
    queue.schedule(later)
    queue.schedule(first)
    queue.schedule(second)

    assert queue.pop_due(1.0) == [first, second]
    assert queue.pop_due(2.0) == [later]


class _FakeSumo:
    def __init__(self) -> None:
        self.now_s = 0.0
        self.subscribed: list[str] = []

    def simulation_step(self) -> None:
        self.now_s += 1.0

    def time_s(self) -> float:
        return self.now_s

    def departed_vehicle_ids(self) -> tuple[str, ...]:
        return ("veh-1",) if self.now_s == 1.0 else ()

    def arrived_vehicle_ids(self) -> tuple[str, ...]:
        return ()

    def vehicle_ids(self) -> tuple[str, ...]:
        return ("veh-1",)

    def subscribe_vehicle(self, vehicle_id: str) -> None:
        self.subscribed.append(vehicle_id)

    def vehicle_snapshot(self, vehicle_id: str) -> VehicleSnapshot:
        return VehicleSnapshot((self.now_s, 0.0), "edge-1", "edge-1_0", 1.0, self.now_s)


def test_engine_orders_departure_subscription_and_state_update() -> None:
    store = AgentStore()
    store.add(
        vehicle=VehicleState("veh-1"),
        driver=DriverState(),
        evacuation=EvacuationState("shelter-1"),
    )
    registry = MessageRegistry()
    scheduler = MultiRateScheduler()
    scheduler.register("communication", interval_s=1.0, first_due_s=1.0)
    scheduler.register("behavior", interval_s=5.0, first_due_s=5.0)
    fake_sumo = _FakeSumo()
    context = SimulationContext(
        sumo=fake_sumo,
        agent_store=store,
        environment=EnvironmentState(),
        message_registry=registry,
        communication_manager=CommunicationManager(
            registry=registry,
            communication_range_m=100.0,
            detector=ContactDetector("spatial_hash"),
        ),
        behavior_manager=BehaviorManager(registry=registry),
        evacuation_manager=EvacuationManager(),
        metrics=MetricsCollector(),
        event_queue=EventQueue(),
        scheduler=scheduler,
    )

    result = SimulationEngine(context).step()

    assert result.departed_vehicle_ids == ("veh-1",)
    assert fake_sumo.subscribed == ["veh-1"]
    assert store.evacuations[0].status is EvacuationStatus.ACTIVE
    assert store.vehicles[0].position == (1.0, 0.0)
    assert context.metrics.event_counts[EventType.VEHICLE_DEPARTED] == 1
