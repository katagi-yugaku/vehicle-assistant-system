from __future__ import annotations

import json

from ijdrr_evacuations.events import EventType, SimulationEvent
from ijdrr_evacuations.metrics import EventLogWriter, MetricsCollector


def test_event_writer_buffers_jsonl(tmp_path) -> None:
    path = tmp_path / "events.jsonl"
    event = SimulationEvent.create(
        time_s=1.5,
        event_type=EventType.CONTACT_STARTED,
        subject_id="0:1",
        data={"distance_m": 12.0},
    )
    collector = MetricsCollector(keep_events=True)
    collector.record(event)

    with EventLogWriter(path, chunk_size=1) as writer:
        writer.append_many(collector.events)

    record = json.loads(path.read_text(encoding="utf-8"))
    assert record["event_type"] == "contact_started"
    assert json.loads(record["data_json"]) == {"distance_m": 12.0}
