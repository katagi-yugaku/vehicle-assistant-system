"""Buffered JSONL or optional Parquet event writer."""

from __future__ import annotations

import json
from pathlib import Path
from typing import Any, Iterable

from ijdrr_evacuations.events import SimulationEvent


class EventLogWriter:
    """Write fixed event records in chunks without DataFrame updates."""

    __slots__ = (
        "path",
        "chunk_size",
        "_buffer",
        "_json_file",
        "_parquet_writer",
        "_closed",
    )

    def __init__(self, path: Path, *, chunk_size: int = 50_000) -> None:
        if chunk_size <= 0:
            raise ValueError("chunk_size must be positive")
        if path.suffix not in {".jsonl", ".parquet"}:
            raise ValueError("event output must end in .jsonl or .parquet")
        path.parent.mkdir(parents=True, exist_ok=True)
        self.path = path
        self.chunk_size = chunk_size
        self._buffer: list[dict[str, Any]] = []
        self._json_file = None
        self._parquet_writer = None
        self._closed = False
        if path.suffix == ".jsonl":
            self._json_file = path.open("w", encoding="utf-8")

    def append(self, event: SimulationEvent) -> None:
        self._ensure_open()
        self._buffer.append(
            {
                "time_s": event.time_s,
                "event_type": event.event_type.value,
                "vehicle_id": event.vehicle_id,
                "subject_id": event.subject_id,
                "data_json": json.dumps(
                    event.data_dict(),
                    ensure_ascii=False,
                    sort_keys=True,
                ),
            }
        )
        if len(self._buffer) >= self.chunk_size:
            self.flush()

    def append_many(self, events: Iterable[SimulationEvent]) -> None:
        for event in events:
            self.append(event)

    def flush(self) -> None:
        self._ensure_open()
        if not self._buffer:
            return
        if self.path.suffix == ".jsonl":
            assert self._json_file is not None
            for record in self._buffer:
                self._json_file.write(
                    json.dumps(record, ensure_ascii=False, sort_keys=True) + "\n"
                )
            self._json_file.flush()
        else:
            self._flush_parquet()
        self._buffer.clear()

    def _flush_parquet(self) -> None:
        try:
            import pyarrow as pa
            import pyarrow.parquet as pq
        except ImportError as exc:
            raise RuntimeError(
                "Parquet output requires pyarrow; use a .jsonl path otherwise"
            ) from exc

        schema = pa.schema(
            [
                ("time_s", pa.float64()),
                ("event_type", pa.string()),
                ("vehicle_id", pa.string()),
                ("subject_id", pa.string()),
                ("data_json", pa.string()),
            ]
        )
        table = pa.Table.from_pylist(self._buffer, schema=schema)
        if self._parquet_writer is None:
            self._parquet_writer = pq.ParquetWriter(self.path, table.schema)
        self._parquet_writer.write_table(table)

    def close(self) -> None:
        if self._closed:
            return
        self.flush()
        if self._json_file is not None:
            self._json_file.close()
        if self._parquet_writer is not None:
            self._parquet_writer.close()
        self._closed = True

    def _ensure_open(self) -> None:
        if self._closed:
            raise RuntimeError("event writer is closed")

    def __enter__(self) -> "EventLogWriter":
        return self

    def __exit__(self, exc_type, exc, traceback) -> None:
        self.close()
