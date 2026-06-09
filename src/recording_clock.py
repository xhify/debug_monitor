"""统一记录会话时钟。"""

from __future__ import annotations

from dataclasses import dataclass, field
from datetime import datetime
import time
import uuid


def _default_session_id() -> str:
    return f"session_{uuid.uuid4().hex[:12]}"


def _default_started_at_iso() -> str:
    return datetime.now().astimezone().isoformat(timespec="seconds")


@dataclass(slots=True)
class RecordingClock:
    """为多源记录提供统一的会话时间基准。"""

    session_id: str = field(default_factory=_default_session_id)
    started_at_iso: str = field(default_factory=_default_started_at_iso)
    start_epoch_s: float = field(default_factory=time.time)
    start_perf_s: float = field(default_factory=time.perf_counter)

    def now_epoch_s(self) -> float:
        return time.time()

    def elapsed_s(self) -> float:
        return max(0.0, time.perf_counter() - self.start_perf_s)

    def elapsed_from_epoch(self, recv_time_epoch_s: float) -> float:
        return max(0.0, float(recv_time_epoch_s) - self.start_epoch_s)

    def now_record_fields(self) -> dict[str, object]:
        return {
            "session_id": self.session_id,
            "recv_time_epoch_s": self.now_epoch_s(),
            "session_elapsed_s": self.elapsed_s(),
        }

    def to_metadata(self) -> dict[str, object]:
        return {
            "session_id": self.session_id,
            "started_at_iso": self.started_at_iso,
            "start_epoch_s": self.start_epoch_s,
        }
