"""Shared live and replay analytics."""

from __future__ import annotations

import numpy as np


def compute_channel_metrics(
    time_s: np.ndarray,
    target: np.ndarray,
    final: np.ndarray,
) -> dict[str, float | None]:
    if time_s.size == 0 or target.size == 0 or final.size == 0:
        return _empty_metrics()

    error = target - final
    metrics = {
        "mean": float(np.mean(final)),
        "std": float(np.std(final)),
        "min": float(np.min(final)),
        "max": float(np.max(final)),
        "peak_to_peak": float(np.max(final) - np.min(final)),
        "mean_error": float(np.mean(error)),
        "max_abs_error": float(np.max(np.abs(error))),
        "steady_state_error": _steady_state_error(error),
        "rise_time_s": None,
        "settling_time_s": None,
        "overshoot_pct": _overshoot_pct(target, final),
    }
    rise_time_s, settling_time_s = _step_response_metrics(time_s, target, final)
    metrics["rise_time_s"] = rise_time_s
    metrics["settling_time_s"] = settling_time_s
    return metrics


def _empty_metrics() -> dict[str, float | None]:
    return {
        "mean": None,
        "std": None,
        "min": None,
        "max": None,
        "peak_to_peak": None,
        "mean_error": None,
        "max_abs_error": None,
        "steady_state_error": None,
        "rise_time_s": None,
        "settling_time_s": None,
        "overshoot_pct": None,
    }


def _steady_state_error(error: np.ndarray) -> float | None:
    if error.size < 3:
        return None
    tail_size = max(1, int(np.ceil(error.size * 0.2)))
    return float(np.mean(error[-tail_size:]))


def _last_step_index(target: np.ndarray) -> int | None:
    step_indices = np.flatnonzero(np.abs(np.diff(target)) > 1e-6)
    if step_indices.size == 0:
        return None
    return int(step_indices[-1] + 1)


def _overshoot_pct(target: np.ndarray, final: np.ndarray) -> float | None:
    start = _last_step_index(target)
    if start is None:
        return None
    initial = float(target[start - 1])
    goal = float(target[start])
    amplitude = goal - initial
    if abs(amplitude) < 1e-6:
        return None

    response = final[start:]
    if response.size == 0:
        return None
    peak = float(np.max(response)) if amplitude > 0 else float(np.min(response))
    overshoot = max(0.0, peak - goal) if amplitude > 0 else max(0.0, goal - peak)
    return float((overshoot / abs(amplitude)) * 100.0)


def _step_response_metrics(
    time_s: np.ndarray,
    target: np.ndarray,
    final: np.ndarray,
) -> tuple[float | None, float | None]:
    start = _last_step_index(target)
    if start is None or time_s.size < 3:
        return None, None

    start_time = float(time_s[start])
    initial = float(target[start - 1])
    goal = float(target[start])
    amplitude = goal - initial
    if abs(amplitude) < 1e-6:
        return None, None

    response = final[start:]
    response_time = time_s[start:]
    low = initial + amplitude * 0.1
    high = initial + amplitude * 0.9

    if amplitude > 0:
        low_hits = np.flatnonzero(response >= low)
        high_hits = np.flatnonzero(response >= high)
    else:
        low_hits = np.flatnonzero(response <= low)
        high_hits = np.flatnonzero(response <= high)

    rise_time = None
    if low_hits.size > 0 and high_hits.size > 0:
        rise_time = float(response_time[high_hits[0]] - response_time[low_hits[0]])

    tolerance = max(abs(goal) * 0.05, 1e-6)
    band = np.abs(response - goal) <= tolerance
    settling_time = None
    for idx in range(band.size):
        if np.all(band[idx:]):
            settling_time = float(response_time[idx] - start_time)
            break

    return rise_time, settling_time
