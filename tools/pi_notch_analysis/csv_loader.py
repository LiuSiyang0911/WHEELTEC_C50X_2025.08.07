from __future__ import annotations

from dataclasses import dataclass
from io import StringIO
from pathlib import Path

import numpy as np


@dataclass(frozen=True)
class DebugCsvData:
    path: Path
    time_s: np.ndarray
    final_a: np.ndarray
    final_b: np.ndarray
    output_a: np.ndarray
    output_b: np.ndarray
    target_a: np.ndarray
    target_b: np.ndarray
    afc_output_a: np.ndarray
    afc_output_b: np.ndarray
    sample_rate_hz: float
    active_slice: slice
    steady_slice: slice

    def motor_feedback(self, motor: str) -> np.ndarray:
        return self.final_a if motor == "a" else self.final_b


def _first_present(table: dict[str, np.ndarray], *names: str) -> np.ndarray:
    for name in names:
        if name in table:
            return table[name]
    raise KeyError(f"Missing required columns: {', '.join(names)}")


def _compute_sample_rate_hz(time_s: np.ndarray) -> float:
    dt = np.diff(time_s)
    dt = dt[dt > 0]
    if dt.size == 0:
        raise ValueError("CSV does not contain increasing time_s samples")
    return float(np.round(1.0 / np.median(dt), 6))


def _time_to_index(time_s: np.ndarray, value_s: float) -> int:
    return int(np.searchsorted(time_s, value_s, side="left"))


def _detect_slices(
    time_s: np.ndarray,
    output_a: np.ndarray,
    output_b: np.ndarray,
    final_a: np.ndarray,
    final_b: np.ndarray,
    settle_time_s: float,
    tail_time_s: float,
) -> tuple[slice, slice]:
    active_mask = (
        (np.abs(output_a) > 1.0)
        | (np.abs(output_b) > 1.0)
        | (np.abs(final_a) > 0.02)
        | (np.abs(final_b) > 0.02)
    )
    active_idx = np.flatnonzero(active_mask)
    if active_idx.size == 0:
        raise ValueError("No active motion segment detected in CSV")

    active_start = int(active_idx[0])
    active_stop = int(active_idx[-1]) + 1
    start_time = float(time_s[active_start] + settle_time_s)
    stop_time = float(time_s[active_stop - 1] - tail_time_s)

    if stop_time <= start_time:
        span = time_s[active_stop - 1] - time_s[active_start]
        start_time = float(time_s[active_start] + 0.25 * span)
        stop_time = float(time_s[active_stop - 1] - 0.25 * span)

    steady_start = max(active_start, _time_to_index(time_s, start_time))
    steady_stop = min(active_stop, _time_to_index(time_s, stop_time) + 1)
    if steady_stop - steady_start < 16:
        raise ValueError("Steady-state segment is too short for analysis")

    return slice(active_start, active_stop), slice(steady_start, steady_stop)


def _load_debug_csv_array(
    table: np.ndarray,
    *,
    source_path: Path,
    settle_time_s: float,
    tail_time_s: float,
) -> DebugCsvData:
    if table.shape == ():
        table = np.array([table], dtype=table.dtype)
    columns = {name: table[name] for name in table.dtype.names or ()}
    time_s = _first_present(columns, "time_s")
    final_a = _first_present(columns, "final_a")
    final_b = _first_present(columns, "final_b")
    output_a = _first_present(columns, "output_a")
    output_b = _first_present(columns, "output_b")

    target_a = columns.get("target_a", np.zeros_like(time_s))
    target_b = columns.get("target_b", np.zeros_like(time_s))
    afc_output_a = columns.get("afc_output_a", np.zeros_like(time_s))
    afc_output_b = columns.get("afc_output_b", np.zeros_like(time_s))

    active_slice, steady_slice = _detect_slices(
        time_s,
        output_a,
        output_b,
        final_a,
        final_b,
        settle_time_s,
        tail_time_s,
    )

    return DebugCsvData(
        path=source_path,
        time_s=time_s,
        final_a=final_a,
        final_b=final_b,
        output_a=output_a,
        output_b=output_b,
        target_a=target_a,
        target_b=target_b,
        afc_output_a=afc_output_a,
        afc_output_b=afc_output_b,
        sample_rate_hz=_compute_sample_rate_hz(time_s),
        active_slice=active_slice,
        steady_slice=steady_slice,
    )


def load_debug_csv(
    path: str | Path,
    *,
    settle_time_s: float = 5.0,
    tail_time_s: float = 5.0,
) -> DebugCsvData:
    csv_path = Path(path)
    table = np.genfromtxt(csv_path, delimiter=",", names=True, dtype=float, encoding="utf-8-sig")
    return _load_debug_csv_array(
        table,
        source_path=csv_path,
        settle_time_s=settle_time_s,
        tail_time_s=tail_time_s,
    )


def load_debug_csv_text(
    text: str,
    *,
    source_name: str = "<uploaded>",
    settle_time_s: float = 5.0,
    tail_time_s: float = 5.0,
) -> DebugCsvData:
    table = np.genfromtxt(StringIO(text), delimiter=",", names=True, dtype=float, encoding="utf-8-sig")
    return _load_debug_csv_array(
        table,
        source_path=Path(source_name),
        settle_time_s=settle_time_s,
        tail_time_s=tail_time_s,
    )
