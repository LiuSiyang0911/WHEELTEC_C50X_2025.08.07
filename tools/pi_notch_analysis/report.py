from __future__ import annotations

import json
from pathlib import Path

from .notch_sim import SimulationReport


def render_text_report(report: SimulationReport) -> str:
    lines = [
        f"Recommended PI Notch ({report.motor.upper()} motor)",
        f"Source CSV: {report.motor.upper()} channel from {report.target_speed_mps:.6f} m/s steady-state mean",
        f"Dominant disturbance: {report.disturbance_hz:.3f} Hz",
        f"Raw error amplitude: {report.raw_error_amplitude:.6f} m/s",
        f"Raw PI amplitude: {report.raw_pi_amplitude:.3f} PWM",
        "",
        "Recommended candidate:",
        f"  gain={report.recommended.gain:.3f}",
        f"  bandwidth={report.recommended.bandwidth_hz:.3f} Hz",
        f"  time_constant={report.recommended.time_constant_s:.3f} s",
        f"  error_reduction={report.recommended.error_reduction_pct:.1f}%",
        f"  pi_reduction={report.recommended.pi_reduction_pct:.1f}%",
        "",
        "Sweep results:",
    ]
    for candidate in report.candidates:
        lines.append(
            "  "
            f"gain={candidate.gain:.3f} "
            f"bw={candidate.bandwidth_hz:.3f}Hz "
            f"err_red={candidate.error_reduction_pct:.1f}% "
            f"pi_red={candidate.pi_reduction_pct:.1f}%"
        )
    return "\n".join(lines)


def write_json_report(report: SimulationReport, path: str | Path) -> None:
    Path(path).write_text(json.dumps(report.to_dict(), indent=2), encoding="utf-8")
