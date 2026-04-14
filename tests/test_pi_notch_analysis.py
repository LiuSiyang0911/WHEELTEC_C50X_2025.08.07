from __future__ import annotations

import json
import subprocess
import sys
from pathlib import Path

import numpy as np
import pytest

from tools.pi_notch_analysis.csv_loader import load_debug_csv
from tools.pi_notch_analysis.notch_sim import SimulationConfig, simulate_closed_loop_notch
from tools.pi_notch_analysis.signal_metrics import dominant_frequency_hz

SAMPLE_CSV = Path(r"D:\radar\car\debug_monitor\pwm2000_20260407_175513.csv")


@pytest.mark.skipif(not SAMPLE_CSV.exists(), reason="requires local pwm2000 sample CSV")
def test_load_debug_csv_and_detect_active_window() -> None:
    data = load_debug_csv(SAMPLE_CSV)

    assert data.sample_rate_hz == 100.0
    assert data.steady_slice.start < data.steady_slice.stop
    assert data.time_s[data.active_slice.start] == pytest.approx(2.57, abs=0.05)
    assert data.time_s[data.steady_slice.start] == pytest.approx(7.57, abs=0.05)
    assert data.time_s[data.steady_slice.stop - 1] == pytest.approx(28.75, abs=0.05)


def test_dominant_frequency_hz_detects_wheel_disturbance() -> None:
    sample_rate_hz = 100.0
    duration_s = 20.0
    t = np.arange(int(sample_rate_hz * duration_s)) / sample_rate_hz
    signal = 0.3 * np.sin(2.0 * np.pi * 0.64 * t) + 0.03 * np.sin(2.0 * np.pi * 1.8 * t)

    dominant = dominant_frequency_hz(signal, sample_rate_hz, min_hz=0.2, max_hz=5.0)

    assert dominant == pytest.approx(0.64, abs=0.03)


@pytest.mark.skipif(not SAMPLE_CSV.exists(), reason="requires local pwm2000 sample CSV")
def test_simulate_closed_loop_notch_prefers_narrow_bandwidth() -> None:
    data = load_debug_csv(SAMPLE_CSV)

    report = simulate_closed_loop_notch(
        data=data,
        motor="a",
        config=SimulationConfig(
            wheel_circ_m=np.pi * 0.125,
            gains=[0.01, 0.015, 0.02, 0.025, 0.03, 0.05, 0.08],
            kp=300.0,
            ki=300.0,
        ),
    )

    assert report.recommended.gain == pytest.approx(0.02, abs=0.001)
    assert report.recommended.error_reduction_pct > 75.0
    assert report.recommended.pi_reduction_pct > 80.0


@pytest.mark.skipif(not SAMPLE_CSV.exists(), reason="requires local pwm2000 sample CSV")
def test_cli_outputs_summary_for_sample_csv() -> None:
    result = subprocess.run(
        [
            sys.executable,
            "-m",
            "tools.pi_notch_analysis.cli",
            str(SAMPLE_CSV),
            "--motor",
            "a",
        ],
        check=False,
        capture_output=True,
        text=True,
    )

    assert result.returncode == 0, result.stderr
    assert "Recommended PI Notch" in result.stdout
    assert "Dominant disturbance" in result.stdout
    assert "gain=0.020" in result.stdout
