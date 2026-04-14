from __future__ import annotations

import math
from pathlib import Path

import pytest
from fastapi.testclient import TestClient

from tools.pi_notch_analysis.webapp import app

SAMPLE_CSV = Path(r"D:\radar\car\debug_monitor\pwm2000_20260407_175513.csv")


@pytest.mark.skipif(not SAMPLE_CSV.exists(), reason="requires local pwm2000 sample CSV")
def test_index_page_serves_html() -> None:
    client = TestClient(app)

    response = client.get("/")

    assert response.status_code == 200
    assert "text/html" in response.headers["content-type"]
    assert "PI Notch Analysis" in response.text


@pytest.mark.skipif(not SAMPLE_CSV.exists(), reason="requires local pwm2000 sample CSV")
def test_upload_analysis_returns_summary_and_chart_payloads() -> None:
    client = TestClient(app)

    with SAMPLE_CSV.open("rb") as handle:
        response = client.post(
            "/api/analyze",
            data={
                "motor": "a",
                "wheel_diameter": "0.125",
                "kp": "300",
                "ki": "300",
                "kd": "0",
                "settle_time": "5.0",
                "tail_time": "5.0",
                "gains": "0.01,0.015,0.02,0.025,0.03,0.04,0.05,0.08",
                "speed_notch_enabled": "true",
                "speed_notch_freq_hz": "0.66",
                "speed_notch_q": "6.0",
            },
            files={"file": ("pwm2000.csv", handle, "text/csv")},
        )

    assert response.status_code == 200
    payload = response.json()
    assert payload["summary"]["motor"] == "a"
    assert payload["summary"]["speed_notch_enabled"] is True
    assert payload["summary"]["speed_notch_freq_hz"] == pytest.approx(0.66, abs=1e-6)
    assert payload["summary"]["speed_notch_q"] == pytest.approx(6.0, abs=1e-6)
    assert payload["summary"]["recommended_gain"] == pytest.approx(0.02, abs=0.001)
    assert payload["summary"]["dominant_disturbance_hz"] == pytest.approx(0.66, abs=0.05)
    assert payload["charts"]["time_series"]["feedback"]["x"]
    assert payload["charts"]["time_series"]["feedback"]["y"]
    assert payload["charts"]["time_series"]["notched_feedback"]["y"]
    assert payload["charts"]["spectrum"]["feedback"]["frequency_hz"]
    assert payload["charts"]["spectrum"]["error"]["frequency_hz"]
    assert payload["charts"]["gain_sweep"]["gain"]
    assert len(payload["charts"]["time_series"]["feedback"]["y"]) == len(payload["charts"]["time_series"]["notched_feedback"]["y"])
    assert payload["summary"]["recommended_bandwidth_hz"] == pytest.approx(
        -math.log(1.0 - 0.02) / (2.0 * math.pi * 0.01),
        abs=1e-3,
    )


@pytest.mark.skipif(not SAMPLE_CSV.exists(), reason="requires local pwm2000 sample CSV")
def test_speed_notch_changes_feedback_trace() -> None:
    client = TestClient(app)

    def post(speed_notch_enabled: str) -> dict[str, object]:
        with SAMPLE_CSV.open("rb") as handle:
            response = client.post(
                "/api/analyze",
                data={
                    "motor": "a",
                    "wheel_diameter": "0.125",
                    "kp": "300",
                    "ki": "300",
                    "kd": "0",
                    "settle_time": "5.0",
                    "tail_time": "5.0",
                    "gains": "0.01,0.015,0.02,0.025,0.03,0.04,0.05,0.08",
                    "speed_notch_enabled": speed_notch_enabled,
                    "speed_notch_freq_hz": "0.66",
                    "speed_notch_q": "6.0",
                },
                files={"file": ("pwm2000.csv", handle, "text/csv")},
            )
        assert response.status_code == 200
        return response.json()

    disabled_payload = post("false")
    enabled_payload = post("true")

    disabled = disabled_payload["charts"]["time_series"]["notched_feedback"]["y"]
    enabled = enabled_payload["charts"]["time_series"]["notched_feedback"]["y"]
    assert disabled != enabled
