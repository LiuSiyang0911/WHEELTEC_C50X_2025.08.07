from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from .csv_loader import load_debug_csv_text
from .notch_sim import SimulationConfig, SimulationReport, simulate_closed_loop_notch
from .signal_metrics import periodogram_amplitude


@dataclass(frozen=True)
class AnalysisRequest:
    csv_text: str
    source_name: str
    motor: str
    wheel_diameter: float
    kp: float
    ki: float
    kd: float
    settle_time: float
    tail_time: float
    gains: list[float]
    speed_notch_enabled: bool
    speed_notch_freq_hz: float
    speed_notch_q: float


def _simulate_incremental_pi(error: np.ndarray, *, kp: float, ki: float, kd: float) -> np.ndarray:
    output = np.zeros_like(error, dtype=float)
    last_bias = 0.0
    lastest_bias = 0.0
    accumulator = 0.0
    for index, bias in enumerate(error):
        delta = kp * (bias - last_bias) + ki * bias + kd * (bias - 2.0 * last_bias + lastest_bias)
        accumulator += delta
        output[index] = accumulator
        lastest_bias = last_bias
        last_bias = float(bias)
    return output


def _simulate_sync_notch(
    raw_error: np.ndarray,
    feedback: np.ndarray,
    target_speed_mps: float,
    gain: float,
    *,
    wheel_circ_m: float,
    sample_rate_hz: float,
) -> np.ndarray:
    phase = 0.0
    sin_state = 0.0
    cos_state = 0.0
    filtered = np.empty_like(raw_error, dtype=float)
    dt = 1.0 / sample_rate_hz

    for index, error in enumerate(raw_error):
        speed = float(feedback[index])
        if abs(target_speed_mps) < 0.10 or abs(speed) < 0.08 or (abs(target_speed_mps) < 0.03 and abs(speed) < 0.03) or (target_speed_mps * speed) < -0.0025:
            phase = 0.0
            sin_state = 0.0
            cos_state = 0.0
            filtered[index] = error
            continue

        phase += 2.0 * np.pi * speed * dt / wheel_circ_m
        phase = float((phase + 2.0 * np.pi) % (2.0 * np.pi))
        sin_ref = float(np.sin(phase))
        cos_ref = float(np.cos(phase))
        sin_state += gain * ((error * sin_ref) - sin_state)
        cos_state += gain * ((error * cos_ref) - cos_state)
        filtered[index] = error - 2.0 * (sin_state * sin_ref + cos_state * cos_ref)

    return filtered


def _apply_biquad_notch(samples: np.ndarray, sample_rate_hz: float, center_hz: float, q: float) -> np.ndarray:
    values = np.asarray(samples, dtype=float)
    if center_hz <= 0.0:
        raise ValueError("speed_notch_freq_hz must be > 0")
    if q <= 0.0:
        raise ValueError("speed_notch_q must be > 0")
    nyquist_hz = 0.5 * sample_rate_hz
    if center_hz >= nyquist_hz:
        raise ValueError("speed_notch_freq_hz must be below Nyquist")

    omega = 2.0 * np.pi * center_hz / sample_rate_hz
    alpha = np.sin(omega) / (2.0 * q)
    cos_omega = np.cos(omega)

    b0 = 1.0
    b1 = -2.0 * cos_omega
    b2 = 1.0
    a0 = 1.0 + alpha
    a1 = -2.0 * cos_omega
    a2 = 1.0 - alpha

    b0 /= a0
    b1 /= a0
    b2 /= a0
    a1 /= a0
    a2 /= a0

    output = np.zeros_like(values)
    x1 = 0.0
    x2 = 0.0
    y1 = 0.0
    y2 = 0.0
    for index, x0 in enumerate(values):
        y0 = b0 * x0 + b1 * x1 + b2 * x2 - a1 * y1 - a2 * y2
        output[index] = y0
        x2 = x1
        x1 = x0
        y2 = y1
        y1 = y0
    return output


def _spectrum_payload(samples: np.ndarray, sample_rate_hz: float) -> dict[str, list[float]]:
    frequency_hz, amplitude = periodogram_amplitude(samples, sample_rate_hz)
    return {
        "frequency_hz": frequency_hz.tolist(),
        "amplitude": amplitude.tolist(),
    }


def _time_series_payload(
    time_s: np.ndarray,
    feedback: np.ndarray,
    notched_feedback: np.ndarray,
    raw_error: np.ndarray,
    filtered_error: np.ndarray,
    raw_pi: np.ndarray,
    filtered_pi: np.ndarray,
) -> dict[str, dict[str, list[float]]]:
    return {
        "feedback": {"x": time_s.tolist(), "y": feedback.tolist()},
        "notched_feedback": {"x": time_s.tolist(), "y": notched_feedback.tolist()},
        "raw_error": {"x": time_s.tolist(), "y": raw_error.tolist()},
        "filtered_error": {"x": time_s.tolist(), "y": filtered_error.tolist()},
        "raw_pi": {"x": time_s.tolist(), "y": raw_pi.tolist()},
        "filtered_pi": {"x": time_s.tolist(), "y": filtered_pi.tolist()},
    }


def _gain_sweep_payload(report: SimulationReport) -> dict[str, list[float]]:
    return {
        "gain": [candidate.gain for candidate in report.candidates],
        "bandwidth_hz": [candidate.bandwidth_hz for candidate in report.candidates],
        "error_reduction_pct": [candidate.error_reduction_pct for candidate in report.candidates],
        "pi_reduction_pct": [candidate.pi_reduction_pct for candidate in report.candidates],
    }


def analyze_uploaded_csv(request: AnalysisRequest) -> dict[str, object]:
    data = load_debug_csv_text(
        request.csv_text,
        source_name=request.source_name,
        settle_time_s=request.settle_time,
        tail_time_s=request.tail_time,
    )
    config = SimulationConfig(
        wheel_circ_m=request.wheel_diameter * np.pi,
        gains=request.gains,
        kp=request.kp,
        ki=request.ki,
        kd=request.kd,
    )
    raw_feedback = data.motor_feedback(request.motor)[data.steady_slice]
    analysis_feedback = raw_feedback
    if request.speed_notch_enabled:
        analysis_feedback = _apply_biquad_notch(
            raw_feedback,
            data.sample_rate_hz,
            request.speed_notch_freq_hz,
            request.speed_notch_q,
        )

    report = simulate_closed_loop_notch(
        data=data,
        motor=request.motor,
        config=config,
        feedback_override=analysis_feedback,
    )

    steady_time = data.time_s[data.steady_slice]
    raw_error = report.target_speed_mps - analysis_feedback
    filtered_error = _simulate_sync_notch(
        raw_error,
        analysis_feedback,
        report.target_speed_mps,
        report.recommended.gain,
        wheel_circ_m=config.wheel_circ_m,
        sample_rate_hz=data.sample_rate_hz,
    )
    raw_pi = _simulate_incremental_pi(raw_error, kp=request.kp, ki=request.ki, kd=request.kd)
    filtered_pi = _simulate_incremental_pi(filtered_error, kp=request.kp, ki=request.ki, kd=request.kd)

    return {
        "summary": {
            "source_name": request.source_name,
            "motor": request.motor,
            "sample_rate_hz": data.sample_rate_hz,
            "active_start_s": float(data.time_s[data.active_slice.start]),
            "active_stop_s": float(data.time_s[data.active_slice.stop - 1]),
            "steady_start_s": float(data.time_s[data.steady_slice.start]),
            "steady_stop_s": float(data.time_s[data.steady_slice.stop - 1]),
            "target_speed_mps": report.target_speed_mps,
            "dominant_disturbance_hz": report.disturbance_hz,
            "recommended_gain": report.recommended.gain,
            "recommended_bandwidth_hz": report.recommended.bandwidth_hz,
            "recommended_time_constant_s": report.recommended.time_constant_s,
            "error_reduction_pct": report.recommended.error_reduction_pct,
            "pi_reduction_pct": report.recommended.pi_reduction_pct,
            "speed_notch_enabled": request.speed_notch_enabled,
            "speed_notch_freq_hz": request.speed_notch_freq_hz,
            "speed_notch_q": request.speed_notch_q,
        },
        "charts": {
            "time_series": _time_series_payload(steady_time, raw_feedback, analysis_feedback, raw_error, filtered_error, raw_pi, filtered_pi),
            "spectrum": {
                "feedback": _spectrum_payload(raw_feedback, data.sample_rate_hz),
                "notched_feedback": _spectrum_payload(analysis_feedback, data.sample_rate_hz),
                "error": _spectrum_payload(raw_error, data.sample_rate_hz),
            },
            "gain_sweep": _gain_sweep_payload(report),
        },
        "candidates": [candidate.to_dict() for candidate in report.candidates],
    }
