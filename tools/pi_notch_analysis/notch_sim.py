from __future__ import annotations

from dataclasses import asdict, dataclass

import numpy as np

from .csv_loader import DebugCsvData
from .signal_metrics import dominant_frequency_hz, gain_to_bandwidth_hz, gain_to_time_constant_s, sinusoid_amplitude


@dataclass(frozen=True)
class SimulationConfig:
    wheel_circ_m: float
    gains: list[float]
    kp: float = 300.0
    ki: float = 300.0
    kd: float = 0.0
    min_hz: float = 0.2
    max_hz: float = 5.0
    burn_in_s: float = 5.0
    min_target_speed: float = 0.10
    min_feedback_speed: float = 0.08
    reset_speed: float = 0.03
    command_flip_guard: float = -0.0025


@dataclass(frozen=True)
class CandidateResult:
    gain: float
    bandwidth_hz: float
    time_constant_s: float
    disturbance_hz: float
    error_amplitude: float
    pi_amplitude: float
    error_reduction_pct: float
    pi_reduction_pct: float

    def to_dict(self) -> dict[str, float]:
        return asdict(self)


@dataclass(frozen=True)
class SimulationReport:
    motor: str
    target_speed_mps: float
    disturbance_hz: float
    raw_error_amplitude: float
    raw_pi_amplitude: float
    recommended: CandidateResult
    candidates: list[CandidateResult]

    def to_dict(self) -> dict[str, object]:
        return {
            "motor": self.motor,
            "target_speed_mps": self.target_speed_mps,
            "disturbance_hz": self.disturbance_hz,
            "raw_error_amplitude": self.raw_error_amplitude,
            "raw_pi_amplitude": self.raw_pi_amplitude,
            "recommended": self.recommended.to_dict(),
            "candidates": [candidate.to_dict() for candidate in self.candidates],
        }


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
    config: SimulationConfig,
    sample_rate_hz: float,
) -> np.ndarray:
    phase = 0.0
    sin_state = 0.0
    cos_state = 0.0
    filtered = np.empty_like(raw_error, dtype=float)
    dt = 1.0 / sample_rate_hz

    for index, error in enumerate(raw_error):
        speed = float(feedback[index])
        if (
            abs(target_speed_mps) < config.min_target_speed
            or abs(speed) < config.min_feedback_speed
            or (abs(target_speed_mps) < config.reset_speed and abs(speed) < config.reset_speed)
            or (target_speed_mps * speed) < config.command_flip_guard
        ):
            phase = 0.0
            sin_state = 0.0
            cos_state = 0.0
            filtered[index] = error
            continue

        phase += 2.0 * np.pi * speed * dt / config.wheel_circ_m
        phase = float((phase + 2.0 * np.pi) % (2.0 * np.pi))
        sin_ref = float(np.sin(phase))
        cos_ref = float(np.cos(phase))
        sin_state += gain * ((error * sin_ref) - sin_state)
        cos_state += gain * ((error * cos_ref) - cos_state)
        filtered[index] = error - 2.0 * (sin_state * sin_ref + cos_state * cos_ref)

    return filtered


def _recommend_candidate(candidates: list[CandidateResult], disturbance_hz: float) -> CandidateResult:
    lower_bw = 0.45 * disturbance_hz
    upper_bw = 1.0 * disturbance_hz
    preferred = [
        candidate
        for candidate in candidates
        if lower_bw <= candidate.bandwidth_hz <= upper_bw
    ]
    pool = preferred or candidates
    return max(pool, key=lambda candidate: (candidate.pi_reduction_pct, candidate.error_reduction_pct, -candidate.gain))


def simulate_closed_loop_notch(
    *,
    data: DebugCsvData,
    motor: str,
    config: SimulationConfig,
    feedback_override: np.ndarray | None = None,
) -> SimulationReport:
    if motor not in {"a", "b"}:
        raise ValueError("motor must be 'a' or 'b'")

    feedback = np.asarray(feedback_override, dtype=float) if feedback_override is not None else data.motor_feedback(motor)[data.steady_slice]
    target_speed_mps = float(np.mean(feedback))
    raw_error = target_speed_mps - feedback
    disturbance_hz = dominant_frequency_hz(raw_error, data.sample_rate_hz, min_hz=config.min_hz, max_hz=config.max_hz)
    raw_pi_output = _simulate_incremental_pi(raw_error, kp=config.kp, ki=config.ki, kd=config.kd)

    burn_in_samples = min(int(round(config.burn_in_s * data.sample_rate_hz)), max(0, raw_error.size // 3))
    sample_slice = slice(burn_in_samples, None)

    raw_error_amplitude = sinusoid_amplitude(raw_error[sample_slice], data.sample_rate_hz, disturbance_hz)
    raw_pi_amplitude = sinusoid_amplitude(raw_pi_output[sample_slice], data.sample_rate_hz, disturbance_hz)

    candidates: list[CandidateResult] = []
    sample_time_s = 1.0 / data.sample_rate_hz
    for gain in config.gains:
        filtered_error = _simulate_sync_notch(
            raw_error,
            feedback,
            target_speed_mps,
            gain,
            config=config,
            sample_rate_hz=data.sample_rate_hz,
        )
        pi_output = _simulate_incremental_pi(filtered_error, kp=config.kp, ki=config.ki, kd=config.kd)
        error_amplitude = sinusoid_amplitude(filtered_error[sample_slice], data.sample_rate_hz, disturbance_hz)
        pi_amplitude = sinusoid_amplitude(pi_output[sample_slice], data.sample_rate_hz, disturbance_hz)
        candidates.append(
            CandidateResult(
                gain=float(gain),
                bandwidth_hz=gain_to_bandwidth_hz(float(gain), sample_time_s),
                time_constant_s=gain_to_time_constant_s(float(gain), sample_time_s),
                disturbance_hz=disturbance_hz,
                error_amplitude=error_amplitude,
                pi_amplitude=pi_amplitude,
                error_reduction_pct=float(100.0 * (1.0 - error_amplitude / raw_error_amplitude)),
                pi_reduction_pct=float(100.0 * (1.0 - pi_amplitude / raw_pi_amplitude)),
            )
        )

    recommended = _recommend_candidate(candidates, disturbance_hz)
    return SimulationReport(
        motor=motor,
        target_speed_mps=target_speed_mps,
        disturbance_hz=disturbance_hz,
        raw_error_amplitude=raw_error_amplitude,
        raw_pi_amplitude=raw_pi_amplitude,
        recommended=recommended,
        candidates=candidates,
    )
