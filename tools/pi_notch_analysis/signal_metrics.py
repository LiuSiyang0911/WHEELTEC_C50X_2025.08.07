from __future__ import annotations

import math

import numpy as np


def periodogram_amplitude(
    samples: np.ndarray,
    sample_rate_hz: float,
) -> tuple[np.ndarray, np.ndarray]:
    values = np.asarray(samples, dtype=float)
    centered = values - float(np.mean(values))
    if centered.size == 0:
        raise ValueError("No samples available for spectrum analysis")

    fft = np.fft.rfft(centered)
    frequency_hz = np.fft.rfftfreq(centered.size, d=1.0 / sample_rate_hz)
    amplitude = 2.0 * np.abs(fft) / centered.size
    if amplitude.size:
        amplitude[0] *= 0.5
    if centered.size % 2 == 0 and amplitude.size > 1:
        amplitude[-1] *= 0.5
    return frequency_hz, amplitude


def dominant_frequency_hz(
    samples: np.ndarray,
    sample_rate_hz: float,
    *,
    min_hz: float = 0.2,
    max_hz: float = 5.0,
) -> float:
    freq_hz, amplitude = periodogram_amplitude(samples, sample_rate_hz)
    mask = (freq_hz >= min_hz) & (freq_hz <= max_hz)
    if not np.any(mask):
        raise ValueError("No spectrum bins in requested frequency range")
    band_freq = freq_hz[mask]
    band_amplitude = amplitude[mask]
    return float(band_freq[int(np.argmax(band_amplitude))])


def sinusoid_amplitude(samples: np.ndarray, sample_rate_hz: float, frequency_hz: float) -> float:
    values = np.asarray(samples, dtype=float)
    t = np.arange(values.size, dtype=float) / sample_rate_hz
    phase = 2.0 * math.pi * frequency_hz * t
    c = np.dot(values, np.cos(phase))
    s = np.dot(values, np.sin(phase))
    return float(2.0 * math.hypot(c, s) / values.size)


def gain_to_bandwidth_hz(gain: float, sample_time_s: float) -> float:
    return float(-math.log(1.0 - gain) / (2.0 * math.pi * sample_time_s))


def gain_to_time_constant_s(gain: float, sample_time_s: float) -> float:
    return float(-sample_time_s / math.log(1.0 - gain))
