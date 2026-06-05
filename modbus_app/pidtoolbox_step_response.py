"""
PIDtoolbox-style step response analysis for Python.

This is a Python port/adaptation of the step-response logic found in
OctavePIDtoolbox's PTstepcalc.m and PTtuningParams.m.

Original algorithm notes:
- Input signal: setpoint / commanded axis rate
- Output signal: gyroADC / measured axis rate
- Splits the selected log window into overlapping 2-second segments
- Estimates an impulse response by frequency-domain deconvolution
- Integrates impulse response to produce a step response
- Keeps only traces whose 200-500 ms steady-state section looks plausible
- Averages kept traces and reports peak and half-height latency

Use with INAV CSV columns:
- Roll:  setpoint_0_ or axisRate_0_, gyroADC_0_
- Pitch: setpoint_1_ or axisRate_1_, gyroADC_1_
- Yaw:   setpoint_2_ or axisRate_2_, gyroADC_2_

For INAV logs, PIDtoolbox maps axisRate_* to setpoint_*.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Optional, Sequence

import numpy as np


@dataclass
class StepResponseResult:
    t_ms: np.ndarray
    traces: np.ndarray          # shape: (n_traces, n_time_points)
    mean_response: np.ndarray   # shape: (n_time_points,)
    peak: float                 # max response before 150 ms
    peak_time_ms: float
    latency_half_height_ms: float
    n_traces: int
    sample_rate_hz: float


def _moving_average(x: np.ndarray, window: int) -> np.ndarray:
    """MATLAB smooth(..., 'moving') style simple centered moving average."""
    if window <= 1:
        return x.astype(float, copy=True)
    kernel = np.ones(int(window), dtype=float) / float(window)
    return np.convolve(x.astype(float), kernel, mode="same")


def _sample_rate_hz_from_time_us(time_us: Sequence[float]) -> float:
    t = np.asarray(time_us, dtype=float)
    if t.size < 2:
        raise ValueError("time_us needs at least two samples")
    dt_us = np.nanmedian(np.diff(t))
    if not np.isfinite(dt_us) or dt_us <= 0:
        raise ValueError("time_us must be increasing and finite")
    return 1_000_000.0 / dt_us


def compute_pidtoolbox_step_response(
    setpoint: Sequence[float],
    gyro: Sequence[float],
    *,
    time_us: Optional[Sequence[float]] = None,
    sample_rate_hz: Optional[float] = None,
    smooth_level: int = 0,
    y_correction: bool = False,
    min_input_deg_s: float = 20.0,
    segment_seconds: float = 2.0,
    response_ms: float = 500.0,
    steady_start_ms: float = 200.0,
    peak_window_ms: float = 150.0,
    regularization: float = 0.0001,
    pad_length: int = 100,
) -> StepResponseResult:
    """
    Compute a PIDtoolbox-style normalized step response from setpoint and gyro.

    Parameters
    ----------
    setpoint, gyro:
        Commanded and measured axis-rate arrays, normally deg/s.
    time_us:
        Optional Blackbox/INAV time column in microseconds. Used to compute sample rate.
    sample_rate_hz:
        Optional explicit sample rate. Provide either this or time_us.
    smooth_level:
        0=off, 1=low, 2=medium, 3=high. Matches PIDtoolbox smoothing choices.
    y_correction:
        Mimics PIDtoolbox's optional steady-state Y correction.
    min_input_deg_s:
        Segments with smaller setpoint magnitude are ignored.

    Returns
    -------
    StepResponseResult
        Contains individual accepted traces, average response, peak, peak time,
        and half-height latency.
    """
    sp = np.asarray(setpoint, dtype=float).ravel()
    gy = np.asarray(gyro, dtype=float).ravel()
    n = min(sp.size, gy.size)
    sp = sp[:n]
    gy = gy[:n]

    if n < 10:
        raise ValueError("setpoint/gyro arrays are too short")

    if sample_rate_hz is None:
        if time_us is None:
            raise ValueError("Provide either time_us or sample_rate_hz")
        sample_rate_hz = _sample_rate_hz_from_time_us(time_us)

    if sample_rate_hz <= 0:
        raise ValueError("sample_rate_hz must be positive")

    samples_per_ms = sample_rate_hz / 1000.0
    smooth_windows = [1, 20, 40, 60]
    smooth_level = int(np.clip(smooth_level, 0, 3))
    gy = _moving_average(gy, smooth_windows[smooth_level])

    segment_len = int(round(sample_rate_hz * segment_seconds))
    wnd = int(round(samples_per_ms * response_ms))
    if segment_len <= wnd + 2 or n <= segment_len:
        raise ValueError("log section is too short for the requested response window")

    t_ms = np.arange(wnd + 1, dtype=float) / samples_per_ms

    file_dur_sec = n / sample_rate_hz
    if file_dur_sec <= 20:
        subsample_factor = 10
    elif file_dur_sec <= 60:
        subsample_factor = 7
    else:
        subsample_factor = 3

    step = max(1, int(round(segment_len / subsample_factor)))
    starts = np.arange(0, n - segment_len, step, dtype=int)

    traces = []
    hann = np.hanning(segment_len + 1)
    steady_mask = (t_ms > steady_start_ms) & (t_ms < response_ms)
    if not np.any(steady_mask):
        raise ValueError("steady-state mask is empty; check response_ms")

    for start in starts:
        stop = start + segment_len + 1
        sp_seg = sp[start:stop]
        gy_seg = gy[start:stop]
        if sp_seg.size != segment_len + 1 or gy_seg.size != segment_len + 1:
            continue
        if np.nanmax(np.abs(sp_seg)) < min_input_deg_s:
            continue

        a = gy_seg * hann
        b = sp_seg * hann
        a = np.fft.fft(np.concatenate([np.zeros(pad_length), a, np.zeros(pad_length)]))
        b = np.fft.fft(np.concatenate([np.zeros(pad_length), b, np.zeros(pad_length)]))

        # Frequency-domain deconvolution: impulse = ifft(GY * conj(SP) / (SP * conj(SP) + reg))
        impulse = np.real(np.fft.ifft((a * np.conj(b)) / (b * np.conj(b) + regularization)))
        response = np.cumsum(impulse)
        response = response[: wnd + 1]

        steady = response[steady_mask]
        steady_mean = np.nanmean(steady)
        if not np.isfinite(steady_mean):
            continue

        if y_correction:
            # This intentionally mimics PTstepcalc.m: response *= (1 + (1 - mean)).
            # A mathematically cleaner normalization would be response /= mean.
            response = response * (2.0 - steady_mean)
            steady = response[steady_mask]

        # PIDtoolbox quality gate: accepted traces settle between 0.5 and 3.
        if np.nanmin(steady) > 0.5 and np.nanmax(steady) < 3.0:
            traces.append(response)

    if not traces:
        empty = np.empty((0, wnd + 1), dtype=float)
        return StepResponseResult(
            t_ms=t_ms,
            traces=empty,
            mean_response=np.full(wnd + 1, np.nan),
            peak=np.nan,
            peak_time_ms=np.nan,
            latency_half_height_ms=np.nan,
            n_traces=0,
            sample_rate_hz=float(sample_rate_hz),
        )

    traces_arr = np.vstack(traces)
    mean_response = np.nanmean(traces_arr, axis=0)

    before_peak = np.where(t_ms < peak_window_ms)[0]
    if before_peak.size:
        local = mean_response[before_peak]
        peak_idx = before_peak[int(np.nanargmax(local))]
        peak = float(mean_response[peak_idx])
        peak_time_ms = float(t_ms[peak_idx])
    else:
        peak = np.nan
        peak_time_ms = np.nan

    half = np.where(mean_response > 0.5)[0]
    latency = float(t_ms[half[0]]) if half.size else np.nan

    return StepResponseResult(
        t_ms=t_ms,
        traces=traces_arr,
        mean_response=mean_response,
        peak=peak,
        peak_time_ms=peak_time_ms,
        latency_half_height_ms=latency,
        n_traces=int(traces_arr.shape[0]),
        sample_rate_hz=float(sample_rate_hz),
    )


def axis_columns_for_inav(axis: str) -> tuple[str, str]:
    """Return preferred INAV CSV column names for an axis."""
    axis = axis.lower().strip()
    mapping = {
        "roll": ("axisRate_0_", "gyroADC_0_"),
        "pitch": ("axisRate_1_", "gyroADC_1_"),
        "yaw": ("axisRate_2_", "gyroADC_2_"),
    }
    if axis not in mapping:
        raise ValueError("axis must be roll, pitch, or yaw")
    return mapping[axis]


def compute_from_dataframe(df, axis: str, *, time_column: str = "time_us_", **kwargs) -> StepResponseResult:
    """Convenience helper for a pandas DataFrame from a decoded INAV CSV."""
    setpoint_col, gyro_col = axis_columns_for_inav(axis)
    if setpoint_col not in df.columns:
        # Some decoded logs already have setpoint_* instead of INAV axisRate_*.
        idx = {"roll": 0, "pitch": 1, "yaw": 2}[axis.lower().strip()]
        fallback = f"setpoint_{idx}_"
        if fallback in df.columns:
            setpoint_col = fallback
        else:
            raise KeyError(f"Missing setpoint column: {setpoint_col} or {fallback}")
    if gyro_col not in df.columns:
        raise KeyError(f"Missing gyro column: {gyro_col}")
    if time_column in df.columns:
        return compute_pidtoolbox_step_response(
            df[setpoint_col].to_numpy(),
            df[gyro_col].to_numpy(),
            time_us=df[time_column].to_numpy(),
            **kwargs,
        )
    return compute_pidtoolbox_step_response(
        df[setpoint_col].to_numpy(),
        df[gyro_col].to_numpy(),
        **kwargs,
    )
