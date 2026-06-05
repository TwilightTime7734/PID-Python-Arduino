# PIDtoolbox Step Response Tool: what to port into the Python project

## Main finding

The Step Response Tool is driven mainly by two MATLAB/Octave files:

- `PTtuneUIcontrol.m` creates the Step Response Tool window and controls.
- `PTtuningParams.m` calls `PTstepcalc.m` for each selected file and each selected axis.
- `PTstepcalc.m` contains the actual algorithm.

The useful part for the Python project is `PTstepcalc.m`. The GUI code is not important except for the settings it exposes.

## Data used

For each selected log and axis, PIDtoolbox does this:

```matlab
H = T{f}.setpoint_0_(tIND{f});   % roll command / setpoint
G = T{f}.gyroADC_0_(tIND{f});    % roll measured gyro
[stepresp_A{p}, tA] = PTstepcalc(H, G, A_lograte(f), Ycorrection, smoothFactor)
```

For pitch and yaw it uses `setpoint_1_`, `gyroADC_1_`, and `setpoint_2_`, `gyroADC_2_`.

For INAV, `PTload.m` maps:

```matlab
setpoint_0_ = axisRate_0_
setpoint_1_ = axisRate_1_
setpoint_2_ = axisRate_2_
setpoint_3_ = rcData_3_ - 1000
```

So for our INAV Python project, use:

| Axis | Setpoint input | Gyro output |
|---|---|---|
| Roll | `axisRate_0_` or `setpoint_0_` | `gyroADC_0_` |
| Pitch | `axisRate_1_` or `setpoint_1_` | `gyroADC_1_` |
| Yaw | `axisRate_2_` or `setpoint_2_` | `gyroADC_2_` |

## Sample rate detail

PIDtoolbox computes:

```matlab
A_lograte = 1000 / median(diff(time_us_))
```

That gives samples per millisecond, also known as kHz. In Python it is clearer to use sample rate in Hz:

```python
sample_rate_hz = 1_000_000 / median(diff(time_us_))
```

## Algorithm summary

`PTstepcalc.m` does the following:

1. Smooths gyro with moving average window `[1, 20, 40, 60]` depending on smoothing setting.
2. Uses 2-second log segments.
3. Ignores segments where `max(abs(setpoint)) < 20 deg/s`.
4. Applies a Hann window to setpoint and gyro segment.
5. Pads both signals by 100 samples at front and back.
6. Computes FFT of both signals.
7. Estimates impulse response by deconvolution:

```text
impulse = ifft((gyro_fft * conj(setpoint_fft)) / (setpoint_fft * conj(setpoint_fft) + 0.0001))
```

8. Integrates impulse response using cumulative sum to get step response.
9. Keeps first 500 ms of that response.
10. Checks the 200-500 ms section. It keeps only traces whose steady-state response stays between 0.5 and 3.0.
11. Averages all accepted traces.
12. Reports:
    - `peak`: max response before 150 ms
    - `peak_time_ms`: time of that peak
    - `latency_half_height_ms`: first time the averaged response crosses 0.5
    - `n_traces`: number of accepted segments

## How to use in the Python project

The included `pidtoolbox_step_response.py` file contains a Python port.

Example:

```python
import pandas as pd
from pidtoolbox_step_response import compute_from_dataframe

csv_path = "decoded_blackbox.csv"
df = pd.read_csv(csv_path)

result = compute_from_dataframe(df, "roll", smooth_level=1, y_correction=False)

print(result.n_traces)
print(result.peak)
print(result.peak_time_ms)
print(result.latency_half_height_ms)
```

For plotting in your GUI:

```python
ax.plot(result.t_ms, result.mean_response)
ax.axhline(1.0, linestyle="--", linewidth=0.8)
ax.set_xlim(0, 500)
ax.set_ylim(0, 1.75)
```

## Tuning interpretation for your assistant

The Step Response Tool itself does not directly say “raise P by 3” or “lower D by 2.” It gives measured response behavior:

- Peak much above 1.0 means overshoot or bounce-back tendency.
- Peak near 1.0 with fast half-height latency means tight tracking.
- Slow half-height latency means sluggish response.
- No accepted traces means the log section did not contain enough useful setpoint movement, or the stand/test data is too noisy.

For your automatic tuner, use this as one more diagnostic next to setpoint-vs-gyro error:

- High overshoot: reduce P and/or D depending on noise and bounce symptoms.
- Low peak and slow latency: increase P or feedforward cautiously.
- Good peak but delayed response: feedforward can help if tracking lags command.
- Noisy accepted traces or very low `n_traces`: ask for a cleaner left/right movement sample before changing PID.
