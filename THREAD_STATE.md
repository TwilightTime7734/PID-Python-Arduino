# Thread State

## Current Focus
- The desktop app is a safer supervised INAV/Blackbox PID tuning helper for a PPM Modbus Arduino output path.
- The main workflow is no longer the old randomized auto-tune button path. `Start Auto Session` now drives the guided PID tuning plan.
- User preference: do not test yaw in the sweep. Provide conservative yaw PID/FF recommendations at final write only.

## Current Code Shape
- Entrypoint: `main.py`.
  - Re-execs into `.venv\Scripts\python.exe` or `pythonw.exe` when the repo virtual environment exists.
  - Then calls `modbus_app.app.main()`.
- App runtime: `modbus_app/app.py`.
  - Refactored into `ModbusApp`.
  - Most UI callbacks are currently local functions inside `ModbusApp.run()`.
  - The bottom-level `main()` creates Tk, instantiates `ModbusApp`, and starts it.
- Hardware output helper: `modbus_app/hardware_controller.py`.
  - Owns `SerialWorker`, Arduino serial state, PPM start/stop, queued live channel updates, and marker-aware output.
  - Pads output to 8 channels and uses channel 8 as the marker channel.
- UI layout: `modbus_app/ui.py`.
  - Main Controls has only adjust/default/offset/idle output controls.
  - Old `Pulse Str`, `Duration`, `Angle`, and `Pulse` controls are removed.
  - Arduino/FC action buttons are in Main Controls in an aligned 4-column grid.
  - `Fly/Log` button width is `18` so pushed states fit.
  - `Simulate` is a checkbox in the Main Controls action grid.
  - Auto Tune Session has `Start Auto Session`, `Fly/Log`, `Progress`, `Chart Step Response`, `PID Tuning Plan`, and the report text box.
  - The old `Report Files` list and its `Open Selected`, `Open All`, and `Clear` buttons are removed.

## Recent Important Changes
- Removed the Report Files UI and code behind it.
  - No `auto_report_files` state remains.
  - No listbox refresh/open/clear handlers remain.
  - Generated reports are still written to disk and the report text panel still shows the current summary.
- Added/hardened a PID progress window.
  - Opened by the `Progress` button.
  - Shows phase flow, current step, current action, plan path, target values, and selected winners.
  - Window is built while hidden and only shown after widgets are created, avoiding blank partial windows.
- Moved simulation from a push button to a checkbox.
  - Simulation requires both Arduino output and FC to be disconnected.
  - Connecting FC or Arduino is blocked while Simulate is checked.
  - Simulation does not touch the FC, Arduino, servos, logs, or MSC.
- Added a `HardwareController` abstraction.
  - The app delegates Arduino output start/stop and queued channel updates to it.
  - The controller supports queued updates with marker state, avoiding duplicate hardware state variables in the app.

## PID Tuning Plan Workflow
- Button: `PID Tuning Plan`.
- Module: `modbus_app/pid_tuning_workflow.py`.
- The generated plan follows `blackbox_imports/pid_tuning_plan_sample.txt` style.
- The dialog asks for:
  - all-up weight
  - motor KV
  - prop diameter and pitch
  - battery cell count
  - battery chemistry
  - motor count
- `Pavo Pico 2` preset fills BETAFPV Pavo Pico II O4 + LAVA II 580mAh defaults.
- Generated artifacts:
  - `pid_tuning_plan.txt`
  - `pid_tuning_plan_summary.json`
  - Stored under `blackbox_imports/reports/pid_tuning_plan_*`.
- Summary JSON includes the same plan text that was generated for the user.
- Current plan sequence:
  - safe start
  - D sweep
  - optional D
  - P sweep
  - D re-check
  - I sweep
  - FF sweep
  - final roll/pitch selection plus conservative yaw write prompt
- Safe-start and every candidate write must happen while disarmed.

## Fly/Log Workflow
- `Fly/Log` is enabled only when a real or simulated PID plan candidate is staged.
- Real `Fly/Log` requires:
  - Arduino output connected
  - FC connected
  - live attitude telemetry
  - FC armed
  - simulation mode off
- Real `Fly/Log` flow:
  - prepares outputs first
  - waits the configured spin-up delay
  - sets CH8 high as the beeper marker
  - runs the bounded roll/pitch movement for `PID_PLAN_FLY_LOG_RUNTIME_S`
  - sets CH8 low when complete or aborted
- Current constants:
  - `BEEPER_MARKER_CHANNEL_INDEX = 7`
  - `BEEPER_MARKER_ON_US = 2000`
  - `BEEPER_MARKER_OFF_US = 1000`
  - `BEEPER_MARKER_SPINUP_DELAY_MS = 2000`
  - `PID_PLAN_FLY_LOG_RUNTIME_S = 30.0`
- There is no throttle-based fallback marker.
- INAV logs only the first 4 RC channels, so the program should not search for CH8 columns in logs.

## Step Response Workflow
- Button: `Chart Step Response`.
- Module: `modbus_app/step_response_report.py`.
- User can select up to `MAX_STEP_RESPONSE_LOGS` Blackbox logs.
- Raw logs are decoded with `tools/blackbox_decode_INAV.exe`.
- Y Correction is always enabled.
- The step-response analyzer uses BEEPERON/BEEPER flight-mode flags as the analysis window when present.
- It uses the first continuous BEEPERON marker run:
  - starts at first BEEPERON
  - stops when that marker turns off
  - does not scan forward for extra marker runs
- If no BEEPERON flag is found, it falls back to the full log and reports that fallback in the summary.
- It intentionally does not look for CH8 columns.

## Simulation Workflow
- Simulation starts through `Start Auto Session` while the `Simulate` checkbox is checked.
- It stages each plan candidate by updating the PID/FF display boxes only.
- Simulated `Fly/Log` runs for the same configured `PID_PLAN_FLY_LOG_RUNTIME_S` duration as real Fly/Log.
- Synthetic roll/pitch motion repeats across the whole runtime instead of stopping after the old 7.5-second preview.
- `Next Sim Step` stays blocked until the simulated `Fly/Log` has run.

## Safety Rules To Preserve
- No auto-connects to FC or Arduino.
- Simulation mode must require FC and Arduino disconnected.
- Do not write PID/FF settings while armed.
- Before Blackbox import/MSC, verify the FC is disarmed or warn if arm state cannot be verified.
- User remains responsible for arming, flying, landing, disarming, reviewing charts, and deciding final winners.
- Final values should not be saved automatically without explicit user confirmation.

## Reports And Files
- Generated report files still exist on disk under `blackbox_imports/reports`.
- The GUI no longer has the Report Files list or open/clear buttons.
- Report summaries are still displayed in the Auto Tune Session text box.
- `run_app.ps1` and `main.py` should use `.venv` to avoid the system Python path problem.

## Current Verification Status
- Latest checks passed under `.venv`:
  - `.venv\Scripts\python.exe -m py_compile main.py modbus_app\app.py modbus_app\ui.py modbus_app\hardware_controller.py modbus_app\pid_tuning_workflow.py modbus_app\step_response_report.py`
  - `.venv\Scripts\python.exe -m unittest discover -s tests`
  - Test result: 23 tests passed.
- Avoid plain `python` or PowerShell commands that resolve to `C:\Program Files\Python314\python.exe`; that environment may not have Plotly/Numpy installed.

## Open Follow-Ups
- Consider adding tests around `HardwareController` marker padding and queued marker updates.
- Consider adding tests that the Progress window can be constructed without a live plan.
- Consider splitting large nested callback sections out of `ModbusApp.run()` once behavior stabilizes.
- Future automation may include stage metadata and chart scoring, but winner selection should remain supervised until the scoring rule is agreed and proven.
