# Thread State

## Current Focus
- The desktop app is being shaped into a safer INAV/Blackbox tuning helper.
- Recent work centered on auto-tune UI simplification, Blackbox step-response charting, and a supervised PID tuning workflow from `Instructions.txt`.
- A new `PID Tuning Plan` workflow now generates a staged roll/pitch tuning plan and saves report artifacts.
- User decision: do not test yaw as part of the sweep; provide a conservative yaw PID/FF recommendation at the end instead.

## Recent Progress
- Simplified the Auto Tune section in `modbus_app/ui.py`.
  - Removed `State`, `Command`, `Safety`.
  - Removed the entire `Calculated Pulse` group.
  - Removed `Calculate Pulse` and `Calculate Missing`.
  - Removed Roll/Pitch confidence labels and progress bars.
  - Moved `Report Files` above the report text box.
  - Report list now fills the section width except for a right-side button column: `Open Selected`, `Open All`, `Clear`.
- Removed old pulse-calibration runtime/code from `modbus_app/app.py`.
  - Auto tune now uses the randomized bounded controller defaults directly.
  - No stale `pulse_cal_*` UI wiring remains.
- Added/kept report file controls.
  - `Clear` deletes contents under `blackbox_imports` and `blackbox_imports/reports` after confirmation.
  - Folders are preserved.
- Added FC armed-state safety around Blackbox import/MSC.
  - App checks whether the FC reports armed.
  - If armed, it prompts to disarm before pulling logs or sending `msc`.
  - If armed state cannot be verified, it warns before continuing.
- Updated adaptive auto session behavior.
  - Random roll/pitch direction selection.
  - 60-second bounded routine.
  - Random force/time within safe limits.
  - Does not allow the drone to exceed the configured attitude safety envelope.
  - `force_max_us` is now `425`.
- Added step-response chart workflow.
  - Button renamed to `Chart Step Response`.
  - User can select up to 6 Blackbox logs.
  - Logs are decoded with the decoder in `tools`.
  - Each log gets its own color.
  - The HTML now only shows step-response charts, not separate Peak/Latency side charts.
  - `Y Correction` is always enabled.
- Added supervised PID tuning plan workflow.
  - New button: `PID Tuning Plan`.
  - New helper module: `modbus_app/pid_tuning_workflow.py`.
  - The dialog asks for all-up weight, motor KV, prop diameter, prop pitch, battery cell count, battery chemistry, and motor count.
  - The dialog now has a `Pavo Pico 2` checkbox that fills the BETAFPV Pavo Pico II O4 + LAVA II 580mAh defaults.
  - Current P, motor heat, and vague "authority" questions were removed after review; this is for estimating a fresh safe start before motors have been used.
  - It suggests safe roll/pitch starting P values and generates the D -> P -> D re-check -> I -> FF workflow.
  - It writes `pid_tuning_plan.txt` and `pid_tuning_plan_summary.json` under `blackbox_imports/reports/pid_tuning_plan_*`.
  - Yaw is not included in test sweeps; final yaw starts from P `45`, I `60`, D `0`, FF `86`, with yaw P scaled down for high-risk hardware.
- Revised `Start Auto Session` behavior for PID tuning.
  - After FC/telemetry checks and the preflight confirmation, it looks first for `blackbox_imports/reports/pid_tuning_plan_20260605_201036/pid_tuning_plan.txt`, then falls back to the newest generated `pid_tuning_plan.txt`.
  - The button becomes `Next PID Plan Step` while the guided plan is active.
  - Each press checks current FC PID/FF values against the next plan target, asks before writing, verifies disarmed state before writes, then tells the user to fly/log and disarm before pressing the button again.
  - Prompts now explicitly say the sequence for each candidate: disarm, write/check PID values only while disarmed, arm, press `Fly/Log`, land, disarm, then continue.
  - The initial safe-start step now checks armed state immediately, waits for disarm if needed, writes the safe starting PID/FF values automatically while disarmed, then tells the user to arm, press `Fly/Log`, and disarm before the next step.
  - Later `Next PID Plan Step` presses also verify disarmed state before advancing to the next candidate prompt.
  - The runner steps through safe start, D sweep, optional D, P sweep, D re-check, I sweep, FF sweep, then asks about final roll/pitch and conservative yaw writes.
  - It does not run the old randomized stick-pulse auto session from this button anymore.
- Added a real `Fly/Log` button for active PID plan candidates.
  - It is enabled only after a candidate PID/FF target has been written and is waiting for a log run.
  - It requires Arduino output connected, FC connected, live attitude telemetry, and FC armed.
  - If the FC is not armed, it reminds the user to arm before pressing `Fly/Log`.
  - It runs the bounded roll/pitch movement routine for logging and does not write PID/FF values while armed.
  - When complete, it tells the user to disarm before pressing `Next PID Plan Step`.
- Added a visual-only `Simulate` button next to `Start Auto Session`.
  - It now forces a no-hardware condition: Arduino output and FC must be disconnected before simulation starts.
  - It loads the PID tuning plan and steps through the plan one simulated candidate at a time.
  - `Simulate` now stages a candidate by writing the simulated target values into the roll/pitch PID/FF UI boxes only; it does not write INAV settings.
  - After a simulated candidate is staged, the same `Fly/Log` button is enabled and runs the synthetic roll/pitch horizon movement for that simulated candidate.
  - `Next Sim Step` stays disabled until the simulated `Fly/Log` movement has run.
  - Simulation text mirrors the real safe sequence: write only while disarmed, then arm/fly/log, land, disarm, continue.
  - It does not move servos, send PPM, pull logs, or touch the FC.
- Extended INAV setting-name maps to include yaw PID/FF names for readback/future final-value support:
  - `mc_p_yaw`, `mc_i_yaw`, `mc_d_yaw`, `mc_cd_yaw`.

## Step Response Details
- Main generator: `modbus_app/step_response_report.py`.
- Core math: `modbus_app/pidtoolbox_step_response.py`.
- Current settings include:
  - `y_correction = True`
  - response window: `500 ms`
  - smoothing off by default
  - detail-preserving overlap/subsample behavior
- Fresh generated example:
  - `blackbox_imports/reports/step_response_20260605_022303/pidtoolbox_step_response_detail.html`

## Full Tuning Suite Discussion
`Instructions.txt` currently describes this workflow:
- Start with only P and D.
- Set I and FF to `0`.
- Start from Roll P `45`, Pitch P `47`, Yaw P `45`, Roll/Pitch D `17`, and Yaw D low/`0`.
- Log D values `17`, `23`, `30`, `36`; optional `42` only if needed and motors stay cool.
- Use Step Response Tool with Y Correction.
- Pick best D from the chart.
- With best D set, sweep P:
  - Roll example: `40`, `45`, `50`, `55`.
  - Pitch example: `42`, `47`, `52`, `57`.
  - Pick the highest P that tracks well without ringing or oscillation.
- Re-check D slightly lower/current/higher with chosen P.
- With correct P/D set, sweep I:
  - Roll/Pitch: `35/40`, `60/65`, `85/90`, `110/115`.
- Pick best I from the chart.
- With correct P/D/I set, sweep FF:
  - Roll/Pitch: `43/44`, `86/89`, `129/134`, `172/179`.
- Pick best FF from the chart.
- Yaw is not tested per user preference; plan recommends a conservative final yaw PID/FF based on P `45`, I `60`, D `0`, FF `86`, with P reduced if the hardware estimator reduces roll/pitch P.

## Safe Starting P Inputs
The program can suggest a conservative roll/pitch P start when it knows:
- All-up weight.
- Motor KV.
- Prop diameter and prop pitch.
- Battery cell count.
- Battery chemistry: LiPo, LiHV, or Li-ion.
- Motor count, defaulting to 4.

Default/no-risk inputs keep the instruction baselines:
- Roll P `45`.
- Pitch P `47`.

The heuristic is intentionally conservative:
- It estimates a no-load RPM/prop-size index from motor KV, battery voltage, and prop diameter.
- It estimates disk loading from AUW, prop diameter, and motor count.
- It applies extra margin for very small props, large props, and high prop pitch.
- It does not claim an official INAV formula; it only nudges the written baseline down before the supervised sweep.

## What Can Be Automated Later
- Write candidate PID/FF values to INAV over MSP.
- Pull/decode logs.
- Generate Y-corrected step-response charts.
- Organize reports by stage: D sweep, P sweep, D re-check, I sweep, FF sweep.
- Add stage metadata so results are easier to compare.
- Add a scoring function for "best follows setpoint" using overshoot, rise behavior, ripple/oscillation, and steady-state error.
- Present a recommended winner per stage before writing final values.

## What Should Stay Supervised
- Arming and flying.
- Any PID setting change while armed.
- Deciding whether to save final values.
- Confirming the "best chart" pick until the scoring heuristic proves reliable.
- Reviewing the final yaw recommendation before saving it, since yaw is not being tested.
- Entering MSC/log-pull mode, because the app should continue requiring the drone to be disarmed first.

## Open Questions Before Full Suite Implementation
- Should sweeps be done while armed in one continuous Blackbox file, or as separate disarmed write/log cycles?
  - The one-file workflow may require live PID writes while armed.
  - Safer implementation is staged: write values while disarmed, fly/log, disarm, pull/analyze.
- What exact scoring rule should decide the best D/P/I/FF candidate?
  - Need agreement before the app writes final values automatically.
- Should the app add a disarmed-only candidate writer next, or stay report-only for one more tuning session?

## Verification Status
- Latest checks:
  - `python -m py_compile main.py modbus_app\app.py modbus_app\ui.py modbus_app\pid_tuning_workflow.py serialUSB\inav_serial_service.py` passed.
  - `python -m unittest tests.test_pid_tuning_workflow tests.test_inav_serial_service` passed: 6 tests.
  - After the P-start input revision, the same py_compile and focused unittest commands passed again.
  - `python -m unittest discover -s tests` was attempted but could not complete because the active Python 3.14 environment does not have `numpy` installed. The failures were import errors in existing NumPy-dependent tests.

## Historical Firmware Notes
- Firmware and app version were previously aligned at `1.0.0`.
- UNO R4 WiFi/Minima builds were previously verified.
- Firmware upload to `COM6` was previously successful.
- Firmware exposes version via Modbus registers.
- Python app checks firmware version and warns on mismatch.
- Known PlatformIO executable on this PC:
  - `C:\Users\I Love Judy\.platformio\penv\Scripts\platformio.exe`
- Build project config used previously:
  - `_buildcheck/pio_main/platformio.ini`
