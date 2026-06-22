"""Supervised PID tuning workflow helpers."""

from __future__ import annotations

import json
import re
from dataclasses import asdict, dataclass
from datetime import datetime
from pathlib import Path

from .constants import PID_PLAN_FLY_LOG_RUNTIME_S

TUNED_AXES = ("roll", "pitch")
START_P_DEFAULTS = {"roll": 45, "pitch": 47, "yaw": 45}
START_I_DEFAULTS = {"roll": 30, "pitch": 35}
D_SWEEP_VALUES = (17, 23, 30, 36, 42)
OPTIONAL_D_VALUE = None
I_SWEEP_VALUES = (
    {"roll": 35, "pitch": 40},
    {"roll": 60, "pitch": 65},
    {"roll": 85, "pitch": 90},
    {"roll": 110, "pitch": 115},
)
FF_SWEEP_VALUES = (
    {"roll": 43, "pitch": 44},
    {"roll": 86, "pitch": 89},
    {"roll": 129, "pitch": 134},
    {"roll": 172, "pitch": 179},
)
YAW_FINAL_DEFAULTS = {"p": 45, "i": 60, "d": 0, "ff": 86}
BATTERY_NOMINAL_VOLTAGE = {
    "lipo": 3.7,
    "lihv": 3.8,
    "liion": 3.6,
}
BATTERY_CHEMISTRY_VALUES = set(BATTERY_NOMINAL_VOLTAGE)
BASELINE_NO_LOAD_RPM_INCH = 2300.0 * 4.0 * BATTERY_NOMINAL_VOLTAGE["lipo"] * 5.0
BASELINE_DISK_LOADING_G_IN2 = 600.0 / (4.0 * 3.141592653589793 * 2.5 * 2.5)

REFERENCE_TEST_THROTTLE_US = 1250
REFERENCE_HOVER_THROTTLE_US = 1350
REFERENCE_MOTOR_KV = 14000
REFERENCE_PROP_DIAMETER_IN = 1.77
REFERENCE_PROP_PITCH_IN = 1.5
REFERENCE_BATTERY_CELLS = 2
REFERENCE_BATTERY_CHEMISTRY = "lihv"
REFERENCE_AUW_G = 83
REFERENCE_MOTOR_COUNT = 4
MIN_LEVEL_TEST_THROTTLE_US = 1125
MAX_LEVEL_TEST_THROTTLE_US = 1350
MIN_HOVER_THROTTLE_US = 1175
MAX_HOVER_THROTTLE_US = 1600


@dataclass(frozen=True)
class PStartInputs:
    """Information used to choose a conservative roll/pitch P starting point."""

    all_up_weight_g: int | None = None
    motor_kv: int | None = None
    prop_diameter_in: float | None = None
    prop_pitch_in: float | None = None
    battery_cells: int | None = None
    battery_chemistry: str = "lipo"
    motor_count: int = 4


PAVO_PICO_II_PRESET_INPUTS = PStartInputs(
    all_up_weight_g=83,
    motor_kv=14000,
    prop_diameter_in=1.77,
    prop_pitch_in=1.5,
    battery_cells=2,
    battery_chemistry="lihv",
    motor_count=4,
)


@dataclass(frozen=True)
class TestThrottleEstimate:
    """Estimated throttle targets for test-stand setup and manual verification."""

    level_test_throttle_us: int
    hover_throttle_us: int
    lift_target_low_g: int | None
    lift_target_mid_g: int | None
    lift_target_high_g: int | None
    notes: tuple[str, ...]


@dataclass(frozen=True)
class PStartRecommendation:
    """Starting P values and the generated supervised tuning workflow."""

    start_p: dict[str, int]
    start_i: dict[str, int]
    p_sweep: dict[str, tuple[int, ...]]
    yaw_final_pid_ff: dict[str, int]
    notes: tuple[str, ...]
    inputs: PStartInputs
    throttle_estimate: TestThrottleEstimate


@dataclass(frozen=True)
class PIDTuningPlanReport:
    report_dir: str
    text_path: str
    summary_json: str


@dataclass(frozen=True)
class LoadedPIDTuningPlan:
    text_path: str
    summary_json: str | None
    text: str
    start_p: dict[str, int]
    start_i: dict[str, int]
    p_sweep: dict[str, tuple[int, ...]]
    yaw_final_pid_ff: dict[str, int]
    d_sweep: tuple[int, ...]
    optional_d: int | None
    i_sweep: tuple[dict[str, int], ...]
    ff_sweep: tuple[dict[str, int], ...]
    level_test_throttle_us: int | None = None


def safe_p_start_information_needed() -> tuple[str, ...]:
    """Return the user-facing inputs that make a P start less guessy."""

    return (
        "All-up weight in grams.",
        "Motor KV.",
        "Prop size, ideally diameter and pitch in inches.",
        "Battery cell count and chemistry, such as 2S LiPo or 4S Li-ion.",
        "Motor count, defaulting to 4 for a quad.",
    )


def suggest_starting_p(inputs: PStartInputs) -> PStartRecommendation:
    """Suggest conservative roll/pitch P starts and keep yaw as an end-state recommendation."""

    factor = 1.0
    notes: list[str] = []
    chemistry = _choice(inputs.battery_chemistry, BATTERY_CHEMISTRY_VALUES, "lipo")
    motor_count = _bounded_int(inputs.motor_count, 1, 16, 4)

    if inputs.motor_kv is not None and inputs.battery_cells is not None and inputs.prop_diameter_in is not None:
        nominal_voltage = float(inputs.battery_cells) * BATTERY_NOMINAL_VOLTAGE[chemistry]
        rpm_inch_index = (float(inputs.motor_kv) * nominal_voltage * inputs.prop_diameter_in) / BASELINE_NO_LOAD_RPM_INCH
        if rpm_inch_index >= 1.45:
            factor *= 0.82
            notes.append("Very high KV/cell-count/prop speed index, so the starting P was reduced strongly.")
        elif rpm_inch_index >= 1.20:
            factor *= 0.90
            notes.append("High KV/cell-count/prop speed index, so the starting P was reduced.")
        elif rpm_inch_index <= 0.55:
            factor *= 0.94
            notes.append("Very low KV/cell-count/prop speed index, so the starting P was kept cautious for first lift-off.")
        else:
            notes.append("KV, cell count, and prop diameter are near the normal baseline range.")
    else:
        notes.append("KV, cell count, and prop diameter were not all provided, so drivetrain speed was not estimated.")

    if inputs.all_up_weight_g is not None and inputs.prop_diameter_in is not None:
        disk_area = motor_count * 3.141592653589793 * (inputs.prop_diameter_in / 2.0) ** 2
        disk_loading = float(inputs.all_up_weight_g) / disk_area if disk_area > 0 else 0.0
        loading_index = disk_loading / BASELINE_DISK_LOADING_G_IN2 if BASELINE_DISK_LOADING_G_IN2 > 0 else 1.0
        if loading_index >= 2.0:
            factor *= 0.86
            notes.append("High disk loading for the prop/motor count, so the starting P was reduced.")
        elif loading_index >= 1.45:
            factor *= 0.92
            notes.append("Moderately high disk loading, so a small safety margin was applied.")
        elif loading_index <= 0.55:
            factor *= 0.95
            notes.append("Low disk loading can feel very responsive, so a small safety margin was applied.")
        else:
            notes.append("Disk loading is near the normal baseline range.")
    else:
        notes.append("AUW and prop diameter were not both provided, so disk loading was not estimated.")

    if inputs.prop_diameter_in is not None:
        if inputs.prop_diameter_in < 3.5:
            factor *= 0.93
            notes.append("Small props usually run high RPM, so the starting P was reduced.")
        elif inputs.prop_diameter_in > 7.0:
            factor *= 0.88
            notes.append("Large props have more inertia, so the starting P was reduced.")
        elif inputs.prop_diameter_in > 6.0:
            factor *= 0.94
            notes.append("Larger-than-typical props selected, so a small safety margin was applied.")

    if inputs.prop_pitch_in is not None:
        if inputs.prop_pitch_in >= 5.5:
            factor *= 0.92
            notes.append("High-pitch props load the motors harder, so the starting P was reduced.")
        elif inputs.prop_pitch_in >= 4.6:
            factor *= 0.96
            notes.append("Moderately high prop pitch selected, so a small safety margin was applied.")

    start_p: dict[str, int] = {}
    p_sweep: dict[str, tuple[int, ...]] = {}
    for axis in TUNED_AXES:
        default = START_P_DEFAULTS[axis]
        value = max(15, min(default, int(round(default * factor))))
        start_p[axis] = value
        p_sweep[axis] = _p_sweep(value)

    yaw_final = dict(YAW_FINAL_DEFAULTS)
    yaw_final["p"] = max(30, min(YAW_FINAL_DEFAULTS["p"], int(round(YAW_FINAL_DEFAULTS["p"] * factor))))
    notes.append("Yaw is not being swept; the final yaw P was scaled conservatively from the same hardware estimate.")

    throttle_estimate = estimate_test_throttle(inputs)

    return PStartRecommendation(
        start_p=start_p,
        start_i=dict(START_I_DEFAULTS),
        p_sweep=p_sweep,
        yaw_final_pid_ff=yaw_final,
        notes=tuple(dict.fromkeys(notes)),
        inputs=inputs,
        throttle_estimate=throttle_estimate,
    )



def estimate_test_throttle(inputs: PStartInputs) -> TestThrottleEstimate:
    """Estimate a safe test-stand throttle from build data.

    This is intentionally a conservative starting estimate, not a final calibration.
    The estimate is referenced to the Pavo Pico II preset and should be verified on
    the actual stand before Level or Fly/Log is trusted.
    """

    notes: list[str] = []
    chemistry = _choice(inputs.battery_chemistry, BATTERY_CHEMISTRY_VALUES, REFERENCE_BATTERY_CHEMISTRY)
    motor_count = _bounded_int(inputs.motor_count, 1, 16, REFERENCE_MOTOR_COUNT)
    auw_g = _positive_float(inputs.all_up_weight_g, REFERENCE_AUW_G)
    motor_kv = _positive_float(inputs.motor_kv, REFERENCE_MOTOR_KV)
    prop_diameter = _positive_float(inputs.prop_diameter_in, REFERENCE_PROP_DIAMETER_IN)
    prop_pitch = _positive_float(inputs.prop_pitch_in, REFERENCE_PROP_PITCH_IN)
    cells = _positive_float(inputs.battery_cells, REFERENCE_BATTERY_CELLS)

    voltage = cells * BATTERY_NOMINAL_VOLTAGE[chemistry]
    reference_voltage = REFERENCE_BATTERY_CELLS * BATTERY_NOMINAL_VOLTAGE[REFERENCE_BATTERY_CHEMISTRY]

    speed_ratio = (motor_kv * voltage) / (REFERENCE_MOTOR_KV * reference_voltage)
    disk_ratio = (motor_count / REFERENCE_MOTOR_COUNT) * (prop_diameter / REFERENCE_PROP_DIAMETER_IN) ** 4
    pitch_ratio = (prop_pitch / REFERENCE_PROP_PITCH_IN) ** 0.5
    weight_ratio = auw_g / REFERENCE_AUW_G
    authority_index = max(0.25, min(4.0, (speed_ratio ** 2) * disk_ratio * pitch_ratio / max(0.25, weight_ratio)))

    level_delta = (REFERENCE_TEST_THROTTLE_US - 1000.0) / (authority_index ** 0.5)
    hover_delta = (REFERENCE_HOVER_THROTTLE_US - 1000.0) / (authority_index ** 0.5)
    level_test_throttle = _clamp_int(round(1000.0 + level_delta), MIN_LEVEL_TEST_THROTTLE_US, MAX_LEVEL_TEST_THROTTLE_US)
    hover_throttle = _clamp_int(round(1000.0 + hover_delta), MIN_HOVER_THROTTLE_US, MAX_HOVER_THROTTLE_US)

    if authority_index >= 1.35:
        notes.append("High estimated thrust authority; suggested test throttle was reduced.")
    elif authority_index <= 0.75:
        notes.append("Low estimated thrust authority; suggested test throttle was raised, but verify carefully.")
    else:
        notes.append("Estimated thrust authority is near the Pavo Pico II reference baseline.")
    notes.append("Use the level-test throttle as a starting point only; verify on the actual stand.")
    notes.append("For scale verification, start around 50-65% weight reduction, not full hover weight.")

    lift_low: int | None = None
    lift_mid: int | None = None
    lift_high: int | None = None
    if inputs.all_up_weight_g is not None:
        lift_low = int(round(float(inputs.all_up_weight_g) * 0.50))
        lift_mid = int(round(float(inputs.all_up_weight_g) * 0.60))
        lift_high = int(round(float(inputs.all_up_weight_g) * 0.70))

    return TestThrottleEstimate(
        level_test_throttle_us=level_test_throttle,
        hover_throttle_us=hover_throttle,
        lift_target_low_g=lift_low,
        lift_target_mid_g=lift_mid,
        lift_target_high_g=lift_high,
        notes=tuple(dict.fromkeys(notes)),
    )

def format_pid_tuning_plan(recommendation: PStartRecommendation) -> str:
    """Format the supervised P/D/I/FF plan as plain text."""

    start_p = recommendation.start_p
    start_i = recommendation.start_i
    p_sweep = recommendation.p_sweep
    yaw = recommendation.yaw_final_pid_ff
    throttle = recommendation.throttle_estimate
    start_d = D_SWEEP_VALUES[0]
    d_values = ", ".join(str(v) for v in D_SWEEP_VALUES)
    roll_p_values = ", ".join(str(v) for v in p_sweep["roll"])
    pitch_p_values = ", ".join(str(v) for v in p_sweep["pitch"])
    roll_i_values = ", ".join(str(row["roll"]) for row in I_SWEEP_VALUES)
    pitch_i_values = ", ".join(str(row["pitch"]) for row in I_SWEEP_VALUES)
    roll_ff_values = ", ".join(str(row["roll"]) for row in FF_SWEEP_VALUES)
    pitch_ff_values = ", ".join(str(row["pitch"]) for row in FF_SWEEP_VALUES)
    fly_log_action = (
        "-   Once the 'Fly/Log' button is pressed, wait for spin-up, set CH8 beeper marker ON, "
        "run the 6-group pitch/roll step routine, then set CH8 beeper marker OFF. "
        "Extract/Analyze reads the marked routine and ignores center-adjust pulses."
    )
    lines: list[str] = [
        "Supervised PID tuning plan",
        "",
        "Tuning order",
        "- Filters first.",
        "- Tune P first; P is the main authority that makes the axis respond.",
        "- Tune D after P; D damps the response once there is enough P to damp.",
        "- Tune I after P/D are close.",
        "- Leave FF/CD at 0 until the PID response is usable; revisit FF/CD last for stick feel only if needed.",
        "",
        "Safety gates",
        "- Keep battery fresh.",
        "- Verify test-stand throttle before relying on Level or Fly/Log.",
        f"- Active Level/FlyLog test throttle: {throttle.level_test_throttle_us} us. This is the value the program will use.",
        f"- Hover throttle estimate only: {throttle.hover_throttle_us} us. This is a reference number, not an automatic throttle change.",
        "- Do not raise the test throttle toward hover until the stand test has been verified safe.",
        "",
        "Throttle verification target",
        _format_throttle_lift_target(throttle),
        "",
        "Safe starting point / first P log",
        "- This first log uses the generated starting P values with starting D, starting I, and FF 0.",
        f"- Roll:  P {start_p['roll']}, D {start_d}, I {start_i['roll']}, FF 0",
        f"- Pitch: P {start_p['pitch']}, D {start_d}, I {start_i['pitch']}, FF 0",
        f"- Yaw:   P {yaw['p']}, D {yaw['d']:2d}, I 0, FF 0",
        "-   Confirm the drone is not armed.",
        "-   Stage/save the safe-start values if needed.",
        "-   Arm the drone and hit the 'Fly/Log' button.",
        fly_log_action,
        "-   Disarm the drone.",
        "",
        "P tuning, roll/pitch only",
        f"- Roll  P candidates: {roll_p_values}.",
        f"- Pitch P candidates: {pitch_p_values}.",
        f"- Keep D at the starting value ({start_d}), keep starting I, and keep FF 0.",
        "- The safe-start log already covers the generated starting P value; do not repeat that exact candidate unless you want a duplicate check.",
        "-   Confirm the drone is not armed.",
        "-   Set the Roll/Pitch 'P' value according to the candidate.",
        "-   Arm the drone and hit the 'Fly/Log' button.",
        fly_log_action,
        "-   Disarm the drone.",
        "-   Repeat with the next 'P' setting.",
        "- Once all the 'P' values have been logged, inspect the logs and select the correct 'P' term.",
        "- Pick the highest P that tracks well without ringing, oscillation, or ugly bounce-back.",
        "",
        "D tuning, roll/pitch only after P is selected",
        f"- Log D values: {d_values}",
        f"- Roll  D candidates: {d_values}",
        f"- Pitch D candidates: {d_values}",
        "- Keep the chosen P from the previous step.",
        "- Keep starting I and FF 0.",
        f"- The starting D ({start_d}) was already used during P selection; compare it against the higher D logs.",
        "-   Confirm the drone is not armed.",
        "-   Set the Roll/Pitch 'D' value according to the candidate.",
        "-   Arm the drone and hit the 'Fly/Log' button.",
        fly_log_action,
        "-   Disarm the drone.",
        "-   Repeat with the next 'D' setting.",
        "- Once all the 'D' values have been logged, inspect the logs and select the correct 'D' term.",
        "- Pick the lowest D that controls overshoot and bounce without adding motor noise or heat.",
        "",
        "- This will be the 'P' and 'D' values used throughout the rest of these tests.",
        "",
        "I tuning, roll/pitch only",
        f"- Roll:  I {roll_i_values}",
        f"- Pitch: I {pitch_i_values}",
        "- Keep chosen P and chosen D.",
        "- Keep FF 0.",
        "-   Confirm the drone is not armed.",
        "-   Set the Roll/Pitch 'I' value according to the candidate.",
        "-   Arm the drone and hit the 'Fly/Log' button.",
        fly_log_action,
        "-   Disarm the drone.",
        "-   Repeat with the next 'I' setting.",
        "- Once all the 'I' values have been logged, inspect the logs and select the correct 'I' term.",
        "- Pick the value that holds attitude without slow wobble or bounce-back.",
        "",
        "FF/CD tuning, last and optional",
        f"- Roll:  FF {roll_ff_values}",
        f"- Pitch: FF {pitch_ff_values}",
        "- Only use this section after P, D, and I are already usable.",
        "-   Confirm the drone is not armed.",
        "-   Set the Roll/Pitch 'FF' value according to the candidate.",
        "-   Arm the drone and hit the 'Fly/Log' button.",
        fly_log_action,
        "-   Disarm the drone.",
        "-   Repeat with the next 'FF' setting.",
        "- Once all the 'FF' values have been logged, inspect the logs and select the correct 'FF' term.",
        "- Pick FF where gyro starts with setpoint without jumping ahead.",
        "",
        "Yaw final recommendation, not tested",
        f"- Yaw P {yaw['p']}, I {yaw['i']}, D {yaw['d']}, FF {yaw['ff']}.",
        "- Treat yaw as a conservative baseline; revisit only if logs or flight feel show yaw-specific problems.",
        "",
        "Why this P start was chosen",
    ]
    lines.extend(f"- {note}" for note in recommendation.notes)
    return "\n".join(lines).strip()

def generate_pid_tuning_plan_report(
    output_root: str | Path,
    recommendation: PStartRecommendation,
) -> PIDTuningPlanReport:
    """Write a text and JSON copy of the current tuning plan."""

    root = Path(output_root).resolve()
    report_dir = _next_report_dir(root / "reports", "pid_tuning_plan")
    report_dir.mkdir(parents=True, exist_ok=True)
    text_path = report_dir / "pid_tuning_plan.txt"
    summary_json = report_dir / "pid_tuning_plan_summary.json"
    plan_text = format_pid_tuning_plan(recommendation)
    text_path.write_text(plan_text + "\n", encoding="utf-8")
    summary_json.write_text(_recommendation_json(recommendation, plan_text), encoding="utf-8")
    return PIDTuningPlanReport(
        report_dir=str(report_dir),
        text_path=str(text_path),
        summary_json=str(summary_json),
    )


def find_latest_pid_tuning_plan(output_root: str | Path) -> Path | None:
    """Return the newest generated PID tuning plan text file, if one exists."""

    reports_dir = Path(output_root).resolve() / "reports"
    if not reports_dir.exists():
        return None
    candidates = [
        path
        for path in reports_dir.glob("pid_tuning_plan_*/pid_tuning_plan.txt")
        if path.is_file()
    ]
    if not candidates:
        return None
    return max(candidates, key=lambda path: (path.stat().st_mtime, str(path)))


def load_pid_tuning_plan(plan_text_path: str | Path) -> LoadedPIDTuningPlan:
    """Load a generated PID tuning plan from text, preferring the adjacent JSON summary."""

    text_path = Path(plan_text_path).resolve()
    if not text_path.exists():
        raise FileNotFoundError(f"PID tuning plan not found: {text_path}")
    text = text_path.read_text(encoding="utf-8", errors="replace")
    summary_path = text_path.with_name("pid_tuning_plan_summary.json")
    if summary_path.exists():
        payload = json.loads(summary_path.read_text(encoding="utf-8"))
        workflow = payload.get("workflow", {})
        throttle_payload = payload.get("throttle_estimate", {})
        return LoadedPIDTuningPlan(
            text_path=str(text_path),
            summary_json=str(summary_path),
            text=text,
            start_p=_int_dict(payload.get("start_p", {}), ("roll", "pitch")),
            start_i=_int_dict(payload.get("start_i", START_I_DEFAULTS), ("roll", "pitch")),
            p_sweep=_tuple_map(payload.get("p_sweep", {}), ("roll", "pitch")),
            yaw_final_pid_ff=_int_dict(payload.get("yaw_final_pid_ff", {}), ("p", "i", "d", "ff")),
            d_sweep=tuple(int(value) for value in workflow.get("d_sweep", D_SWEEP_VALUES)),
            optional_d=_optional_int(workflow.get("optional_d", OPTIONAL_D_VALUE)),
            i_sweep=_pair_rows(workflow.get("i_sweep", I_SWEEP_VALUES)),
            ff_sweep=_pair_rows(workflow.get("ff_sweep", FF_SWEEP_VALUES)),
            level_test_throttle_us=_optional_int(throttle_payload.get("level_test_throttle_us")),
        )
    return _load_pid_tuning_plan_from_text(text_path, text)


def _recommendation_json(recommendation: PStartRecommendation, plan_text: str) -> str:
    payload = asdict(recommendation)
    payload["plan"] = {
        "format": "sample",
        "text": plan_text,
    }
    payload["workflow"] = {
        "tuned_axes": TUNED_AXES,
        "yaw_tested": False,
        "d_sweep": D_SWEEP_VALUES,
        "optional_d": OPTIONAL_D_VALUE,
        "i_sweep": I_SWEEP_VALUES,
        "ff_sweep": FF_SWEEP_VALUES,
    }
    return json.dumps(payload, indent=2)


def _load_pid_tuning_plan_from_text(text_path: Path, text: str) -> LoadedPIDTuningPlan:
    roll_start = _line_ints(text, r"-\s*Roll:\s*P\s*(\d+),\s*D\s*(\d+),\s*I\s*(\d+),\s*FF\s*(\d+)")
    pitch_start = _line_ints(text, r"-\s*Pitch:\s*P\s*(\d+),\s*D\s*(\d+),\s*I\s*(\d+),\s*FF\s*(\d+)")
    d_sweep = _line_ints(text, r"-\s*Log D values:\s*([0-9,\s]+)")
    roll_p = _line_ints(text, r"-\s*Roll\s+P candidates:\s*([0-9,\s]+)")
    pitch_p = _line_ints(text, r"-\s*Pitch\s+P candidates:\s*([0-9,\s]+)")
    yaw = _line_ints(text, r"-\s*Yaw P\s*(\d+),\s*I\s*(\d+),\s*D\s*(\d+),\s*FF\s*(\d+)")
    optional_match = re.search(r"Optional D\s*(\d+)", text, flags=re.IGNORECASE)
    level_throttle = _line_ints(text, r"Active Level/FlyLog test throttle:\s*(\d+)\s*us")
    if not level_throttle:
        level_throttle = _line_ints(text, r"Estimated Level-test throttle start:\s*(\d+)\s*us")

    i_rows = tuple(
        {"roll": int(match.group(1)), "pitch": int(match.group(2))}
        for match in re.finditer(r"-\s*Roll/Pitch I:\s*(\d+)\s*/\s*(\d+)", text, flags=re.IGNORECASE)
    )
    if not i_rows:
        roll_i = _line_ints(text, r"-\s*Roll:\s*I\s*([0-9,\s]+)")
        pitch_i = _line_ints(text, r"-\s*Pitch:\s*I\s*([0-9,\s]+)")
        i_rows = tuple(
            {"roll": int(roll), "pitch": int(pitch)}
            for roll, pitch in zip(roll_i, pitch_i)
        )

    ff_rows = tuple(
        {"roll": int(match.group(1)), "pitch": int(match.group(2))}
        for match in re.finditer(r"-\s*Roll/Pitch FF:\s*(\d+)\s*/\s*(\d+)", text, flags=re.IGNORECASE)
    )
    if not ff_rows:
        roll_ff = _line_ints(text, r"-\s*Roll:\s*FF\s*([0-9,\s]+)")
        pitch_ff = _line_ints(text, r"-\s*Pitch:\s*FF\s*([0-9,\s]+)")
        ff_rows = tuple(
            {"roll": int(roll), "pitch": int(pitch)}
            for roll, pitch in zip(roll_ff, pitch_ff)
        )

    return LoadedPIDTuningPlan(
        text_path=str(text_path),
        summary_json=None,
        text=text,
        start_p={
            "roll": roll_start[0] if roll_start else START_P_DEFAULTS["roll"],
            "pitch": pitch_start[0] if pitch_start else START_P_DEFAULTS["pitch"],
        },
        start_i={
            "roll": roll_start[2] if len(roll_start) >= 3 else START_I_DEFAULTS["roll"],
            "pitch": pitch_start[2] if len(pitch_start) >= 3 else START_I_DEFAULTS["pitch"],
        },
        p_sweep={
            "roll": tuple(roll_p) if roll_p else _p_sweep(START_P_DEFAULTS["roll"]),
            "pitch": tuple(pitch_p) if pitch_p else _p_sweep(START_P_DEFAULTS["pitch"]),
        },
        yaw_final_pid_ff={
            "p": yaw[0] if yaw else YAW_FINAL_DEFAULTS["p"],
            "i": yaw[1] if yaw else YAW_FINAL_DEFAULTS["i"],
            "d": yaw[2] if yaw else YAW_FINAL_DEFAULTS["d"],
            "ff": yaw[3] if yaw else YAW_FINAL_DEFAULTS["ff"],
        },
        d_sweep=tuple(d_sweep) if d_sweep else D_SWEEP_VALUES,
        optional_d=int(optional_match.group(1)) if optional_match else OPTIONAL_D_VALUE,
        i_sweep=i_rows if i_rows else I_SWEEP_VALUES,
        ff_sweep=ff_rows if ff_rows else FF_SWEEP_VALUES,
        level_test_throttle_us=level_throttle[0] if level_throttle else None,
    )


def _line_ints(text: str, pattern: str) -> tuple[int, ...]:
    match = re.search(pattern, text, flags=re.IGNORECASE)
    if not match:
        return ()
    return tuple(int(value) for value in re.findall(r"\d+", match.group(0)))


def _int_dict(payload: object, keys: tuple[str, ...]) -> dict[str, int]:
    if not isinstance(payload, dict):
        return {key: 0 for key in keys}
    return {key: int(payload[key]) for key in keys}


def _tuple_map(payload: object, keys: tuple[str, ...]) -> dict[str, tuple[int, ...]]:
    if not isinstance(payload, dict):
        return {key: () for key in keys}
    return {key: tuple(int(value) for value in payload.get(key, ())) for key in keys}


def _pair_rows(payload: object) -> tuple[dict[str, int], ...]:
    if not isinstance(payload, (list, tuple)):
        return ()
    rows: list[dict[str, int]] = []
    for row in payload:
        if not isinstance(row, dict):
            continue
        rows.append({"roll": int(row["roll"]), "pitch": int(row["pitch"])})
    return tuple(rows)


def _optional_int(value: object) -> int | None:
    if value is None:
        return None
    return int(value)


def _p_sweep(start: int) -> tuple[int, ...]:
    candidates = (start - 5, start, start + 5, start + 10)
    return tuple(dict.fromkeys(max(15, int(value)) for value in candidates))


def _choice(value: str, allowed: set[str], default: str) -> str:
    parsed = str(value or "").strip().lower()
    return parsed if parsed in allowed else default



def _format_throttle_lift_target(throttle: TestThrottleEstimate) -> str:
    if (
        throttle.lift_target_low_g is None
        or throttle.lift_target_mid_g is None
        or throttle.lift_target_high_g is None
    ):
        return "- Scale check: no AUW supplied, so use a small movement test and verify manually."
    return (
        f"- Scale check: aim for about {throttle.lift_target_low_g}-{throttle.lift_target_high_g} g "
        f"removed from the scale first; {throttle.lift_target_mid_g} g is the middle target."
    )


def _positive_float(value: int | float | None, default: float) -> float:
    try:
        parsed = float(value)
    except (TypeError, ValueError):
        return float(default)
    if parsed <= 0:
        return float(default)
    return parsed


def _clamp_int(value: int, minimum: int, maximum: int) -> int:
    return max(minimum, min(maximum, int(value)))

def _bounded_int(value: int | None, minimum: int, maximum: int, default: int) -> int:
    try:
        parsed = int(value)
    except (TypeError, ValueError):
        return default
    return max(minimum, min(maximum, parsed))


def _next_report_dir(parent: Path, prefix: str) -> Path:
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    base = parent / f"{prefix}_{timestamp}"
    if not base.exists():
        return base
    for index in range(2, 1000):
        candidate = parent / f"{prefix}_{timestamp}_{index:02d}"
        if not candidate.exists():
            return candidate
    raise RuntimeError("Could not allocate a PID tuning plan report folder.")
