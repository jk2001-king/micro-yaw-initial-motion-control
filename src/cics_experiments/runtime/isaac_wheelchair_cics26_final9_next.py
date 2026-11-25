"""Run missing arms of the final seven-condition wheelchair comparison.

Final methods are B0 immediate, B1 fixed slow, B2 acceleration ramp,
P response-stepped initial yaw, and proposed B3H fixed micro-yaw plus body-yaw
hold. The 40 kg empty wheelchair uses only its native center CoM. The occupied
90 and 127.5 kg conditions each use rear/center/front occupant CoM positions.
Existing protocol-matched B2/P/B3H results are reused by exact label.
"""

from cics_experiments.paths import workspace_root, resolve_path

import csv
from pathlib import Path

import omni.timeline
import omni.usd
from pxr import Sdf, Usd, UsdPhysics


ROOT = workspace_root()
SCRIPTS = resolve_path('isaac/scripts')
RESULTS = resolve_path('studies/CICS26/results/wheelchair_lab')
FLIP_ROOT = RESULTS / "natural_flip_gate"
STATIC_ROOT = RESULTS / "load_share_gate"
BASE_PATH = "/World/wheelchair_lab_screening/base_link"
JOINT_ROOT = "/World/wheelchair_lab_screening/joints"

CONDITIONS = (
    ("40p0_center", 40.0, 0.0, 0.120, "cics26_empty0_center", 0.29411960784313723),
    ("90p0_rearx0p07", 90.0, 50.0, 0.070, "cics26_occupied50_rearx0p07", 0.20697254901960782),
    ("90p0_center", 90.0, 50.0, 0.120, "cics26_occupied50_center", 0.261438779956427),
    ("90p0_frontx0p17", 90.0, 50.0, 0.170, "cics26_occupied50_frontx0p17", 0.3159050108932462),
    ("127p5_rearx0p07", 127.5, 87.5, 0.070, "cics26_occupied87p5_rearx0p07", 0.18646735870818915),
    ("127p5_center", 127.5, 87.5, 0.120, "cics26_occupied87p5_center", 0.2537491733948481),
    ("127p5_frontx0p17", 127.5, 87.5, 0.170, "cics26_occupied87p5_frontx0p17", 0.3210309880815071),
)
METHODS = ("B0", "B1", "B2", "P", "B3H")

REUSED = {
    ("40p0_center", "B2"): "cics26_caster_swivel_kd0030000_B2_40p0_center_d090",
    ("40p0_center", "P"): "cics26_p10_lowdamping_d090_v1_40p0_center",
    ("40p0_center", "B3H"): "cics26_kd003_d090_B3H_40p0_center",
    ("90p0_center", "B2"): "cics26_caster_swivel_kd0030000_B2_90p0_center_d090",
    ("90p0_center", "P"): "cics26_p11_90kg_heldout_d090_v1",
    ("127p5_center", "B2"): "cics26_caster_swivel_kd0030000_B2_127p5_center_d090",
    ("127p5_center", "P"): "cics26_p12_127p5kg_heldout_d090_v1",
    ("127p5_center", "B3H"): "cics26_kd003_d090_B3H_127p5_center",
    ("127p5_rearx0p07", "B2"): "cics26_kd003_d090_B2_127p5_rearx0p07",
    ("127p5_rearx0p07", "P"): "cics26_kd003_d090_P_127p5_rearx0p07",
    ("127p5_frontx0p17", "B2"): "cics26_kd003_d090_B2_127p5_frontx0p17",
    ("127p5_frontx0p17", "P"): "cics26_kd003_d090_P_127p5_frontx0p17",
    ("127p5_frontx0p17", "B3H"): "cics26_kd003_d090_B3H_127p5_frontx0p17",
}


def result_label(condition, method):
    return REUSED.get(
        (condition, method), f"cics26_final9_kd003_d090_{method}_{condition}"
    )


def read_rows(path):
    with path.open(newline="", encoding="utf-8") as stream:
        return list(csv.DictReader(stream))


def run_script(name, extra=None):
    namespace = {"__builtins__": __builtins__}
    if extra:
        namespace.update(extra)
    path = SCRIPTS / name
    exec(compile(path.read_text(encoding="utf-8"), str(path), "exec"), namespace)


def prepare_condition(stage, condition):
    condition_id, total_mass, occupant_mass, occupant_x, load_label, target_share = condition
    common = {
        "wheelchair_lab_occupant_mass_kg": occupant_mass,
        "wheelchair_lab_occupant_condition_label": load_label,
        "wheelchair_lab_occupant_com_m": (occupant_x, 0.0, 0.500),
    }
    run_script("isaac_prepare_wheelchair_lab_ground_support.py", common)
    run_script("isaac_prepare_wheelchair_lab_occupant_load.py", common)
    run_script("isaac_prepare_wheelchair_lab_contact_material.py", common)
    run_script("isaac_prepare_wheelchair_lab_compliant_contact.py", common)
    base = stage.GetPrimAtPath(BASE_PATH)
    share_attr = base.GetAttribute("experiment:targetCasterLoadShare")
    label_attr = base.GetAttribute("experiment:loadShareCondition")
    mass_attr = base.GetAttribute("experiment:totalMassKg")
    if (not share_attr.IsValid() or share_attr.Get() is None
            or abs(float(share_attr.Get()) - target_share) > 2e-6):
        raise RuntimeError(f"Target caster share mismatch for {condition_id}")
    if not label_attr.IsValid() or str(label_attr.Get()) != load_label:
        raise RuntimeError(f"Load label mismatch for {condition_id}")
    if not mass_attr.IsValid() or abs(float(mass_attr.Get()) - total_mass) > 1e-6:
        raise RuntimeError(f"Total mass mismatch for {condition_id}")
    with Usd.EditContext(stage, stage.GetSessionLayer()):
        base.CreateAttribute(
            "experiment:cics26Final9Condition", Sdf.ValueTypeNames.String
        ).Set(condition_id)


def validate_static(path, total_mass, target_share):
    forces = {row["role"]: float(row["mean_abs_Fz_N"]) for row in read_rows(path)}
    weight = total_mass * 9.81
    caster = forces["caster_left"] + forces["caster_right"]
    drive = forces["drive_left"] + forces["drive_right"]
    other = sum(forces[key] for key in ("base", "fork_left", "fork_right"))
    imbalance = abs(
        forces["caster_left"] + forces["drive_left"]
        - forces["caster_right"] - forces["drive_right"]
    ) / weight
    if not (0.85 <= (caster + drive + other) / weight <= 1.15
            and abs(caster / weight - target_share) <= 0.05
            and other / weight <= 0.02 and imbalance <= 0.10):
        raise RuntimeError(f"Static load gate failed: {path}")


def set_caster_damping(stage):
    for side in ("left", "right"):
        prim = stage.GetPrimAtPath(f"{JOINT_ROOT}/base_link_to_{side}_caster")
        drive = UsdPhysics.DriveAPI.Get(prim, "angular") if prim.IsValid() else None
        if not drive:
            raise RuntimeError(f"Missing caster angular drive: {side}")
        stiffness = float(drive.GetStiffnessAttr().Get() or 0.0)
        damping = float(drive.GetDampingAttr().Get() or 0.0)
        if abs(stiffness) > 1e-6 or not (
            abs(damping - 0.060287) <= 0.002 or abs(damping - 0.03) <= 1e-6
        ):
            raise RuntimeError(f"Unexpected caster gains for {side}: {stiffness}, {damping}")
    with Usd.EditContext(stage, stage.GetSessionLayer()):
        for side in ("left", "right"):
            prim = stage.GetPrimAtPath(f"{JOINT_ROOT}/base_link_to_{side}_caster")
            UsdPhysics.DriveAPI.Get(prim, "angular").CreateDampingAttr(0.03).Set(0.03)


def dynamic_config(label, method, load_label):
    profile = "immediate" if method == "B0" else (
        "constant_slow" if method == "B1" else "accel_ramp"
    )
    speed = 0.025 if method == "B1" else 0.05
    adaptive = method == "P"
    fixed = method == "B3H"
    return {
        "wheelchair_lab_flip_run_label": label,
        "wheelchair_lab_flip_repeats": 3,
        "wheelchair_lab_flip_warmup_repeats": 0,
        "wheelchair_lab_flip_forward_speed_m_s": 0.10,
        "wheelchair_lab_flip_forward_distance_m": 1.20,
        "wheelchair_lab_flip_forward_max_distance_m": 1.50,
        "wheelchair_lab_flip_forward_timeout_s": 40.0,
        "wheelchair_lab_flip_preparation_seed_deg": 0.50,
        "wheelchair_lab_flip_stop_settle_s": 1.50,
        "wheelchair_lab_flip_controlled_reverse_start_deg": (179.5, -179.5),
        "wheelchair_lab_flip_controlled_start_settle_s": 0.50,
        "wheelchair_lab_flip_reverse_profile": profile,
        "wheelchair_lab_flip_reverse_speed_m_s": speed,
        # The shared gate parses this field as float even when the immediate
        # and constant-speed profiles do not apply the acceleration limiter.
        "wheelchair_lab_flip_reverse_accel_limit_m_s2": 0.025,
        "wheelchair_lab_flip_reverse_body_speed_kp": 1.0,
        "wheelchair_lab_flip_reverse_max_command_speed_m_s": 0.08,
        "wheelchair_lab_flip_reverse_distance_m": 0.90,
        "wheelchair_lab_flip_reverse_timeout_s": 90.0 if method == "B1" else 70.0,
        "wheelchair_lab_flip_wall_timeout_s": 900.0,
        "wheelchair_lab_flip_initial_yaw_pulse_rate_rad_s": 0.03203125 if fixed else 0.0,
        "wheelchair_lab_flip_initial_yaw_pulse_duration_s": 0.50 if fixed else 0.0,
        "wheelchair_lab_flip_initial_yaw_hold_translation": False,
        "wheelchair_lab_flip_boost_yaw_rate_rad_s": 0.0,
        "wheelchair_lab_flip_boost_yaw_start_s": 0.0,
        "wheelchair_lab_flip_boost_yaw_duration_s": 0.0,
        "wheelchair_lab_flip_adaptive_initial_enabled": adaptive,
        "wheelchair_lab_flip_adaptive_initial_sign": 1,
        "wheelchair_lab_flip_adaptive_initial_rate_rad_s": 0.010,
        "wheelchair_lab_flip_adaptive_rate_step_rad_s": 0.005,
        "wheelchair_lab_flip_adaptive_step_period_s": 0.50,
        "wheelchair_lab_flip_adaptive_max_rate_rad_s": 0.080,
        "wheelchair_lab_flip_adaptive_max_duration_s": 8.0,
        "wheelchair_lab_flip_adaptive_response_mode": "caster_any_opposed",
        "wheelchair_lab_flip_adaptive_target_direction_mode": "opposed_inward",
        "wheelchair_lab_flip_adaptive_response_yaw_deg": 0.05,
        "wheelchair_lab_flip_adaptive_response_rate_deg_s": 0.10,
        "wheelchair_lab_flip_adaptive_response_dwell_s": 0.10,
        "wheelchair_lab_flip_adaptive_response_window_s": 0.20,
        "wheelchair_lab_flip_adaptive_selection_speed_m_s": None,
        "wheelchair_lab_flip_adaptive_ramp_after_response": False,
        "wheelchair_lab_flip_adaptive_reverse_on_wrong_direction": False,
        "wheelchair_lab_flip_adaptive_reverse_after_s": 4.0 if adaptive else None,
        "wheelchair_lab_flip_direction_confirm_min_rotation_deg": 3.0,
        "wheelchair_lab_flip_direction_confirm_min_speed_deg_s": 1.0,
        "wheelchair_lab_flip_direction_confirm_window_s": 0.20,
        "wheelchair_lab_flip_direction_confirm_dwell_s": 0.10,
        "wheelchair_lab_flip_counter_yaw_rate_rad_s": 0.0,
        "wheelchair_lab_flip_counter_yaw_start_s": 0.0,
        "wheelchair_lab_flip_counter_yaw_duration_s": 0.0,
        "wheelchair_lab_flip_counter_trigger_abs_yaw_deg": None,
        "wheelchair_lab_flip_counter_trigger_min_time_s": 0.0,
        "wheelchair_lab_flip_counter_require_adaptive_response": False,
        "wheelchair_lab_flip_counter_require_direction_confirmed": False,
        "wheelchair_lab_flip_counter_use_measured_yaw_sign": False,
        "wheelchair_lab_flip_counter_stop_on_yaw_recovery": False,
        "wheelchair_lab_flip_counter_stop_abs_yaw_deg": 0.05,
        "wheelchair_lab_flip_counter_rearm_enabled": False,
        "wheelchair_lab_flip_yaw_hold_enabled": method in ("P", "B3H"),
        "wheelchair_lab_flip_yaw_hold_require_adaptive_response": adaptive,
        "wheelchair_lab_flip_yaw_hold_require_flip_complete": False,
        "wheelchair_lab_flip_yaw_hold_start_s": 0.50 if fixed else 0.0,
        "wheelchair_lab_flip_yaw_hold_trigger_abs_deg": 0.15,
        "wheelchair_lab_flip_yaw_hold_trigger_dwell_s": 0.05,
        "wheelchair_lab_flip_yaw_hold_stop_abs_deg": 0.05,
        "wheelchair_lab_flip_yaw_hold_kp_rad_s_per_rad": 10.0,
        "wheelchair_lab_flip_yaw_hold_max_rate_rad_s": 0.12,
    }


timeline = omni.timeline.get_timeline_interface()
if not timeline.is_stopped():
    raise RuntimeError("Press Stop before running the final wheelchair matrix")
stage = omni.usd.get_context().get_stage()
base = stage.GetPrimAtPath(BASE_PATH) if stage else None
if base is None or not base.IsValid():
    raise RuntimeError("Open a fresh wheelchair_lab.usd")

arms = [(condition, method) for condition in CONDITIONS for method in METHODS]
pending = []
completed = 0
new_completed = 0
for condition, method in arms:
    label = result_label(condition[0], method)
    output = FLIP_ROOT / label
    if (output / "summary.csv").exists():
        if not all((output / f"natural_flip_r{repeat:02d}.csv").exists()
                   for repeat in (1, 2, 3)):
            raise RuntimeError(f"Incomplete traces: {output}")
        completed += 1
        if (condition[0], method) not in REUSED:
            new_completed += 1
        continue
    if output.exists() and any(output.iterdir()):
        raise RuntimeError(f"Incomplete output exists: {output}")
    pending.append((condition, method, label))

if not pending:
    print("WHEELCHAIR FINAL SEVEN-CONDITION MATRIX COMPLETE: 35/35 arms.")
    print("Run compare_wheelchair_cics26_final9.py in the terminal.")
else:
    condition, method, label = pending[0]
    condition_id, total_mass, _, _, load_label, target_share = condition
    planned_new = len(arms) - len(REUSED)
    print(
        f"WHEELCHAIR REMAINING ARM {new_completed + 1}/{planned_new} "
        f"(overall {completed + 1}/{len(arms)}): {condition_id} {method}"
    )
    marker = base.GetAttribute("experiment:cics26Final9Condition")
    prepared = str(marker.Get()) if marker.IsValid() and marker.Get() is not None else None
    if prepared is None:
        prepare_condition(stage, condition)
    elif prepared != condition_id:
        raise RuntimeError(f"Prepared for {prepared}; reopen a fresh USD for {condition_id}")
    static_label = f"cics26_final9_{condition_id}_static"
    static_csv = STATIC_ROOT / f"{static_label}.csv"
    if not static_csv.exists():
        print(f"Scheduling shared static gate: {condition_id}")
        run_script("isaac_wheelchair_lab_static_contact_load_gate.py", {
            "wheelchair_lab_load_share_label": static_label,
            "wheelchair_lab_target_caster_share": target_share,
        })
        print("After PASS: Stop, rerun THIS file on the same USD.")
    else:
        validate_static(static_csv, total_mass, target_share)
        set_caster_damping(stage)
        run_script("isaac_wheelchair_lab_natural_flip_gate.py",
                   dynamic_config(label, method, load_label))
        print("After summary: Stop, reopen fresh USD, rerun THIS file.")
        print("Final paper metrics: lateral, yaw, jerk, time, repeat spread.")
