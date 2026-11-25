"""Run the final CICS patient-transport lateral insertion/exit matrix.

The primary matrix compares B0/B1/B2/P/B3H under three total masses and
three base-CoM positions at every mass.
Every dynamic arm is three repeats and uses insertion-stop-extraction history.
"""

from cics_experiments.paths import workspace_root, resolve_path

import csv
import math
from pathlib import Path

import omni.timeline
import omni.usd
from pxr import Gf, Usd, UsdGeom, UsdPhysics


WORKSPACE = workspace_root()
SCRIPTS = resolve_path('isaac/scripts')
RESULTS = resolve_path('studies/CICS26/results/patient_transport_lab')
EXPECTED_USD = (
    resolve_path('isaac/assets/patient_transport_lab/patient_transport_lab.usd')
).resolve()
ROBOT_ROOT = "/World/patient_transport_lab_screening"
BASE = ROBOT_ROOT + "/base_link"
GROUND = "/World/PatientTransportLabGround/CollisionBox"
MATERIAL = "/World/PatientTransportLabGround/WheelGroundContact"
FIXTURE = "/World/CICS26LowClearanceFixture"
PILOT_LABELS = (
    "cics26_pt_265center_lateral_lowbed_frontpos_B2_v5",
    "cics26_pt_265center_lateral_lowbed_frontpos_B3H_v5",
    "cics26_pt_265center_lateral_lowbed_frontneg_B2_v5",
    "cics26_pt_265center_lateral_lowbed_frontneg_B3H_v5",
)

# These are provisional sensitivity conditions, not measured payload models.
CONDITIONS = (
    ("240rearx0p10", 240.0, (-0.10, 0.0, 0.270)),
    ("240center", 240.0, (0.0, 0.0, 0.270)),
    ("240frontx0p10", 240.0, (0.10, 0.0, 0.270)),
    ("265rearx0p10", 265.0, (-0.10, 0.0, 0.270)),
    ("265center", 265.0, (0.0, 0.0, 0.270)),
    ("265frontx0p10", 265.0, (0.10, 0.0, 0.270)),
    ("290rearx0p10", 290.0, (-0.10, 0.0, 0.270)),
    ("290center", 290.0, (0.0, 0.0, 0.270)),
    ("290frontx0p10", 290.0, (0.10, 0.0, 0.270)),
)
METHODS = (
    ("B0", "immediate", 0.05, False, False, False),
    ("B1", "constant_slow", 0.025, False, False, False),
    ("B2", "accel_ramp", 0.05, False, False, False),
    ("P", "accel_ramp", 0.05, False, True, True),
    ("B3H", "accel_ramp", 0.05, True, False, True),
)
INSERT_ALIGNED_DEG = -90.0
OFFSET_DEG = 2.359375
FRONT_POS = (
    INSERT_ALIGNED_DEG + OFFSET_DEG, INSERT_ALIGNED_DEG + OFFSET_DEG,
    INSERT_ALIGNED_DEG - OFFSET_DEG, INSERT_ALIGNED_DEG - OFFSET_DEG,
)
FRONT_NEG = (
    INSERT_ALIGNED_DEG - OFFSET_DEG, INSERT_ALIGNED_DEG - OFFSET_DEG,
    INSERT_ALIGNED_DEG + OFFSET_DEG, INSERT_ALIGNED_DEG + OFFSET_DEG,
)


def run_script(path, extra=None):
    namespace = {"__builtins__": __builtins__}
    if extra:
        namespace.update(extra)
    exec(compile(path.read_text(encoding="utf-8"), str(path), "exec"), namespace)


def prepare_common(stage):
    if not stage.GetPrimAtPath(GROUND).IsValid():
        run_script(SCRIPTS / "isaac_prepare_patient_transport_lab_static_contact.py")
        run_script(SCRIPTS / "isaac_prepare_patient_transport_lab_compliant_contact.py")
        run_script(SCRIPTS / "isaac_prepare_patient_transport_lab_powered_idle.py")
    if not stage.GetPrimAtPath(GROUND).IsValid() or not stage.GetPrimAtPath(MATERIAL).IsValid():
        raise RuntimeError("Floor/contact/powered-idle preparation failed")


def prepare_mass_com(stage, total_mass, target_com):
    target_base_mass = total_mass - 20.0
    mass_api = UsdPhysics.MassAPI(stage.GetPrimAtPath(BASE))
    mass = mass_api.GetMassAttr().Get()
    com = mass_api.GetCenterOfMassAttr().Get()
    inertia = mass_api.GetDiagonalInertiaAttr().Get()
    if mass is None or com is None or inertia is None:
        raise RuntimeError("Missing base mass, CoM, or inertia")
    if abs(float(mass) - 220.0) > 0.01 and abs(float(mass) - target_base_mass) > 0.01:
        raise RuntimeError(f"Expected fresh or prepared base mass, found {mass}")
    previous = stage.GetEditTarget()
    stage.SetEditTarget(stage.GetSessionLayer())
    try:
        if abs(float(mass) - target_base_mass) > 0.01:
            mass_api.CreateMassAttr().Set(target_base_mass)
            mass_api.CreateDiagonalInertiaAttr().Set(Gf.Vec3f(*(
                float(value) * target_base_mass / float(mass) for value in inertia
            )))
        mass_api.CreateCenterOfMassAttr().Set(Gf.Vec3f(*target_com))
    finally:
        stage.SetEditTarget(previous)
    masses = []
    for prim in Usd.PrimRange(stage.GetPrimAtPath(ROBOT_ROOT)):
        if prim.HasAPI(UsdPhysics.RigidBodyAPI):
            value = UsdPhysics.MassAPI(prim).GetMassAttr().Get()
            if value is None:
                raise RuntimeError(f"Missing rigid-body mass: {prim.GetPath()}")
            masses.append(float(value))
    if len(masses) != 13 or abs(sum(masses) - total_mass) > 0.01:
        raise RuntimeError(f"Expected {total_mass} kg, got {sum(masses)} kg")


def prepare_lateral_mode(stage):
    previous = stage.GetEditTarget()
    stage.SetEditTarget(stage.GetSessionLayer())
    try:
        for name in ("base_link_to_front_active_steering",
                     "base_link_to_rear_active_steering"):
            joint = UsdPhysics.RevoluteJoint(
                stage.GetPrimAtPath(f"{ROBOT_ROOT}/joints/{name}")
            )
            if not joint.GetPrim().IsValid():
                raise RuntimeError(f"Missing steering joint: {name}")
            joint.CreateLowerLimitAttr(-90.0).Set(-90.0)
            joint.CreateUpperLimitAttr(90.0).Set(90.0)
    finally:
        stage.SetEditTarget(previous)


def define_box(stage, path, center, size, color):
    cube = UsdGeom.Cube.Define(stage, path)
    cube.CreateSizeAttr(1.0)
    cube.AddTranslateOp().Set(Gf.Vec3d(*center))
    cube.AddScaleOp().Set(Gf.Vec3f(*size))
    cube.CreateDisplayColorAttr([Gf.Vec3f(*color)])
    UsdPhysics.CollisionAPI.Apply(cube.GetPrim()).CreateCollisionEnabledAttr(True).Set(True)


def prepare_fixture(stage):
    if stage.GetPrimAtPath(FIXTURE).IsValid():
        return
    origin = UsdGeom.Xformable(stage.GetPrimAtPath(BASE)).ComputeLocalToWorldTransform(
        Usd.TimeCode.Default()
    ).ExtractTranslation()
    center_x, center_y = float(origin[0]), float(origin[1]) + 0.70
    previous = stage.GetEditTarget()
    stage.SetEditTarget(stage.GetSessionLayer())
    try:
        UsdGeom.Xform.Define(stage, FIXTURE)
        for longitudinal, y_sign in (("Near", -1.0), ("Far", 1.0)):
            for lateral, x_sign in (("Front", 1.0), ("Rear", -1.0)):
                define_box(stage, f"{FIXTURE}/{longitudinal}{lateral}Leg",
                           (center_x + x_sign * 0.47, center_y + y_sign * 0.67, 0.04),
                           (0.06, 0.06, 0.08), (0.38, 0.55, 0.78))
        define_box(stage, FIXTURE + "/LowSlab", (center_x, center_y, 0.10),
                   (1.00, 1.40, 0.04), (0.55, 0.68, 0.86))
        define_box(stage, FIXTURE + "/LowPad", (center_x, center_y, 0.135),
                   (0.94, 1.74, 0.03), (0.72, 0.82, 0.92))
    finally:
        stage.SetEditTarget(previous)


def static_passed(path, total_mass):
    with path.open(newline="", encoding="utf-8") as stream:
        load = {row["role"]: float(row["mean_abs_Fz_N"]) for row in csv.DictReader(stream)}
    wheels = ("active_front", "active_rear", "caster_fl", "caster_fr",
              "caster_rl", "caster_rr")
    return (set(wheels).issubset(load)
            and all(load[role] > 1.0 for role in wheels)
            and 0.85 <= sum(load.values()) / (total_mass * 9.81) <= 1.15)


arms = []
for condition, mass, com in CONDITIONS:
    for method, profile, speed, pulse, adaptive, hold in METHODS:
        label = f"cics26_pt_{condition}_lateral_lowbed_frontpos_{method}_v5"
        # Preserve the already completed, protocol-matched 265-center B2 arm.
        if condition == "265center" and method == "B2":
            label = "cics26_pt_265center_lateral_lowbed_frontpos_B2_v5"
        arms.append((condition, mass, com, "front_pos", FRONT_POS,
                     method, profile, speed, pulse, adaptive, hold, label))


timeline = omni.timeline.get_timeline_interface()
if not timeline.is_stopped():
    raise RuntimeError("Press Timeline Stop before running this file")
missing_pilot = [
    label for label in PILOT_LABELS
    if not (RESULTS / "natural_flip_pilot" / label / "summary.csv").exists()
]
if missing_pilot:
    raise RuntimeError(
        "Complete and assess the B2/B3H low-clearance pilot before the full matrix. "
        "Run isaac_patient_transport_cics26_b3h_lowclearance_next.py; missing: "
        + ", ".join(missing_pilot)
    )
stage = omni.usd.get_context().get_stage()
if stage is None or Path(stage.GetRootLayer().realPath).resolve() != EXPECTED_USD:
    raise RuntimeError("Open a fresh patient_transport_lab.usd before running")

target = None
done = 0
for arm in arms:
    output = RESULTS / "natural_flip_pilot" / arm[-1]
    if (output / "summary.csv").exists():
        if not all((output / f"trace_r{repeat:02d}.csv").exists()
                   and (output / f"insertion_trace_r{repeat:02d}.csv").exists()
                   for repeat in (1, 2, 3)):
            raise RuntimeError(f"Incomplete final traces: {output}")
        done += 1
        continue
    if output.exists():
        raise RuntimeError(f"Incomplete result exists; inspect first: {output}")
    target = arm
    break

if target is None:
    print("PATIENT-TRANSPORT CICS LATERAL FULL MATRIX complete.")
    print("Run compare_patient_transport_cics26_lateral_fullmatrix.py.")
else:
    (condition, mass, com, state, caster_start, method, profile, speed,
     pulse, adaptive, hold, label) = target
    static_label = f"cics26_pt_{condition}_lateral_lowbed_static_v5"
    if condition == "265center":
        static_label = "cics26_pt_265center_b3h_lateral_lowbed_static_v5"
    static_csv = RESULTS / "static_contact_gate" / f"{static_label}.csv"
    print(f"PATIENT-TRANSPORT CICS MATRIX {done + 1}/{len(arms)}: "
          f"{condition} {state} {method}")
    prepare_common(stage)
    prepare_mass_com(stage, mass, com)
    prepare_lateral_mode(stage)
    prepare_fixture(stage)
    if not static_csv.exists():
        print(f"Scheduling static gate: {condition}, total={mass} kg, base CoM={com}")
        run_script(SCRIPTS / "isaac_patient_transport_lab_static_contact_gate.py",
                   {"patient_transport_lab_contact_label": static_label})
        print("After PASSED: Stop, rerun THIS file on the same USD.")
    else:
        if not static_passed(static_csv, mass):
            raise RuntimeError(f"Static contact gate failed: {static_csv}")
        run_script(SCRIPTS / "isaac_patient_transport_lab_lateral_repeat_gate.py", {
            "patient_lab_lateral_run_label": label,
            "patient_lab_lateral_method": method,
            "patient_lab_lateral_expected_mass_kg": mass,
            "patient_lab_lateral_expected_base_com_m": com,
            "patient_lab_lateral_controlled_start_deg": caster_start,
            "patient_lab_lateral_repeats": 3,
            "patient_lab_lateral_target_m": 0.70,
            "patient_lab_lateral_insert_target_m": 0.70,
            "patient_lab_lateral_profile": profile,
            "patient_lab_lateral_speed_m_s": speed,
            "patient_lab_lateral_accel_m_s2": 0.025,
            "patient_lab_lateral_active_pulse_enabled": pulse,
            "patient_lab_lateral_adaptive_pulse_enabled": adaptive,
            "patient_lab_lateral_yaw_hold_enabled": hold,
            "patient_lab_lateral_pulse_rate_rad_s": 0.03203125,
            "patient_lab_lateral_pulse_duration_s": 0.5,
            "patient_lab_lateral_adaptive_start_rad_s": 0.0080078125,
            "patient_lab_lateral_adaptive_step_rad_s": 0.0080078125,
            "patient_lab_lateral_adaptive_step_interval_s": 0.5,
            "patient_lab_lateral_adaptive_max_rad_s": 0.0640625,
            "patient_lab_lateral_adaptive_max_duration_s": 2.5,
            "patient_lab_lateral_response_yaw_deg": 0.0002,
            "patient_lab_lateral_response_yaw_rate_deg_s": 0.001,
            "patient_lab_lateral_response_dwell_s": 0.10,
            "patient_lab_lateral_yaw_hold_kp": 10.0,
            "patient_lab_lateral_yaw_hold_max_rad_s": 0.12,
        })
        print("After summary: Stop, reopen fresh USD, rerun THIS file.")
        print("Primary CICS metrics: lateral, yaw, jerk, time, repeat spread.")
