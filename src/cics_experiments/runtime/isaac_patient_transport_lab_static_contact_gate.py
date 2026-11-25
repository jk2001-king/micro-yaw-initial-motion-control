"""Numerically check six-wheel ground support in the laboratory platform USD.

Run after isaac_prepare_patient_transport_lab_static_contact.py with Timeline
stopped. The script starts/pauses the Timeline itself and saves a CSV. It does
not change the USD root layer. The 240 kg mass remains a provisional model
assumption, not a measurement of the laboratory robot.
"""

from cics_experiments.paths import workspace_root, resolve_path

import asyncio
import csv
import math
import time
from pathlib import Path

import numpy as np
import omni.kit.app
import omni.physx
import omni.timeline
import omni.usd
from isaacsim.core.api.sensors import RigidContactView
from pxr import PhysicsSchemaTools, PhysxSchema, Usd, UsdPhysics, UsdUtils


WORKSPACE = workspace_root()
CONDITION_LABEL = str(
    globals().get("patient_transport_lab_contact_label", "baseline")
)
if not CONDITION_LABEL or not all(
    char.isascii() and (char.isalnum() or char in "_-")
    for char in CONDITION_LABEL
):
    raise ValueError("Contact gate label must contain only ASCII letters, digits, _ or -")
OUTPUT = (
    resolve_path('studies/CICS26/results/patient_transport_lab/static_contact_gate')
    / f"{CONDITION_LABEL}.csv"
)
ROBOT_ROOT = "/World/patient_transport_lab_screening"
ARTICULATION_ROOT = ROBOT_ROOT + "/base_link"
GROUND = "/World/PatientTransportLabGround/CollisionBox"
PHYSICS_DT_S = 1.0 / 120.0
SETTLE_S = 1.0
SAMPLE_S = 2.0
MIN_WHEEL_LOAD_N = 1.0
MIN_MEAN_WHEEL_WEIGHT_SHARE = 0.005
MIN_LOAD_BEARING_DUTY = 0.95
MAX_TOTAL_FORCE_CV = 0.15
MAX_CASTER_SHARE_STD = 0.05
MAX_BASE_Z_SPAN_M = 0.005
MAX_BASE_TILT_DEG = 1.0
MAX_BASE_TILT_SPAN_DEG = 1.0

WHEELS = {
    "active_front": "front_active_drive_wheel_link",
    "active_rear": "rear_active_drive_wheel_link",
    "caster_fl": "front_left_caster_wheel_link",
    "caster_fr": "front_right_caster_wheel_link",
    "caster_rl": "rear_left_caster_wheel_link",
    "caster_rr": "rear_right_caster_wheel_link",
}
BODY_PATHS = {role: ROBOT_ROOT + "/" + name for role, name in WHEELS.items()}
BODY_PATHS["base"] = ARTICULATION_ROOT
for corner in ("front_left", "front_right", "rear_left", "rear_right"):
    BODY_PATHS[f"fork_{corner}"] = ROBOT_ROOT + f"/{corner}_caster_link"
for end in ("front", "rear"):
    BODY_PATHS[f"steering_{end}"] = ROBOT_ROOT + f"/{end}_active_steering_link"

timeline = omni.timeline.get_timeline_interface()
if not timeline.is_stopped():
    raise RuntimeError("Press Stop before running the static contact gate")
stage = omni.usd.get_context().get_stage()
if stage is None:
    raise RuntimeError("No USD stage is open")
for path in (GROUND, *BODY_PATHS.values()):
    if not stage.GetPrimAtPath(path).IsValid():
        raise RuntimeError(f"Missing required prim: {path}")

mass_rows = []
for prim in Usd.PrimRange(stage.GetPrimAtPath(ROBOT_ROOT)):
    if prim.HasAPI(UsdPhysics.RigidBodyAPI):
        value = UsdPhysics.MassAPI(prim).GetMassAttr().Get()
        if value is None or float(value) <= 0.0:
            raise RuntimeError(f"Missing positive authored mass: {prim.GetPath()}")
        mass_rows.append(float(value))
if len(mass_rows) != 13:
    raise RuntimeError(f"Expected 13 massive rigid bodies, found {len(mass_rows)}")
EXPECTED_MASS_KG = sum(mass_rows)
EXPECTED_WEIGHT_N = EXPECTED_MASS_KG * 9.81

previous_target = stage.GetEditTarget()
stage.SetEditTarget(stage.GetSessionLayer())
try:
    for path in (GROUND, *BODY_PATHS.values()):
        PhysxSchema.PhysxContactReportAPI.Apply(
            stage.GetPrimAtPath(path)
        ).CreateThresholdAttr(0.0).Set(0.0)
finally:
    stage.SetEditTarget(previous_target)

roles = list(BODY_PATHS)
views = [
    RigidContactView(
        prim_paths_expr=BODY_PATHS[role],
        filter_paths_expr=[],
        name=f"patient_transport_lab_contact_{role}",
        prepare_contact_sensors=False,
        disable_stablization=False,
    )
    for role in roles
]
physics_time_s = 0.0


def _physics_step(dt):
    global physics_time_s
    physics_time_s += float(dt)


physics_subscription = omni.physx.get_physx_interface().subscribe_physics_step_events(
    _physics_step
)


def _base_pose_sample():
    transform = omni.physx.get_physx_interface().get_rigidbody_transformation(
        ARTICULATION_ROOT
    )
    if not transform.get("ret_val", False):
        raise RuntimeError("Could not read the live PhysX base pose")
    position = transform["position"]
    rotation = transform["rotation"]
    x, y, z, w = (float(rotation[index]) for index in range(4))
    roll = math.degrees(math.atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x*x + y*y)))
    pitch_arg = max(-1.0, min(1.0, 2.0 * (w * y - z * x)))
    pitch = math.degrees(math.asin(pitch_arg))
    return float(position[2]), roll, pitch


async def _run():
    timeline.play()
    app = omni.kit.app.get_app()
    for _ in range(5):
        await app.next_update_async()
    if not timeline.is_playing():
        raise RuntimeError("Timeline did not start")
    for view in views:
        view.initialize()

    stage_id = UsdUtils.StageCache.Get().GetId(stage).ToLongInt()
    encoded_root = PhysicsSchemaTools.sdfPathToInt(ARTICULATION_ROOT)
    simulation = omni.physx.get_physx_simulation_interface()
    wall_start = time.perf_counter()
    settle_start = physics_time_s
    while physics_time_s - settle_start < SETTLE_S:
        simulation.wake_up(stage_id, encoded_root)
        await app.next_update_async()
        if not timeline.is_playing():
            raise RuntimeError("Timeline stopped while settling")
        if time.perf_counter() - wall_start > 20.0:
            raise RuntimeError("Timeout while settling")

    samples = []
    pose_samples = []
    sample_times = []
    sample_start = physics_time_s
    while physics_time_s - sample_start < SAMPLE_S:
        simulation.wake_up(stage_id, encoded_root)
        await app.next_update_async()
        if not timeline.is_playing():
            raise RuntimeError("Timeline stopped while sampling")
        samples.append(
            np.stack(
                [
                    np.asarray(view.get_net_contact_forces(dt=PHYSICS_DT_S), dtype=float)
                    .reshape(-1, 3)[0]
                    for view in views
                ],
                axis=0,
            )
        )
        pose_samples.append(_base_pose_sample())
        sample_times.append(physics_time_s - sample_start)
        if time.perf_counter() - wall_start > 30.0:
            raise RuntimeError("Timeout while sampling")

    timeline.pause()
    data = np.asarray(samples, dtype=float)
    pose_data = np.asarray(pose_samples, dtype=float)
    if not np.all(np.isfinite(data)) or not np.all(np.isfinite(pose_data)):
        raise RuntimeError("Non-finite contact force or base pose during sampling")
    rows = []
    print("=== Laboratory patient-transport six-wheel static contact gate ===")
    print("condition:", CONDITION_LABEL)
    print("role, mean_Fx_N, mean_Fy_N, mean_Fz_N, mean_abs_Fz_N")
    for index, role in enumerate(roles):
        mean_force = np.mean(data[:, index, :], axis=0)
        row = {
            "role": role,
            "prim_path": BODY_PATHS[role],
            "mean_Fx_N": float(mean_force[0]),
            "mean_Fy_N": float(mean_force[1]),
            "mean_Fz_N": float(mean_force[2]),
            "mean_abs_Fz_N": float(np.mean(np.abs(data[:, index, 2]))),
            "max_abs_Fz_N": float(np.max(np.abs(data[:, index, 2]))),
            "load_bearing_duty_above_1N": float(
                np.mean(np.abs(data[:, index, 2]) > MIN_WHEEL_LOAD_N)
            ),
            "samples": int(len(samples)),
        }
        rows.append(row)
        print(
            f"{role}, {row['mean_Fx_N']:+.3f}, {row['mean_Fy_N']:+.3f}, "
            f"{row['mean_Fz_N']:+.3f}, {row['mean_abs_Fz_N']:.3f}"
        )

    load = {row["role"]: row["mean_abs_Fz_N"] for row in rows}
    caster_load = sum(load[role] for role in WHEELS if role.startswith("caster_"))
    active_load = load["active_front"] + load["active_rear"]
    unintended_load = sum(load[role] for role in roles if role not in WHEELS)
    tracked_load = caster_load + active_load + unintended_load
    tracked_ratio = tracked_load / EXPECTED_WEIGHT_N
    caster_share = caster_load / tracked_load if tracked_load > 0.0 else float("nan")
    min_mean_wheel_load = EXPECTED_WEIGHT_N * MIN_MEAN_WHEEL_WEIGHT_SHARE
    six_wheels_loaded = all(load[role] > min_mean_wheel_load for role in WHEELS)
    load_bearing_duty = {
        row["role"]: row["load_bearing_duty_above_1N"] for row in rows
        if row["role"] in WHEELS
    }
    persistent_wheel_load = all(
        duty >= MIN_LOAD_BEARING_DUTY for duty in load_bearing_duty.values()
    )
    unintended_share = unintended_load / EXPECTED_WEIGHT_N
    total_force_series = np.sum(np.abs(data[:, :, 2]), axis=1)
    total_force_cv = float(np.std(total_force_series) / np.mean(total_force_series))
    wheel_force_series = np.sum(np.abs(data[:, :len(WHEELS), 2]), axis=1)
    caster_force_series = np.sum(
        np.abs(data[:, [roles.index(role) for role in WHEELS if role.startswith("caster_")], 2]),
        axis=1,
    )
    caster_share_series = np.divide(
        caster_force_series,
        wheel_force_series,
        out=np.full_like(caster_force_series, np.nan),
        where=wheel_force_series > 0.0,
    )
    caster_share_std = float(np.nanstd(caster_share_series))
    base_z_span = float(np.ptp(pose_data[:, 0]))
    roll_mean = float(np.mean(pose_data[:, 1]))
    pitch_mean = float(np.mean(pose_data[:, 2]))
    roll_span = float(np.ptp(pose_data[:, 1]))
    pitch_span = float(np.ptp(pose_data[:, 2]))
    failure_reasons = []
    if not six_wheels_loaded:
        failure_reasons.append("one or more wheel mean loads below 0.5% of model weight")
    if not persistent_wheel_load:
        failure_reasons.append("one or more wheels lack >=95% load-bearing duty (>1 N)")
    if not 0.85 <= tracked_ratio <= 1.15:
        failure_reasons.append("mean tracked load outside 85-115% of model weight")
    if unintended_share > 0.02:
        failure_reasons.append("non-wheel support exceeds 2% of model weight")
    if total_force_cv > MAX_TOTAL_FORCE_CV:
        failure_reasons.append("total vertical force coefficient of variation exceeds 15%")
    if not math.isfinite(caster_share_std) or caster_share_std > MAX_CASTER_SHARE_STD:
        failure_reasons.append("instantaneous caster-share standard deviation exceeds 0.05")
    if base_z_span > MAX_BASE_Z_SPAN_M:
        failure_reasons.append("base vertical peak-to-peak motion exceeds 5 mm")
    if max(abs(roll_mean), abs(pitch_mean)) > MAX_BASE_TILT_DEG:
        failure_reasons.append("mean base roll or pitch exceeds 1 deg")
    if max(roll_span, pitch_span) > MAX_BASE_TILT_SPAN_DEG:
        failure_reasons.append("base roll or pitch peak-to-peak motion exceeds 1 deg")
    passed = not failure_reasons

    OUTPUT.parent.mkdir(parents=True, exist_ok=True)
    with OUTPUT.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)
    timeseries_output = OUTPUT.with_name(OUTPUT.stem + "_timeseries.csv")
    with timeseries_output.open("w", newline="", encoding="utf-8") as stream:
        fieldnames = ["time_s", "base_z_m", "base_roll_deg", "base_pitch_deg"]
        fieldnames += ["caster_share_of_wheel_load"]
        fieldnames += [f"{role}_abs_Fz_N" for role in roles]
        writer = csv.DictWriter(stream, fieldnames=fieldnames)
        writer.writeheader()
        for sample_index, sample_time in enumerate(sample_times):
            row = {
                "time_s": sample_time,
                "base_z_m": pose_data[sample_index, 0],
                "base_roll_deg": pose_data[sample_index, 1],
                "base_pitch_deg": pose_data[sample_index, 2],
                "caster_share_of_wheel_load": caster_share_series[sample_index],
            }
            row.update({
                f"{role}_abs_Fz_N": abs(data[sample_index, role_index, 2])
                for role_index, role in enumerate(roles)
            })
            writer.writerow(row)

    print("PROVISIONAL authored mass/weight:", round(EXPECTED_MASS_KG, 3), "kg", round(EXPECTED_WEIGHT_N, 3), "N")
    print("active/caster/unintended load:", round(active_load, 3), round(caster_load, 3), round(unintended_load, 3), "N")
    print("tracked load ratio:", round(tracked_ratio, 4))
    print("caster share of tracked load (model only):", round(caster_share, 4))
    print("all wheel mean loads > 0.5% model weight:", six_wheels_loaded)
    print("wheel load-bearing duty (>1 N per sample):", {role: round(value, 3) for role, value in load_bearing_duty.items()})
    print("unintended support share:", round(unintended_share, 4))
    print("total vertical force CV:", round(total_force_cv, 4))
    print("instantaneous caster share min/max/std:",
          round(float(np.nanmin(caster_share_series)), 4),
          round(float(np.nanmax(caster_share_series)), 4),
          round(caster_share_std, 4))
    print("base z peak-to-peak [mm]:", round(base_z_span * 1000.0, 3))
    print("base mean roll/pitch [deg]:", round(roll_mean, 4), round(pitch_mean, 4))
    print("base roll/pitch peak-to-peak [deg]:", round(roll_span, 4), round(pitch_span, 4))
    print("failure reasons:", failure_reasons)
    print("RESULT:", "PASSED" if passed else "REVIEW REQUIRED")
    print("saved:", OUTPUT)
    print("saved timeseries:", timeseries_output)
    print("Timeline paused automatically. Press Stop after copying the result.")
    print("Do not save the stage; the floor and contact-report settings are session-only.")


def _done(task):
    try:
        task.result()
    except asyncio.CancelledError:
        print("Laboratory patient-transport contact gate cancelled.")
    except Exception as error:
        if timeline.is_playing():
            timeline.pause()
        print(f"Laboratory patient-transport contact gate aborted: {type(error).__name__}: {error}")


old_task = globals().get("patient_transport_lab_contact_task")
if old_task is not None and not old_task.done():
    old_task.cancel()
patient_transport_lab_contact_task = asyncio.ensure_future(_run())
patient_transport_lab_contact_task.add_done_callback(_done)
print("Laboratory patient-transport six-wheel static contact gate scheduled; do not use Timeline controls.")
