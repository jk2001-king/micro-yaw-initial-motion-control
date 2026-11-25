"""Automatic four-wheel static contact-load gate for the capsule wheelchair."""

from cics_experiments.paths import workspace_root, resolve_path

import asyncio
import csv
import time
from pathlib import Path

import numpy as np
import omni.kit.app
import omni.physx
import omni.timeline
import omni.usd
from isaacsim.core.api.sensors import RigidContactView
from pxr import PhysicsSchemaTools, PhysxSchema, UsdPhysics, UsdUtils


WORKSPACE = workspace_root()
TARGET_CASTER_SHARE = float(globals().get("wheelchair_lab_target_caster_share", 0.30))
SHARE_TOLERANCE = 0.05
CONDITION_LABEL = str(globals().get("wheelchair_lab_load_share_label", "baseline_assumed_share30"))
OUTPUT = (
    resolve_path('studies/CICS26/results/wheelchair_lab/load_share_gate')
    / f"{CONDITION_LABEL}.csv"
)
ROBOT_ROOT = "/World/wheelchair_lab_screening"
ARTICULATION_ROOT = ROBOT_ROOT + "/base_link"
GROUND = "/World/WheelchairGround/CollisionBox"
PHYSICS_DT_S = 1.0 / 120.0
SETTLE_S = 1.0
SAMPLE_S = 2.0
DEFAULT_EXPECTED_MASS_KG = 40.0

BODY_PATHS = {
    "drive_left": ROBOT_ROOT + "/left_wheel_link",
    "drive_right": ROBOT_ROOT + "/right_wheel_link",
    "caster_left": ROBOT_ROOT + "/left_caster_wheel_link",
    "caster_right": ROBOT_ROOT + "/right_caster_wheel_link",
    "base": ROBOT_ROOT + "/base_link",
    "fork_left": ROBOT_ROOT + "/left_caster_link",
    "fork_right": ROBOT_ROOT + "/right_caster_link",
}

timeline = omni.timeline.get_timeline_interface()
if timeline.is_playing() or not timeline.is_stopped():
    raise RuntimeError("Press Stop before running the static contact-load gate")

stage = omni.usd.get_context().get_stage()
for path in (ARTICULATION_ROOT, GROUND, *BODY_PATHS.values()):
    if not stage.GetPrimAtPath(path).IsValid():
        raise RuntimeError(f"Missing required prim: {path}")
base_prim = stage.GetPrimAtPath(ARTICULATION_ROOT)
total_mass_attr = base_prim.GetAttribute("experiment:totalMassKg")
EXPECTED_MASS_KG = (
    float(total_mass_attr.Get())
    if total_mass_attr.IsValid() and total_mass_attr.Get() is not None
    else DEFAULT_EXPECTED_MASS_KG
)
EXPECTED_WEIGHT_N = EXPECTED_MASS_KG * 9.81

previous_target = stage.GetEditTarget()
stage.SetEditTarget(stage.GetSessionLayer())
try:
    for path in (*BODY_PATHS.values(), GROUND):
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
        name=f"wheelchair_lab_static_contact_{role}",
        prepare_contact_sensors=False,
        disable_stablization=False,
    )
    for role in roles
]

physics_time_s = 0.0


def _physics_step(dt):
    global physics_time_s
    physics_time_s += float(dt)


physics_subscription = omni.physx.get_physx_interface().subscribe_physics_step_events(_physics_step)


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

    settle_start = physics_time_s
    wall_start = time.perf_counter()
    while physics_time_s - settle_start < SETTLE_S:
        simulation.wake_up(stage_id, encoded_root)
        await app.next_update_async()
        if time.perf_counter() - wall_start > 20.0:
            raise RuntimeError("Timeout during contact-load settling")

    samples = []
    sample_start = physics_time_s
    while physics_time_s - sample_start < SAMPLE_S:
        simulation.wake_up(stage_id, encoded_root)
        await app.next_update_async()
        if not timeline.is_playing():
            raise RuntimeError("Timeline stopped during contact-load sampling")
        forces = np.stack(
            [
                np.asarray(view.get_net_contact_forces(dt=PHYSICS_DT_S), dtype=float).reshape(-1, 3)[0]
                for view in views
            ],
            axis=0,
        )
        samples.append(forces)
        if time.perf_counter() - wall_start > 30.0:
            raise RuntimeError("Timeout during contact-load sampling")

    timeline.pause()
    data = np.asarray(samples, dtype=float)
    rows = []
    print("=== Measured wheelchair static contact-load numerical gate ===")
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
            "samples": int(len(samples)),
        }
        rows.append(row)
        print(
            f"{role}, {row['mean_Fx_N']:+.3f}, {row['mean_Fy_N']:+.3f}, "
            f"{row['mean_Fz_N']:+.3f}, {row['mean_abs_Fz_N']:.3f}"
        )

    load = {row["role"]: row["mean_abs_Fz_N"] for row in rows}
    caster_load = load["caster_left"] + load["caster_right"]
    drive_load = load["drive_left"] + load["drive_right"]
    unintended_load = load["base"] + load["fork_left"] + load["fork_right"]
    tracked_load = caster_load + drive_load + unintended_load
    caster_share = caster_load / EXPECTED_WEIGHT_N
    tracked_ratio = tracked_load / EXPECTED_WEIGHT_N
    lr_imbalance = abs(
        (load["caster_left"] + load["drive_left"])
        - (load["caster_right"] + load["drive_right"])
    ) / EXPECTED_WEIGHT_N
    unintended_share = unintended_load / EXPECTED_WEIGHT_N
    caster_share_error = caster_share - TARGET_CASTER_SHARE
    passed = (
        0.85 <= tracked_ratio <= 1.15
        and abs(caster_share_error) <= SHARE_TOLERANCE
        and lr_imbalance <= 0.10
        and unintended_share <= 0.02
    )

    OUTPUT.parent.mkdir(parents=True, exist_ok=True)
    with OUTPUT.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)

    print("expected mass/weight:", round(EXPECTED_MASS_KG, 3), "kg", round(EXPECTED_WEIGHT_N, 3), "N")
    print("drive/caster/unintended load:", round(drive_load, 3), round(caster_load, 3), round(unintended_load, 3), "N")
    print("tracked load ratio:", round(tracked_ratio, 4))
    print("target caster share:", round(TARGET_CASTER_SHARE, 4))
    print("caster share of weight:", round(caster_share, 4))
    print("caster share error:", round(caster_share_error, 4))
    print("left-right load imbalance:", round(lr_imbalance, 4))
    print("unintended support share:", round(unintended_share, 4))
    print("RESULT:", "PASSED" if passed else "REVIEW REQUIRED")
    print("saved:", OUTPUT)
    print("Timeline paused automatically. Press Stop after copying the result.")


def _done(task):
    try:
        task.result()
    except asyncio.CancelledError:
        print("Measured wheelchair contact-load gate cancelled.")
    except Exception as error:
        if timeline.is_playing():
            timeline.pause()
        print(f"Measured wheelchair contact-load gate aborted: {type(error).__name__}: {error}")


old_task = globals().get("wheelchair_lab_contact_load_task")
if old_task is not None and not old_task.done():
    old_task.cancel()
wheelchair_lab_contact_load_task = asyncio.ensure_future(_run())
wheelchair_lab_contact_load_task.add_done_callback(_done)
print("Measured wheelchair static contact-load numerical gate scheduled; do not use Timeline controls.")
