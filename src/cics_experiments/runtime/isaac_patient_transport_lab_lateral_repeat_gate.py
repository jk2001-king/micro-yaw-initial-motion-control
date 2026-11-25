"""Lateral insertion-stop-extraction repeat gate for the lab platform.

Both active steering axes are held at +90 deg so the front/rear drive wheels
form a differential pair for body-Y translation and yaw. This gate is separate
from the longitudinal Ackermann-like repeat gate.
"""

from cics_experiments.paths import workspace_root, resolve_path

import asyncio
import csv
import math
import time
from collections import deque
from pathlib import Path

import numpy as np
import omni.kit.app
import omni.physx
import omni.timeline
import omni.usd
from isaacsim.core.prims import SingleArticulation
from isaacsim.core.utils.types import ArticulationAction


WORKSPACE = workspace_root()
RUN_LABEL = str(globals().get("patient_lab_lateral_run_label", "lateral_repeat"))
METHOD = str(globals().get("patient_lab_lateral_method", "B2"))
PROFILE = str(globals().get("patient_lab_lateral_profile", "accel_ramp"))
OUTPUT_DIR = (
    resolve_path('studies/CICS26/results/patient_transport_lab/natural_flip_pilot') / RUN_LABEL
)
ROOT = "/World/patient_transport_lab_screening"
ARTICULATION_ROOT = ROOT + "/base_link"
CASTER_JOINTS = tuple(
    f"base_link_to_{name}_caster"
    for name in ("front_left", "front_right", "rear_left", "rear_right")
)
DRIVE_JOINTS = (
    "front_active_steering_to_drive_wheel",
    "rear_active_steering_to_drive_wheel",
)
STEERING_JOINTS = (
    "base_link_to_front_active_steering",
    "base_link_to_rear_active_steering",
)
CONTROLLED_START_DEG = tuple(
    float(value) for value in globals()["patient_lab_lateral_controlled_start_deg"]
)
if len(CONTROLLED_START_DEG) != 4:
    raise ValueError("patient_lab_lateral_controlled_start_deg requires four values")
EXPECTED_MASS_KG = float(globals().get("patient_lab_lateral_expected_mass_kg", 265.0))
EXPECTED_BASE_COM_M = tuple(float(value) for value in globals().get(
    "patient_lab_lateral_expected_base_com_m", (0.0, 0.0, 0.270)
))
if len(EXPECTED_BASE_COM_M) != 3:
    raise ValueError("patient_lab_lateral_expected_base_com_m requires three values")
REPEATS = int(globals().get("patient_lab_lateral_repeats", 3))
VIDEO_DEMO = bool(globals().get("patient_lab_lateral_video_demo", False))
TARGET_M = float(globals().get("patient_lab_lateral_target_m", 0.70))
INSERT_TARGET_M = float(globals().get("patient_lab_lateral_insert_target_m", 0.70))
SPEED_M_S = float(globals().get("patient_lab_lateral_speed_m_s", 0.05))
ACCEL_M_S2 = float(globals().get("patient_lab_lateral_accel_m_s2", 0.025))
ACTIVE_PULSE = bool(globals().get("patient_lab_lateral_active_pulse_enabled", False))
ADAPTIVE_PULSE = bool(globals().get("patient_lab_lateral_adaptive_pulse_enabled", False))
YAW_HOLD = bool(globals().get("patient_lab_lateral_yaw_hold_enabled", False))
PULSE_RATE_RAD_S = float(globals().get("patient_lab_lateral_pulse_rate_rad_s", 0.03203125))
PULSE_DURATION_S = float(globals().get("patient_lab_lateral_pulse_duration_s", 0.5))
ADAPTIVE_START_RAD_S = float(globals().get(
    "patient_lab_lateral_adaptive_start_rad_s", 0.0080078125
))
ADAPTIVE_STEP_RAD_S = float(globals().get(
    "patient_lab_lateral_adaptive_step_rad_s", 0.0080078125
))
ADAPTIVE_STEP_INTERVAL_S = float(globals().get(
    "patient_lab_lateral_adaptive_step_interval_s", 0.5
))
ADAPTIVE_MAX_RAD_S = float(globals().get(
    "patient_lab_lateral_adaptive_max_rad_s", 0.0640625
))
ADAPTIVE_MAX_DURATION_S = float(globals().get(
    "patient_lab_lateral_adaptive_max_duration_s", 2.5
))
RESPONSE_YAW_DEG = float(globals().get(
    "patient_lab_lateral_response_yaw_deg", 0.0002
))
RESPONSE_YAW_RATE_DEG_S = float(globals().get(
    "patient_lab_lateral_response_yaw_rate_deg_s", 0.001
))
RESPONSE_DWELL_S = float(globals().get(
    "patient_lab_lateral_response_dwell_s", 0.10
))
RESPONSE_MODE = str(globals().get(
    "patient_lab_lateral_response_mode", "legacy_or"
))
RESPONSE_FILTER_WINDOW_S = float(globals().get(
    "patient_lab_lateral_response_filter_window_s", 0.20
))
YAW_HOLD_KP = float(globals().get("patient_lab_lateral_yaw_hold_kp", 10.0))
YAW_HOLD_MAX_RAD_S = float(globals().get("patient_lab_lateral_yaw_hold_max_rad_s", 0.12))
WHEEL_RADIUS_M = 0.074
ACTIVE_HALF_SPACING_M = 0.665
PHYSICS_DT_S = 1.0 / 120.0
WALL_TIMEOUT_S = 900.0
EXTRACT_TIMEOUT_S = 35.0
INSERT_TIMEOUT_S = 35.0
SETTLE_S = 2.0

if OUTPUT_DIR.exists():
    raise FileExistsError(f"Existing result would be overwritten: {OUTPUT_DIR}")
expected_repeats = 1 if VIDEO_DEMO else 3
if (REPEATS != expected_repeats or not 0.69 <= TARGET_M <= 0.71
        or not 0.69 <= INSERT_TARGET_M <= 0.71):
    if VIDEO_DEMO:
        raise ValueError("Video-demo mode requires exactly one repeat and 0.70 m insertion/extraction")
    raise ValueError("The frozen experiment gate requires three repeats and 0.70 m insertion/extraction")
if RESPONSE_MODE not in ("legacy_or", "body_yaw_filtered_and"):
    raise ValueError(f"Unknown lateral response mode: {RESPONSE_MODE}")
if not all(math.isfinite(value) and value > 0.0 for value in (
        RESPONSE_YAW_DEG, RESPONSE_YAW_RATE_DEG_S, RESPONSE_DWELL_S,
        RESPONSE_FILTER_WINDOW_S)):
    raise ValueError("Response thresholds, dwell, and filter window must be positive")


def wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def yaw_from_wxyz(q):
    w, x, y, z = (float(value) for value in q)
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def local_xy(position, origin, heading):
    delta = np.asarray(position, dtype=float) - np.asarray(origin, dtype=float)
    return (
        float(math.cos(heading) * delta[0] + math.sin(heading) * delta[1]),
        float(-math.sin(heading) * delta[0] + math.cos(heading) * delta[1]),
    )


timeline = omni.timeline.get_timeline_interface()
if not timeline.is_stopped():
    raise RuntimeError("Press Timeline Stop before this lateral repeat gate")
stage = omni.usd.get_context().get_stage()
if stage is None:
    raise RuntimeError("Open patient_transport_lab.usd first")

physics_clock = {"time_s": 0.0}


def on_physics_step(dt):
    physics_clock["time_s"] += float(dt)


physics_subscription = omni.physx.get_physx_interface().subscribe_physics_step_events(
    on_physics_step
)


async def run_gate():
    timeline.play()
    app = omni.kit.app.get_app()
    wall_start = time.perf_counter()

    async def next_step():
        await app.next_update_async()
        if not timeline.is_playing():
            raise RuntimeError("Simulation stopped during lateral repeat gate")
        if time.perf_counter() - wall_start > WALL_TIMEOUT_S:
            raise TimeoutError("Lateral repeat gate exceeded wall-clock timeout")

    try:
        for _ in range(5):
            await next_step()
        robot = SingleArticulation(ARTICULATION_ROOT)
        robot.initialize()
        indices = {name: index for index, name in enumerate(robot.dof_names)}
        required = (*CASTER_JOINTS, *DRIVE_JOINTS, *STEERING_JOINTS)
        missing = [name for name in required if name not in indices]
        if missing:
            raise RuntimeError(f"Missing articulation DOFs: {missing}")
        caster_idx = np.asarray([indices[name] for name in CASTER_JOINTS], dtype=np.int32)
        drive_idx = np.asarray([indices[name] for name in DRIVE_JOINTS], dtype=np.int32)
        steering_idx = np.asarray([indices[name] for name in STEERING_JOINTS], dtype=np.int32)

        def command_steering_90():
            robot.apply_action(ArticulationAction(
                joint_positions=np.asarray([math.pi / 2.0, math.pi / 2.0], dtype=np.float32),
                joint_indices=steering_idx,
            ))

        def command_lateral(speed_m_s, yaw_rate_rad_s=0.0):
            front = speed_m_s + yaw_rate_rad_s * ACTIVE_HALF_SPACING_M
            rear = speed_m_s - yaw_rate_rad_s * ACTIVE_HALF_SPACING_M
            robot.apply_action(ArticulationAction(
                joint_velocities=np.asarray(
                    [front / WHEEL_RADIUS_M, rear / WHEEL_RADIUS_M], dtype=np.float32
                ),
                joint_indices=drive_idx,
            ))
            command_steering_90()

        reset_position, reset_orientation = robot.get_world_pose()
        reset_position = np.asarray(reset_position, dtype=np.float32)
        reset_orientation = np.asarray(reset_orientation, dtype=np.float32)
        OUTPUT_DIR.mkdir(parents=True, exist_ok=False)
        summaries = []

        for repeat in range(1, REPEATS + 1):
            robot.set_world_pose(position=reset_position, orientation=reset_orientation)
            robot.set_linear_velocity(np.zeros(3, dtype=np.float32))
            robot.set_angular_velocity(np.zeros(3, dtype=np.float32))
            q0 = np.asarray(robot.get_joint_positions(), dtype=np.float32)
            q0[caster_idx] = np.radians(
                np.asarray(CONTROLLED_START_DEG, dtype=np.float32)
            )
            q0[steering_idx] = math.pi / 2.0
            robot.set_joint_positions(q0)
            robot.set_joint_velocities(np.zeros(len(robot.dof_names), dtype=np.float32))
            settle_start = physics_clock["time_s"]
            while physics_clock["time_s"] - settle_start < 1.0:
                command_lateral(0.0)
                await next_step()
            actual_steering = np.asarray(robot.get_joint_positions(), dtype=float)[steering_idx]
            actual_deg = np.degrees(actual_steering)
            if max(abs(actual_deg - 90.0)) > 2.0:
                raise RuntimeError(
                    f"Lateral mode steering did not reach +90 deg: {actual_deg.tolist()}"
                )

            # Insert along +body-Y, stop, then reverse without resetting the
            # body pose or the caster state formed during insertion.
            insert_origin, insert_orientation = robot.get_world_pose()
            reference_heading = yaw_from_wxyz(insert_orientation)
            insert_start = physics_clock["time_s"]
            insert_rows = []
            while physics_clock["time_s"] - insert_start < INSERT_TIMEOUT_S:
                elapsed = physics_clock["time_s"] - insert_start
                speed = min(SPEED_M_S, ACCEL_M_S2 * elapsed)
                command_lateral(speed)
                await next_step()
                elapsed = physics_clock["time_s"] - insert_start
                position, orientation = robot.get_world_pose()
                local_x, local_y = local_xy(position, insert_origin, reference_heading)
                caster_q = np.asarray(robot.get_joint_positions(), dtype=float)[caster_idx]
                insert_rows.append({
                    "time_s": float(elapsed),
                    "insertion_distance_m": float(max(0.0, local_y)),
                    "longitudinal_m": float(local_y),
                    "lateral_m": float(local_x),
                    "yaw_error_deg": float(math.degrees(wrap(
                        yaw_from_wxyz(orientation) - reference_heading
                    ))),
                    "speed_command_m_s": float(speed),
                    **{
                        f"caster_{name}_angle_rad": float(caster_q[index])
                        for index, name in enumerate(
                            ("front_left", "front_right", "rear_left", "rear_right")
                        )
                    },
                })
                if local_y >= INSERT_TARGET_M:
                    break
            if (not insert_rows
                    or insert_rows[-1]["insertion_distance_m"] < INSERT_TARGET_M):
                reached = 0.0 if not insert_rows else insert_rows[-1]["insertion_distance_m"]
                raise RuntimeError(f"Lateral insertion failed: y={reached:.4f} m")

            stop_start = physics_clock["time_s"]
            while physics_clock["time_s"] - stop_start < SETTLE_S:
                command_lateral(0.0)
                await next_step()

            extract_origin, _ = robot.get_world_pose()
            extraction_start_caster_deg = np.degrees(
                np.asarray(robot.get_joint_positions(), dtype=float)[caster_idx]
            )
            extract_start = physics_clock["time_s"]
            previous_time = 0.0
            previous_x = 0.0
            previous_y = 0.0
            rows = []
            flip_since = None
            flip_time = None
            flip_distance = None
            adaptive_response_since = None
            adaptive_response_reached = False
            adaptive_response_time = None
            adaptive_selected_rate = None
            response_yaw_history = deque()
            filtered_yaw_rate_deg_s = 0.0

            while physics_clock["time_s"] - extract_start < EXTRACT_TIMEOUT_S:
                elapsed = physics_clock["time_s"] - extract_start
                speed = (
                    -SPEED_M_S if PROFILE in ("immediate", "constant_slow")
                    else -min(SPEED_M_S, ACCEL_M_S2 * elapsed)
                )
                previous_yaw = rows[-1]["yaw_error_deg"] if rows else 0.0
                if len(rows) >= 2:
                    yaw_dt = max(PHYSICS_DT_S, rows[-1]["time_s"] - rows[-2]["time_s"])
                    previous_yaw_rate = (
                        rows[-1]["yaw_error_deg"] - rows[-2]["yaw_error_deg"]
                    ) / yaw_dt
                else:
                    previous_yaw_rate = 0.0
                if ADAPTIVE_PULSE and not adaptive_response_reached:
                    if RESPONSE_MODE == "body_yaw_filtered_and":
                        response_now = (
                            abs(previous_yaw) >= RESPONSE_YAW_DEG
                            and abs(filtered_yaw_rate_deg_s)
                            >= RESPONSE_YAW_RATE_DEG_S
                            and previous_yaw * filtered_yaw_rate_deg_s > 0.0
                        )
                    else:
                        response_now = (
                            abs(previous_yaw) >= RESPONSE_YAW_DEG
                            or abs(previous_yaw_rate) >= RESPONSE_YAW_RATE_DEG_S
                        )
                    if response_now:
                        if adaptive_response_since is None:
                            adaptive_response_since = elapsed
                        elif elapsed - adaptive_response_since >= RESPONSE_DWELL_S:
                            adaptive_response_reached = True
                            adaptive_response_time = elapsed
                    else:
                        adaptive_response_since = None
                if (ADAPTIVE_PULSE and not adaptive_response_reached
                        and elapsed < ADAPTIVE_MAX_DURATION_S):
                    step_index = int(elapsed // ADAPTIVE_STEP_INTERVAL_S)
                    adaptive_selected_rate = min(
                        ADAPTIVE_MAX_RAD_S,
                        ADAPTIVE_START_RAD_S + step_index * ADAPTIVE_STEP_RAD_S,
                    )
                    yaw_rate_command = adaptive_selected_rate
                    phase = "response_step_micro_yaw"
                elif ACTIVE_PULSE and elapsed < PULSE_DURATION_S:
                    yaw_rate_command = PULSE_RATE_RAD_S
                    phase = "fixed_micro_yaw"
                elif YAW_HOLD:
                    yaw_rate_command = float(np.clip(
                        -YAW_HOLD_KP * math.radians(previous_yaw),
                        -YAW_HOLD_MAX_RAD_S,
                        YAW_HOLD_MAX_RAD_S,
                    ))
                    phase = "yaw_compensation"
                else:
                    yaw_rate_command = 0.0
                    phase = "baseline"
                command_lateral(speed, yaw_rate_command)
                await next_step()

                elapsed = physics_clock["time_s"] - extract_start
                position, orientation = robot.get_world_pose()
                local_x, local_y = local_xy(position, extract_origin, reference_heading)
                distance = max(0.0, -local_y)
                yaw_error = math.degrees(wrap(yaw_from_wxyz(orientation) - reference_heading))
                response_yaw_history.append((elapsed, yaw_error))
                while (
                    len(response_yaw_history) > 1
                    and response_yaw_history[1][0]
                    <= elapsed - RESPONSE_FILTER_WINDOW_S
                ):
                    response_yaw_history.popleft()
                oldest_response_t, oldest_response_yaw = response_yaw_history[0]
                response_span_s = elapsed - oldest_response_t
                filtered_yaw_rate_deg_s = (
                    (yaw_error - oldest_response_yaw) / response_span_s
                    if response_span_s >= 0.75 * RESPONSE_FILTER_WINDOW_S
                    else 0.0
                )
                dt = max(PHYSICS_DT_S, elapsed - previous_time)
                vx = (local_y - previous_y) / dt
                vy = (local_x - previous_x) / dt
                caster_q = np.asarray(robot.get_joint_positions(), dtype=float)[caster_idx]
                caster_qd = np.asarray(robot.get_joint_velocities(), dtype=float)[caster_idx]
                # Extraction travels along -Y, so trailing caster angle is +90 deg.
                errors = [abs(math.degrees(wrap(value - math.pi / 2.0))) for value in caster_q]
                speeds = [abs(math.degrees(value)) for value in caster_qd]
                if max(errors) <= 10.0 and max(speeds) <= 10.0:
                    if flip_since is None:
                        flip_since = (elapsed, distance)
                    elif flip_time is None and elapsed - flip_since[0] >= 0.2:
                        flip_time, flip_distance = flip_since
                else:
                    flip_since = None
                drive_qd = np.asarray(robot.get_joint_velocities(), dtype=float)[drive_idx]
                row = {
                    "time_s": float(elapsed),
                    "reverse_distance_m": float(distance),
                    "longitudinal_m": float(local_y),
                    "lateral_m": float(local_x),
                    "yaw_error_deg": float(yaw_error),
                    "response_filtered_yaw_rate_deg_s": float(filtered_yaw_rate_deg_s),
                    "body_longitudinal_speed_m_s": float(vx),
                    "body_lateral_speed_m_s": float(vy),
                    "speed_command_m_s": float(speed),
                    "yaw_rate_command_rad_s": float(yaw_rate_command),
                    "control_phase": phase,
                    "front_drive_linear_speed_m_s": float(drive_qd[0] * WHEEL_RADIUS_M),
                    "rear_drive_linear_speed_m_s": float(drive_qd[1] * WHEEL_RADIUS_M),
                }
                for index, name in enumerate(("front_left", "front_right", "rear_left", "rear_right")):
                    row[f"caster_{name}_angle_rad"] = float(caster_q[index])
                    row[f"caster_{name}_velocity_rad_s"] = float(caster_qd[index])
                rows.append(row)
                previous_time = elapsed
                previous_x = local_x
                previous_y = local_y
                if distance >= TARGET_M:
                    break

            hold_start = physics_clock["time_s"]
            while physics_clock["time_s"] - hold_start < 0.5:
                command_lateral(0.0)
                await next_step()
            if not rows:
                raise RuntimeError("Lateral extraction produced no samples")
            reached = max(row["reverse_distance_m"] for row in rows) >= TARGET_M
            fields = list(rows[0])
            with (OUTPUT_DIR / f"trace_r{repeat:02d}.csv").open(
                "w", newline="", encoding="utf-8"
            ) as stream:
                writer = csv.DictWriter(stream, fieldnames=fields)
                writer.writeheader()
                writer.writerows(rows)
            with (OUTPUT_DIR / f"insertion_trace_r{repeat:02d}.csv").open(
                "w", newline="", encoding="utf-8"
            ) as stream:
                writer = csv.DictWriter(stream, fieldnames=list(insert_rows[0]))
                writer.writeheader()
                writer.writerows(insert_rows)
            summary = {
                "run_label": RUN_LABEL,
                "method": METHOD,
                "analysis_role": "video_demo_single_repeat" if VIDEO_DEMO else "recorded",
                "motion_mode": "lateral_differential_steering_90deg",
                "repeat": repeat,
                "model_mass_kg_provisional": EXPECTED_MASS_KG,
                "base_com_x_m": EXPECTED_BASE_COM_M[0],
                "base_com_y_m": EXPECTED_BASE_COM_M[1],
                "base_com_z_m": EXPECTED_BASE_COM_M[2],
                "controlled_reverse_start_deg": ";".join(str(v) for v in CONTROLLED_START_DEG),
                "insertion_target_distance_m": INSERT_TARGET_M,
                "insertion_target_reached": True,
                "insertion_time_s": insert_rows[-1]["time_s"],
                "insertion_max_abs_lateral_mm": 1000.0 * max(
                    abs(row["lateral_m"]) for row in insert_rows
                ),
                "insertion_max_abs_yaw_deg": max(
                    abs(row["yaw_error_deg"]) for row in insert_rows
                ),
                "extraction_start_caster_deg": ";".join(
                    f"{value:.6f}" for value in extraction_start_caster_deg
                ),
                "stop_settle_s": SETTLE_S,
                "reverse_target_distance_m": TARGET_M,
                "target_reached": reached,
                "flip_completed": flip_time is not None,
                "flip_time_s": flip_time,
                "flip_distance_m": flip_distance,
                "max_abs_lateral_full_mm": 1000.0 * max(abs(row["lateral_m"]) for row in rows),
                "max_abs_yaw_full_deg": max(abs(row["yaw_error_deg"]) for row in rows),
                "travel_time_s": rows[-1]["time_s"],
                "profile": PROFILE,
                "active_pulse_enabled": ACTIVE_PULSE,
                "adaptive_pulse_enabled": ADAPTIVE_PULSE,
                "adaptive_response_reached": adaptive_response_reached,
                "adaptive_response_time_s": adaptive_response_time,
                "adaptive_selected_rate_rad_s": adaptive_selected_rate,
                "adaptive_response_mode": RESPONSE_MODE if ADAPTIVE_PULSE else None,
                "adaptive_response_yaw_threshold_deg": (
                    RESPONSE_YAW_DEG if ADAPTIVE_PULSE else None
                ),
                "adaptive_response_yaw_rate_threshold_deg_s": (
                    RESPONSE_YAW_RATE_DEG_S if ADAPTIVE_PULSE else None
                ),
                "adaptive_response_dwell_s": RESPONSE_DWELL_S if ADAPTIVE_PULSE else None,
                "adaptive_response_filter_window_s": (
                    RESPONSE_FILTER_WINDOW_S if ADAPTIVE_PULSE else None
                ),
                "yaw_hold_enabled": YAW_HOLD,
                "initial_yaw_pulse_rate_rad_s": PULSE_RATE_RAD_S if ACTIVE_PULSE else 0.0,
                "yaw_hold_kp": YAW_HOLD_KP if YAW_HOLD else 0.0,
                "yaw_hold_max_rad_s": YAW_HOLD_MAX_RAD_S if YAW_HOLD else 0.0,
            }
            summaries.append(summary)
            print(
                f"repeat {repeat}/{REPEATS}: reached={reached}, flip={flip_distance}, "
                f"max lateral={summary['max_abs_lateral_full_mm']:.3f} mm, "
                f"max yaw={summary['max_abs_yaw_full_deg']:.4f} deg"
            )

        with (OUTPUT_DIR / "summary.csv").open("w", newline="", encoding="utf-8") as stream:
            writer = csv.DictWriter(stream, fieldnames=list(summaries[0]))
            writer.writeheader()
            writer.writerows(summaries)
        print("=== Patient-transport lateral differential B2/B3H repeat gate ===")
        print("run label:", RUN_LABEL)
        if VIDEO_DEMO:
            print("VIDEO DEMO ONLY: one run; not a repeatability/statistical result")
        print("method / motion mode:", METHOD, "lateral differential at +90 deg steering")
        print("sequence: +Y insertion -> stop/settle -> -Y extraction (no state reset)")
        print("controlled caster starts [deg]:", CONTROLLED_START_DEG)
        print("saved:", OUTPUT_DIR / "summary.csv")
        print("Timeline paused automatically. Press Stop; do not save the stage.")
    except Exception as exc:
        print(f"Patient-transport lateral repeat gate aborted: {type(exc).__name__}: {exc}")
        raise
    finally:
        timeline.pause()


old_task = globals().get("patient_transport_lateral_gate_task")
if old_task is not None and not old_task.done():
    raise RuntimeError("A patient-transport lateral gate is already running")
patient_transport_lateral_gate_task = asyncio.ensure_future(run_gate())
