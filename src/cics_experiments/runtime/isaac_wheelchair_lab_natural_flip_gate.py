"""Measure the unassisted caster flip after a natural forward-stop-reverse cycle.

Run this script from Isaac Sim's Script Editor with the Timeline stopped after
the measured wheelchair ground, powered-hold, contact-material, and compliant
contact preparations have been applied.  The script controls the Timeline;
do not press Play manually.  Session Layer only.
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
from isaacsim.core.api.sensors import RigidContactView
from isaacsim.core.prims import SingleArticulation
from isaacsim.core.utils.types import ArticulationAction
from pxr import PhysxSchema, UsdPhysics


WORKSPACE = workspace_root()
RUN_LABEL = str(globals().get("wheelchair_lab_flip_run_label", "empty_nominal_zero_yaw01"))
REPEATS = int(globals().get("wheelchair_lab_flip_repeats", 3))
WARMUP_REPEATS = int(globals().get("wheelchair_lab_flip_warmup_repeats", 0))
FORWARD_SPEED_M_S = float(globals().get("wheelchair_lab_flip_forward_speed_m_s", 0.10))
FORWARD_DISTANCE_M = float(globals().get("wheelchair_lab_flip_forward_distance_m", 0.80))
FORWARD_MAX_DISTANCE_M = float(
    globals().get("wheelchair_lab_flip_forward_max_distance_m", 3.00)
)
FORWARD_TIMEOUT_S = float(globals().get("wheelchair_lab_flip_forward_timeout_s", 40.0))
PREPARATION_SEED_DEG = float(
    globals().get("wheelchair_lab_flip_preparation_seed_deg", 0.50)
)
REVERSE_SPEED_M_S = float(globals().get("wheelchair_lab_flip_reverse_speed_m_s", 0.05))
REVERSE_PROFILE = str(globals().get("wheelchair_lab_flip_reverse_profile", "immediate"))
REVERSE_ACCEL_LIMIT_M_S2 = float(
    globals().get("wheelchair_lab_flip_reverse_accel_limit_m_s2", 0.025)
)
REVERSE_BODY_SPEED_KP = float(
    globals().get("wheelchair_lab_flip_reverse_body_speed_kp", 0.0)
)
REVERSE_MAX_COMMAND_SPEED_M_S = float(
    globals().get("wheelchair_lab_flip_reverse_max_command_speed_m_s", 0.08)
)
REVERSE_DISTANCE_M = float(globals().get("wheelchair_lab_flip_reverse_distance_m", 0.50))
STOP_SETTLE_S = float(globals().get("wheelchair_lab_flip_stop_settle_s", 1.50))
REVERSE_TIMEOUT_S = float(globals().get("wheelchair_lab_flip_reverse_timeout_s", 15.0))
WALL_TIMEOUT_S = float(globals().get("wheelchair_lab_flip_wall_timeout_s", 300.0))
INITIAL_YAW_PULSE_RATE_RAD_S = float(
    globals().get("wheelchair_lab_flip_initial_yaw_pulse_rate_rad_s", 0.0)
)
INITIAL_YAW_PULSE_DURATION_S = float(
    globals().get("wheelchair_lab_flip_initial_yaw_pulse_duration_s", 0.0)
)
INITIAL_YAW_HOLD_TRANSLATION = bool(
    globals().get("wheelchair_lab_flip_initial_yaw_hold_translation", False)
)
BOOST_YAW_RATE_RAD_S = float(
    globals().get("wheelchair_lab_flip_boost_yaw_rate_rad_s", 0.0)
)
BOOST_YAW_START_S = float(
    globals().get("wheelchair_lab_flip_boost_yaw_start_s", 0.0)
)
BOOST_YAW_DURATION_S = float(
    globals().get("wheelchair_lab_flip_boost_yaw_duration_s", 0.0)
)
ADAPTIVE_INITIAL_ENABLED = bool(
    globals().get("wheelchair_lab_flip_adaptive_initial_enabled", False)
)
ADAPTIVE_FIXED_PULSE_FALLBACK = bool(
    globals().get("wheelchair_lab_flip_adaptive_fixed_pulse_fallback", False)
)
ADAPTIVE_START_S = float(
    globals().get("wheelchair_lab_flip_adaptive_start_s", 0.0)
)
ADAPTIVE_INITIAL_SIGN = int(globals().get("wheelchair_lab_flip_adaptive_initial_sign", 1))
ADAPTIVE_INITIAL_RATE_RAD_S = float(
    globals().get("wheelchair_lab_flip_adaptive_initial_rate_rad_s", 0.010)
)
ADAPTIVE_RATE_STEP_RAD_S = float(
    globals().get("wheelchair_lab_flip_adaptive_rate_step_rad_s", 0.005)
)
ADAPTIVE_STEP_PERIOD_S = float(
    globals().get("wheelchair_lab_flip_adaptive_step_period_s", 0.25)
)
_adaptive_pulse_on = globals().get("wheelchair_lab_flip_adaptive_pulse_on_s", None)
ADAPTIVE_PULSE_ON_S = (
    None if _adaptive_pulse_on is None else float(_adaptive_pulse_on)
)
ADAPTIVE_MAX_RATE_RAD_S = float(
    globals().get("wheelchair_lab_flip_adaptive_max_rate_rad_s", 0.050)
)
ADAPTIVE_MAX_DURATION_S = float(
    globals().get("wheelchair_lab_flip_adaptive_max_duration_s", 2.0)
)
ADAPTIVE_RESPONSE_YAW_DEG = float(
    globals().get("wheelchair_lab_flip_adaptive_response_yaw_deg", 0.50)
)
ADAPTIVE_RESPONSE_MODE = str(
    globals().get("wheelchair_lab_flip_adaptive_response_mode", "yaw_angle")
)
ADAPTIVE_TARGET_DIRECTION_MODE = str(
    globals().get("wheelchair_lab_flip_adaptive_target_direction_mode", "opposed_inward")
)
ADAPTIVE_RESPONSE_RATE_DEG_S = float(
    globals().get("wheelchair_lab_flip_adaptive_response_rate_deg_s", 0.40)
)
ADAPTIVE_RESPONSE_DWELL_S = float(
    globals().get("wheelchair_lab_flip_adaptive_response_dwell_s", 0.10)
)
ADAPTIVE_RESPONSE_WINDOW_S = float(
    globals().get("wheelchair_lab_flip_adaptive_response_window_s", 0.20)
)
_adaptive_impulse_cap = globals().get(
    "wheelchair_lab_flip_adaptive_max_abs_impulse_rad", None
)
ADAPTIVE_MAX_ABS_IMPULSE_RAD = (
    None if _adaptive_impulse_cap is None else float(_adaptive_impulse_cap)
)
_adaptive_selection_speed = globals().get(
    "wheelchair_lab_flip_adaptive_selection_speed_m_s", None
)
ADAPTIVE_SELECTION_SPEED_M_S = (
    None if _adaptive_selection_speed is None else float(_adaptive_selection_speed)
)
ADAPTIVE_RAMP_AFTER_RESPONSE = bool(
    globals().get("wheelchair_lab_flip_adaptive_ramp_after_response", False)
)
ADAPTIVE_REVERSE_ON_WRONG_DIRECTION = bool(
    globals().get("wheelchair_lab_flip_adaptive_reverse_on_wrong_direction", False)
)
_adaptive_reverse_after = globals().get(
    "wheelchair_lab_flip_adaptive_reverse_after_s", None
)
ADAPTIVE_REVERSE_AFTER_S = (
    None if _adaptive_reverse_after is None else float(_adaptive_reverse_after)
)
COUNTER_YAW_RATE_RAD_S = float(
    globals().get("wheelchair_lab_flip_counter_yaw_rate_rad_s", 0.0)
)
COUNTER_YAW_START_S = float(
    globals().get("wheelchair_lab_flip_counter_yaw_start_s", 0.0)
)
COUNTER_YAW_DURATION_S = float(
    globals().get("wheelchair_lab_flip_counter_yaw_duration_s", 0.0)
)
_counter_trigger = globals().get(
    "wheelchair_lab_flip_counter_trigger_abs_yaw_deg", None
)
COUNTER_TRIGGER_ABS_YAW_DEG = (
    None if _counter_trigger is None else float(_counter_trigger)
)
COUNTER_TRIGGER_MIN_TIME_S = float(
    globals().get("wheelchair_lab_flip_counter_trigger_min_time_s", 0.0)
)
COUNTER_REQUIRE_ADAPTIVE_RESPONSE = bool(
    globals().get("wheelchair_lab_flip_counter_require_adaptive_response", False)
)
COUNTER_REQUIRE_DIRECTION_CONFIRMED = bool(
    globals().get("wheelchair_lab_flip_counter_require_direction_confirmed", False)
)
COUNTER_USE_MEASURED_YAW_SIGN = bool(
    globals().get("wheelchair_lab_flip_counter_use_measured_yaw_sign", False)
)
COUNTER_STOP_ON_YAW_RECOVERY = bool(
    globals().get("wheelchair_lab_flip_counter_stop_on_yaw_recovery", False)
)
COUNTER_STOP_ABS_YAW_DEG = float(
    globals().get("wheelchair_lab_flip_counter_stop_abs_yaw_deg", 0.10)
)
COUNTER_REARM_ENABLED = bool(
    globals().get("wheelchair_lab_flip_counter_rearm_enabled", False)
)
COUNTER_REARM_TRIGGER_ABS_YAW_DEG = float(
    globals().get("wheelchair_lab_flip_counter_rearm_trigger_abs_yaw_deg", 1.50)
)
COUNTER_REARM_MIN_GAP_S = float(
    globals().get("wheelchair_lab_flip_counter_rearm_min_gap_s", 0.30)
)
YAW_HOLD_ENABLED = bool(
    globals().get("wheelchair_lab_flip_yaw_hold_enabled", False)
)
YAW_HOLD_REQUIRE_ADAPTIVE_RESPONSE = bool(
    globals().get("wheelchair_lab_flip_yaw_hold_require_adaptive_response", False)
)
YAW_HOLD_RESPONSE_LATCH = bool(
    globals().get("wheelchair_lab_flip_yaw_hold_response_latch", False)
)
YAW_HOLD_AFTER_ADAPTIVE_STOP = bool(
    globals().get("wheelchair_lab_flip_yaw_hold_after_adaptive_stop", False)
)
YAW_HOLD_REQUIRE_FLIP_COMPLETE = bool(
    globals().get("wheelchair_lab_flip_yaw_hold_require_flip_complete", False)
)
YAW_HOLD_START_S = float(
    globals().get("wheelchair_lab_flip_yaw_hold_start_s", 0.50)
)
YAW_HOLD_TRIGGER_ABS_DEG = float(
    globals().get("wheelchair_lab_flip_yaw_hold_trigger_abs_deg", 0.25)
)
YAW_HOLD_TRIGGER_DWELL_S = float(
    globals().get("wheelchair_lab_flip_yaw_hold_trigger_dwell_s", 0.10)
)
YAW_HOLD_STOP_ABS_DEG = float(
    globals().get("wheelchair_lab_flip_yaw_hold_stop_abs_deg", 0.05)
)
YAW_HOLD_KP_RAD_S_PER_RAD = float(
    globals().get("wheelchair_lab_flip_yaw_hold_kp_rad_s_per_rad", 4.0)
)
YAW_HOLD_MAX_RATE_RAD_S = float(
    globals().get("wheelchair_lab_flip_yaw_hold_max_rate_rad_s", 0.03)
)
_direction_target = globals().get(
    "wheelchair_lab_flip_counter_target_direction_mode", None
)
COUNTER_TARGET_DIRECTION_MODE = (
    None if _direction_target is None else str(_direction_target)
)
DIRECTION_CONFIRM_MIN_ROTATION_DEG = float(
    globals().get("wheelchair_lab_flip_direction_confirm_min_rotation_deg", 20.0)
)
DIRECTION_CONFIRM_MIN_SPEED_DEG_S = float(
    globals().get("wheelchair_lab_flip_direction_confirm_min_speed_deg_s", 2.0)
)
DIRECTION_CONFIRM_WINDOW_S = float(
    globals().get("wheelchair_lab_flip_direction_confirm_window_s", 0.40)
)
DIRECTION_CONFIRM_DWELL_S = float(
    globals().get("wheelchair_lab_flip_direction_confirm_dwell_s", 0.30)
)
CONTROLLED_REVERSE_START_DEG = globals().get(
    "wheelchair_lab_flip_controlled_reverse_start_deg", None
)
CONTROLLED_START_SETTLE_S = float(
    globals().get("wheelchair_lab_flip_controlled_start_settle_s", 0.50)
)
if CONTROLLED_REVERSE_START_DEG is not None:
    CONTROLLED_REVERSE_START_DEG = tuple(
        float(value) for value in CONTROLLED_REVERSE_START_DEG
    )
    if len(CONTROLLED_REVERSE_START_DEG) != 2:
        raise RuntimeError(
            "wheelchair_lab_flip_controlled_reverse_start_deg must contain left and right angles"
        )

WHEEL_RADIUS_M = 0.323085
WHEEL_TRACK_M = 0.500
BASELINE_CONTROLLER_KD = math.degrees(50.0)
DRIVE_MAX_FORCE = float(
    globals().get("wheelchair_lab_flip_expected_drive_max_force", 250.0)
)
PHYSICS_DT_S = 1.0 / 120.0
FORWARD_ALIGNMENT_TOL_DEG = 15.0
FORWARD_ALIGNMENT_DWELL_S = 0.30
FLIP_ALIGNMENT_TOL_DEG = 10.0
FLIP_SPEED_TOL_DEG_S = 10.0
FLIP_DWELL_S = 0.20
CONTACT_LOSS_THRESHOLD_N = 1.0

ROBOT_ROOT = "/World/wheelchair_lab_screening"
ARTICULATION_ROOT = ROBOT_ROOT + "/base_link"
GROUND = "/World/WheelchairGround/CollisionBox"
MATERIAL = "/World/WheelchairLabMaterials/WheelGroundContact"
CASTER_JOINTS = ("base_link_to_left_caster", "base_link_to_right_caster")
DRIVE_JOINTS = ("base_link_to_left_wheel", "base_link_to_right_wheel")
CASTER_WHEEL_PATHS = (
    ROBOT_ROOT + "/left_caster_wheel_link",
    ROBOT_ROOT + "/right_caster_wheel_link",
)
OUTPUT_DIR = (
    resolve_path('studies/CICS26/results/wheelchair_lab/natural_flip_gate')
    / RUN_LABEL
)


def _wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def _yaw(q):
    w, x, y, z = [float(v) for v in q]
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def _roll_pitch_deg(q):
    w, x, y, z = [float(v) for v in q]
    roll = math.atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
    sin_pitch = max(-1.0, min(1.0, 2.0 * (w * y - z * x)))
    pitch = math.asin(sin_pitch)
    return math.degrees(roll), math.degrees(pitch)


def _local_displacement(position, origin, origin_yaw):
    delta = np.asarray(position, dtype=float) - np.asarray(origin, dtype=float)
    longitudinal = math.cos(origin_yaw) * delta[0] + math.sin(origin_yaw) * delta[1]
    lateral = -math.sin(origin_yaw) * delta[0] + math.cos(origin_yaw) * delta[1]
    return float(longitudinal), float(lateral)


def _angle_error_deg(value, target):
    return abs(math.degrees(_wrap(float(value) - float(target))))


def _opposed_direction_candidate(caster_q, caster_start, recent_q, recent_dt):
    """Use windowed swivel motion; instantaneous joint velocity is contact-noisy."""
    if recent_q is None or recent_dt <= 0.0:
        return None
    delta = [
        math.degrees(float(position) - float(start))
        for position, start in zip(caster_q, caster_start)
    ]
    speed = [
        math.degrees(float(position) - float(recent)) / recent_dt
        for position, recent in zip(caster_q, recent_q)
    ]
    if any(
        abs(turn) < DIRECTION_CONFIRM_MIN_ROTATION_DEG
        or abs(rate) < DIRECTION_CONFIRM_MIN_SPEED_DEG_S
        or turn * rate <= 0.0
        for turn, rate in zip(delta, speed)
    ):
        return None
    if delta[0] < 0.0 < delta[1]:
        return "opposed_outward"
    if delta[0] > 0.0 > delta[1]:
        return "opposed_inward"
    return None


if REPEATS < 1:
    raise RuntimeError("wheelchair_lab_flip_repeats must be at least 1")
if WARMUP_REPEATS < 0 or WARMUP_REPEATS >= REPEATS:
    raise RuntimeError(
        "wheelchair_lab_flip_warmup_repeats must be nonnegative and less than repeats"
    )
if WALL_TIMEOUT_S <= 0.0:
    raise RuntimeError("wheelchair_lab_flip_wall_timeout_s must be positive")
if REVERSE_PROFILE not in ("immediate", "constant_slow", "accel_ramp"):
    raise RuntimeError("Unknown wheelchair_lab_flip_reverse_profile")
if REVERSE_SPEED_M_S <= 0.0 or REVERSE_ACCEL_LIMIT_M_S2 <= 0.0:
    raise RuntimeError("Reverse speed and acceleration limit must be positive")
if not math.isfinite(REVERSE_BODY_SPEED_KP) or REVERSE_BODY_SPEED_KP < 0.0:
    raise RuntimeError("Reverse body-speed Kp must be finite and nonnegative")
if (
    not math.isfinite(REVERSE_MAX_COMMAND_SPEED_M_S)
    or REVERSE_MAX_COMMAND_SPEED_M_S < REVERSE_SPEED_M_S
):
    raise RuntimeError("Reverse maximum command speed must cover the target speed")
if REVERSE_PROFILE == "constant_slow" and REVERSE_TIMEOUT_S <= REVERSE_DISTANCE_M / REVERSE_SPEED_M_S:
    raise RuntimeError("Reverse timeout is too short for constant_slow profile")
if ADAPTIVE_INITIAL_ENABLED:
    if not math.isfinite(ADAPTIVE_START_S) or ADAPTIVE_START_S < 0.0:
        raise RuntimeError("Adaptive start time must be finite and nonnegative")
    if ADAPTIVE_FIXED_PULSE_FALLBACK:
        if not (
            INITIAL_YAW_PULSE_RATE_RAD_S != 0.0
            and INITIAL_YAW_PULSE_DURATION_S > 0.0
            and ADAPTIVE_START_S >= INITIAL_YAW_PULSE_DURATION_S
        ):
            raise RuntimeError("Fixed-pulse fallback requires a nonoverlapping adaptive start")
    elif ADAPTIVE_START_S != 0.0:
        raise RuntimeError("Delayed adaptive start requires fixed-pulse fallback mode")
    if ADAPTIVE_RESPONSE_MODE not in (
        "yaw_angle", "signed_yaw_motion", "body_yaw_filtered_and",
        "caster_direction", "caster_any_opposed"
    ):
        raise RuntimeError("Unknown adaptive response mode")
    if ADAPTIVE_INITIAL_SIGN not in (-1, 1):
        raise RuntimeError("Adaptive initial sign must be -1 or +1")
    if not all(math.isfinite(value) and value > 0.0 for value in (
        ADAPTIVE_INITIAL_RATE_RAD_S, ADAPTIVE_RATE_STEP_RAD_S,
        ADAPTIVE_STEP_PERIOD_S, ADAPTIVE_MAX_RATE_RAD_S,
        ADAPTIVE_MAX_DURATION_S, ADAPTIVE_RESPONSE_YAW_DEG,
    )):
        raise RuntimeError("Adaptive initial parameters must be positive and finite")
    if ADAPTIVE_MAX_RATE_RAD_S < ADAPTIVE_INITIAL_RATE_RAD_S:
        raise RuntimeError("Adaptive max rate must be at least the initial rate")
    if ADAPTIVE_PULSE_ON_S is not None and not (
        math.isfinite(ADAPTIVE_PULSE_ON_S)
        and 0.0 < ADAPTIVE_PULSE_ON_S <= ADAPTIVE_STEP_PERIOD_S
    ):
        raise RuntimeError("Adaptive probe-on time must be within one step period")
    if ADAPTIVE_RESPONSE_MODE in (
        "signed_yaw_motion", "body_yaw_filtered_and"
    ) and not all(
        math.isfinite(value) and value > 0.0 for value in (
            ADAPTIVE_RESPONSE_RATE_DEG_S,
            ADAPTIVE_RESPONSE_DWELL_S,
            ADAPTIVE_RESPONSE_WINDOW_S,
        )
    ):
        raise RuntimeError("Signed-yaw-motion response parameters must be positive")
    if ADAPTIVE_MAX_ABS_IMPULSE_RAD is not None and not (
        math.isfinite(ADAPTIVE_MAX_ABS_IMPULSE_RAD)
        and ADAPTIVE_MAX_ABS_IMPULSE_RAD > 0.0
    ):
        raise RuntimeError("Adaptive yaw-impulse cap must be positive and finite")
    if ADAPTIVE_SELECTION_SPEED_M_S is not None and not (
        math.isfinite(ADAPTIVE_SELECTION_SPEED_M_S)
        and 0.0 < ADAPTIVE_SELECTION_SPEED_M_S < REVERSE_SPEED_M_S
    ):
        raise RuntimeError(
            "Adaptive selection speed must be positive and below reverse target speed"
        )
    if ADAPTIVE_RAMP_AFTER_RESPONSE and ADAPTIVE_SELECTION_SPEED_M_S is None:
        raise RuntimeError("Post-response ramp requires an adaptive selection speed")
    if ADAPTIVE_REVERSE_AFTER_S is not None and not (
        math.isfinite(ADAPTIVE_REVERSE_AFTER_S)
        and ADAPTIVE_REVERSE_AFTER_S > 0.0
        and ADAPTIVE_REVERSE_AFTER_S < ADAPTIVE_MAX_DURATION_S
    ):
        raise RuntimeError("Adaptive reverse time must be positive and precede the timeout")
    if (INITIAL_YAW_PULSE_RATE_RAD_S != 0.0 and not ADAPTIVE_FIXED_PULSE_FALLBACK) or BOOST_YAW_RATE_RAD_S != 0.0:
        raise RuntimeError("Adaptive initial control cannot overlap fixed yaw pulses")
elif ADAPTIVE_FIXED_PULSE_FALLBACK or ADAPTIVE_START_S != 0.0:
    raise RuntimeError("Adaptive fallback settings require adaptive initial control")
if INITIAL_YAW_PULSE_DURATION_S < 0.0:
    raise RuntimeError("wheelchair_lab_flip_initial_yaw_pulse_duration_s cannot be negative")
if INITIAL_YAW_HOLD_TRANSLATION and not (
    INITIAL_YAW_PULSE_RATE_RAD_S != 0.0
    and 0.0 < INITIAL_YAW_PULSE_DURATION_S <= 1.0
    and abs(INITIAL_YAW_PULSE_RATE_RAD_S) <= 0.10
):
    raise RuntimeError("Pre-translation yaw requires a pulse of at most 0.10 rad/s for at most 1 s")
if not all(math.isfinite(value) for value in (
    INITIAL_YAW_PULSE_RATE_RAD_S, INITIAL_YAW_PULSE_DURATION_S,
    BOOST_YAW_RATE_RAD_S, BOOST_YAW_START_S, BOOST_YAW_DURATION_S,
    COUNTER_YAW_RATE_RAD_S, COUNTER_YAW_START_S, COUNTER_YAW_DURATION_S,
)):
    raise RuntimeError("Yaw command parameters must be finite")
if BOOST_YAW_RATE_RAD_S != 0.0:
    if INITIAL_YAW_PULSE_RATE_RAD_S == 0.0 or INITIAL_YAW_PULSE_DURATION_S == 0.0:
        raise RuntimeError("Yaw step-up requires an initial yaw pulse")
    if BOOST_YAW_RATE_RAD_S * INITIAL_YAW_PULSE_RATE_RAD_S <= 0.0:
        raise RuntimeError("Yaw step-up must have the same sign as the initial pulse")
    if abs(BOOST_YAW_RATE_RAD_S) <= abs(INITIAL_YAW_PULSE_RATE_RAD_S):
        raise RuntimeError("Yaw step-up rate must exceed the initial pulse magnitude")
    if not 0.0 < BOOST_YAW_DURATION_S <= 2.0:
        raise RuntimeError("Yaw step-up duration must be in (0, 2] s")
    if not INITIAL_YAW_PULSE_DURATION_S <= BOOST_YAW_START_S <= 5.0:
        raise RuntimeError("Yaw step-up must start after the initial pulse and within 5 s")
    if abs(BOOST_YAW_RATE_RAD_S) > 0.20:
        raise RuntimeError("Yaw step-up rate must be within +/-0.20 rad/s")
if COUNTER_REQUIRE_DIRECTION_CONFIRMED:
    if COUNTER_TARGET_DIRECTION_MODE not in ("opposed_outward", "opposed_inward"):
        raise RuntimeError(
            "Direction-confirmed counter-yaw requires an opposed target mode"
        )
    if not all(math.isfinite(value) and value > 0.0 for value in (
        DIRECTION_CONFIRM_MIN_ROTATION_DEG,
        DIRECTION_CONFIRM_MIN_SPEED_DEG_S,
        DIRECTION_CONFIRM_WINDOW_S,
        DIRECTION_CONFIRM_DWELL_S,
    )):
        raise RuntimeError("Direction confirmation thresholds must be positive and finite")
    if DIRECTION_CONFIRM_MIN_ROTATION_DEG >= 90.0:
        raise RuntimeError("Direction rotation threshold must be below 90 degrees")
    if DIRECTION_CONFIRM_WINDOW_S < 2.0 * PHYSICS_DT_S:
        raise RuntimeError("Direction confirmation window is too short")
    if COUNTER_TRIGGER_ABS_YAW_DEG is None:
        raise RuntimeError("Direction-confirmed counter-yaw requires a yaw trigger")
if YAW_HOLD_ENABLED:
    if YAW_HOLD_RESPONSE_LATCH and not (
        ADAPTIVE_INITIAL_ENABLED and YAW_HOLD_REQUIRE_ADAPTIVE_RESPONSE
    ):
        raise RuntimeError("Response-latched yaw hold requires response-gated adaptive selection")
    if (
        (ADAPTIVE_INITIAL_ENABLED and not (
            YAW_HOLD_REQUIRE_ADAPTIVE_RESPONSE or YAW_HOLD_AFTER_ADAPTIVE_STOP
            or ADAPTIVE_FIXED_PULSE_FALLBACK
        ))
        or BOOST_YAW_RATE_RAD_S != 0.0
        or COUNTER_YAW_RATE_RAD_S != 0.0
    ):
        raise RuntimeError("Yaw hold with adaptive yaw requires response, adaptive-stop, or fixed-pulse fallback gating; no boost/counter yaw")
    if YAW_HOLD_AFTER_ADAPTIVE_STOP and not ADAPTIVE_INITIAL_ENABLED:
        raise RuntimeError("Adaptive-stop-gated yaw hold requires adaptive selection")
    if YAW_HOLD_AFTER_ADAPTIVE_STOP and YAW_HOLD_REQUIRE_ADAPTIVE_RESPONSE:
        raise RuntimeError("Choose one yaw-hold activation gate")
    if YAW_HOLD_REQUIRE_ADAPTIVE_RESPONSE and not ADAPTIVE_INITIAL_ENABLED:
        raise RuntimeError("Response-gated yaw hold requires adaptive selection")
    if not all(math.isfinite(value) and value > 0.0 for value in (
        YAW_HOLD_TRIGGER_ABS_DEG,
        YAW_HOLD_TRIGGER_DWELL_S,
        YAW_HOLD_KP_RAD_S_PER_RAD,
        YAW_HOLD_MAX_RATE_RAD_S,
    )):
        raise RuntimeError("Yaw-hold thresholds and gains must be positive and finite")
    if not math.isfinite(YAW_HOLD_START_S) or YAW_HOLD_START_S < 0.0:
        raise RuntimeError("Yaw-hold start time must be finite and nonnegative")
    if not 0.0 <= YAW_HOLD_STOP_ABS_DEG < YAW_HOLD_TRIGGER_ABS_DEG:
        raise RuntimeError("Yaw-hold stop threshold must be below its trigger threshold")
if COUNTER_USE_MEASURED_YAW_SIGN and COUNTER_TRIGGER_ABS_YAW_DEG is None:
    raise RuntimeError("Measured-yaw-sign counter-yaw requires a yaw trigger")
if COUNTER_REQUIRE_ADAPTIVE_RESPONSE and not (
    ADAPTIVE_INITIAL_ENABLED
    and COUNTER_YAW_RATE_RAD_S != 0.0
    and COUNTER_TRIGGER_ABS_YAW_DEG is not None
):
    raise RuntimeError(
        "Adaptive-response counter gate requires adaptive selection and a triggered counter"
    )
if COUNTER_STOP_ON_YAW_RECOVERY:
    if not COUNTER_USE_MEASURED_YAW_SIGN:
        raise RuntimeError("Yaw-recovery stop requires measured-yaw-sign counter-yaw")
    if not (math.isfinite(COUNTER_STOP_ABS_YAW_DEG) and COUNTER_STOP_ABS_YAW_DEG > 0.0):
        raise RuntimeError("Yaw-recovery stop tolerance must be positive and finite")
    if COUNTER_STOP_ABS_YAW_DEG >= COUNTER_TRIGGER_ABS_YAW_DEG:
        raise RuntimeError("Yaw-recovery stop tolerance must be below the trigger threshold")
if COUNTER_YAW_RATE_RAD_S != 0.0:
    if not ADAPTIVE_INITIAL_ENABLED and (
        INITIAL_YAW_PULSE_RATE_RAD_S == 0.0 or INITIAL_YAW_PULSE_DURATION_S == 0.0
    ):
        raise RuntimeError("Counter-yaw requires an initial yaw pulse")
    initial_selection_sign = (
        ADAPTIVE_INITIAL_SIGN if ADAPTIVE_INITIAL_ENABLED else INITIAL_YAW_PULSE_RATE_RAD_S
    )
    if COUNTER_YAW_RATE_RAD_S * initial_selection_sign >= 0.0:
        raise RuntimeError("Counter-yaw must have the opposite sign")
    if not 0.0 < COUNTER_YAW_DURATION_S <= 5.0:
        raise RuntimeError("Counter-yaw duration must be in (0, 5] s")
    if COUNTER_TRIGGER_ABS_YAW_DEG is None:
        if not INITIAL_YAW_PULSE_DURATION_S <= COUNTER_YAW_START_S <= 10.0:
            raise RuntimeError("Counter-yaw must start after the initial pulse and within 10 s")
    elif not (
        math.isfinite(COUNTER_TRIGGER_ABS_YAW_DEG)
        and COUNTER_TRIGGER_ABS_YAW_DEG > 0.0
        and math.isfinite(COUNTER_TRIGGER_MIN_TIME_S)
        and INITIAL_YAW_PULSE_DURATION_S <= COUNTER_TRIGGER_MIN_TIME_S <= 10.0
    ):
        raise RuntimeError("Invalid response-triggered counter-yaw threshold or minimum time")
    if abs(COUNTER_YAW_RATE_RAD_S) > 0.20:
        raise RuntimeError("Counter-yaw rate must be within +/-0.20 rad/s")
    if BOOST_YAW_RATE_RAD_S != 0.0 and COUNTER_TRIGGER_ABS_YAW_DEG is None:
        if COUNTER_YAW_START_S < BOOST_YAW_START_S + BOOST_YAW_DURATION_S:
            raise RuntimeError("Counter-yaw cannot overlap yaw step-up")
    if (
        BOOST_YAW_RATE_RAD_S != 0.0
        and COUNTER_TRIGGER_ABS_YAW_DEG is not None
        and COUNTER_TRIGGER_MIN_TIME_S < BOOST_YAW_START_S + BOOST_YAW_DURATION_S
    ):
        raise RuntimeError("Response-triggered counter-yaw cannot overlap yaw step-up")
elif COUNTER_TRIGGER_ABS_YAW_DEG is not None:
    raise RuntimeError("Response-triggered counter-yaw requires a nonzero counter command")
if COUNTER_REARM_ENABLED:
    if not (
        COUNTER_YAW_RATE_RAD_S != 0.0
        and COUNTER_TRIGGER_ABS_YAW_DEG is not None
        and COUNTER_REQUIRE_DIRECTION_CONFIRMED
        and COUNTER_USE_MEASURED_YAW_SIGN
        and COUNTER_STOP_ON_YAW_RECOVERY
    ):
        raise RuntimeError(
            "Counter rearm requires direction-confirmed, measured-sign, "
            "response-triggered counter-yaw with recovery stop"
        )
    if not (
        math.isfinite(COUNTER_REARM_TRIGGER_ABS_YAW_DEG)
        and COUNTER_REARM_TRIGGER_ABS_YAW_DEG > COUNTER_STOP_ABS_YAW_DEG
        and math.isfinite(COUNTER_REARM_MIN_GAP_S)
        and COUNTER_REARM_MIN_GAP_S >= 0.0
    ):
        raise RuntimeError("Invalid counter rearm threshold or minimum gap")

timeline = omni.timeline.get_timeline_interface()
if timeline.is_playing() or not timeline.is_stopped():
    raise RuntimeError("Press Stop before running the natural caster-flip gate")
if OUTPUT_DIR.exists():
    raise FileExistsError(f"Existing result would be overwritten: {OUTPUT_DIR}")

stage = omni.usd.get_context().get_stage()
for path in (ARTICULATION_ROOT, GROUND, MATERIAL, *CASTER_WHEEL_PATHS):
    if not stage.GetPrimAtPath(path).IsValid():
        raise RuntimeError(f"Missing prepared prim: {path}")

physx_material = PhysxSchema.PhysxMaterialAPI(stage.GetPrimAtPath(MATERIAL))
if float(physx_material.GetCompliantContactStiffnessAttr().Get() or 0.0) <= 0.0:
    raise RuntimeError("Compliant wheel-ground contact is not prepared")
usd_material = UsdPhysics.MaterialAPI(stage.GetPrimAtPath(MATERIAL))
STATIC_FRICTION = float(usd_material.GetStaticFrictionAttr().Get())
DYNAMIC_FRICTION = float(usd_material.GetDynamicFrictionAttr().Get())

base_prim = stage.GetPrimAtPath(ARTICULATION_ROOT)
share_attr = base_prim.GetAttribute("experiment:targetCasterLoadShare")
TARGET_CASTER_SHARE = (
    float(share_attr.Get())
    if share_attr.IsValid() and share_attr.Get() is not None
    else 0.30
)
share_label_attr = base_prim.GetAttribute("experiment:loadShareCondition")
LOAD_CONDITION = (
    str(share_label_attr.Get())
    if share_label_attr.IsValid() and share_label_attr.Get() is not None
    else "baseline_assumed_share30"
)

previous_target = stage.GetEditTarget()
stage.SetEditTarget(stage.GetSessionLayer())
try:
    for path in (*CASTER_WHEEL_PATHS, GROUND):
        PhysxSchema.PhysxContactReportAPI.Apply(
            stage.GetPrimAtPath(path)
        ).CreateThresholdAttr(0.0).Set(0.0)
finally:
    stage.SetEditTarget(previous_target)

contact_views = [
    RigidContactView(
        prim_paths_expr=path,
        filter_paths_expr=[],
        name=f"wheelchair_lab_flip_contact_{side}",
        prepare_contact_sensors=False,
        disable_stablization=False,
    )
    for path, side in zip(CASTER_WHEEL_PATHS, ("left", "right"))
]

physics_time_s = 0.0


def _physics_step(dt):
    global physics_time_s
    physics_time_s += float(dt)


physics_subscription = omni.physx.get_physx_interface().subscribe_physics_step_events(
    _physics_step
)


async def _run():
    timeline.play()
    app = omni.kit.app.get_app()
    for _ in range(5):
        await app.next_update_async()

    robot = SingleArticulation(ARTICULATION_ROOT)
    robot.initialize()
    for view in contact_views:
        view.initialize()

    indices = {name: index for index, name in enumerate(robot.dof_names)}
    missing = [name for name in (*CASTER_JOINTS, *DRIVE_JOINTS) if name not in indices]
    if missing:
        raise RuntimeError(f"Missing required DOFs: {missing}")
    caster_indices = np.asarray([indices[name] for name in CASTER_JOINTS], dtype=np.int32)
    drive_indices = np.asarray([indices[name] for name in DRIVE_JOINTS], dtype=np.int32)
    controller = robot.get_articulation_controller()
    drive_stiffnesses, drive_dampings = controller.get_gains()
    drive_stiffnesses = np.asarray(drive_stiffnesses, dtype=float)[drive_indices]
    drive_dampings = np.asarray(drive_dampings, dtype=float)[drive_indices]
    drive_max_efforts = np.asarray(controller.get_max_efforts(), dtype=float)[drive_indices]
    if not np.allclose(drive_stiffnesses, 0.0, atol=1.0e-6):
        raise RuntimeError(f"Drive stiffness is not zero: {drive_stiffnesses}")
    if not np.allclose(
        drive_dampings, BASELINE_CONTROLLER_KD, rtol=1.0e-5, atol=1.0e-3
    ):
        raise RuntimeError(f"Expected baseline drive Kd=50 USD: {drive_dampings}")
    if not np.allclose(drive_max_efforts, DRIVE_MAX_FORCE, atol=1.0e-4):
        raise RuntimeError(
            f"Expected drive max effort {DRIVE_MAX_FORCE:g}: {drive_max_efforts}"
        )

    def command_twist(speed_m_s, yaw_rate_rad_s=0.0):
        left = float(speed_m_s) - 0.5 * WHEEL_TRACK_M * float(yaw_rate_rad_s)
        right = float(speed_m_s) + 0.5 * WHEEL_TRACK_M * float(yaw_rate_rad_s)
        target_rates = np.asarray(
            [left / WHEEL_RADIUS_M, right / WHEEL_RADIUS_M], dtype=np.float32
        )
        robot.apply_action(
            ArticulationAction(
                joint_velocities=target_rates,
                joint_indices=drive_indices,
            )
        )
        return target_rates

    # Settle compliant contact once and capture a common floating-base reset pose.
    precondition_start = physics_time_s
    while physics_time_s - precondition_start < 2.0:
        command_twist(0.0)
        await app.next_update_async()
    reset_position, reset_orientation = robot.get_world_pose()
    reset_position = np.asarray(reset_position, dtype=np.float32)
    reset_orientation = np.asarray(reset_orientation, dtype=np.float32)

    OUTPUT_DIR.mkdir(parents=True, exist_ok=False)
    summaries = []
    wall_start = time.perf_counter()

    for repeat in range(1, REPEATS + 1):
        command_twist(0.0)
        robot.set_world_pose(position=reset_position, orientation=reset_orientation)
        robot.set_linear_velocity(np.zeros(3, dtype=np.float32))
        robot.set_angular_velocity(np.zeros(3, dtype=np.float32))
        reset_q = np.zeros(len(robot.dof_names), dtype=np.float32)
        # Exact leading (0 deg) is a symmetric unstable equilibrium. A tiny,
        # opposed preparation-only seed makes forward alignment deterministic
        # without prescribing the measured reverse-flip direction.
        reset_q[caster_indices] = np.radians(
            np.asarray([PREPARATION_SEED_DEG, -PREPARATION_SEED_DEG], dtype=np.float32)
        )
        robot.set_joint_positions(reset_q)
        robot.set_joint_velocities(np.zeros(len(robot.dof_names), dtype=np.float32))

        reset_settle_start = physics_time_s
        while physics_time_s - reset_settle_start < 0.5:
            command_twist(0.0)
            await app.next_update_async()

        # Natural preparation: drive forward until both casters trail the motion.
        forward_origin, forward_orientation = robot.get_world_pose()
        forward_yaw = _yaw(forward_orientation)
        forward_start = physics_time_s
        forward_longitudinal = 0.0
        forward_aligned_since = None
        while True:
            command_twist(FORWARD_SPEED_M_S, 0.0)
            await app.next_update_async()
            position, _ = robot.get_world_pose()
            forward_longitudinal, _ = _local_displacement(
                position, forward_origin, forward_yaw
            )
            q_forward = np.asarray(robot.get_joint_positions(), dtype=float)
            forward_errors_now = [
                _angle_error_deg(value, math.pi)
                for value in q_forward[caster_indices]
            ]
            forward_elapsed = physics_time_s - forward_start
            if (
                forward_longitudinal >= FORWARD_DISTANCE_M
                and max(forward_errors_now) <= FORWARD_ALIGNMENT_TOL_DEG
            ):
                if forward_aligned_since is None:
                    forward_aligned_since = forward_elapsed
                elif forward_elapsed - forward_aligned_since >= FORWARD_ALIGNMENT_DWELL_S:
                    break
            else:
                forward_aligned_since = None
            if (
                forward_longitudinal >= FORWARD_MAX_DISTANCE_M
                or forward_elapsed >= FORWARD_TIMEOUT_S
            ):
                raise RuntimeError(
                    "Forward caster-alignment preparation did not converge: "
                    f"distance_m={forward_longitudinal:.3f}, "
                    f"errors_deg={forward_errors_now}"
                )

        command_twist(0.0)
        stop_start = physics_time_s
        while physics_time_s - stop_start < STOP_SETTLE_S:
            command_twist(0.0)
            await app.next_update_async()

        pre_reapply_caster_deg = (None, None)
        pre_reapply_base_z_m = None
        if CONTROLLED_REVERSE_START_DEG is not None:
            controlled_q = np.asarray(robot.get_joint_positions(), dtype=np.float32)
            controlled_q[caster_indices] = np.radians(
                np.asarray(CONTROLLED_REVERSE_START_DEG, dtype=np.float32)
            )
            robot.set_joint_positions(controlled_q)
            robot.set_joint_velocities(np.zeros(len(robot.dof_names), dtype=np.float32))
            robot.set_linear_velocity(np.zeros(3, dtype=np.float32))
            robot.set_angular_velocity(np.zeros(3, dtype=np.float32))
            controlled_settle_start = physics_time_s
            while physics_time_s - controlled_settle_start < CONTROLLED_START_SETTLE_S:
                command_twist(0.0)
                await app.next_update_async()
            pre_reapply_q = np.asarray(robot.get_joint_positions(), dtype=float)
            pre_reapply_caster_deg = tuple(
                math.degrees(value) for value in pre_reapply_q[caster_indices]
            )
            pre_reapply_position, _ = robot.get_world_pose()
            pre_reapply_base_z_m = float(pre_reapply_position[2])
            # Re-apply the exact diagnostic angle after contact settling, then
            # release it directly into the reverse maneuver. This prevents the
            # load-dependent zero-speed drift from contaminating the controlled
            # initial-angle comparison.
            controlled_q = np.asarray(robot.get_joint_positions(), dtype=np.float32)
            controlled_q[caster_indices] = np.radians(
                np.asarray(CONTROLLED_REVERSE_START_DEG, dtype=np.float32)
            )
            robot.set_joint_positions(controlled_q)
            robot.set_joint_velocities(np.zeros(len(robot.dof_names), dtype=np.float32))
            robot.set_linear_velocity(np.zeros(3, dtype=np.float32))
            robot.set_angular_velocity(np.zeros(3, dtype=np.float32))

        q_start = np.asarray(robot.get_joint_positions(), dtype=float)
        qd_start = np.asarray(robot.get_joint_velocities(), dtype=float)
        base_start_position, base_start_orientation = robot.get_world_pose()
        base_start_roll_deg, base_start_pitch_deg = _roll_pitch_deg(base_start_orientation)
        caster_start_fz = [
            float(
                np.asarray(view.get_net_contact_forces(dt=PHYSICS_DT_S), dtype=float)
                .reshape(-1, 3)[0, 2]
            )
            for view in contact_views
        ]
        caster_start = q_start[caster_indices]
        forward_errors = [
            _angle_error_deg(value, math.pi) for value in caster_start
        ]
        if max(forward_errors) > FORWARD_ALIGNMENT_TOL_DEG:
            raise RuntimeError(
                "Natural forward preparation did not align both casters: "
                f"errors_deg={forward_errors}"
            )

        reverse_origin, reverse_orientation = robot.get_world_pose()
        reverse_yaw = _yaw(reverse_orientation)
        reverse_start = physics_time_s
        rows = []
        aligned_since = None
        individual_aligned_since = [None, None]
        individual_aligned_distance = [None, None]
        individual_flip_time_s = [None, None]
        individual_flip_distance_m = [None, None]
        flip_time_s = None
        flip_distance_m = None
        previous_t = reverse_start
        previous_yaw_error = 0.0
        previous_longitudinal = 0.0
        previous_lateral = 0.0
        previous_longitudinal_velocity = 0.0
        previous_lateral_velocity = 0.0
        adaptive_response_at_s = None
        adaptive_last_rate_rad_s = 0.0
        adaptive_step_count = 0
        adaptive_abs_impulse_rad = 0.0
        adaptive_fallback_started_s = None
        adaptive_active_sign = ADAPTIVE_INITIAL_SIGN
        adaptive_sign_switch_count = 0
        adaptive_wrong_direction_handled = False
        adaptive_cap_hit_at_s = None
        adaptive_motion_history = deque()
        adaptive_motion_rate_deg_s = 0.0
        adaptive_response_candidate_since_s = None
        counter_started_s = None
        counter_trigger_yaw_deg = None
        counter_trigger_distance_m = None
        counter_trigger_direction_mode = None
        counter_trigger_direction_confirmed_s = None
        counter_actual_rate_rad_s = None
        counter_actual_applied_duration_s = 0.0
        counter_stopped_s = None
        counter_stop_reason = None
        rearm_started_s = None
        rearm_trigger_yaw_deg = None
        rearm_trigger_distance_m = None
        rearm_actual_rate_rad_s = None
        rearm_actual_applied_duration_s = 0.0
        rearm_stopped_s = None
        rearm_stop_reason = None
        direction_candidate_mode = None
        direction_candidate_since_s = None
        direction_confirmed_mode = None
        direction_confirmed_at_s = None
        direction_motion_history = deque()
        first_direction_confirmed_mode = None
        first_direction_confirmed_s = None
        first_direction_confirmed_distance_m = None
        yaw_hold_active = False
        yaw_hold_candidate_since_s = None
        yaw_hold_first_started_s = None
        yaw_hold_activation_count = 0
        yaw_hold_applied_duration_s = 0.0
        yaw_hold_last_rate_rad_s = 0.0

        while physics_time_s - reverse_start < REVERSE_TIMEOUT_S:
            command_elapsed = physics_time_s - reverse_start
            adaptive_elapsed = command_elapsed - ADAPTIVE_START_S
            adaptive_deadline_s = ADAPTIVE_START_S + ADAPTIVE_MAX_DURATION_S
            yaw_error_deg_before_command = math.degrees(previous_yaw_error)
            if YAW_HOLD_ENABLED:
                if (
                    YAW_HOLD_RESPONSE_LATCH
                    and adaptive_response_at_s is not None
                    and command_elapsed >= YAW_HOLD_START_S
                ):
                    if not yaw_hold_active:
                        yaw_hold_active = True
                        yaw_hold_activation_count += 1
                        yaw_hold_first_started_s = command_elapsed
                    yaw_hold_candidate_since_s = None
                elif yaw_hold_active:
                    if abs(yaw_error_deg_before_command) <= YAW_HOLD_STOP_ABS_DEG:
                        yaw_hold_active = False
                        yaw_hold_candidate_since_s = None
                elif (
                    command_elapsed >= YAW_HOLD_START_S
                    and (
                        not YAW_HOLD_REQUIRE_ADAPTIVE_RESPONSE
                        or adaptive_response_at_s is not None
                    )
                    and (
                        not YAW_HOLD_AFTER_ADAPTIVE_STOP
                        or adaptive_response_at_s is not None
                        or adaptive_cap_hit_at_s is not None
                        or command_elapsed >= adaptive_deadline_s
                    )
                    and (
                        not YAW_HOLD_REQUIRE_FLIP_COMPLETE
                        or flip_time_s is not None
                    )
                    and abs(yaw_error_deg_before_command) >= YAW_HOLD_TRIGGER_ABS_DEG
                ):
                    if yaw_hold_candidate_since_s is None:
                        yaw_hold_candidate_since_s = command_elapsed
                    elif (
                        command_elapsed - yaw_hold_candidate_since_s
                        >= YAW_HOLD_TRIGGER_DWELL_S
                    ):
                        yaw_hold_active = True
                        yaw_hold_activation_count += 1
                        if yaw_hold_first_started_s is None:
                            yaw_hold_first_started_s = command_elapsed
                else:
                    yaw_hold_candidate_since_s = None
            else:
                yaw_hold_active = False
                yaw_hold_candidate_since_s = None
            if ADAPTIVE_INITIAL_ENABLED and adaptive_response_at_s is None:
                if (
                    ADAPTIVE_REVERSE_AFTER_S is not None
                    and adaptive_sign_switch_count == 0
                    and adaptive_elapsed >= ADAPTIVE_REVERSE_AFTER_S
                ):
                    adaptive_active_sign *= -1
                    adaptive_sign_switch_count += 1
                if (
                    ADAPTIVE_REVERSE_ON_WRONG_DIRECTION
                    and not adaptive_wrong_direction_handled
                    and direction_confirmed_mode in ("opposed_inward", "opposed_outward")
                    and direction_confirmed_mode != ADAPTIVE_TARGET_DIRECTION_MODE
                ):
                    adaptive_active_sign *= -1
                    adaptive_sign_switch_count += 1
                    adaptive_wrong_direction_handled = True
                if ADAPTIVE_RESPONSE_MODE == "yaw_angle":
                    if (
                        0.0 < command_elapsed <= adaptive_deadline_s
                        and abs(math.degrees(previous_yaw_error))
                        >= ADAPTIVE_RESPONSE_YAW_DEG
                    ):
                        adaptive_response_at_s = command_elapsed
                elif ADAPTIVE_RESPONSE_MODE == "signed_yaw_motion":
                    # Both cues must have the commanded sign. The caster oracle
                    # and contact forces are intentionally absent from this rule.
                    response_now = (
                        command_elapsed <= adaptive_deadline_s
                        and adaptive_cap_hit_at_s is None
                        and ADAPTIVE_INITIAL_SIGN * math.degrees(previous_yaw_error)
                        >= ADAPTIVE_RESPONSE_YAW_DEG
                        and ADAPTIVE_INITIAL_SIGN * adaptive_motion_rate_deg_s
                        >= ADAPTIVE_RESPONSE_RATE_DEG_S
                    )
                    if response_now:
                        if adaptive_response_candidate_since_s is None:
                            adaptive_response_candidate_since_s = command_elapsed
                        elif (
                            command_elapsed - adaptive_response_candidate_since_s
                            >= ADAPTIVE_RESPONSE_DWELL_S
                        ):
                            adaptive_response_at_s = command_elapsed
                    else:
                        adaptive_response_candidate_since_s = None
                elif ADAPTIVE_RESPONSE_MODE == "body_yaw_filtered_and":
                    # Platform-common IMU-compatible response rule: both the
                    # body-yaw magnitude and its window-filtered rate must be
                    # sustained. No caster-angle oracle enters this decision.
                    response_now = (
                        command_elapsed <= adaptive_deadline_s
                        and adaptive_cap_hit_at_s is None
                        and abs(math.degrees(previous_yaw_error))
                        >= ADAPTIVE_RESPONSE_YAW_DEG
                        and abs(adaptive_motion_rate_deg_s)
                        >= ADAPTIVE_RESPONSE_RATE_DEG_S
                        and math.degrees(previous_yaw_error)
                        * adaptive_motion_rate_deg_s > 0.0
                    )
                    if response_now:
                        if adaptive_response_candidate_since_s is None:
                            adaptive_response_candidate_since_s = command_elapsed
                        elif (
                            command_elapsed - adaptive_response_candidate_since_s
                            >= ADAPTIVE_RESPONSE_DWELL_S
                        ):
                            adaptive_response_at_s = command_elapsed
                    else:
                        adaptive_response_candidate_since_s = None
                elif (
                    ADAPTIVE_RESPONSE_MODE == "caster_any_opposed"
                    and command_elapsed <= adaptive_deadline_s
                    and adaptive_cap_hit_at_s is None
                    and direction_confirmed_mode in ("opposed_inward", "opposed_outward")
                ):
                    adaptive_response_at_s = command_elapsed
                elif (
                    ADAPTIVE_RESPONSE_MODE == "caster_direction"
                    and command_elapsed <= adaptive_deadline_s
                    and adaptive_cap_hit_at_s is None
                    and direction_confirmed_mode == ADAPTIVE_TARGET_DIRECTION_MODE
                ):
                    adaptive_response_at_s = command_elapsed
            if (
                COUNTER_TRIGGER_ABS_YAW_DEG is not None
                and counter_started_s is None
                and command_elapsed >= COUNTER_TRIGGER_MIN_TIME_S
                and abs(math.degrees(previous_yaw_error)) >= COUNTER_TRIGGER_ABS_YAW_DEG
                and (
                    not COUNTER_REQUIRE_ADAPTIVE_RESPONSE
                    or adaptive_response_at_s is not None
                )
                and (
                    not COUNTER_REQUIRE_DIRECTION_CONFIRMED
                    or direction_confirmed_mode == COUNTER_TARGET_DIRECTION_MODE
                )
                and (not COUNTER_REQUIRE_DIRECTION_CONFIRMED or flip_time_s is None)
            ):
                counter_started_s = command_elapsed
                counter_trigger_yaw_deg = math.degrees(previous_yaw_error)
                counter_trigger_distance_m = max(0.0, -previous_longitudinal)
                counter_trigger_direction_mode = direction_confirmed_mode
                counter_trigger_direction_confirmed_s = direction_confirmed_at_s
                counter_actual_rate_rad_s = (
                    -math.copysign(abs(COUNTER_YAW_RATE_RAD_S), previous_yaw_error)
                    if COUNTER_USE_MEASURED_YAW_SIGN
                    else COUNTER_YAW_RATE_RAD_S
                )
            if counter_started_s is not None and counter_stopped_s is None:
                if COUNTER_STOP_ON_YAW_RECOVERY and command_elapsed > counter_started_s:
                    yaw_now_deg = math.degrees(previous_yaw_error)
                    if (
                        abs(yaw_now_deg) <= COUNTER_STOP_ABS_YAW_DEG
                        or yaw_now_deg * counter_trigger_yaw_deg <= 0.0
                    ):
                        counter_stopped_s = command_elapsed
                        counter_stop_reason = "yaw_recovered_or_crossed_zero"
                if (
                    counter_stopped_s is None
                    and command_elapsed >= counter_started_s + COUNTER_YAW_DURATION_S
                ):
                    counter_stopped_s = command_elapsed
                    counter_stop_reason = "max_duration"
            if (
                COUNTER_REARM_ENABLED
                and counter_stop_reason == "yaw_recovered_or_crossed_zero"
                and rearm_started_s is None
                and command_elapsed >= counter_stopped_s + COUNTER_REARM_MIN_GAP_S
                and abs(math.degrees(previous_yaw_error)) >= COUNTER_REARM_TRIGGER_ABS_YAW_DEG
                and direction_confirmed_mode == COUNTER_TARGET_DIRECTION_MODE
                and flip_time_s is None
            ):
                rearm_started_s = command_elapsed
                rearm_trigger_yaw_deg = math.degrees(previous_yaw_error)
                rearm_trigger_distance_m = max(0.0, -previous_longitudinal)
                rearm_actual_rate_rad_s = -math.copysign(
                    abs(COUNTER_YAW_RATE_RAD_S), previous_yaw_error
                )
            if rearm_started_s is not None and rearm_stopped_s is None:
                if flip_time_s is not None:
                    rearm_stopped_s = command_elapsed
                    rearm_stop_reason = "flip_completed"
                elif command_elapsed > rearm_started_s:
                    yaw_now_deg = math.degrees(previous_yaw_error)
                    if (
                        abs(yaw_now_deg) <= COUNTER_STOP_ABS_YAW_DEG
                        or yaw_now_deg * rearm_trigger_yaw_deg <= 0.0
                    ):
                        rearm_stopped_s = command_elapsed
                        rearm_stop_reason = "yaw_recovered_or_crossed_zero"
                if (
                    rearm_stopped_s is None
                    and command_elapsed >= rearm_started_s + COUNTER_YAW_DURATION_S
                ):
                    rearm_stopped_s = command_elapsed
                    rearm_stop_reason = "max_duration"
            if (
                ADAPTIVE_INITIAL_ENABLED
                and adaptive_response_at_s is None
                and 0.0 <= adaptive_elapsed < ADAPTIVE_MAX_DURATION_S
                and (
                    ADAPTIVE_MAX_ABS_IMPULSE_RAD is None
                    or adaptive_abs_impulse_rad < ADAPTIVE_MAX_ABS_IMPULSE_RAD
                )
            ):
                adaptive_step_count = 1 + int(adaptive_elapsed / ADAPTIVE_STEP_PERIOD_S)
                adaptive_last_rate_rad_s = adaptive_active_sign * min(
                    ADAPTIVE_MAX_RATE_RAD_S,
                    ADAPTIVE_INITIAL_RATE_RAD_S
                    + (adaptive_step_count - 1) * ADAPTIVE_RATE_STEP_RAD_S,
                )
                probe_active = (
                    ADAPTIVE_PULSE_ON_S is None
                    or adaptive_elapsed % ADAPTIVE_STEP_PERIOD_S < ADAPTIVE_PULSE_ON_S
                )
                if probe_active:
                    commanded_yaw_rate = adaptive_last_rate_rad_s
                    if ADAPTIVE_FIXED_PULSE_FALLBACK and adaptive_fallback_started_s is None:
                        adaptive_fallback_started_s = command_elapsed
                    if ADAPTIVE_MAX_ABS_IMPULSE_RAD is not None:
                        remaining = max(
                            0.0,
                            ADAPTIVE_MAX_ABS_IMPULSE_RAD - adaptive_abs_impulse_rad,
                        )
                        commanded_yaw_rate = adaptive_active_sign * min(
                            abs(commanded_yaw_rate), remaining / (2.0 * PHYSICS_DT_S)
                        )
                    command_phase = "reverse_adaptive_yaw_selection"
                else:
                    commanded_yaw_rate = 0.0
                    command_phase = "reverse_adaptive_probe_wait"
            elif command_elapsed < INITIAL_YAW_PULSE_DURATION_S:
                commanded_yaw_rate = INITIAL_YAW_PULSE_RATE_RAD_S
                command_phase = "reverse_initial_yaw_pulse"
            elif (
                BOOST_YAW_RATE_RAD_S != 0.0
                and BOOST_YAW_START_S <= command_elapsed
                < BOOST_YAW_START_S + BOOST_YAW_DURATION_S
            ):
                commanded_yaw_rate = BOOST_YAW_RATE_RAD_S
                command_phase = "reverse_yaw_step_up"
            elif (
                rearm_started_s is not None
                and rearm_stopped_s is None
                and command_elapsed < rearm_started_s + COUNTER_YAW_DURATION_S
            ):
                commanded_yaw_rate = rearm_actual_rate_rad_s
                command_phase = "reverse_counter_yaw_rearm_pulse"
            elif (
                COUNTER_YAW_RATE_RAD_S != 0.0
                and (
                    (
                        COUNTER_TRIGGER_ABS_YAW_DEG is not None
                        and counter_started_s is not None
                        and counter_stopped_s is None
                        and command_elapsed < counter_started_s + COUNTER_YAW_DURATION_S
                    )
                    or (
                        COUNTER_TRIGGER_ABS_YAW_DEG is None
                        and COUNTER_YAW_START_S <= command_elapsed
                        < COUNTER_YAW_START_S + COUNTER_YAW_DURATION_S
                    )
                )
            ):
                if counter_started_s is None:
                    counter_started_s = command_elapsed
                    counter_trigger_yaw_deg = math.degrees(previous_yaw_error)
                    counter_trigger_distance_m = max(0.0, -previous_longitudinal)
                    counter_trigger_direction_mode = direction_confirmed_mode
                    counter_trigger_direction_confirmed_s = direction_confirmed_at_s
                    counter_actual_rate_rad_s = COUNTER_YAW_RATE_RAD_S
                commanded_yaw_rate = counter_actual_rate_rad_s
                command_phase = "reverse_counter_yaw_pulse"
            elif YAW_HOLD_ENABLED and yaw_hold_active:
                yaw_hold_last_rate_rad_s = max(
                    -YAW_HOLD_MAX_RATE_RAD_S,
                    min(
                        YAW_HOLD_MAX_RATE_RAD_S,
                        -YAW_HOLD_KP_RAD_S_PER_RAD * previous_yaw_error,
                    ),
                )
                commanded_yaw_rate = yaw_hold_last_rate_rad_s
                command_phase = "reverse_measured_yaw_hold"
            else:
                commanded_yaw_rate = 0.0
                command_phase = "reverse_zero_yaw"
            if REVERSE_PROFILE == "accel_ramp":
                reference_speed = -min(
                    REVERSE_SPEED_M_S, REVERSE_ACCEL_LIMIT_M_S2 * command_elapsed
                )
            else:
                reference_speed = -REVERSE_SPEED_M_S
            if ADAPTIVE_INITIAL_ENABLED and ADAPTIVE_SELECTION_SPEED_M_S is not None:
                if adaptive_response_at_s is None:
                    reference_speed = -min(
                        ADAPTIVE_SELECTION_SPEED_M_S,
                        REVERSE_ACCEL_LIMIT_M_S2 * command_elapsed,
                    )
                elif ADAPTIVE_RAMP_AFTER_RESPONSE:
                    post_response_elapsed = max(
                        0.0, command_elapsed - adaptive_response_at_s
                    )
                    selection_speed_at_response = min(
                        ADAPTIVE_SELECTION_SPEED_M_S,
                        REVERSE_ACCEL_LIMIT_M_S2 * adaptive_response_at_s,
                    )
                    reference_speed = -min(
                        REVERSE_SPEED_M_S,
                        selection_speed_at_response
                        + REVERSE_ACCEL_LIMIT_M_S2 * post_response_elapsed,
                    )
            if INITIAL_YAW_HOLD_TRANSLATION and command_elapsed < INITIAL_YAW_PULSE_DURATION_S:
                reference_speed = 0.0
            velocity_before = np.asarray(robot.get_linear_velocity(), dtype=float)
            _, orientation_before = robot.get_world_pose()
            body_yaw_before = _yaw(orientation_before)
            actual_body_speed_before = (
                math.cos(body_yaw_before) * velocity_before[0]
                + math.sin(body_yaw_before) * velocity_before[1]
            )
            commanded_speed = reference_speed + REVERSE_BODY_SPEED_KP * (
                reference_speed - actual_body_speed_before
            )
            commanded_speed = max(
                -REVERSE_MAX_COMMAND_SPEED_M_S, min(0.0, commanded_speed)
            )
            target_drive_rates = command_twist(commanded_speed, commanded_yaw_rate)
            await app.next_update_async()

            elapsed = physics_time_s - reverse_start
            if command_phase == "reverse_adaptive_yaw_selection":
                adaptive_abs_impulse_rad += (
                    abs(commanded_yaw_rate) * max(0.0, elapsed - command_elapsed)
                )
                if (
                    adaptive_cap_hit_at_s is None
                    and ADAPTIVE_MAX_ABS_IMPULSE_RAD is not None
                    and adaptive_abs_impulse_rad
                    >= ADAPTIVE_MAX_ABS_IMPULSE_RAD - 1.0e-9
                ):
                    adaptive_cap_hit_at_s = elapsed
            if command_phase == "reverse_counter_yaw_pulse":
                counter_actual_applied_duration_s += max(0.0, elapsed - command_elapsed)
            elif command_phase == "reverse_counter_yaw_rearm_pulse":
                rearm_actual_applied_duration_s += max(0.0, elapsed - command_elapsed)
            elif command_phase == "reverse_measured_yaw_hold":
                yaw_hold_applied_duration_s += max(0.0, elapsed - command_elapsed)
            position, orientation = robot.get_world_pose()
            yaw_error = _wrap(_yaw(orientation) - reverse_yaw)
            longitudinal, lateral = _local_displacement(
                position, reverse_origin, reverse_yaw
            )
            reverse_distance = max(0.0, -longitudinal)
            q = np.asarray(robot.get_joint_positions(), dtype=float)
            qd = np.asarray(robot.get_joint_velocities(), dtype=float)
            world_velocity = np.asarray(robot.get_linear_velocity(), dtype=float)
            body_yaw = _yaw(orientation)
            actual_body_speed = (
                math.cos(body_yaw) * world_velocity[0]
                + math.sin(body_yaw) * world_velocity[1]
            )
            requested_drive_efforts = drive_dampings * (
                np.asarray(target_drive_rates, dtype=float) - qd[drive_indices]
            )
            measured_efforts = np.asarray(robot.get_measured_joint_efforts(), dtype=float)
            caster_q = q[caster_indices]
            caster_qd = qd[caster_indices]
            caster_errors = [_angle_error_deg(value, 0.0) for value in caster_q]
            caster_speeds_deg_s = [abs(math.degrees(value)) for value in caster_qd]
            forces = [
                np.asarray(view.get_net_contact_forces(dt=PHYSICS_DT_S), dtype=float)
                .reshape(-1, 3)[0]
                for view in contact_views
            ]
            direction_motion_history.append((elapsed, caster_q.copy()))
            while (
                len(direction_motion_history) > 1
                and direction_motion_history[1][0]
                <= elapsed - DIRECTION_CONFIRM_WINDOW_S
            ):
                direction_motion_history.popleft()
            recent_elapsed, recent_q = direction_motion_history[0]
            candidate_now = (
                _opposed_direction_candidate(
                    caster_q,
                    caster_start,
                    recent_q if elapsed - recent_elapsed >= DIRECTION_CONFIRM_WINDOW_S else None,
                    elapsed - recent_elapsed,
                )
                if all(abs(force[2]) >= CONTACT_LOSS_THRESHOLD_N for force in forces)
                else None
            )
            if candidate_now != direction_candidate_mode:
                direction_candidate_mode = candidate_now
                direction_candidate_since_s = elapsed if candidate_now is not None else None
                direction_confirmed_mode = None
                direction_confirmed_at_s = None
            if (
                candidate_now is not None
                and direction_confirmed_mode is None
                and elapsed - direction_candidate_since_s >= DIRECTION_CONFIRM_DWELL_S
            ):
                direction_confirmed_mode = candidate_now
                direction_confirmed_at_s = elapsed
                if first_direction_confirmed_s is None:
                    first_direction_confirmed_mode = candidate_now
                    first_direction_confirmed_s = elapsed
                    first_direction_confirmed_distance_m = reverse_distance

            for caster_index in range(2):
                caster_aligned = (
                    caster_errors[caster_index] <= FLIP_ALIGNMENT_TOL_DEG
                    and caster_speeds_deg_s[caster_index]
                    <= FLIP_SPEED_TOL_DEG_S
                )
                if caster_aligned:
                    if individual_aligned_since[caster_index] is None:
                        individual_aligned_since[caster_index] = elapsed
                        individual_aligned_distance[caster_index] = reverse_distance
                    elif (
                        individual_flip_time_s[caster_index] is None
                        and elapsed - individual_aligned_since[caster_index]
                        >= FLIP_DWELL_S
                    ):
                        individual_flip_time_s[caster_index] = float(
                            individual_aligned_since[caster_index]
                        )
                        individual_flip_distance_m[caster_index] = float(
                            individual_aligned_distance[caster_index]
                        )
                else:
                    individual_aligned_since[caster_index] = None
                    individual_aligned_distance[caster_index] = None

            if (
                max(caster_errors) <= FLIP_ALIGNMENT_TOL_DEG
                and max(caster_speeds_deg_s) <= FLIP_SPEED_TOL_DEG_S
            ):
                if aligned_since is None:
                    aligned_since = elapsed
                elif flip_time_s is None and elapsed - aligned_since >= FLIP_DWELL_S:
                    flip_time_s = float(aligned_since)
                    flip_distance_m = float(reverse_distance)
            else:
                aligned_since = None

            now = physics_time_s
            dt = max(now - previous_t, 1.0e-9)
            longitudinal_velocity = (longitudinal - previous_longitudinal) / dt
            lateral_velocity = (lateral - previous_lateral) / dt
            longitudinal_acceleration = (
                longitudinal_velocity - previous_longitudinal_velocity
            ) / dt
            lateral_acceleration = (
                lateral_velocity - previous_lateral_velocity
            ) / dt
            yaw_rate_deg_s = math.degrees(_wrap(yaw_error - previous_yaw_error)) / dt
            adaptive_motion_history.append((elapsed, math.degrees(yaw_error)))
            while (
                len(adaptive_motion_history) > 1
                and adaptive_motion_history[1][0]
                <= elapsed - ADAPTIVE_RESPONSE_WINDOW_S
            ):
                adaptive_motion_history.popleft()
            oldest_t, oldest_yaw_deg = adaptive_motion_history[0]
            adaptive_motion_rate_deg_s = (
                (math.degrees(yaw_error) - oldest_yaw_deg) / (elapsed - oldest_t)
                if elapsed - oldest_t >= 0.75 * ADAPTIVE_RESPONSE_WINDOW_S
                else 0.0
            )
            rows.append(
                {
                    "time_s": float(elapsed),
                    "repeat": repeat,
                    "phase": command_phase,
                    "reference_speed_m_s": reference_speed,
                    "commanded_speed_m_s": commanded_speed,
                    "actual_body_speed_m_s": actual_body_speed,
                    "body_speed_error_m_s": reference_speed - actual_body_speed,
                    "commanded_yaw_rate_rad_s": commanded_yaw_rate,
                    "adaptive_response_reached": adaptive_response_at_s is not None,
                    "adaptive_step_count": adaptive_step_count,
                    "adaptive_abs_impulse_rad": adaptive_abs_impulse_rad,
                    "adaptive_motion_rate_deg_s": adaptive_motion_rate_deg_s,
                    "longitudinal_m": longitudinal,
                    "reverse_distance_m": reverse_distance,
                    "lateral_m": lateral,
                    "yaw_error_deg": math.degrees(yaw_error),
                    "yaw_rate_deg_s": yaw_rate_deg_s,
                    "longitudinal_velocity_m_s": longitudinal_velocity,
                    "lateral_velocity_m_s": lateral_velocity,
                    "longitudinal_acceleration_m_s2_raw": longitudinal_acceleration,
                    "lateral_acceleration_m_s2_raw": lateral_acceleration,
                    "drive_left_velocity_rad_s": float(qd[drive_indices[0]]),
                    "drive_right_velocity_rad_s": float(qd[drive_indices[1]]),
                    "drive_left_target_velocity_rad_s": float(target_drive_rates[0]),
                    "drive_right_target_velocity_rad_s": float(target_drive_rates[1]),
                    "drive_left_requested_effort_est": float(requested_drive_efforts[0]),
                    "drive_right_requested_effort_est": float(requested_drive_efforts[1]),
                    "drive_left_effort_sim": float(measured_efforts[drive_indices[0]]),
                    "drive_right_effort_sim": float(measured_efforts[drive_indices[1]]),
                    "caster_left_angle_rad": float(caster_q[0]),
                    "caster_right_angle_rad": float(caster_q[1]),
                    "caster_left_velocity_rad_s": float(caster_qd[0]),
                    "caster_right_velocity_rad_s": float(caster_qd[1]),
                    "caster_left_reverse_error_deg": caster_errors[0],
                    "caster_right_reverse_error_deg": caster_errors[1],
                    "direction_candidate_mode": direction_candidate_mode,
                    "direction_confirmed_mode": direction_confirmed_mode,
                    "counter_actual_rate_rad_s": counter_actual_rate_rad_s,
                    "counter_stopped_s": counter_stopped_s,
                    "counter_stop_reason": counter_stop_reason,
                    "counter_rearm_started_s": rearm_started_s,
                    "counter_rearm_actual_rate_rad_s": rearm_actual_rate_rad_s,
                    "counter_rearm_stopped_s": rearm_stopped_s,
                    "counter_rearm_stop_reason": rearm_stop_reason,
                    "yaw_hold_active": yaw_hold_active,
                    "yaw_hold_rate_rad_s": (
                        yaw_hold_last_rate_rad_s if yaw_hold_active else 0.0
                    ),
                    "caster_left_Fz_N": float(forces[0][2]),
                    "caster_right_Fz_N": float(forces[1][2]),
                    "both_casters_flip_complete": flip_time_s is not None,
                }
            )
            previous_t = now
            previous_yaw_error = yaw_error
            previous_longitudinal = longitudinal
            previous_lateral = lateral
            previous_longitudinal_velocity = longitudinal_velocity
            previous_lateral_velocity = lateral_velocity

            if reverse_distance >= REVERSE_DISTANCE_M:
                break
            if time.perf_counter() - wall_start > WALL_TIMEOUT_S:
                raise RuntimeError("Wall-clock timeout during natural flip gate")

        command_twist(0.0)
        hold_start = physics_time_s
        while physics_time_s - hold_start < 0.5:
            command_twist(0.0)
            await app.next_update_async()

        if not rows:
            raise RuntimeError("Natural flip trial produced no samples")

        left_unwrapped = np.unwrap(
            np.asarray([row["caster_left_angle_rad"] for row in rows], dtype=float)
        )
        right_unwrapped = np.unwrap(
            np.asarray([row["caster_right_angle_rad"] for row in rows], dtype=float)
        )
        left_delta = float(left_unwrapped[-1] - left_unwrapped[0])
        right_delta = float(right_unwrapped[-1] - right_unwrapped[0])

        def direction(delta):
            if abs(delta) < math.radians(45.0):
                return "undetermined"
            return "positive" if delta > 0.0 else "negative"

        left_direction = direction(left_delta)
        right_direction = direction(right_delta)

        def flip_mode(left, right):
            # Both swivel joints use +Z axes and both wheel centers have a
            # local +X trail offset. Thus (+, -) swings the wheel centers
            # inward, whereas (-, +) swings them outward.
            modes = {
                ("positive", "negative"): "opposed_inward",
                ("negative", "positive"): "opposed_outward",
                ("positive", "positive"): "same_positive",
                ("negative", "negative"): "same_negative",
            }
            return modes.get((left, right), "undetermined")

        trace_path = OUTPUT_DIR / f"natural_flip_r{repeat:02d}.csv"
        with trace_path.open("w", newline="", encoding="utf-8") as stream:
            writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
            writer.writeheader()
            writer.writerows(rows)

        caster_left_fz = np.asarray([row["caster_left_Fz_N"] for row in rows])
        caster_right_fz = np.asarray([row["caster_right_Fz_N"] for row in rows])
        ramp_duration_s = (
            REVERSE_SPEED_M_S / REVERSE_ACCEL_LIMIT_M_S2
            if REVERSE_PROFILE == "accel_ramp" else 0.0
        )
        steady_rows = [
            row for row in rows
            if row["time_s"] >= ramp_duration_s + 0.50
        ]
        ramp_rows = [
            row for row in rows
            if REVERSE_PROFILE == "accel_ramp"
            and 0.25 <= row["time_s"] <= max(0.25, ramp_duration_s - 0.25)
        ]
        mean_actual_body_speed = (
            float(np.mean([row["actual_body_speed_m_s"] for row in steady_rows]))
            if steady_rows else None
        )
        steady_body_speed_rms_error = (
            math.sqrt(float(np.mean([
                row["body_speed_error_m_s"] ** 2 for row in steady_rows
            ]))) if steady_rows else None
        )
        ramp_body_speed_rms_error = (
            math.sqrt(float(np.mean([
                row["body_speed_error_m_s"] ** 2 for row in ramp_rows
            ]))) if ramp_rows else None
        )
        achieved_ramp_accel = (
            float(np.polyfit(
                [row["time_s"] for row in ramp_rows],
                [row["actual_body_speed_m_s"] for row in ramp_rows],
                1,
            )[0]) if len(ramp_rows) >= 2 else None
        )
        requested_drive_efforts = [
            row[key] for row in rows
            for key in (
                "drive_left_requested_effort_est",
                "drive_right_requested_effort_est",
            )
        ]
        drive_saturation_fraction = sum(
            abs(value) >= DRIVE_MAX_FORCE for value in requested_drive_efforts
        ) / len(requested_drive_efforts)
        longest_saturation_steps = 0
        current_saturation_steps = 0
        for row in rows:
            saturated = any(
                abs(row[key]) >= DRIVE_MAX_FORCE
                for key in (
                    "drive_left_requested_effort_est",
                    "drive_right_requested_effort_est",
                )
            )
            current_saturation_steps = current_saturation_steps + 1 if saturated else 0
            longest_saturation_steps = max(
                longest_saturation_steps, current_saturation_steps
            )
        pre_flip_rows = (
            [row for row in rows if row["time_s"] <= flip_time_s]
            if flip_time_s is not None
            else rows
        )
        summary = {
            "repeat": repeat,
            "analysis_role": "warmup" if repeat <= WARMUP_REPEATS else "recorded",
            "run_label": RUN_LABEL,
            "reverse_start_mode": (
                "controlled" if CONTROLLED_REVERSE_START_DEG is not None else "natural"
            ),
            "controlled_reverse_start_left_deg": (
                CONTROLLED_REVERSE_START_DEG[0]
                if CONTROLLED_REVERSE_START_DEG is not None else None
            ),
            "controlled_reverse_start_right_deg": (
                CONTROLLED_REVERSE_START_DEG[1]
                if CONTROLLED_REVERSE_START_DEG is not None else None
            ),
            "load_condition": LOAD_CONDITION,
            "target_caster_load_share": TARGET_CASTER_SHARE,
            "static_friction": STATIC_FRICTION,
            "dynamic_friction": DYNAMIC_FRICTION,
            "forward_speed_m_s": FORWARD_SPEED_M_S,
            "forward_min_distance_m": FORWARD_DISTANCE_M,
            "forward_actual_distance_m": forward_longitudinal,
            "controlled_start_settle_s": CONTROLLED_START_SETTLE_S,
            "pre_reapply_caster_left_deg": pre_reapply_caster_deg[0],
            "pre_reapply_caster_right_deg": pre_reapply_caster_deg[1],
            "pre_reapply_base_z_m": pre_reapply_base_z_m,
            "reverse_start_base_z_m": float(base_start_position[2]),
            "reverse_start_base_roll_deg": base_start_roll_deg,
            "reverse_start_base_pitch_deg": base_start_pitch_deg,
            "reverse_start_base_yaw_deg": math.degrees(_yaw(base_start_orientation)),
            "reverse_start_drive_left_position_rad": float(q_start[drive_indices[0]]),
            "reverse_start_drive_right_position_rad": float(q_start[drive_indices[1]]),
            "reverse_start_drive_left_velocity_rad_s": float(qd_start[drive_indices[0]]),
            "reverse_start_drive_right_velocity_rad_s": float(qd_start[drive_indices[1]]),
            "pre_reverse_last_step_caster_left_Fz_N": caster_start_fz[0],
            "pre_reverse_last_step_caster_right_Fz_N": caster_start_fz[1],
            "preparation_seed_deg": PREPARATION_SEED_DEG,
            "reverse_speed_m_s": REVERSE_SPEED_M_S,
            "reverse_profile": REVERSE_PROFILE,
            "reverse_body_speed_kp": REVERSE_BODY_SPEED_KP,
            "reverse_max_command_speed_m_s": REVERSE_MAX_COMMAND_SPEED_M_S,
            "reverse_accel_limit_m_s2": (
                REVERSE_ACCEL_LIMIT_M_S2 if REVERSE_PROFILE == "accel_ramp" else None
            ),
            "mean_actual_body_speed_m_s": mean_actual_body_speed,
            "steady_body_speed_rms_error_m_s": steady_body_speed_rms_error,
            "ramp_body_speed_rms_error_m_s": ramp_body_speed_rms_error,
            "achieved_ramp_accel_m_s2": achieved_ramp_accel,
            "peak_abs_requested_drive_effort_est": max(
                abs(value) for value in requested_drive_efforts
            ),
            "drive_effort_saturation_fraction_est": drive_saturation_fraction,
            "max_consecutive_drive_saturation_s_est": (
                longest_saturation_steps * PHYSICS_DT_S
            ),
            "reverse_target_distance_m": REVERSE_DISTANCE_M,
            "initial_yaw_pulse_rate_rad_s": INITIAL_YAW_PULSE_RATE_RAD_S,
            "initial_yaw_pulse_duration_s": INITIAL_YAW_PULSE_DURATION_S,
            "initial_yaw_hold_translation": INITIAL_YAW_HOLD_TRANSLATION,
            "initial_yaw_pulse_signed_impulse_rad": (
                INITIAL_YAW_PULSE_RATE_RAD_S * INITIAL_YAW_PULSE_DURATION_S
            ),
            "initial_yaw_pulse_abs_impulse_rad": (
                abs(INITIAL_YAW_PULSE_RATE_RAD_S) * INITIAL_YAW_PULSE_DURATION_S
            ),
            "boost_yaw_rate_rad_s": BOOST_YAW_RATE_RAD_S,
            "boost_yaw_start_s": BOOST_YAW_START_S if BOOST_YAW_RATE_RAD_S != 0.0 else None,
            "boost_yaw_duration_s": BOOST_YAW_DURATION_S if BOOST_YAW_RATE_RAD_S != 0.0 else 0.0,
            "boost_yaw_signed_impulse_rad": BOOST_YAW_RATE_RAD_S * BOOST_YAW_DURATION_S,
            "adaptive_initial_enabled": ADAPTIVE_INITIAL_ENABLED,
            "adaptive_fixed_pulse_fallback": ADAPTIVE_FIXED_PULSE_FALLBACK,
            "adaptive_start_s": ADAPTIVE_START_S if ADAPTIVE_INITIAL_ENABLED else None,
            "adaptive_fallback_started_s": adaptive_fallback_started_s,
            "adaptive_pulse_on_s": ADAPTIVE_PULSE_ON_S if ADAPTIVE_INITIAL_ENABLED else None,
            "adaptive_response_mode": (
                ADAPTIVE_RESPONSE_MODE if ADAPTIVE_INITIAL_ENABLED else None
            ),
            "adaptive_initial_sign": ADAPTIVE_INITIAL_SIGN if ADAPTIVE_INITIAL_ENABLED else None,
            "adaptive_response_yaw_threshold_deg": (
                ADAPTIVE_RESPONSE_YAW_DEG if ADAPTIVE_INITIAL_ENABLED else None
            ),
            "adaptive_response_reached": adaptive_response_at_s is not None,
            "adaptive_response_rate_threshold_deg_s": (
                ADAPTIVE_RESPONSE_RATE_DEG_S
                if ADAPTIVE_INITIAL_ENABLED
                and ADAPTIVE_RESPONSE_MODE in (
                    "signed_yaw_motion", "body_yaw_filtered_and"
                ) else None
            ),
            "adaptive_response_dwell_s": (
                ADAPTIVE_RESPONSE_DWELL_S
                if ADAPTIVE_INITIAL_ENABLED
                and ADAPTIVE_RESPONSE_MODE in (
                    "signed_yaw_motion", "body_yaw_filtered_and"
                ) else None
            ),
            "adaptive_response_at_s": adaptive_response_at_s,
            "adaptive_last_rate_rad_s": (
                adaptive_last_rate_rad_s if ADAPTIVE_INITIAL_ENABLED else None
            ),
            "adaptive_step_count": adaptive_step_count if ADAPTIVE_INITIAL_ENABLED else None,
            "adaptive_abs_impulse_rad": (
                adaptive_abs_impulse_rad if ADAPTIVE_INITIAL_ENABLED else None
            ),
            "adaptive_max_abs_impulse_rad": (
                ADAPTIVE_MAX_ABS_IMPULSE_RAD if ADAPTIVE_INITIAL_ENABLED else None
            ),
            "adaptive_stopped_by_impulse_cap": (
                adaptive_cap_hit_at_s is not None
            ) if ADAPTIVE_INITIAL_ENABLED else None,
            "adaptive_cap_hit_at_s": (
                adaptive_cap_hit_at_s if ADAPTIVE_INITIAL_ENABLED else None
            ),
            "adaptive_selection_speed_m_s": (
                ADAPTIVE_SELECTION_SPEED_M_S if ADAPTIVE_INITIAL_ENABLED else None
            ),
            "adaptive_ramp_after_response": (
                ADAPTIVE_RAMP_AFTER_RESPONSE if ADAPTIVE_INITIAL_ENABLED else None
            ),
            "adaptive_reverse_on_wrong_direction": (
                ADAPTIVE_REVERSE_ON_WRONG_DIRECTION if ADAPTIVE_INITIAL_ENABLED else None
            ),
            "adaptive_reverse_after_s": (
                ADAPTIVE_REVERSE_AFTER_S if ADAPTIVE_INITIAL_ENABLED else None
            ),
            "adaptive_sign_switch_count": (
                adaptive_sign_switch_count if ADAPTIVE_INITIAL_ENABLED else None
            ),
            "counter_yaw_rate_rad_s": COUNTER_YAW_RATE_RAD_S,
            "counter_yaw_start_s": (
                COUNTER_YAW_START_S
                if COUNTER_YAW_RATE_RAD_S != 0.0 and COUNTER_TRIGGER_ABS_YAW_DEG is None
                else None
            ),
            "counter_yaw_duration_s": COUNTER_YAW_DURATION_S if COUNTER_YAW_RATE_RAD_S != 0.0 else 0.0,
            "counter_yaw_signed_impulse_rad": (
                counter_actual_rate_rad_s * COUNTER_YAW_DURATION_S
                if counter_started_s is not None else 0.0
            ),
            "counter_trigger_abs_yaw_deg": COUNTER_TRIGGER_ABS_YAW_DEG,
            "counter_trigger_min_time_s": (
                COUNTER_TRIGGER_MIN_TIME_S if COUNTER_TRIGGER_ABS_YAW_DEG is not None else None
            ),
            "counter_require_adaptive_response": COUNTER_REQUIRE_ADAPTIVE_RESPONSE,
            "counter_actual_start_s": counter_started_s,
            "counter_actual_rate_rad_s": counter_actual_rate_rad_s,
            "counter_actual_applied_duration_s": counter_actual_applied_duration_s,
            "counter_actual_signed_impulse_rad": (
                counter_actual_rate_rad_s * counter_actual_applied_duration_s
                if counter_actual_rate_rad_s is not None else 0.0
            ),
            "counter_stop_on_yaw_recovery": COUNTER_STOP_ON_YAW_RECOVERY,
            "counter_stop_abs_yaw_deg": (
                COUNTER_STOP_ABS_YAW_DEG if COUNTER_STOP_ON_YAW_RECOVERY else None
            ),
            "counter_stopped_s": counter_stopped_s,
            "counter_stop_reason": counter_stop_reason,
            "counter_trigger_yaw_deg": counter_trigger_yaw_deg,
            "counter_trigger_reverse_distance_m": counter_trigger_distance_m,
            "counter_require_direction_confirmed": COUNTER_REQUIRE_DIRECTION_CONFIRMED,
            "counter_use_measured_yaw_sign": COUNTER_USE_MEASURED_YAW_SIGN,
            "counter_target_direction_mode": COUNTER_TARGET_DIRECTION_MODE,
            "counter_rearm_enabled": COUNTER_REARM_ENABLED,
            "yaw_hold_enabled": YAW_HOLD_ENABLED,
            "yaw_hold_response_latch": YAW_HOLD_RESPONSE_LATCH,
            "yaw_hold_after_adaptive_stop": YAW_HOLD_AFTER_ADAPTIVE_STOP,
            "yaw_hold_require_flip_complete": YAW_HOLD_REQUIRE_FLIP_COMPLETE,
            "yaw_hold_start_s": YAW_HOLD_START_S if YAW_HOLD_ENABLED else None,
            "yaw_hold_trigger_abs_deg": (
                YAW_HOLD_TRIGGER_ABS_DEG if YAW_HOLD_ENABLED else None
            ),
            "yaw_hold_trigger_dwell_s": (
                YAW_HOLD_TRIGGER_DWELL_S if YAW_HOLD_ENABLED else None
            ),
            "yaw_hold_stop_abs_deg": (
                YAW_HOLD_STOP_ABS_DEG if YAW_HOLD_ENABLED else None
            ),
            "yaw_hold_kp_rad_s_per_rad": (
                YAW_HOLD_KP_RAD_S_PER_RAD if YAW_HOLD_ENABLED else None
            ),
            "yaw_hold_max_rate_rad_s": (
                YAW_HOLD_MAX_RATE_RAD_S if YAW_HOLD_ENABLED else None
            ),
            "yaw_hold_first_started_s": yaw_hold_first_started_s,
            "yaw_hold_activation_count": yaw_hold_activation_count,
            "yaw_hold_applied_duration_s": yaw_hold_applied_duration_s,
            "counter_rearm_trigger_abs_yaw_deg": (
                COUNTER_REARM_TRIGGER_ABS_YAW_DEG if COUNTER_REARM_ENABLED else None
            ),
            "counter_rearm_min_gap_s": (
                COUNTER_REARM_MIN_GAP_S if COUNTER_REARM_ENABLED else None
            ),
            "counter_rearm_started_s": rearm_started_s,
            "counter_rearm_trigger_yaw_deg": rearm_trigger_yaw_deg,
            "counter_rearm_trigger_reverse_distance_m": rearm_trigger_distance_m,
            "counter_rearm_actual_rate_rad_s": rearm_actual_rate_rad_s,
            "counter_rearm_actual_applied_duration_s": rearm_actual_applied_duration_s,
            "counter_rearm_actual_signed_impulse_rad": (
                rearm_actual_rate_rad_s * rearm_actual_applied_duration_s
                if rearm_actual_rate_rad_s is not None else 0.0
            ),
            "counter_rearm_stopped_s": rearm_stopped_s,
            "counter_rearm_stop_reason": rearm_stop_reason,
            "counter_rearm_before_flip": (
                rearm_started_s < flip_time_s
                if rearm_started_s is not None and flip_time_s is not None else None
            ),
            "counter_rearm_rate_opposes_trigger_yaw": (
                rearm_actual_rate_rad_s * rearm_trigger_yaw_deg < 0.0
                if rearm_started_s is not None else None
            ),
            "direction_confirm_min_rotation_deg": DIRECTION_CONFIRM_MIN_ROTATION_DEG,
            "direction_confirm_min_speed_deg_s": DIRECTION_CONFIRM_MIN_SPEED_DEG_S,
            "direction_confirm_window_s": DIRECTION_CONFIRM_WINDOW_S,
            "direction_confirm_dwell_s": DIRECTION_CONFIRM_DWELL_S,
            "first_direction_confirmed_mode": first_direction_confirmed_mode,
            "first_direction_confirmed_s": first_direction_confirmed_s,
            "first_direction_confirmed_distance_m": first_direction_confirmed_distance_m,
            "counter_trigger_direction_mode": counter_trigger_direction_mode,
            "counter_trigger_direction_confirmed_s": counter_trigger_direction_confirmed_s,
            "direction_confirmed_mode_matches_final": (
                first_direction_confirmed_mode == flip_mode(left_direction, right_direction)
                if first_direction_confirmed_mode is not None else None
            ),
            "counter_trigger_mode_matches_final": (
                counter_trigger_direction_mode == flip_mode(left_direction, right_direction)
                if counter_trigger_direction_mode is not None else None
            ),
            "counter_started_before_flip": (
                counter_started_s < flip_time_s
                if counter_started_s is not None and flip_time_s is not None else None
            ),
            "counter_started_after_direction_confirmation": (
                counter_started_s is not None
                and counter_trigger_direction_confirmed_s is not None
                and counter_started_s > counter_trigger_direction_confirmed_s
                and counter_trigger_direction_mode == COUNTER_TARGET_DIRECTION_MODE
            ) if COUNTER_REQUIRE_DIRECTION_CONFIRMED else None,
            "counter_rate_opposes_trigger_yaw": (
                counter_actual_rate_rad_s * counter_trigger_yaw_deg < 0.0
                if counter_started_s is not None else None
            ),
            "caster_left_start_deg": math.degrees(caster_start[0]),
            "caster_right_start_deg": math.degrees(caster_start[1]),
            "caster_left_start_velocity_deg_s": math.degrees(qd_start[caster_indices[0]]),
            "caster_right_start_velocity_deg_s": math.degrees(qd_start[caster_indices[1]]),
            "caster_left_flip_direction": left_direction,
            "caster_right_flip_direction": right_direction,
            "caster_flip_mode": flip_mode(left_direction, right_direction),
            "caster_left_unwrapped_delta_deg": math.degrees(left_delta),
            "caster_right_unwrapped_delta_deg": math.degrees(right_delta),
            "caster_left_flip_time_s": individual_flip_time_s[0],
            "caster_right_flip_time_s": individual_flip_time_s[1],
            "caster_left_flip_distance_m": individual_flip_distance_m[0],
            "caster_right_flip_distance_m": individual_flip_distance_m[1],
            "flip_completed": flip_time_s is not None,
            "flip_time_s": flip_time_s,
            "flip_distance_m": flip_distance_m,
            "final_reverse_distance_m": rows[-1]["reverse_distance_m"],
            "max_abs_lateral_before_flip_mm": 1000.0 * max(
                abs(row["lateral_m"]) for row in pre_flip_rows
            ),
            "max_abs_yaw_before_flip_deg": max(
                abs(row["yaw_error_deg"]) for row in pre_flip_rows
            ),
            "max_abs_lateral_mm": 1000.0 * max(abs(row["lateral_m"]) for row in rows),
            "max_abs_yaw_error_deg": max(abs(row["yaw_error_deg"]) for row in rows),
            "peak_abs_yaw_rate_deg_s": max(abs(row["yaw_rate_deg_s"]) for row in rows),
            "peak_abs_lateral_acceleration_m_s2_raw": max(
                abs(row["lateral_acceleration_m_s2_raw"]) for row in rows
            ),
            "caster_left_mean_Fz_N": float(np.mean(caster_left_fz)),
            "caster_right_mean_Fz_N": float(np.mean(caster_right_fz)),
            "caster_left_Fz_std_N": float(np.std(caster_left_fz)),
            "caster_right_Fz_std_N": float(np.std(caster_right_fz)),
            "caster_left_contact_loss_fraction": float(
                np.mean(np.abs(caster_left_fz) < CONTACT_LOSS_THRESHOLD_N)
            ),
            "caster_right_contact_loss_fraction": float(
                np.mean(np.abs(caster_right_fz) < CONTACT_LOSS_THRESHOLD_N)
            ),
        }
        summaries.append(summary)

    timeline.pause()
    summary_path = OUTPUT_DIR / "summary.csv"
    with summary_path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(summaries[0]))
        writer.writeheader()
        writer.writerows(summaries)

    print("=== Measured wheelchair natural caster-flip gate ===")
    print("run label:", RUN_LABEL)
    print("load condition/target caster share:", LOAD_CONDITION, TARGET_CASTER_SHARE)
    print("static/dynamic friction:", STATIC_FRICTION, DYNAMIC_FRICTION)
    print("reverse speed profile / target / accel limit:",
          REVERSE_PROFILE, REVERSE_SPEED_M_S,
          REVERSE_ACCEL_LIMIT_M_S2 if REVERSE_PROFILE == "accel_ramp" else None)
    print(
        "reverse body-speed Kp / max command:",
        REVERSE_BODY_SPEED_KP, REVERSE_MAX_COMMAND_SPEED_M_S,
    )
    print("adaptive selection / response reached:", ADAPTIVE_INITIAL_ENABLED,
          [row["adaptive_response_reached"] for row in summaries])
    print("adaptive step probe-on duration [s]:", ADAPTIVE_PULSE_ON_S)
    if ADAPTIVE_FIXED_PULSE_FALLBACK:
        print("fixed-pulse fallback / adaptive start / actually used:",
              True, ADAPTIVE_START_S,
              [row["adaptive_fallback_started_s"] is not None for row in summaries])
    print(
        "adaptive selection creep / ramp after response:",
        ADAPTIVE_SELECTION_SPEED_M_S, ADAPTIVE_RAMP_AFTER_RESPONSE,
    )
    print("adaptive yaw sign switches / timeout:", ADAPTIVE_REVERSE_AFTER_S,
          [row["adaptive_sign_switch_count"] for row in summaries])
    print(
        "sequence: forward -> stop/settle -> reverse with initial yaw pulse"
        " -> optional yaw step-up"
        " -> optional counter-yaw/measured-yaw hold -> zero yaw"
    )
    print(
        "initial yaw pulse rate/duration/signed impulse:",
        INITIAL_YAW_PULSE_RATE_RAD_S,
        INITIAL_YAW_PULSE_DURATION_S,
        INITIAL_YAW_PULSE_RATE_RAD_S * INITIAL_YAW_PULSE_DURATION_S,
    )
    print("initial yaw pulse holds translation:", INITIAL_YAW_HOLD_TRANSLATION)
    print(
        "yaw step-up rate/start/duration [rad/s, s, s]:",
        BOOST_YAW_RATE_RAD_S, BOOST_YAW_START_S, BOOST_YAW_DURATION_S,
    )
    print(
        "counter-yaw rate/start/duration [rad/s, s, s]:",
        COUNTER_YAW_RATE_RAD_S, COUNTER_YAW_START_S, COUNTER_YAW_DURATION_S,
    )
    print(
        "response trigger abs yaw/min time [deg, s]:",
        COUNTER_TRIGGER_ABS_YAW_DEG, COUNTER_TRIGGER_MIN_TIME_S,
    )
    print("counter requires adaptive response:", COUNTER_REQUIRE_ADAPTIVE_RESPONSE)
    print(
        "direction-confirmed counter / target / measured-yaw sign:",
        COUNTER_REQUIRE_DIRECTION_CONFIRMED,
        COUNTER_TARGET_DIRECTION_MODE,
        COUNTER_USE_MEASURED_YAW_SIGN,
    )
    print(
        "counter stop on yaw recovery / tolerance [deg]:",
        COUNTER_STOP_ON_YAW_RECOVERY,
        COUNTER_STOP_ABS_YAW_DEG if COUNTER_STOP_ON_YAW_RECOVERY else None,
    )
    print(
        "counter rearm enabled / yaw trigger / minimum gap:",
        COUNTER_REARM_ENABLED,
        COUNTER_REARM_TRIGGER_ABS_YAW_DEG if COUNTER_REARM_ENABLED else None,
        COUNTER_REARM_MIN_GAP_S if COUNTER_REARM_ENABLED else None,
    )
    print(
        "measured-yaw hold / trigger-stop / kp-max:",
        YAW_HOLD_ENABLED,
        (YAW_HOLD_TRIGGER_ABS_DEG, YAW_HOLD_STOP_ABS_DEG),
        (YAW_HOLD_KP_RAD_S_PER_RAD, YAW_HOLD_MAX_RATE_RAD_S),
    )
    if YAW_HOLD_ENABLED:
        print("measured-yaw hold online inputs: elapsed time and body yaw error only")
        if YAW_HOLD_RESPONSE_LATCH:
            print("measured-yaw hold: latched immediately after adaptive response")
    if COUNTER_REQUIRE_DIRECTION_CONFIRMED:
        print(
            "direction confirmation min rotation/windowed speed/window/dwell:",
            DIRECTION_CONFIRM_MIN_ROTATION_DEG,
            DIRECTION_CONFIRM_MIN_SPEED_DEG_S,
            DIRECTION_CONFIRM_WINDOW_S,
            DIRECTION_CONFIRM_DWELL_S,
        )
    print("warmup/recorded repeats:", WARMUP_REPEATS, REPEATS - WARMUP_REPEATS)
    print(
        "reverse start mode/controlled angles [deg]:",
        "controlled" if CONTROLLED_REVERSE_START_DEG is not None else "natural",
        CONTROLLED_REVERSE_START_DEG,
    )
    for row in summaries:
        print(
            f"r{row['repeat']:02d}",
            "start caster [deg]=",
            (round(row["caster_left_start_deg"], 3), round(row["caster_right_start_deg"], 3)),
            "direction=",
            (row["caster_left_flip_direction"], row["caster_right_flip_direction"]),
            "mode=", row["caster_flip_mode"],
            "flip time/distance=",
            (row["flip_time_s"], row["flip_distance_m"]),
            "max lat before/full [mm]=",
            (
                round(row["max_abs_lateral_before_flip_mm"], 3),
                round(row["max_abs_lateral_mm"], 3),
            ),
            "max yaw before/full [deg]=",
            (
                round(row["max_abs_yaw_before_flip_deg"], 4),
                round(row["max_abs_yaw_error_deg"], 4),
            ),
            "contact loss=",
            (
                round(row["caster_left_contact_loss_fraction"], 4),
                round(row["caster_right_contact_loss_fraction"], 4),
            ),
            "body speed/accel/peak effort/saturation/max consecutive sat [s]=",
            (
                round(row["mean_actual_body_speed_m_s"], 6)
                if row["mean_actual_body_speed_m_s"] is not None else None,
                round(row["achieved_ramp_accel_m_s2"], 6)
                if row["achieved_ramp_accel_m_s2"] is not None else None,
                round(row["peak_abs_requested_drive_effort_est"], 3),
                round(row["drive_effort_saturation_fraction_est"], 4),
                round(row["max_consecutive_drive_saturation_s_est"], 4),
            ),
            "completed=", row["flip_completed"],
            "counter actual start/trigger yaw=",
            (row["counter_actual_start_s"], row["counter_trigger_yaw_deg"]),
            "direction confirmation/mode=",
            (row["first_direction_confirmed_s"], row["first_direction_confirmed_mode"]),
            "counter actual rate/opposes yaw=",
            (row["counter_actual_rate_rad_s"], row["counter_rate_opposes_trigger_yaw"]),
            "counter stop/reason/applied duration=",
            (
                row["counter_stopped_s"],
                row["counter_stop_reason"],
                row["counter_actual_applied_duration_s"],
            ),
            "counter rearm start/stop/rate/duration=",
            (
                row["counter_rearm_started_s"],
                row["counter_rearm_stopped_s"],
                row["counter_rearm_actual_rate_rad_s"],
                row["counter_rearm_actual_applied_duration_s"],
            ) if COUNTER_REARM_ENABLED else None,
            "confirmation/trigger match final, counter before flip=",
            (
                row["direction_confirmed_mode_matches_final"],
                row["counter_trigger_mode_matches_final"],
                row["counter_started_before_flip"],
            ),
        )
    recorded_modes = [
        row["caster_flip_mode"]
        for row in summaries
        if row["analysis_role"] == "recorded"
    ]
    print(
        "recorded mode consistency:",
        f"{max(recorded_modes.count(mode) for mode in set(recorded_modes))}/{len(recorded_modes)}",
        sorted(set(recorded_modes)),
    )
    print("saved:", summary_path)
    print("trace also records actual drive-wheel speeds and simulated measured joint efforts.")
    if YAW_HOLD_ENABLED:
        if ADAPTIVE_INITIAL_ENABLED and ADAPTIVE_RESPONSE_MODE in ("caster_direction", "caster_any_opposed"):
            print("MEASURED-YAW HOLD PILOT — hold uses body yaw; adaptive selection uses simulated caster-angle state online.")
        elif YAW_HOLD_REQUIRE_FLIP_COMPLETE:
            print("MEASURED-YAW HOLD PILOT — hold activation uses simulated caster-angle flip completion online.")
        else:
            print("MEASURED-YAW HOLD PILOT — online yaw hold uses body yaw only.")
    elif ADAPTIVE_INITIAL_ENABLED:
        print(
            "RESPONSE-STEP INITIAL YAW — eval_v1 prespecified trial; "
            "retain failures and do not claim overall performance from one run."
            if "_eval_v1_" in RUN_LABEL
            else "RESPONSE-STEP INITIAL YAW SCREEN — pilot parameters; not an evaluated result."
        )
    elif INITIAL_YAW_PULSE_RATE_RAD_S == 0.0 or INITIAL_YAW_PULSE_DURATION_S == 0.0:
        print("DIAGNOSTIC BASELINE ONLY — no yaw-bias or compensation controller was used.")
    elif BOOST_YAW_RATE_RAD_S != 0.0 and COUNTER_YAW_RATE_RAD_S == 0.0:
        print("FIXED STEP-UP FEASIBILITY SCREEN — no response-triggered increment yet.")
    elif COUNTER_YAW_RATE_RAD_S == 0.0:
        print(
            "DIRECTION-SELECTION SCREEN — initial yaw pulse only; "
            "no counter-yaw compensation."
        )
    else:
        print(
            "DIRECTION-CONFIRMED RESPONSE COUNTER-YAW SCREEN — initial selection command is still fixed."
            if COUNTER_REQUIRE_DIRECTION_CONFIRMED
            else "RESPONSE-TRIGGERED COUNTER-YAW SCREEN — initial selection command is still fixed."
            if COUNTER_TRIGGER_ABS_YAW_DEG is not None
            else "FIXED TWO-PULSE FEASIBILITY SCREEN — not yet a response-adaptive controller."
        )
    print("Timeline paused automatically. Press Stop after copying the result.")


def _done(task):
    try:
        task.result()
    except asyncio.CancelledError:
        print("Measured wheelchair natural caster-flip gate cancelled.")
    except Exception as error:
        if timeline.is_playing():
            timeline.pause()
        print(
            "Measured wheelchair natural caster-flip gate aborted: "
            f"{type(error).__name__}: {error}"
        )


old_task = globals().get("wheelchair_lab_natural_flip_gate_task")
if old_task is not None and not old_task.done():
    old_task.cancel()
wheelchair_lab_natural_flip_gate_task = asyncio.ensure_future(_run())
wheelchair_lab_natural_flip_gate_task.add_done_callback(_done)
print("Measured wheelchair natural caster-flip gate scheduled; do not use Timeline controls.")
