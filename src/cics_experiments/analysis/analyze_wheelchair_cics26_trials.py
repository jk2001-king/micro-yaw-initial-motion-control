"""Read-only common-window metrics for CICS26 wheelchair trial directories.

Usage: python3 analyze_wheelchair_cics26_trials.py [--trial-prefix eval_v1]
Only cics26_main_* runs are included. A run without 0.80 m completion remains
censored; it is not silently dropped from the outcome table.
"""

from cics_experiments.paths import workspace_root, resolve_path

import argparse
import csv
import math
import re
import sys
from collections import defaultdict
from pathlib import Path

import numpy as np
from scipy.signal import butter, sosfiltfilt


ROOT = workspace_root()
RESULTS = resolve_path('studies/CICS26/results/wheelchair_lab/natural_flip_gate')
NAME = re.compile(r"^cics26_main_(40p0|90p0|127p5)kg_(B0|B1|B2|B3|P)_(.+)$")
WINDOW_M = 0.80
SAMPLE_HZ = 120.0
JERK_CUTOFF_HZ = 5.0
GRID_M = np.linspace(0.0, WINDOW_M, 81)


def _read_csv(path):
    with path.open(newline="", encoding="utf-8") as stream:
        return list(csv.DictReader(stream))


def _number(row, key):
    value = row.get(key)
    return float(value) if value not in (None, "", "None") else math.nan


def _trajectory(distance, lateral):
    progress = np.maximum.accumulate(distance)
    unique, indexes = np.unique(progress, return_index=True)
    if unique[0] > 0.01 or unique[-1] < WINDOW_M:
        return None
    return np.interp(GRID_M, unique, lateral[indexes])


def _jerk(time, vx, vy, end_time):
    keep = time <= end_time
    time, vx, vy = time[keep], vx[keep], vy[keep]
    unique, indexes = np.unique(time, return_index=True)
    if len(unique) < 60:
        return math.nan, math.nan
    grid = np.arange(unique[0], unique[-1], 1.0 / SAMPLE_HZ)
    if len(grid) < 60:
        return math.nan, math.nan
    sos = butter(4, JERK_CUTOFF_HZ, fs=SAMPLE_HZ, output="sos")
    vx_filtered = sosfiltfilt(sos, np.interp(grid, unique, vx[indexes]))
    vy_filtered = sosfiltfilt(sos, np.interp(grid, unique, vy[indexes]))
    ax = np.gradient(vx_filtered, 1.0 / SAMPLE_HZ)
    ay = np.gradient(vy_filtered, 1.0 / SAMPLE_HZ)
    jerk = np.hypot(
        np.gradient(ax, 1.0 / SAMPLE_HZ),
        np.gradient(ay, 1.0 / SAMPLE_HZ),
    )
    return float(np.max(jerk)), float(np.sqrt(np.mean(jerk * jerk)))


def analyze(directory):
    match = NAME.match(directory.name)
    if match is None:
        return None
    summary_path = directory / "summary.csv"
    trace_path = directory / "natural_flip_r01.csv"
    if not summary_path.exists() or not trace_path.exists():
        return None
    summary_rows = _read_csv(summary_path)
    trace_rows = _read_csv(trace_path)
    if len(summary_rows) != 1 or not trace_rows:
        raise ValueError(f"Expected one completed trial in {directory}")
    summary = summary_rows[0]
    time = np.array([_number(row, "time_s") for row in trace_rows])
    distance = np.array([_number(row, "reverse_distance_m") for row in trace_rows])
    lateral = np.array([_number(row, "lateral_m") for row in trace_rows])
    yaw = np.array([_number(row, "yaw_error_deg") for row in trace_rows])
    vx = np.array([_number(row, "longitudinal_velocity_m_s") for row in trace_rows])
    vy = np.array([_number(row, "lateral_velocity_m_s") for row in trace_rows])
    if not all(np.all(np.isfinite(array)) for array in (time, distance, lateral, yaw, vx, vy)):
        raise ValueError(f"Non-finite trace values in {directory}")
    if len(time) < 2 or np.any(np.diff(time) <= 0):
        raise ValueError(f"Non-increasing trace time in {directory}")
    reached = bool(np.max(distance) >= WINDOW_M)
    if reached:
        first = int(np.argmax(distance >= WINDOW_M))
        if first == 0:
            end_time = float(time[0])
        else:
            end_time = float(np.interp(
                WINDOW_M, distance[first - 1:first + 1], time[first - 1:first + 1]
            ))
        included = time <= time[first]
    else:
        end_time = math.nan
        included = np.ones(len(time), dtype=bool)
    peak_jerk, rms_jerk = (
        _jerk(time, vx, vy, end_time) if reached else (math.nan, math.nan)
    )
    flip_distance = _number(summary, "flip_distance_m")
    flip_completed_raw = summary["flip_completed"] == "True"
    # The simulator may confirm its dwell condition on the first sample just
    # beyond 0.80 m. Preserve that raw result, but do not count it as a flip
    # confirmed inside the prespecified common-distance window.
    flip_within_window = bool(
        flip_completed_raw and math.isfinite(flip_distance)
        and flip_distance <= WINDOW_M
    )
    result = {
        "run_label": directory.name,
        "mass_kg": match.group(1).replace("p", "."),
        "method": match.group(2),
        "trial_id": match.group(3),
        "start_left_deg": _number(summary, "caster_left_start_deg"),
        "start_right_deg": _number(summary, "caster_right_start_deg"),
        "forward_actual_m": _number(summary, "forward_actual_distance_m"),
        "reached_0p8m": reached,
        "travel_time_s": end_time,
        "max_abs_lateral_mm": float(1000.0 * np.max(np.abs(lateral[included]))),
        "max_abs_yaw_deg": float(np.max(np.abs(yaw[included]))),
        "peak_jerk_m_s3_5hz": peak_jerk,
        "rms_jerk_m_s3_5hz": rms_jerk,
        "flip_mode": summary["caster_flip_mode"],
        "flip_completed": summary["flip_completed"],
        "flip_distance_m": flip_distance,
        "flip_completed_within_0p8m": flip_within_window,
        "flip_boundary_censored": flip_completed_raw and not flip_within_window,
        "adaptive_response_reached": summary.get("adaptive_response_reached", ""),
    }
    return result, _trajectory(distance, lateral)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--trial-prefix", default=None,
                        help="Keep one prespecified trial block separate from pilots")
    args = parser.parse_args()
    results = []
    curves = defaultdict(list)
    paired = defaultdict(list)
    cross_load = defaultdict(list)
    for directory in sorted(RESULTS.glob("cics26_main_*")):
        analyzed = analyze(directory)
        if analyzed is None:
            continue
        row, curve = analyzed
        if args.trial_prefix and not row["trial_id"].startswith(args.trial_prefix):
            continue
        results.append(row)
        paired[(row["mass_kg"], row["trial_id"])].append(row)
        cross_load[(row["method"], row["trial_id"])].append(row)
        if curve is not None:
            curves[(row["mass_kg"], row["method"])].append((
                (round(row["start_left_deg"], 2), round(row["start_right_deg"], 2)),
                curve,
            ))
    if not results:
        print("No cics26_main_* trial results yet.")
        return
    for key, group in paired.items():
        if (
            max(row["forward_actual_m"] for row in group)
            - min(row["forward_actual_m"] for row in group) > 0.02
            or max(row["start_left_deg"] for row in group)
            - min(row["start_left_deg"] for row in group) > 0.02
            or max(row["start_right_deg"] for row in group)
            - min(row["start_right_deg"] for row in group) > 0.02
        ):
            print(f"INVALID paired start condition: {key}", file=sys.stderr)
    for key, group in cross_load.items():
        if len(group) >= 2 and (
            max(row["forward_actual_m"] for row in group)
            - min(row["forward_actual_m"] for row in group) > 0.02
        ):
            print(f"INVALID cross-load forward distance: {key}", file=sys.stderr)
    for row in results:
        group = curves[(row["mass_kg"], row["method"])]
        distinct = {angle for angle, _ in group}
        row["trajectory_dispersion_rms_mm"] = (
            float(1000.0 * np.sqrt(np.mean(np.var(
                [curve for _, curve in group], axis=0, ddof=1
            ))))
            if len(group) >= 3 and len(distinct) >= 3 else math.nan
        )
        row["dispersion_n"] = len(group)
    writer = csv.DictWriter(sys.stdout, fieldnames=list(results[0]))
    writer.writeheader()
    writer.writerows(results)


if __name__ == "__main__":
    main()
