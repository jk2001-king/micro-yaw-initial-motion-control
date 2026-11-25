"""Print read-only CICS26 wheelchair evidence tables from saved Isaac CSVs.

The 45-run eval_v1 block and later Kd=0.03 matched-repeat block are separate
protocols. Do not combine their rows into a single treatment effect.
"""

from cics_experiments.paths import workspace_root, resolve_path

import csv
import math
from collections import defaultdict
from pathlib import Path

import numpy as np

from analyze_wheelchair_cics26_trials import _jerk, analyze


ROOT = workspace_root()
FLIP = resolve_path('studies/CICS26/results/wheelchair_lab/natural_flip_gate')
METHODS = ("B0", "B1", "B2", "B3", "P")
MASSES = ("40.0", "90.0", "127.5")
MATCHED = (
    ("40 center", "B2", "cics26_caster_swivel_kd0030000_B2_40p0_center_d090"),
    ("40 center", "P", "cics26_p10_lowdamping_d090_v1_40p0_center"),
    ("40 center", "B3H", "cics26_kd003_d090_B3H_40p0_center"),
    ("90 center", "B2", "cics26_caster_swivel_kd0030000_B2_90p0_center_d090"),
    ("90 center", "P", "cics26_p11_90kg_heldout_d090_v1"),
    ("127.5 center", "B2", "cics26_caster_swivel_kd0030000_B2_127p5_center_d090"),
    ("127.5 center", "P", "cics26_p12_127p5kg_heldout_d090_v1"),
    ("127.5 center", "B3H", "cics26_kd003_d090_B3H_127p5_center"),
    ("127.5 rear", "B2", "cics26_kd003_d090_B2_127p5_rearx0p07"),
    ("127.5 rear", "P", "cics26_kd003_d090_P_127p5_rearx0p07"),
    ("127.5 front", "B2", "cics26_kd003_d090_B2_127p5_frontx0p17"),
    ("127.5 front", "P", "cics26_kd003_d090_P_127p5_frontx0p17"),
    ("127.5 front", "B3H", "cics26_kd003_d090_B3H_127p5_frontx0p17"),
)


def read_csv(path):
    with path.open(newline="", encoding="utf-8") as stream:
        return list(csv.DictReader(stream))


def median(rows, key):
    values = sorted(float(row[key]) for row in rows if row[key] not in ("", "None", None))
    return values[len(values) // 2] if len(values) == 3 else math.nan


def fmt(value, digits=3):
    return f"{value:.{digits}f}" if math.isfinite(value) else "NA"


def matched_trace(directory, repeat, distance_limit=0.90):
    rows = read_csv(directory / f"natural_flip_r{repeat:02d}.csv")
    time = np.asarray([float(row["time_s"]) for row in rows])
    distance = np.asarray([float(row["reverse_distance_m"]) for row in rows])
    lateral = np.asarray([float(row["lateral_m"]) for row in rows])
    vx = np.asarray([float(row["longitudinal_velocity_m_s"]) for row in rows])
    vy = np.asarray([float(row["lateral_velocity_m_s"]) for row in rows])
    if not all(np.all(np.isfinite(array)) for array in (time, distance, lateral, vx, vy)):
        raise ValueError(f"Non-finite trace: {directory.name} r{repeat:02d}")
    if np.any(np.diff(time) <= 0) or np.max(distance) < distance_limit:
        raise ValueError(f"Incomplete/invalid trace: {directory.name} r{repeat:02d}")
    first = int(np.argmax(distance >= distance_limit))
    elapsed = float(np.interp(
        distance_limit, distance[first - 1:first + 1], time[first - 1:first + 1]
    ))
    peak_jerk, _ = _jerk(time, vx, vy, elapsed)
    progress = np.maximum.accumulate(distance)
    unique, index = np.unique(progress, return_index=True)
    if unique[0] > 0.01:
        raise ValueError(f"Trace starts too late: {directory.name} r{repeat:02d}")
    curve = np.interp(np.linspace(0.0, distance_limit, 91), unique, lateral[index])
    return elapsed, peak_jerk, curve


def main():
    eval_groups = defaultdict(list)
    eval_curves = defaultdict(list)
    for directory in FLIP.glob("cics26_main_*eval_v1_*"):
        result = analyze(directory)
        if result is None:
            continue
        row, curve = result
        key = (row["mass_kg"], row["method"])
        eval_groups[key].append(row)
        if curve is not None:
            eval_curves[key].append(curve)
    if sum(map(len, eval_groups.values())) != 45:
        raise ValueError("Expected all 45 eval_v1 trials")

    print("# eval_v1 central initial angle: 0.80 m common window")
    print("mass kg | arm | lateral mm | yaw deg | peak jerk m/s³ | time s | flip m | flip≤0.8")
    for mass in MASSES:
        for method in METHODS:
            row = next(row for row in eval_groups[(mass, method)]
                       if row["trial_id"] == "eval_v1_z0")
            print(" | ".join((mass, method, fmt(row["max_abs_lateral_mm"]),
                              fmt(row["max_abs_yaw_deg"]),
                              fmt(row["peak_jerk_m_s3_5hz"]),
                              fmt(row["travel_time_s"]),
                              fmt(row["flip_distance_m"]),
                              str(row["flip_completed_within_0p8m"]))))

    print("# eval_v1 initial-angle perturbation sensitivity: -0.25/0/+0.25 deg")
    print("mass kg | arm | reached 0.8/3 | flip≤0.8/3 | lateral-curve RMS spread mm | response/3")
    for mass in MASSES:
        for method in METHODS:
            key = (mass, method)
            rows = eval_groups[key]
            curves = eval_curves[key]
            dispersion = (1000.0 * np.sqrt(np.mean(np.var(curves, axis=0, ddof=1)))
                          if len(curves) == 3 else math.nan)
            print(" | ".join((mass, method,
                              str(sum(row["reached_0p8m"] for row in rows)),
                              str(sum(row["flip_completed_within_0p8m"] for row in rows)),
                              fmt(dispersion),
                              str(sum(row["adaptive_response_reached"] == "True" for row in rows)))))

    print("# Kd=0.03 matched 3-repeat block: 0.90 m common window")
    print("condition | arm | preflip lat mm | preflip yaw deg | full lat mm | full yaw deg | peak jerk m/s³ | time s | flip m | trajectory spread mm | unique curves | saturation est")
    for condition, method, label in MATCHED:
        directory = FLIP / label
        rows = read_csv(directory / "summary.csv")
        if len(rows) != 3:
            raise ValueError(f"Expected three repeats: {label}")
        if any(row["flip_completed"] != "True" for row in rows):
            raise ValueError(f"Incomplete flip needs censoring: {label}")
        if any(abs(float(row["reverse_target_distance_m"]) - 0.90) > 1e-9
               for row in rows):
            raise ValueError(f"Protocol mismatch: {label}")
        traces = [matched_trace(directory, repeat) for repeat in (1, 2, 3)]
        times = sorted(trace[0] for trace in traces)
        jerks = sorted(trace[1] for trace in traces)
        curves = [trace[2] for trace in traces]
        dispersion = 1000.0 * np.sqrt(np.mean(np.var(curves, axis=0, ddof=1)))
        unique = len({curve.tobytes() for curve in curves})
        print(" | ".join((condition, method,
                          fmt(median(rows, "max_abs_lateral_before_flip_mm")),
                          fmt(median(rows, "max_abs_yaw_before_flip_deg")),
                          fmt(median(rows, "max_abs_lateral_mm")),
                          fmt(median(rows, "max_abs_yaw_error_deg")),
                          fmt(jerks[1]), fmt(times[1]),
                          fmt(median(rows, "flip_distance_m")), fmt(dispersion),
                          str(unique), fmt(median(rows, "drive_effort_saturation_fraction_est"), 4))))


if __name__ == "__main__":
    main()
