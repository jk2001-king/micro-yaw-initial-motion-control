"""Read-only five-metric summary for the 265 kg patient-transport five-arm pilot.

Missing/incomplete arms remain visible. All metrics use the common 0.70 m
reverse-travel window; no failure is silently dropped.
"""

from cics_experiments.paths import workspace_root, resolve_path

import csv
import math
from pathlib import Path

import numpy as np

from analyze_wheelchair_cics26_trials import _jerk


ROOT = workspace_root()
RESULTS = resolve_path('studies/CICS26/results/patient_transport_lab/natural_flip_pilot')
METHODS = ("B0", "B1", "B2", "B3", "P")
WINDOW_M = 0.70
GRID = np.linspace(0.0, WINDOW_M, 71)


def read_csv(path):
    with path.open(newline="", encoding="utf-8") as stream:
        return list(csv.DictReader(stream))


def analyze_trace(path):
    rows = read_csv(path)
    if len(rows) < 60:
        raise ValueError(f"Too few trace rows: {path}")
    get = lambda key: np.asarray([float(row[key]) for row in rows])
    time = get("time_s")
    distance = get("reverse_distance_m")
    lateral = get("lateral_m")
    yaw = get("yaw_error_deg")
    vx = get("body_longitudinal_speed_m_s")
    vy = get("body_lateral_speed_m_s")
    if any(not np.all(np.isfinite(array)) for array in (time, distance, lateral, yaw, vx, vy)):
        raise ValueError(f"Non-finite trace values: {path}")
    if np.any(np.diff(time) <= 0):
        raise ValueError(f"Non-increasing trace time: {path}")
    reached = bool(np.max(distance) >= WINDOW_M)
    if not reached:
        return {"reached": False, "max_lat_mm": math.nan, "max_yaw_deg": math.nan,
                "peak_jerk": math.nan, "travel_time_s": math.nan, "curve": None}
    first = int(np.argmax(distance >= WINDOW_M))
    if first == 0:
        raise ValueError(f"0.70 m reached at first sample: {path}")
    elapsed = float(np.interp(WINDOW_M, distance[first - 1:first + 1],
                              time[first - 1:first + 1]))
    included = slice(None, first + 1)
    peak_jerk, _ = _jerk(time, vx, vy, elapsed)
    progress = np.maximum.accumulate(distance)
    unique, indices = np.unique(progress, return_index=True)
    if unique[0] > 0.01:
        raise ValueError(f"Late-start trace: {path}")
    curve = np.interp(GRID, unique, lateral[indices])
    return {
        "reached": True,
        "max_lat_mm": float(1000.0 * np.max(np.abs(lateral[included]))),
        "max_yaw_deg": float(np.max(np.abs(yaw[included]))),
        "peak_jerk": peak_jerk,
        "travel_time_s": elapsed,
        "curve": curve,
    }


def number(value, decimals=3):
    return f"{value:.{decimals}f}" if math.isfinite(value) else "NA"


def main():
    print("# Patient-transport 265 kg center: five-arm pilot; common 0.70 m window")
    print("method | reached/3 | max lateral mm | max yaw deg | peak jerk m/s^3 | travel s | repeat spread mm | unique curves/3 | flip confirmed <=0.70/3")
    completed = 0
    for method in METHODS:
        directory = RESULTS / f"cics26_pt_265center_{method}_fivearm_v1"
        summary_path = directory / "summary.csv"
        paths = [directory / f"trace_r{repeat:02d}.csv" for repeat in (1, 2, 3)]
        if not summary_path.exists() or not all(path.exists() for path in paths):
            print(f"{method} | PENDING")
            continue
        summaries = read_csv(summary_path)
        if len(summaries) != 3 or any(row["analysis_role"] != "recorded" for row in summaries):
            raise ValueError(f"Expected three recorded repeats: {directory}")
        if any(abs(float(row["reverse_target_distance_m"]) - WINDOW_M) > 1e-9
               or abs(float(row["model_mass_kg_provisional"]) - 265.0) > 0.01
               for row in summaries):
            raise ValueError(f"Protocol mismatch: {directory}")
        traces = [analyze_trace(path) for path in paths]
        reached = sum(item["reached"] for item in traces)
        metric = lambda key: float(np.median([item[key] for item in traces])) if reached == 3 else math.nan
        curves = [item["curve"] for item in traces if item["curve"] is not None]
        spread = (1000.0 * np.sqrt(np.mean(np.var(curves, axis=0, ddof=1)))
                  if len(curves) == 3 else math.nan)
        unique = len({curve.tobytes() for curve in curves}) if len(curves) == 3 else 0
        flip = sum(row["flip_completed"] == "True"
                   and row["flip_confirm_distance_m"] not in ("", "None")
                   and float(row["flip_confirm_distance_m"]) <= WINDOW_M
                   for row in summaries)
        print(" | ".join((method, str(reached), number(metric("max_lat_mm")),
                          number(metric("max_yaw_deg"), 5),
                          number(metric("peak_jerk")),
                          number(metric("travel_time_s")), number(spread),
                          str(unique), str(flip))))
        if method == "P":
            responses = sum(row["adaptive_response_reached"] == "True"
                            for row in summaries)
            selected = np.median([float(row["adaptive_selected_steer_deg"])
                                  for row in summaries])
            counters = sum(row["counter_yaw_actual_start_s"] not in ("", "None")
                           for row in summaries)
            print(f"P control diagnostic: response {responses}/3, "
                  f"selected steering median {selected:.5f} deg, "
                  f"counter-yaw started {counters}/3")
        completed += 1
    print(f"Complete arms: {completed}/5. P is a pilot, not a validated winner.")
    print("B1 target speed differs; deterministic duplicate traces are not physical repeatability.")


if __name__ == "__main__":
    main()
