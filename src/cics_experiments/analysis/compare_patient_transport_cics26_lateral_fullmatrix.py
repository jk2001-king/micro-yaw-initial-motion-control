"""Read-only CICS five-method, load, and CoM comparison."""

from cics_experiments.paths import workspace_root, resolve_path

import csv
import math
import statistics
from pathlib import Path

import numpy as np

from build_patient_transport_cics26_paper_evidence import analyze_trace


ROOT = resolve_path('studies/CICS26/results/patient_transport_lab/natural_flip_pilot')
CONDITIONS = (
    "240rearx0p10", "240center", "240frontx0p10",
    "265rearx0p10", "265center", "265frontx0p10",
    "290rearx0p10", "290center", "290frontx0p10",
)
METHODS = ("B0", "B1", "B2", "P", "B3H")


def label(condition, state, method):
    return f"cics26_pt_{condition}_lateral_lowbed_{state}_{method}_v5"


def read_csv(path):
    with path.open(newline="", encoding="utf-8") as stream:
        return list(csv.DictReader(stream))


def metrics(condition, state, method):
    folder = ROOT / label(condition, state, method)
    summaries = read_csv(folder / "summary.csv")
    traces = [analyze_trace(folder / f"trace_r{repeat:02d}.csv")
              for repeat in (1, 2, 3)]
    reached = sum(item["reached"] for item in traces)
    if reached != 3:
        return {"reached": reached, "flip": 0, "lat": math.nan, "yaw": math.nan,
                "jerk": math.nan, "time": math.nan, "spread": math.nan}
    curves = [item["curve"] for item in traces]
    med = lambda key: statistics.median(item[key] for item in traces)
    return {
        "reached": reached,
        "flip": sum(row["flip_completed"] == "True" for row in summaries),
        "lat": med("max_lat_mm"),
        "yaw": med("max_yaw_deg"),
        "jerk": med("peak_jerk"),
        "time": med("travel_time_s"),
        "spread": 1000.0 * np.sqrt(np.mean(np.var(curves, axis=0, ddof=1))),
    }


def main():
    results = {}
    print("condition method reached flip lateral_mm yaw_deg jerk_m_s3 time_s spread_mm")
    for condition in CONDITIONS:
        for method in METHODS:
            value = metrics(condition, "frontpos", method)
            results[condition, method] = value
            print(condition, method, f'{value["reached"]}/3', f'{value["flip"]}/3',
                  f'{value["lat"]:.6f}', f'{value["yaw"]:.6f}',
                  f'{value["jerk"]:.6f}', f'{value["time"]:.6f}',
                  f'{value["spread"]:.6f}')
    for condition in CONDITIONS:
        proposed = results[condition, "B3H"]
        ranks = {}
        for key in ("lat", "yaw", "jerk", "time", "spread"):
            ordered = sorted(METHODS, key=lambda method: results[condition, method][key])
            ranks[key] = ordered.index("B3H") + 1
        print("B3H_RANK", condition, ranks)

    print("B0 immediate; B1 fixed slow; B2 acceleration ramp; "
          "P response-step micro-yaw plus hold; "
          "B3H fixed micro-yaw plus opposite yaw compensation.")
    print("All mass and shifted base-CoM cases are provisional simulation sensitivities.")


if __name__ == "__main__":
    main()
