"""Build the final seven-condition wheelchair evidence table and figures."""

from cics_experiments.paths import workspace_root, resolve_path

import csv
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np

from build_wheelchair_paper_evidence import matched_trace


ROOT = workspace_root()
FLIP = resolve_path('studies/CICS26/results/wheelchair_lab/natural_flip_gate')
OUT = resolve_path('studies/CICS26/paper_assets/wheelchair/final7')
REPORT = resolve_path('studies/CICS26/docs/wheelchair_final7_results.md')

CONDITIONS = (
    ("40p0_center", "40 kg center"),
    ("90p0_rearx0p07", "90 kg rear"),
    ("90p0_center", "90 kg center"),
    ("90p0_frontx0p17", "90 kg front"),
    ("127p5_rearx0p07", "127.5 kg rear"),
    ("127p5_center", "127.5 kg center"),
    ("127p5_frontx0p17", "127.5 kg front"),
)
METHODS = ("B0", "B1", "B2", "P", "B3H")
METHOD_ROLE_KO = {
    "B0": "즉시 목표속도 인가",
    "B1": "고정 저속 구동",
    "B2": "가속도 제한 램프",
    "P": "응답 기반 단계형 미세 yaw 및 차체 yaw 보상",
    "B3H": "고정 미세 yaw 초기 구동 및 차체 yaw 보상 (제안)",
}
METHOD_ROLE_EN = {
    "B0": "Immediate target-speed command",
    "B1": "Fixed low-speed command",
    "B2": "Acceleration-limited ramp",
    "P": "Response-stepped micro-yaw with body-yaw hold",
    "B3H": "Proposed fixed micro-yaw with body-yaw hold",
}
METHOD_SHORT_EN = {
    "B0": "Immediate",
    "B1": "Fixed low-speed",
    "B2": "Acceleration ramp",
    "P": "Response-step + hold",
    "B3H": "Proposed micro-yaw + hold",
}
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


def label(condition, method):
    return REUSED.get(
        (condition, method), f"cics26_final9_kd003_d090_{method}_{condition}"
    )


def read_csv(path):
    with path.open(newline="", encoding="utf-8") as stream:
        return list(csv.DictReader(stream))


def median(rows, key):
    return float(np.median([float(row[key]) for row in rows]))


def collect():
    records = []
    for condition, display in CONDITIONS:
        for method in METHODS:
            run_label = label(condition, method)
            directory = FLIP / run_label
            rows = read_csv(directory / "summary.csv")
            if len(rows) != 3:
                raise ValueError(f"Expected three repeats: {run_label}")
            if any(row["flip_completed"] != "True" for row in rows):
                raise ValueError(f"Incomplete flip: {run_label}")
            if any(abs(float(row["reverse_target_distance_m"]) - 0.90) > 1e-9
                   for row in rows):
                raise ValueError(f"Protocol mismatch: {run_label}")
            traces = [matched_trace(directory, repeat) for repeat in (1, 2, 3)]
            curves = np.asarray([trace[2] for trace in traces])
            records.append({
                "condition": condition,
                "condition_display": display,
                "method": method,
                "method_role_ko": METHOD_ROLE_KO[method],
                "method_role_en": METHOD_ROLE_EN[method],
                "run_label": run_label,
                "max_lateral_before_flip_mm": median(
                    rows, "max_abs_lateral_before_flip_mm"
                ),
                "max_yaw_before_flip_deg": median(
                    rows, "max_abs_yaw_before_flip_deg"
                ),
                "max_lateral_mm": median(rows, "max_abs_lateral_mm"),
                "max_yaw_deg": median(rows, "max_abs_yaw_error_deg"),
                "peak_jerk_m_s3": float(np.median([trace[1] for trace in traces])),
                "travel_time_s": float(np.median([trace[0] for trace in traces])),
                "flip_distance_m": median(rows, "flip_distance_m"),
                "trajectory_spread_mm": float(
                    1000.0 * np.sqrt(np.mean(np.var(curves, axis=0, ddof=1)))
                ),
                "effort_saturation_fraction": median(
                    rows, "drive_effort_saturation_fraction_est"
                ),
                "unique_trajectories": len({curve.tobytes() for curve in curves}),
            })
    return records


def write_csv(records):
    path = OUT / "wheelchair_final7_metrics.csv"
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=records[0].keys())
        writer.writeheader()
        writer.writerows(records)


def plot(records):
    colors = dict(zip(METHODS, ("#777777", "#b279a2", "#59a14f", "#f28e2b", "#4e79a7")))
    x = np.arange(len(CONDITIONS))
    short_conditions = ("40 C", "90 R", "90 C", "90 F",
                        "127.5 R", "127.5 C", "127.5 F")
    width = 0.16
    fig, axes = plt.subplots(2, 1, figsize=(10.5, 6.8), sharex=True)
    for offset, method in enumerate(METHODS):
        subset = [row for row in records if row["method"] == method]
        axes[0].bar(x + (offset - 2) * width,
                    [row["max_lateral_mm"] for row in subset], width,
                    label=METHOD_SHORT_EN[method], color=colors[method])
        axes[1].bar(x + (offset - 2) * width,
                    [row["max_yaw_deg"] for row in subset], width,
                    color=colors[method])
    axes[0].set_ylabel("Maximum lateral deviation [mm]")
    axes[1].set_ylabel("Maximum yaw deviation [deg]")
    axes[0].set_yscale("log")
    axes[1].set_yscale("log")
    axes[0].grid(axis="y", alpha=0.25, which="both")
    axes[1].grid(axis="y", alpha=0.25, which="both")
    axes[0].legend(ncol=5, frameon=False, loc="upper left")
    axes[1].set_xticks(x, [display.replace(" ", "\n", 2) for _, display in CONDITIONS])
    fig.tight_layout()
    fig.savefig(OUT / "wheelchair_final7_tracking_errors.png", dpi=240)
    plt.close(fig)

    fig, axes = plt.subplots(1, 3, figsize=(11.5, 3.5))
    keys = ("peak_jerk_m_s3", "travel_time_s", "trajectory_spread_mm")
    labels = ("Peak jerk [m/s³]", "Travel time to 0.9 m [s]",
              "Trajectory RMS spread [mm]")
    for axis, key, ylabel in zip(axes, keys, labels):
        for method in METHODS:
            subset = [row for row in records if row["method"] == method]
            axis.plot(x, [row[key] for row in subset], marker="o", linewidth=1.5,
                      label=METHOD_SHORT_EN[method], color=colors[method])
        axis.set_ylabel(ylabel)
        axis.set_xticks(x, short_conditions, rotation=35, ha="right")
        axis.grid(alpha=0.25)
    axes[0].legend(ncol=1, frameon=False)
    fig.tight_layout()
    fig.savefig(OUT / "wheelchair_final7_secondary_metrics.png", dpi=240)
    plt.close(fig)


def write_report(records):
    by_key = {(row["condition"], row["method"]): row for row in records}
    metrics = (
        ("max_lateral_mm", "maximum lateral deviation"),
        ("max_yaw_deg", "maximum yaw deviation"),
        ("peak_jerk_m_s3", "peak jerk"),
        ("travel_time_s", "travel time"),
        ("trajectory_spread_mm", "trajectory spread"),
    )
    wins = {method: {key: 0 for key, _ in metrics} for method in METHODS}
    for condition, _ in CONDITIONS:
        for key, _ in metrics:
            best = min(by_key[(condition, method)][key] for method in METHODS)
            for method in METHODS:
                if np.isclose(by_key[(condition, method)][key], best):
                    wins[method][key] += 1

    lines = [
        "# Wheelchair final seven-condition results",
        "",
        "The final matrix is complete: 7 load/CoM conditions × 5 methods × "
        "3 repeats = 105 recorded runs. All runs completed the caster flip and "
        "the common 0.90 m reverse window.",
        "",
        "## Main finding",
        "",
        "The proposed fixed micro-yaw initial drive with body-yaw compensation "
        "ranked first for full-window maximum lateral deviation in 5/7 "
        "conditions and maximum yaw deviation in 6/7 conditions. It therefore "
        "supports a constrained claim of broadly improved departure tracking, "
        "not universal superiority across every metric.",
        "",
        "The proposed method did not minimize travel time: immediate target-speed "
        "command was fastest in all 7 conditions. The proposed method ranked first for "
        "peak jerk in 2/7 conditions and trajectory spread in 2/7 conditions.",
        "",
        "The main exceptions were the rear-CoM cases. At 90 kg rear CoM, the "
        "response-stepped method had the smallest lateral deviation (0.903 mm "
        "versus the proposed method's 1.293 mm). At 127.5 kg rear CoM, the "
        "immediate command had the smallest lateral and yaw deviations (1.103 mm "
        "and 0.138 deg versus 1.423 mm and 0.199 deg).",
        "",
        "## Full-window median results",
        "",
        "| Condition | Method | Lateral [mm] | Yaw [deg] | Jerk [m/s³] | "
        "Time [s] | Spread [mm] |",
        "|---|---:|---:|---:|---:|---:|---:|",
    ]
    for condition, display in CONDITIONS:
        for method in METHODS:
            row = by_key[(condition, method)]
            lines.append(
                f"| {display} | {METHOD_ROLE_KO[method]} | "
                f"{row['max_lateral_mm']:.3f} | "
                f"{row['max_yaw_deg']:.3f} | {row['peak_jerk_m_s3']:.3f} | "
                f"{row['travel_time_s']:.3f} | {row['trajectory_spread_mm']:.3f} |"
            )
    lines.extend((
        "",
        "## Metric win counts",
        "",
        "| Method | Lateral | Yaw | Jerk | Time | Spread |",
        "|---|---:|---:|---:|---:|---:|",
    ))
    for method in METHODS:
        lines.append(
            f"| {METHOD_ROLE_KO[method]} | {wins[method]['max_lateral_mm']} | "
            f"{wins[method]['max_yaw_deg']} | {wins[method]['peak_jerk_m_s3']} | "
            f"{wins[method]['travel_time_s']} | "
            f"{wins[method]['trajectory_spread_mm']} |"
        )
    lines.extend((
        "",
        "## Interpretation limits",
        "",
        "The fixed low-speed method uses 0.025 m/s while the others target 0.05 "
        "m/s, so its travel-time result is an inherent method trade-off rather "
        "than a speed-matched controller comparison. The response-stepped method "
        "uses simulated caster angle for online response detection; the proposed "
        "method uses body yaw for its hold. "
        "The caster damping and contact parameters remain provisional. Many "
        "second and third traces are bit-identical, so the reported trajectory "
        "spread describes deterministic simulation repeatability and must not "
        "be presented as independent physical trial variability.",
        "",
        "Figures: `wheelchair_final7_tracking_errors.png` and "
        "`wheelchair_final7_secondary_metrics.png`.",
    ))
    REPORT.write_text("\n".join(lines) + "\n", encoding="utf-8")


def main():
    OUT.mkdir(parents=True, exist_ok=True)
    records = collect()
    if len(records) != 35:
        raise ValueError(f"Expected 35 arms, got {len(records)}")
    write_csv(records)
    plot(records)
    write_report(records)
    print(f"validated arms/repeats: {len(records)}/{len(records) * 3}")
    print(f"report: {REPORT}")
    print(f"assets: {OUT}")


if __name__ == "__main__":
    main()
