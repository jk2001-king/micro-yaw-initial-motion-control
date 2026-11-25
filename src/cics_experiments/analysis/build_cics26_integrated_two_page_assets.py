"""Build compact, traceable assets for the two-page CICS26 paper.

The proposed method is one control framework with a platform-fixed realization:

* wheelchair: preset micro-yaw initiation (internal code B3H)
* patient transport: response-terminated minimum micro-yaw initiation (P)

No condition-wise selection between B3H and P is performed.  The two internal
codes remain in exported CSV files for traceability and ablation reporting.
"""

from cics_experiments.paths import workspace_root, resolve_path

import csv
import os
from collections import defaultdict
from pathlib import Path

os.environ.setdefault("MPLCONFIGDIR", "/tmp/cics26-integrated-mpl")

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.lines import Line2D
from matplotlib.patches import FancyArrowPatch, FancyBboxPatch


ROOT = workspace_root()
WHEELCHAIR_CSV = (
    resolve_path('studies/CICS26/paper_assets/wheelchair/final7/wheelchair_final7_metrics.csv')
)
PATIENT_CSV = (
    resolve_path('studies/CICS26/paper_assets/patient_transport/final9/patient_transport_final9_metrics.csv')
)
OUT = resolve_path('studies/CICS26/paper_assets/cics26_final_integrated')

METHODS = ("B0", "B1", "B2", "P", "B3H")
BASELINES = ("B0", "B1", "B2")
PROPOSED = {"wheelchair": "B3H", "patient_transport": "P"}
ALTERNATIVE = {"wheelchair": "P", "patient_transport": "B3H"}
PUBLIC_LABEL = {
    "B0": "Immediate",
    "B1": "Fixed low-speed",
    "B2": "Acceleration ramp",
    "P": "Response-terminated variant",
    "B3H": "Preset variant",
}
PLATFORM_LABEL = {
    "wheelchair": "Wheelchair",
    "patient_transport": "Patient transport",
}
IMPLEMENTATION_LABEL = {
    "wheelchair": "Proposed—preset initiation",
    "patient_transport": "Proposed—response-terminated initiation",
}
COLORS = {
    "B0": "#7A7A7A",
    "B1": "#B279A2",
    "B2": "#59A14F",
    "P": "#F28E2B",
    "B3H": "#4E79A7",
}
METRICS = (
    ("lateral_mm", "Lateral", "mm"),
    ("yaw_deg", "Yaw", "deg"),
    ("jerk_m_s3", "Jerk", "m/s³"),
    ("time_s", "Time", "s"),
    ("spread_mm", "Spread", "mm"),
)
PATIENT_DISPLAY = {
    "240rearx0p10": "240 R", "240center": "240 C",
    "240frontx0p10": "240 F", "265rearx0p10": "265 R",
    "265center": "265 C", "265frontx0p10": "265 F",
    "290rearx0p10": "290 R", "290center": "290 C",
    "290frontx0p10": "290 F",
}


def read_csv(path):
    with path.open(newline="", encoding="utf-8") as stream:
        return list(csv.DictReader(stream))


def write_csv(path, rows):
    if not rows:
        return
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)


def load_records():
    records = []
    for row in read_csv(WHEELCHAIR_CSV):
        records.append({
            "platform": "wheelchair",
            "condition": row["condition"],
            "condition_label": row["condition_display"].replace(" kg ", " "),
            "method": row["method"],
            "lateral_mm": float(row["max_lateral_mm"]),
            "yaw_deg": float(row["max_yaw_deg"]),
            "jerk_m_s3": float(row["peak_jerk_m_s3"]),
            "time_s": float(row["travel_time_s"]),
            "spread_mm": float(row["trajectory_spread_mm"]),
            "target_count": 3,
            "flip_count": 3,
            "run_label": row["run_label"],
        })
    for row in read_csv(PATIENT_CSV):
        records.append({
            "platform": "patient_transport",
            "condition": row["condition"],
            "condition_label": PATIENT_DISPLAY[row["condition"]],
            "method": row["method"],
            "lateral_mm": float(row["median_max_lateral_mm"]),
            "yaw_deg": float(row["median_max_yaw_deg"]),
            "jerk_m_s3": float(row["median_peak_jerk_m_s3"]),
            "time_s": float(row["median_travel_time_s"]),
            "spread_mm": float(row["trajectory_spread_mm"]),
            "target_count": int(row["reached_count"]),
            "flip_count": int(row["flip_count"]),
            "run_label": (
                f'cics26_pt_{row["condition"]}_lateral_lowbed_frontpos_'
                f'{row["method"]}_v5'
            ),
        })
    return records


def index_records(records):
    return {
        (row["platform"], row["condition"], row["method"]): row
        for row in records
    }


def ordered_conditions(records, platform):
    seen = []
    for row in records:
        if row["platform"] == platform and row["condition"] not in seen:
            seen.append(row["condition"])
    return seen


def export_exact_records(records):
    rows = []
    for row in records:
        exported = dict(row)
        exported["method_public"] = PUBLIC_LABEL[row["method"]]
        exported["selected_platform_implementation"] = (
            row["method"] == PROPOSED[row["platform"]]
        )
        rows.append(exported)
    write_csv(OUT / "all_platform_condition_metrics.csv", rows)


def summarize(records):
    by_key = index_records(records)
    paired_rows = []
    summary_rows = []
    ablation_rows = []
    for platform in ("wheelchair", "patient_transport"):
        conditions = ordered_conditions(records, platform)
        selected = PROPOSED[platform]
        alternative = ALTERNATIVE[platform]
        for condition in conditions:
            proposed = by_key[platform, condition, selected]
            ramp = by_key[platform, condition, "B2"]
            best_conventional = {
                key: min(by_key[platform, condition, method][key]
                         for method in BASELINES)
                for key, _, _ in METRICS
            }
            item = {
                "platform": platform,
                "condition": condition,
                "implementation_code": selected,
            }
            for key, _, _ in METRICS:
                item[f"proposed_{key}"] = proposed[key]
                item[f"ramp_{key}"] = ramp[key]
                item[f"ratio_to_ramp_{key}"] = proposed[key] / ramp[key]
                item[f"reduction_vs_ramp_pct_{key}"] = (
                    100.0 * (1.0 - proposed[key] / ramp[key])
                )
                item[f"beats_best_conventional_{key}"] = (
                    proposed[key] < best_conventional[key]
                )
            paired_rows.append(item)

        selected_rows = [by_key[platform, c, selected] for c in conditions]
        ramp_rows = [by_key[platform, c, "B2"] for c in conditions]
        summary = {
            "platform": platform,
            "implementation": IMPLEMENTATION_LABEL[platform],
            "internal_code": selected,
            "conditions": len(conditions),
            "recorded_runs": len(conditions) * 3,
            "target_reached": sum(row["target_count"] for row in selected_rows),
            "flip_completed": sum(row["flip_count"] for row in selected_rows),
        }
        for key, _, _ in METRICS:
            summary[f"proposed_median_{key}"] = float(np.median(
                [row[key] for row in selected_rows]
            ))
            summary[f"ramp_median_{key}"] = float(np.median(
                [row[key] for row in ramp_rows]
            ))
            reductions = [
                100.0 * (1.0 - by_key[platform, c, selected][key]
                         / by_key[platform, c, "B2"][key])
                for c in conditions
            ]
            summary[f"median_reduction_vs_ramp_pct_{key}"] = float(
                np.median(reductions)
            )
            summary[f"wins_vs_ramp_{key}"] = sum(value > 0 for value in reductions)
            summary[f"wins_vs_best_conventional_{key}"] = sum(
                by_key[platform, c, selected][key]
                < min(by_key[platform, c, method][key] for method in BASELINES)
                for c in conditions
            )
        summary_rows.append(summary)

        for method in (selected, alternative):
            method_rows = [by_key[platform, c, method] for c in conditions]
            item = {
                "platform": platform,
                "variant": PUBLIC_LABEL[method],
                "internal_code": method,
                "selected_for_main_paper": method == selected,
            }
            for key, _, _ in METRICS:
                item[f"median_{key}"] = float(np.median(
                    [row[key] for row in method_rows]
                ))
                item[f"condition_wins_over_other_variant_{key}"] = sum(
                    by_key[platform, c, method][key]
                    < by_key[platform, c,
                             alternative if method == selected else selected][key]
                    for c in conditions
                )
            ablation_rows.append(item)

    write_csv(OUT / "proposed_vs_ramp_paired_metrics.csv", paired_rows)
    write_csv(OUT / "platform_proposed_summary.csv", summary_rows)
    write_csv(OUT / "platform_variant_ablation.csv", ablation_rows)
    return paired_rows, summary_rows, ablation_rows


def save(fig, stem):
    fig.savefig(OUT / f"{stem}.png", dpi=300, bbox_inches="tight")
    fig.savefig(OUT / f"{stem}.pdf", bbox_inches="tight")
    plt.close(fig)


def add_box(ax, xy, width, height, text, facecolor, fontsize=9,
            edgecolor="#4F4F4F"):
    box = FancyBboxPatch(
        xy, width, height, boxstyle="round,pad=0.012,rounding_size=0.015",
        linewidth=1.1, edgecolor=edgecolor, facecolor=facecolor,
        transform=ax.transAxes,
    )
    ax.add_patch(box)
    ax.text(xy[0] + width / 2, xy[1] + height / 2, text,
            ha="center", va="center", fontsize=fontsize,
            transform=ax.transAxes)


def add_arrow(ax, start, end):
    ax.add_patch(FancyArrowPatch(
        start, end, arrowstyle="-|>", mutation_scale=13, linewidth=1.2,
        color="#555555", transform=ax.transAxes,
    ))


def plot_framework():
    fig, ax = plt.subplots(figsize=(11.8, 2.55))
    ax.set_axis_off()
    add_box(ax, (0.015, 0.39), 0.16, 0.28,
            "Departure command\n+ caster misalignment", "#F2F2F2")
    add_box(ax, (0.225, 0.39), 0.18, 0.28,
            "Micro-yaw initiation\n(direction induction)", "#D9EAF7")
    add_box(ax, (0.455, 0.39), 0.17, 0.28,
            "Caster transition\nresponse", "#E8F1E2")
    add_box(ax, (0.675, 0.39), 0.17, 0.28,
            "Opposite yaw\ncompensation", "#FCE4D6")
    add_box(ax, (0.895, 0.39), 0.09, 0.28,
            "Reduced\ndeparture\nerror", "#E2F0D9", fontsize=8.5)
    for start, end in (
        ((0.175, 0.53), (0.225, 0.53)),
        ((0.405, 0.53), (0.455, 0.53)),
        ((0.625, 0.53), (0.675, 0.53)),
        ((0.845, 0.53), (0.895, 0.53)),
    ):
        add_arrow(ax, start, end)
    add_box(ax, (0.185, 0.075), 0.30, 0.18,
            "Wheelchair: preset initiation (B3H)", "#EAF2F8",
            fontsize=8.2)
    add_box(ax, (0.535, 0.075), 0.38, 0.18,
            "Patient transport: response-terminated initiation (P)",
            "#FFF0E2", fontsize=8.2)
    ax.text(0.5, 0.93,
            "Common framework: micro-yaw directional initiation and body-yaw recovery",
            transform=ax.transAxes, ha="center", va="center",
            fontsize=12, fontweight="semibold")
    ax.text(0.5, 0.30,
            "The initiation termination rule is platform-specific; the control principle is shared.",
            transform=ax.transAxes, ha="center", va="center",
            fontsize=8.5, color="#555555")
    save(fig, "fig1_common_control_framework")


def plot_integrated_validation(records, paired_rows):
    by_key = index_records(records)
    paired_index = defaultdict(list)
    for row in paired_rows:
        paired_index[row["platform"]].append(row)

    fig, axes = plt.subplots(2, 3, figsize=(11.8, 6.7),
                             gridspec_kw={"width_ratios": [1.35, 1.35, 1.0]})
    fig.subplots_adjust(left=0.065, right=0.985, bottom=0.12, top=0.90,
                        wspace=0.34, hspace=0.46)
    panel = iter("abcdef")
    for row_index, platform in enumerate(("wheelchair", "patient_transport")):
        conditions = ordered_conditions(records, platform)
        selected = PROPOSED[platform]
        shown_methods = (*BASELINES, selected)
        x = np.arange(len(conditions))
        width = 0.19
        labels = [by_key[platform, c, selected]["condition_label"] for c in conditions]

        for col_index, (metric, ylabel) in enumerate((
            ("lateral_mm", "Maximum lateral deviation (mm)"),
            ("yaw_deg", "Maximum yaw deviation (deg)"),
        )):
            ax = axes[row_index, col_index]
            for offset, method in enumerate(shown_methods):
                values = [by_key[platform, c, method][metric] for c in conditions]
                label = (IMPLEMENTATION_LABEL[platform]
                         if method == selected else PUBLIC_LABEL[method])
                ax.bar(x + (offset - 1.5) * width, values, width,
                       color=COLORS[method], label=label)
            ax.set_yscale("log")
            ax.set_ylabel(ylabel, fontsize=8.5)
            ax.set_xticks(x, labels, rotation=30, ha="right", fontsize=7.5)
            ax.tick_params(axis="y", labelsize=7.5)
            ax.grid(axis="y", alpha=0.20, which="both")
            ax.set_title(
                f"({next(panel)}) {PLATFORM_LABEL[platform]} — "
                f'{"lateral" if col_index == 0 else "yaw"}',
                fontsize=9.5,
            )
            lower, upper = ax.get_ylim()
            ax.set_ylim(lower, upper * 3.0)
            ax.legend(loc="upper left", ncol=1, fontsize=6.4,
                      frameon=True, facecolor="white", edgecolor="black",
                      framealpha=1.0, borderpad=0.35, handlelength=1.4)

        ax = axes[row_index, 2]
        ratios = []
        for metric, _, _ in METRICS:
            ratios.append([
                row[f"ratio_to_ramp_{metric}"]
                for row in paired_index[platform]
            ])
        positions = np.arange(len(METRICS))
        parts = ax.boxplot(ratios, positions=positions, widths=0.55,
                           patch_artist=True, showfliers=False,
                           medianprops={"color": "black", "linewidth": 1.1})
        for patch in parts["boxes"]:
            patch.set_facecolor(COLORS[selected])
            patch.set_alpha(0.75)
        # Show every condition as well as the distribution.  This keeps the
        # near-unity time ratios visible even when their box is very thin.
        for position, values in zip(positions, ratios):
            jitter = np.linspace(-0.10, 0.10, len(values))
            ax.scatter(position + jitter, values, s=8, color=COLORS[selected],
                       edgecolor="white", linewidth=0.25, zorder=3)
        ax.axhline(1.0, color="#444444", linewidth=1.0, linestyle="--")
        ax.set_yscale("log")
        ax.set_xticks(positions, [label for _, label, _ in METRICS],
                      rotation=30, ha="right", fontsize=7.5)
        ax.tick_params(axis="y", labelsize=7.5)
        ax.set_ylabel("Proposed / acceleration-ramp ratio", fontsize=8.5)
        ax.grid(axis="y", alpha=0.20, which="both")
        ax.set_title(
            f"({next(panel)}) Paired ratios across "
            f'{len(conditions)} conditions', fontsize=9.5,
        )
        ax.text(0.98, 0.04, "< 1: proposed is lower",
                transform=ax.transAxes, ha="right", va="bottom",
                fontsize=7, color="#555555")

    fig.suptitle(
        "Platform-fixed validation of the common micro-yaw initiation and recovery framework",
        fontsize=12, fontweight="semibold", y=0.985,
    )
    save(fig, "fig2_integrated_platform_validation")


def plot_compact_table(summary_rows):
    columns = [
        "Platform / realization", "N", "Lat.\n(mm)", "Yaw\n(deg)",
        "Jerk\n(m/s³)", "Time\n(s)", "Spread\n(mm)",
        "Δ lateral\nvs ramp", "Δ yaw\nvs ramp", "Δ spread\nvs ramp",
    ]
    table_rows = []
    for row in summary_rows:
        table_rows.append([
            ("Wheelchair / preset" if row["platform"] == "wheelchair"
             else "Patient transport / response"),
            f'{row["conditions"]}×3',
            f'{row["proposed_median_lateral_mm"]:.3f}',
            f'{row["proposed_median_yaw_deg"]:.3f}',
            f'{row["proposed_median_jerk_m_s3"]:.3f}',
            f'{row["proposed_median_time_s"]:.3f}',
            f'{row["proposed_median_spread_mm"]:.3f}',
            f'{row["median_reduction_vs_ramp_pct_lateral_mm"]:+.1f}%',
            f'{row["median_reduction_vs_ramp_pct_yaw_deg"]:+.1f}%',
            f'{row["median_reduction_vs_ramp_pct_spread_mm"]:+.1f}%',
        ])
    fig, ax = plt.subplots(figsize=(11.8, 1.85))
    ax.axis("off")
    table = ax.table(
        cellText=table_rows, colLabels=columns, cellLoc="center", loc="center",
        colWidths=[0.215, 0.06, 0.073, 0.073, 0.078, 0.068, 0.078,
                   0.102, 0.092, 0.102],
    )
    table.auto_set_font_size(False)
    table.set_fontsize(7.7)
    table.scale(1.0, 1.6)
    for (row, _), cell in table.get_celld().items():
        cell.set_edgecolor("#AAAAAA")
        cell.set_linewidth(0.55)
        if row == 0:
            cell.set_facecolor("#D9E2F3")
            cell.set_text_props(weight="bold")
        elif row == 1:
            cell.set_facecolor("#EAF2F8")
        else:
            cell.set_facecolor("#FFF0E2")
    ax.set_title(
        "Platform-level medians and paired median reductions relative to the acceleration ramp",
        fontsize=10, fontweight="semibold", pad=5,
    )
    save(fig, "table1_platform_proposed_summary")


def write_readme(summary_rows):
    counts = []
    for row in summary_rows:
        counts.append(
            f'- {PLATFORM_LABEL[row["platform"]]}: '
            f'{row["conditions"]} conditions × 3 repeats; '
            f'{row["target_reached"]}/{row["conditions"] * 3} target reaches, '
            f'{row["flip_completed"]}/{row["conditions"] * 3} completed flips.'
        )
    text = f"""# CICS26 final integrated two-page assets

The main-paper method is one framework: **micro-yaw directional initiation and
body-yaw recovery**. Its realization is fixed by platform, not selected by
condition:

- wheelchair: preset initiation (internal traceability code `B3H`);
- patient transport: minimum response-terminated initiation (`P`).

{chr(10).join(counts)}

## Recommended two-page allocation

1. Use `fig1_common_control_framework` as a shallow full-width method figure.
2. Use `fig2_integrated_platform_validation` as the only large result figure.
3. Use `table1_platform_proposed_summary` only if the template remains legible;
   otherwise typeset its two rows directly as Table 1.

The condition-level data and both framework variants are retained in CSV form.
Do not compute a condition-wise envelope between `P` and `B3H`; that would be a
post-hoc best-case construction rather than a tested controller.

## Files

- `all_platform_condition_metrics.csv`: all 80 platform-condition-method arms.
- `proposed_vs_ramp_paired_metrics.csv`: exact paired ratios and reductions.
- `platform_proposed_summary.csv`: the two platform-fixed main-paper rows.
- `platform_variant_ablation.csv`: preset/response-terminated ablation.
- PNG/PDF files: paper-ready figures and compact table.

Rebuild with:

```bash
python3 studies/CICS26/scripts/build_cics26_integrated_two_page_assets.py
```
"""
    (OUT / "README.md").write_text(text, encoding="utf-8")


def main():
    OUT.mkdir(parents=True, exist_ok=True)
    records = load_records()
    export_exact_records(records)
    paired_rows, summary_rows, _ = summarize(records)
    plot_framework()
    plot_integrated_validation(records, paired_rows)
    plot_compact_table(summary_rows)
    write_readme(summary_rows)
    print(f"Saved integrated CICS26 assets to {OUT}")


if __name__ == "__main__":
    main()
