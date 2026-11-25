# Experiments and results

## Fixed matrix

| Platform | Conditions | Methods | Repeats | Recorded trials |
|---|---|---|---|---:|
| Wheelchair | 40 kg center; 90 / 127.5 kg rear, center, front | B0, B1, B2, P, B3H | 3 | 105 |
| Patient transport | 240 / 265 / 290 kg × rear, center, front CoM | B0, B1, B2, P, B3H | 3 | 135 |

Wheelchair mass is total platform-plus-occupant mass, not patient mass. Exact identifiers, CoM values and per-platform settings are read from the retained scheduler definitions by `protocol.available/select/dynamic_settings`; list them with `ros2 run cics_experiments experiment --list`. Original wheelchair records include explicitly reused run labels in `REUSED`; the 105 records are not claimed to be an independently collected new campaign.

| Code | Meaning |
|---|---|
| B0 | Immediate nominal-speed departure |
| B1 | Fixed low speed, 0.025 m/s |
| B2 | Acceleration-limited ramp, nominal 0.050 m/s |
| P | Response-terminated micro-yaw initiation and recovery |
| B3H | Preset micro-yaw initiation and recovery |

P and B3H nominal speed is 0.050 m/s. Their transient commands differ, so equal nominal speed is not an assertion of identical velocity histories. The main proposed method is B3H for all wheelchair conditions and P for all patient conditions. Both variants are preserved in the ablation table; there is no condition-wise best-case envelope.

## Measurements and provenance

The original evaluation windows, trajectory interpolation, jerk filtering and repeat-spread definitions remain in the retained `analysis` functions. The patient-transfer evaluation concerns 0.70 m extraction after insertion; the wheelchair matched window is 0.90 m reverse travel. Table columns explicitly distinguish maximum lateral/yaw deviation, peak jerk, travel time and repeat-trajectory spread; none is IJCAS path-tracking RMS.

- [All 80 condition-method summaries](data/all_platform_condition_metrics.csv)
- [Condition-paired proposed/B2 ratios and reductions](data/proposed_vs_ramp_paired_metrics.csv)
- [Two platform-fixed summaries](data/platform_proposed_summary.csv)
- [Preset/response-terminated variant comparison](data/platform_variant_ablation.csv)
- [Patient original metric table](data/patient_transport_final9_metrics.csv)
- [Wheelchair original metric table](data/wheelchair_final7_metrics.csv)

These are unchanged outputs from the locally recomputed six-table verification, based on the 240 original records. The source labels in CSVs identify original runs; public CSVs omit raw traces and absolute local paths. Regeneration uses `ros2 run cics_experiments reproduce_results --figures` with the original bundle. Each generated table is compared with the original for columns, row count, labels and numeric values (`rel_tol=1e-12`, `abs_tol=1e-9`).

For a metric X, each condition's reduction is `100*(X_B2-X_proposed)/X_B2`; reported reductions are the median across the platform's seven/nine conditions. Negative reductions indicate worse performance. They are not reductions obtained by dividing two cross-condition medians.

## Interpretation and limits

The fixed proposed implementations reached their targets and completed caster reversal in 21/21 wheelchair and 27/27 patient records. This does not mean other methods failed all these targets. Accuracy improvements coexist with jerk/time trade-offs. The repeats are from the configured simulation, not a clinical cohort or evidence of stochastic robustness to all contact conditions.

After packaging, two representative conditions were newly executed three times each and matched common original trace columns and recomputed metrics. They are commissioning checks, not additions to the original 240-trial statistics or a full replay of the matrix. No new trials were performed to prepare this public release.
