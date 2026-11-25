# Architecture

## Responsibility and entry points

| Layer | Implementation | Responsibility |
|---|---|---|
| ROS interface | `launch/experiment.launch.py`, `cics_experiments/cli.py` | Explicit selector, motion consent, SDK process and completion receipt |
| Frozen recipes | `cics_experiments/protocol.py` | Extract original condition/method definitions without importing Kit |
| SDK orchestration | `cics_experiments/sdk_runner.py` | New model, original preparation/static gate, selected original three-repeat arm, queued Pause confirmation |
| Patient controller | `runtime/isaac_patient_transport_lab_lateral_repeat_gate.py` | Patient-transfer dynamic trial |
| Wheelchair controller | `runtime/isaac_wheelchair_lab_natural_flip_gate.py` | Wheelchair dynamic trial |
| Original matrix | `runtime/isaac_*cics26*next.py` | Condition lists and exact original call parameters |
| Evaluation | `analysis/`, `cics_experiments/reproduce.py` | Original trace metrics and six-table numerical comparison |

`runtime` files are retained dependency modules, not separate user-facing scripts to copy into Script Editor. Their original names remain for traceability. The wheelchair scheduler's `final9` name is historical; its retained matrix has seven conditions.

## Control scope

Micro-yaw initiation guides caster realignment; body-yaw recovery subsequently reduces heading error. The preset variant uses configured initiation. The response-terminated variant ends initiation from measured body response. The main realization is fixed as wheelchair B3H and patient-transfer P, not selected after viewing each condition's performance.

The patient protocol inserts 0.70 m, stops, and extracts 0.70 m without resetting the caster configuration formed during insertion. The wheelchair protocol uses its original preparation and 0.90 m reverse-travel window. The SDK uses simulator observations; this is not an AMCL/encoder-only controller or IJCAS Nav2 hospital system.

## Source and evidence

`config/source_manifest.json` maps 21 retained source files to original paths and SHA-256 values. Control, safety and metric function/class ASTs are unchanged; relocation is in module-level paths/imports. The new ROS/process wrapper is separately tested. Original data remain under `archive/original`; new outputs use `log/cics`. The predecessor Git database remains under `archive/migration/original_git`, separate from the public CICS Git history.
