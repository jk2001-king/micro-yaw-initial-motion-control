# Troubleshooting

| Symptom | Check |
|---|---|
| `ros2` cannot find the package | Build from the source workspace with system Python; source `install/setup.bash` in that terminal |
| SDK import or ROS ABI failure | Use Isaac Python 3.11, not ROS Python 3.10; do not manually combine binary PYTHONPATHs |
| Missing original traces | Restore `archive/original` with unchanged run labels; public summary CSVs are insufficient |
| `check` fails but selector works | `check` audits raw evidence; `stage:=check` only selects and reports a recipe |
| Existing output rejected | Choose a new folder; source, archive and previous outputs are protected |
| Static gate fails | Preserve the failure and inspect contact/support/model preparation; do not lower thresholds to force a run |
| Process exits zero but no success receipt | Check `failure.json` and `completed.json`; a zero Kit exit alone is not trial completion |
| Pause not confirmed | The original queued Pause must be acknowledged by the SDK update; do not substitute an automatic retry |

An offline check, model preparation, source test or previously recorded success is not authority to move. New simulation requires explicit `stage:=run allow_motion:=true`. Keep new failures and never replace a recorded paper failure with a successful retry.
