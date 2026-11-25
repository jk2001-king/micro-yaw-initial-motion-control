# Setup

## Environment and build

Use Ubuntu 22.04, ROS 2 Humble, colcon and rosdep. Install NVIDIA Isaac Sim 5.0 with its supported GPU/driver environment separately. ROS uses system Python 3.10; SDK execution uses its own Python 3.11. Do not add ROS Python 3.10 binary paths to SDK Python.

```bash
git clone https://github.com/jk2001-king/micro-yaw-initial-motion-control.git ~/CICS_ws
cd ~/CICS_ws
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --base-paths src --symlink-install
source install/setup.bash
ros2 run cics_experiments experiment --list
```

SDK execution needs NumPy and the original analysis stack uses SciPy and Matplotlib. Specify `sdk_python:=/path/to/isaac_env/bin/python` when the launcher cannot find your installation. Keep the repository inside a source workspace so `src/cics_experiments/package.xml` identifies its root.

## Code-only clone versus original-result reproduction

The repository includes the final code, model layers and summary CSV/PNG assets. It excludes original traces, development archives and old Git objects. Listing recipes and `stage:=check` work without the original recordings. Model preparation/new trials additionally require a working Isaac SDK. Recomputing published measurements requires the original bundle:

```text
CICS_ws/archive/original/
├── isaac/scripts/                         # original source snapshots for AST/hash audit
└── studies/CICS26/
    ├── results/patient_transport_lab/      # original trace_r01..03.csv / summary.csv
    ├── results/wheelchair_lab/             # original natural_flip_r01..03.csv / summary.csv
    └── paper_assets/                      # six original comparison tables
```

Keep original relative paths and run labels; never replace missing traces with summary CSVs. Public data hosting is not currently provided. The local migration audit additionally requires `archive/migration/original_manifest.json` and the original Git directory. That PC-specific preservation check is not a portable clone validation test.

```bash
# Requires original traces, no new motion:
ros2 run cics_experiments check
ros2 run cics_experiments reproduce_results
```

New outputs go to `log/cics/experiments` or `log/cics/reproduced`, never into `archive/original`. No helper sourcing script or old-workspace symlink is required.
