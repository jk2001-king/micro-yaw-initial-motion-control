# Assets and licensing

- NVIDIA Isaac Sim 5.0 / Omniverse / PhysX are external dependencies, installed under their own terms. This repository does not include their SDK/runtime binaries.
- ROS 2 Humble, ament, launch and colcon are system dependencies, not vendored implementations.
- NumPy, SciPy and Matplotlib retain their upstream licenses and are installed separately.
- `patient_transport_description` and `wheelchair_description` retain their original `Proprietary` package declarations. They include the final laboratory model layers and required meshes, not all historical CAD/development assets.
- `cics_experiments` retains `UNLICENSED`. Public visibility is not a grant of an umbrella MIT/Apache license.
- Source provenance is in `config/source_manifest.json`. Earlier prototype code remains in the separate `patient-transport-initial-motion` repository and is not substituted for the final micro-yaw control kernel.

Original run data, scientific manuscripts, development snapshots, credentials and generated build products are excluded. No claim is made that simulation completion establishes patient-transport safety or hardware certification.
