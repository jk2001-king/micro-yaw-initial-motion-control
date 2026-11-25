"""Session-only powered standstill for the two active steering/drive modules.

Run after the capsule compliant-contact preparation while Timeline is stopped.
Passive caster swivel and wheel joints are not changed. This represents active
motor holding at zero command; gains are provisional simulation settings.
"""

import omni.timeline
import omni.usd
from pxr import UsdPhysics


ROOT = "/World/patient_transport_lab_screening"
MATERIAL = "/World/PatientTransportLabGround/WheelGroundContact"
JOINT_SETTINGS = {
    "base_link_to_front_active_steering": ("steering", 30.0, 5.0, 250.0),
    "base_link_to_rear_active_steering": ("steering", 30.0, 5.0, 250.0),
    "front_active_steering_to_drive_wheel": ("drive", 0.0, 50.0, 500.0),
    "rear_active_steering_to_drive_wheel": ("drive", 0.0, 50.0, 500.0),
}

if not omni.timeline.get_timeline_interface().is_stopped():
    raise RuntimeError("Press Stop before preparing powered idle")
stage = omni.usd.get_context().get_stage()
if stage is None or not stage.GetPrimAtPath(MATERIAL).IsValid():
    raise RuntimeError("Prepare the temporary floor and compliant material first")

for name in JOINT_SETTINGS:
    path = f"{ROOT}/joints/{name}"
    prim = stage.GetPrimAtPath(path)
    if not prim.IsValid() or not prim.IsA(UsdPhysics.RevoluteJoint):
        raise RuntimeError(f"Missing active revolute joint: {path}")
    if not UsdPhysics.DriveAPI.Get(prim, "angular"):
        raise RuntimeError(f"Missing angular drive on {path}")

previous_target = stage.GetEditTarget()
stage.SetEditTarget(stage.GetSessionLayer())
try:
    for name, (role, stiffness, damping, max_force) in JOINT_SETTINGS.items():
        prim = stage.GetPrimAtPath(f"{ROOT}/joints/{name}")
        drive = UsdPhysics.DriveAPI.Get(prim, "angular")
        drive.CreateTypeAttr("force").Set("force")
        drive.CreateStiffnessAttr(stiffness).Set(stiffness)
        drive.CreateDampingAttr(damping).Set(damping)
        drive.CreateMaxForceAttr(max_force).Set(max_force)
        drive.CreateTargetPositionAttr(0.0).Set(0.0)
        drive.CreateTargetVelocityAttr(0.0).Set(0.0)
finally:
    stage.SetEditTarget(previous_target)

globals()["patient_transport_lab_contact_label"] = "capsule_compliant_powered_idle_01"
print("=== Laboratory patient-transport powered idle prepared ===")
for name, (role, stiffness, damping, max_force) in JOINT_SETTINGS.items():
    print(
        name, role,
        f"stiffness={stiffness}, damping={damping}, maxForce={max_force}",
    )
print("Steering target is zero angle; wheel target is zero velocity.")
print("Passive caster swivel and wheel joints remain unchanged.")
print("PROVISIONAL CONTROL GAINS. Session Layer only; do not save the stage.")
print("Next: run isaac_patient_transport_lab_static_contact_gate.py.")
