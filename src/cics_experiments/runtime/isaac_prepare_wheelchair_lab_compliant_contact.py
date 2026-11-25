"""Apply provisional tire compliance before repeating the static-load gate.

The values approximate about 1--3 mm static tire compression at the measured
wheel loads. They are screening values and require later physical calibration.
"""

from cics_experiments.paths import workspace_root, resolve_path

from pathlib import Path

import omni.timeline
import omni.usd
from pxr import PhysxSchema


WORKSPACE = workspace_root()
POWERED_HOLD = resolve_path('isaac/scripts/isaac_prepare_wheelchair_lab_powered_hold.py')
MATERIAL_PATH = "/World/WheelchairLabMaterials/WheelGroundContact"
CONTACT_STIFFNESS_N_M = 100000.0
CONTACT_DAMPING_N_S_M = 1500.0

timeline = omni.timeline.get_timeline_interface()
if timeline.is_playing():
    raise RuntimeError("Press Stop before preparing compliant contact")

# Restore the operational state in case a previous diagnostic left joints locked.
exec(compile(POWERED_HOLD.read_text(), str(POWERED_HOLD), "exec"), globals())

stage = omni.usd.get_context().get_stage()
material = stage.GetPrimAtPath(MATERIAL_PATH)
if not material.IsValid():
    raise RuntimeError(f"Missing wheel-ground material: {MATERIAL_PATH}")

previous_target = stage.GetEditTarget()
stage.SetEditTarget(stage.GetSessionLayer())
try:
    api = PhysxSchema.PhysxMaterialAPI.Apply(material)
    api.CreateCompliantContactAccelerationSpringAttr(False).Set(False)
    api.CreateCompliantContactStiffnessAttr(CONTACT_STIFFNESS_N_M).Set(CONTACT_STIFFNESS_N_M)
    api.CreateCompliantContactDampingAttr(CONTACT_DAMPING_N_S_M).Set(CONTACT_DAMPING_N_S_M)
    api.CreateRestitutionCombineModeAttr("max").Set("max")
    api.CreateDampingCombineModeAttr("max").Set("max")
finally:
    stage.SetEditTarget(previous_target)

print("=== Measured wheelchair compliant tire-contact baseline prepared ===")
print("material:", MATERIAL_PATH)
print("contact stiffness:", CONTACT_STIFFNESS_N_M, "N/m")
print("contact damping:", CONTACT_DAMPING_N_S_M, "N s/m")
print("acceleration spring: False")
print("expected static compression range: approximately 1--3 mm")
print("powered zero-speed hold restored; caster joints remain passive")
print("PROVISIONAL SCREENING VALUES — physical calibration is still required.")
print("Run isaac_wheelchair_static_contact_load_gate.py next.")
