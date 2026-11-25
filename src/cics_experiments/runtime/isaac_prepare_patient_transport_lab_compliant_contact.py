"""Session-only A/B trial of compliant six-wheel contact on the capsule USD.

Run after the temporary floor preparation with Timeline stopped. The 100 kN/m
and 1.5 kN s/m values are inherited screening assumptions from the wheelchair
experiment, not measurements of the laboratory patient-transport platform.
"""

import omni.timeline
import omni.usd
from pxr import PhysxSchema, Usd, UsdPhysics, UsdShade


ROOT = "/World/patient_transport_lab_screening"
GROUND = "/World/PatientTransportLabGround/CollisionBox"
MATERIAL_PATH = "/World/PatientTransportLabGround/WheelGroundContact"
WHEEL_NAMES = (
    "front_active_drive_wheel_link",
    "rear_active_drive_wheel_link",
    "front_left_caster_wheel_link",
    "front_right_caster_wheel_link",
    "rear_left_caster_wheel_link",
    "rear_right_caster_wheel_link",
)
STATIC_FRICTION = 0.8
DYNAMIC_FRICTION = 0.6
STIFFNESS_N_M = 100000.0
DAMPING_N_S_M = 1500.0

if not omni.timeline.get_timeline_interface().is_stopped():
    raise RuntimeError("Press Stop before preparing compliant contact")
stage = omni.usd.get_context().get_stage()
if stage is None or not stage.GetPrimAtPath(GROUND).IsValid():
    raise RuntimeError("Prepare the temporary laboratory floor first")
if stage.GetPrimAtPath(MATERIAL_PATH).IsValid():
    raise RuntimeError("Compliant material already exists; reopen the clean USD")

wheel_bodies = []
for name in WHEEL_NAMES:
    body = stage.GetPrimAtPath(ROOT + "/" + name)
    if not body.IsValid():
        raise RuntimeError(f"Missing wheel body: {name}")
    colliders = [
        prim for prim in Usd.PrimRange(body, Usd.TraverseInstanceProxies())
        if prim.HasAPI(UsdPhysics.CollisionAPI)
        and UsdPhysics.CollisionAPI(prim).GetCollisionEnabledAttr().Get() is not False
    ]
    if len(colliders) != 1 or colliders[0].GetTypeName() != "Capsule":
        raise RuntimeError(f"Expected one enabled Capsule collider for {name}")
    wheel_bodies.append(body)

previous_target = stage.GetEditTarget()
stage.SetEditTarget(stage.GetSessionLayer())
try:
    material = UsdShade.Material.Define(stage, MATERIAL_PATH)
    physics = UsdPhysics.MaterialAPI.Apply(material.GetPrim())
    physics.CreateStaticFrictionAttr(STATIC_FRICTION).Set(STATIC_FRICTION)
    physics.CreateDynamicFrictionAttr(DYNAMIC_FRICTION).Set(DYNAMIC_FRICTION)
    physics.CreateRestitutionAttr(0.0).Set(0.0)
    physx = PhysxSchema.PhysxMaterialAPI.Apply(material.GetPrim())
    physx.CreateCompliantContactAccelerationSpringAttr(False).Set(False)
    physx.CreateCompliantContactStiffnessAttr(STIFFNESS_N_M).Set(STIFFNESS_N_M)
    physx.CreateCompliantContactDampingAttr(DAMPING_N_S_M).Set(DAMPING_N_S_M)
    physx.CreateFrictionCombineModeAttr("max").Set("max")
    physx.CreateRestitutionCombineModeAttr("min").Set("min")
    physx.CreateDampingCombineModeAttr("max").Set("max")
    for prim in [stage.GetPrimAtPath(GROUND), *wheel_bodies]:
        UsdShade.MaterialBindingAPI.Apply(prim).Bind(
            material, UsdShade.Tokens.strongerThanDescendants, "physics"
        )
finally:
    stage.SetEditTarget(previous_target)

globals()["patient_transport_lab_contact_label"] = "capsule_compliant_01"
print("=== Laboratory patient-transport compliant contact trial prepared ===")
print("material:", MATERIAL_PATH)
print("bound to floor and six wheel bodies")
print("provisional static/dynamic friction:", STATIC_FRICTION, DYNAMIC_FRICTION)
print("provisional contact stiffness/damping:", STIFFNESS_N_M, DAMPING_N_S_M)
print("restitution: 0; caster forks remain collision-enabled")
print("Session Layer only. Do not save the stage.")
print("Next: run isaac_patient_transport_lab_static_contact_gate.py.")
