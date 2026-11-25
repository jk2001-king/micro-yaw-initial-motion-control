"""Prepare a temporary flat floor for the laboratory platform's six-wheel gate.

Run in Isaac Sim Script Editor while the Timeline is stopped. This script writes
only to the Session Layer; do not save the stage after running it.
"""

import omni.timeline
import omni.usd
from pxr import Gf, PhysxSchema, Sdf, Usd, UsdGeom, UsdPhysics


ROBOT_ROOT = "/World/patient_transport_lab_screening"
GROUND_ROOT = "/World/PatientTransportLabGround"
GROUND_COLLIDER = GROUND_ROOT + "/CollisionBox"
GROUND_MATERIAL = GROUND_ROOT + "/ProvisionalMaterial"
PHYSICS_HZ = 120
PROVISIONAL_STATIC_FRICTION = 0.8
PROVISIONAL_DYNAMIC_FRICTION = 0.6

timeline = omni.timeline.get_timeline_interface()
if not timeline.is_stopped():
    raise RuntimeError("Press Stop before preparing the static contact gate")

stage = omni.usd.get_context().get_stage()
if stage is None or not stage.GetPrimAtPath(ROBOT_ROOT + "/base_link").IsValid():
    raise RuntimeError("Open the measured laboratory platform USD first")
if stage.GetPrimAtPath(GROUND_ROOT).IsValid():
    raise RuntimeError(
        f"{GROUND_ROOT} already exists. Reopen the clean USD before preparing again."
    )

external_colliders = [
    str(prim.GetPath())
    for prim in stage.Traverse()
    if prim.HasAPI(UsdPhysics.CollisionAPI)
    and not prim.GetPath().HasPrefix(Sdf.Path(ROBOT_ROOT))
    and UsdPhysics.CollisionAPI(prim).GetCollisionEnabledAttr().Get() is not False
]
if external_colliders:
    raise RuntimeError(
        "Existing non-robot colliders found; no second floor was created: "
        + ", ".join(external_colliders)
    )

scenes = [prim for prim in stage.Traverse() if prim.IsA(UsdPhysics.Scene)]
if len(scenes) != 1:
    raise RuntimeError(f"Expected exactly one PhysicsScene, found {len(scenes)}")

previous_target = stage.GetEditTarget()
stage.SetEditTarget(stage.GetSessionLayer())
try:
    UsdGeom.Xform.Define(stage, GROUND_ROOT)
    cube = UsdGeom.Cube.Define(stage, GROUND_COLLIDER)
    cube.CreateSizeAttr(1.0)
    xform = UsdGeom.Xformable(cube.GetPrim())
    xform.AddTranslateOp().Set(Gf.Vec3d(0.0, 0.0, -0.05))
    xform.AddScaleOp().Set(Gf.Vec3f(20.0, 20.0, 0.1))
    UsdPhysics.CollisionAPI.Apply(cube.GetPrim()).CreateCollisionEnabledAttr(True).Set(True)

    material_prim = stage.DefinePrim(GROUND_MATERIAL, "Material")
    material = UsdPhysics.MaterialAPI.Apply(material_prim)
    material.CreateStaticFrictionAttr(PROVISIONAL_STATIC_FRICTION).Set(
        PROVISIONAL_STATIC_FRICTION
    )
    material.CreateDynamicFrictionAttr(PROVISIONAL_DYNAMIC_FRICTION).Set(
        PROVISIONAL_DYNAMIC_FRICTION
    )
    material.CreateRestitutionAttr(0.0).Set(0.0)
    cube.GetPrim().CreateRelationship("material:binding:physics").SetTargets(
        [Sdf.Path(GROUND_MATERIAL)]
    )

    PhysxSchema.PhysxSceneAPI.Apply(scenes[0]).CreateTimeStepsPerSecondAttr(
        PHYSICS_HZ
    ).Set(PHYSICS_HZ)
finally:
    stage.SetEditTarget(previous_target)

globals()["patient_transport_lab_contact_label"] = "baseline"
print("=== Laboratory patient-transport temporary floor prepared ===")
print("floor:", GROUND_COLLIDER, "top z=0 m")
print("physics rate:", PHYSICS_HZ, "Hz")
print(
    "PROVISIONAL floor static/dynamic friction:",
    PROVISIONAL_STATIC_FRICTION,
    PROVISIONAL_DYNAMIC_FRICTION,
)
print("Session Layer only. Do not save the stage.")
print("Next: run isaac_patient_transport_lab_static_contact_gate.py while stopped.")
