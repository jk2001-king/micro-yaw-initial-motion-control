"""Validate passive damping and prepare a near-contact ground-support test.

Run on a clean re-import with the Timeline stopped. This helper preserves the
imported sleep/stabilization settings and joint damping. Session Layer only.
"""

import math

import omni.timeline
import omni.usd
from pxr import Gf, PhysxSchema, Sdf, Usd, UsdGeom, UsdPhysics


ROBOT_ROOT = "/World/wheelchair_lab_screening"
ARTICULATION_ROOT = ROBOT_ROOT + "/base_link"
GROUND_ROOT = "/World/WheelchairGround"
GROUND_COLLIDER = GROUND_ROOT + "/CollisionBox"
MATERIAL_ROOT = "/World/WheelchairLabMaterials"
GROUND_MATERIAL = MATERIAL_ROOT + "/Ground"
INITIAL_ROOT_Z_M = 0.3255
PHYSICS_HZ = 120
PROVISIONAL_FRICTION = 0.7

EXPECTED_DAMPING = {
    "base_link_to_left_caster": 0.060287,
    "base_link_to_right_caster": 0.060287,
    "base_link_to_left_wheel": 0.049817,
    "base_link_to_right_wheel": 0.049817,
    "left_caster_to_left_caster_wheel": 0.002189,
    "right_caster_to_right_caster_wheel": 0.002189,
}

timeline = omni.timeline.get_timeline_interface()
if timeline.is_playing():
    raise RuntimeError("Press Stop before preparing the wheelchair ground-support test")

stage = omni.usd.get_context().get_stage()
robot_root = stage.GetPrimAtPath(ROBOT_ROOT)
articulation = stage.GetPrimAtPath(ARTICULATION_ROOT)
if not robot_root.IsValid() or not articulation.IsValid():
    raise RuntimeError("The re-imported wheelchair prims are missing")

joint_rows = []
for prim in Usd.PrimRange(robot_root):
    if not prim.IsA(UsdPhysics.RevoluteJoint):
        continue
    drive = UsdPhysics.DriveAPI.Get(prim, "angular")
    if not drive:
        raise RuntimeError(f"Missing angular drive schema: {prim.GetPath()}")
    stiffness = float(drive.GetStiffnessAttr().Get() or 0.0)
    damping = float(drive.GetDampingAttr().Get() or 0.0)
    joint_rows.append((prim.GetName(), stiffness, damping))

observed_names = {name for name, _, _ in joint_rows}
if observed_names != set(EXPECTED_DAMPING):
    raise RuntimeError(f"Unexpected joint set: {sorted(observed_names)}")
bad_stiffness = [(name, value) for name, value, _ in joint_rows if abs(value) > 1.0e-8]
bad_damping = [
    (name, value, EXPECTED_DAMPING[name])
    for name, _, value in joint_rows
    if not math.isclose(value, EXPECTED_DAMPING[name], rel_tol=0.03, abs_tol=0.001)
]
if bad_stiffness:
    raise RuntimeError(f"Nonzero joint Strength detected: {bad_stiffness}")
if bad_damping:
    raise RuntimeError(f"Unexpected joint Damping detected: {bad_damping}")

art_api = PhysxSchema.PhysxArticulationAPI(articulation)
self_collision = art_api.GetEnabledSelfCollisionsAttr().Get()
sleep_threshold = art_api.GetSleepThresholdAttr().Get()
stabilization_threshold = art_api.GetStabilizationThresholdAttr().Get()
if self_collision is True:
    raise RuntimeError("Articulation self-collision must be disabled")

physics_scenes = [p for p in stage.Traverse() if p.IsA(UsdPhysics.Scene)]
if len(physics_scenes) != 1:
    raise RuntimeError(f"Expected one PhysicsScene, got {[str(p.GetPath()) for p in physics_scenes]}")

previous_target = stage.GetEditTarget()
stage.SetEditTarget(stage.GetSessionLayer())
try:
    xformable = UsdGeom.Xformable(robot_root)
    local = xformable.GetLocalTransformation()
    if isinstance(local, tuple):
        local = local[0]
    local = Gf.Matrix4d(local)
    previous_translation = local.ExtractTranslation()
    local.SetTranslateOnly(Gf.Vec3d(previous_translation[0], previous_translation[1], INITIAL_ROOT_Z_M))
    op_attr = robot_root.GetAttribute("xformOp:transform:groundSupport")
    root_op = UsdGeom.XformOp(op_attr) if op_attr and op_attr.IsValid() else xformable.AddTransformOp(
        UsdGeom.XformOp.PrecisionDouble, "groundSupport"
    )
    root_op.Set(local)
    xformable.SetXformOpOrder([root_op], resetXformStack=False)

    if stage.GetPrimAtPath(GROUND_ROOT).IsValid():
        stage.RemovePrim(GROUND_ROOT)
    UsdGeom.Xform.Define(stage, GROUND_ROOT)
    floor = UsdGeom.Cube.Define(stage, GROUND_COLLIDER)
    floor.CreateSizeAttr(1.0)
    floor_xform = UsdGeom.Xformable(floor.GetPrim())
    floor_xform.AddTranslateOp().Set(Gf.Vec3d(0.0, 0.0, -0.05))
    floor_xform.AddScaleOp().Set(Gf.Vec3f(20.0, 20.0, 0.1))
    UsdPhysics.CollisionAPI.Apply(floor.GetPrim()).CreateCollisionEnabledAttr(True).Set(True)

    stage.DefinePrim(MATERIAL_ROOT, "Scope")
    material_prim = stage.DefinePrim(GROUND_MATERIAL, "Material")
    material = UsdPhysics.MaterialAPI.Apply(material_prim)
    material.CreateStaticFrictionAttr(PROVISIONAL_FRICTION).Set(PROVISIONAL_FRICTION)
    material.CreateDynamicFrictionAttr(PROVISIONAL_FRICTION).Set(PROVISIONAL_FRICTION)
    material.CreateRestitutionAttr(0.0).Set(0.0)
    floor.GetPrim().CreateRelationship("material:binding:physics").SetTargets([Sdf.Path(GROUND_MATERIAL)])

    PhysxSchema.PhysxSceneAPI.Apply(physics_scenes[0]).CreateTimeStepsPerSecondAttr(PHYSICS_HZ).Set(PHYSICS_HZ)
    rigid_count = 0
    for prim in Usd.PrimRange(robot_root):
        if prim.HasAPI(UsdPhysics.RigidBodyAPI):
            PhysxSchema.PhysxRigidBodyAPI.Apply(prim).CreateDisableGravityAttr(False).Set(False)
            rigid_count += 1
finally:
    stage.SetEditTarget(previous_target)

print("=== Measured wheelchair ground stage prepared ===")
print("validated joint Strength/Damping:")
for name, stiffness, damping in sorted(joint_rows):
    print(f"  {name}: Strength={stiffness:.6f}, Damping={damping:.6f}")
print("self-collision:", self_collision)
print("preserved sleep threshold:", sleep_threshold)
print("preserved stabilization threshold:", stabilization_threshold)
print("prepared root z:", INITIAL_ROOT_Z_M, "m")
print("initial caster/drive clearance: 0.000500 / 0.000500 m")
print("physics ground:", GROUND_COLLIDER)
print("physics rate:", PHYSICS_HZ, "Hz")
print("provisional ground friction:", PROVISIONAL_FRICTION)
print("rigid bodies with gravity enabled:", rigid_count)
print("Session Layer only. Do not save the stage.")
