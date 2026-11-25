"""Bind an explicit physics material to the floor and all wheel bodies.

Run stopped after the locked-articulation diagnostic. Session Layer only.
"""

import omni.timeline
import omni.usd
from pxr import Gf, PhysxSchema, Sdf, UsdGeom, UsdPhysics, UsdShade


ROBOT_ROOT = "/World/wheelchair_lab_screening"
GROUND = "/World/WheelchairGround/CollisionBox"
MATERIAL_PATH = "/World/WheelchairLabMaterials/WheelGroundContact"
INITIAL_ROOT_Z_M = 0.3255
STATIC_FRICTION = 0.8
DYNAMIC_FRICTION = 0.6
WHEEL_BODIES = (
    ROBOT_ROOT + "/left_wheel_link",
    ROBOT_ROOT + "/right_wheel_link",
    ROBOT_ROOT + "/left_caster_wheel_link",
    ROBOT_ROOT + "/right_caster_wheel_link",
)

timeline = omni.timeline.get_timeline_interface()
if timeline.is_playing():
    raise RuntimeError("Press Stop before preparing the contact-material test")

stage = omni.usd.get_context().get_stage()
robot_root = stage.GetPrimAtPath(ROBOT_ROOT)
ground = stage.GetPrimAtPath(GROUND)
if not robot_root.IsValid() or not ground.IsValid():
    raise RuntimeError("Wheelchair or physics ground is missing")

previous_target = stage.GetEditTarget()
stage.SetEditTarget(stage.GetSessionLayer())
try:
    xformable = UsdGeom.Xformable(robot_root)
    local = xformable.GetLocalTransformation()
    if isinstance(local, tuple):
        local = local[0]
    local = Gf.Matrix4d(local)
    translation = local.ExtractTranslation()
    local.SetTranslateOnly(Gf.Vec3d(translation[0], translation[1], INITIAL_ROOT_Z_M))
    op_attr = robot_root.GetAttribute("xformOp:transform:groundSupport")
    root_op = UsdGeom.XformOp(op_attr) if op_attr and op_attr.IsValid() else xformable.AddTransformOp(
        UsdGeom.XformOp.PrecisionDouble, "groundSupport"
    )
    root_op.Set(local)
    xformable.SetXformOpOrder([root_op], resetXformStack=False)

    material = UsdShade.Material.Define(stage, MATERIAL_PATH)
    physics_material = UsdPhysics.MaterialAPI.Apply(material.GetPrim())
    physics_material.CreateStaticFrictionAttr(STATIC_FRICTION).Set(STATIC_FRICTION)
    physics_material.CreateDynamicFrictionAttr(DYNAMIC_FRICTION).Set(DYNAMIC_FRICTION)
    physics_material.CreateRestitutionAttr(0.0).Set(0.0)
    try:
        physx_material = PhysxSchema.PhysxMaterialAPI.Apply(material.GetPrim())
        physx_material.CreateFrictionCombineModeAttr("max").Set("max")
        physx_material.CreateRestitutionCombineModeAttr("min").Set("min")
    except Exception as exc:
        print("PhysX combine-mode note:", type(exc).__name__, str(exc))

    bound = []
    for path in (GROUND,) + WHEEL_BODIES:
        prim = stage.GetPrimAtPath(path)
        if not prim.IsValid():
            raise RuntimeError(f"Missing material binding target: {path}")
        UsdShade.MaterialBindingAPI.Apply(prim).Bind(
            material,
            UsdShade.Tokens.strongerThanDescendants,
            "physics",
        )
        bound.append(path)
finally:
    stage.SetEditTarget(previous_target)

print("=== Measured wheelchair explicit contact material prepared ===")
print("material:", MATERIAL_PATH)
print("static/dynamic friction:", STATIC_FRICTION, DYNAMIC_FRICTION)
print("restitution: 0.0")
print("binding strength: strongerThanDescendants, purpose=physics")
print("bound prims:")
for path in bound:
    print("  ", path)
print("prepared root z:", INITIAL_ROOT_Z_M, "m")
print("Powered zero-speed hold is applied in the next preparation step.")
print("Session Layer only. Do not save the stage.")
