"""Operational standstill gate: powered drive wheels, passive caster joints.

Requires the capsule import and an already prepared ground/material session.
Run stopped. Session Layer only.
"""

import omni.timeline
import omni.usd
from pxr import Gf, Usd, UsdGeom, UsdPhysics


ROBOT_ROOT = "/World/wheelchair_lab_screening"
GROUND = "/World/WheelchairGround/CollisionBox"
INITIAL_ROOT_Z_M = 0.3255
DRIVE_GAIN = 50.0

JOINT_SETTINGS = {
    "base_link_to_left_caster": (0.060287, 50.0, "passive caster swivel"),
    "base_link_to_right_caster": (0.060287, 50.0, "passive caster swivel"),
    "left_caster_to_left_caster_wheel": (0.002189, 50.0, "passive caster roll"),
    "right_caster_to_right_caster_wheel": (0.002189, 50.0, "passive caster roll"),
    "base_link_to_left_wheel": (DRIVE_GAIN, 250.0, "powered zero-speed hold"),
    "base_link_to_right_wheel": (DRIVE_GAIN, 250.0, "powered zero-speed hold"),
}

timeline = omni.timeline.get_timeline_interface()
if timeline.is_playing():
    raise RuntimeError("Press Stop before preparing the capsule powered-hold gate")

stage = omni.usd.get_context().get_stage()
robot_root = stage.GetPrimAtPath(ROBOT_ROOT)
if not robot_root.IsValid() or not stage.GetPrimAtPath(GROUND).IsValid():
    raise RuntimeError("Capsule wheelchair or prepared physics ground is missing")

colliders = [
    p for p in Usd.PrimRange(robot_root, Usd.TraverseInstanceProxies())
    if p.HasAPI(UsdPhysics.CollisionAPI)
]
capsule_count = sum(p.GetTypeName() == "Capsule" for p in colliders)
if capsule_count != 4:
    raise RuntimeError(f"Expected 4 wheel Capsules, got {capsule_count}")

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

    configured = []
    joints_root = stage.GetPrimAtPath(ROBOT_ROOT + "/joints")
    for prim in Usd.PrimRange(joints_root):
        if not prim.IsA(UsdPhysics.RevoluteJoint):
            continue
        name = prim.GetName()
        if name not in JOINT_SETTINGS:
            raise RuntimeError(f"Unexpected revolute joint: {prim.GetPath()}")
        damping, max_force, role = JOINT_SETTINGS[name]
        joint = UsdPhysics.Joint(prim)
        joint.CreateCollisionEnabledAttr(False).Set(False)
        drive = UsdPhysics.DriveAPI.Get(prim, "angular")
        if not drive:
            drive = UsdPhysics.DriveAPI.Apply(prim, "angular")
        drive.CreateTypeAttr("force").Set("force")
        drive.CreateStiffnessAttr(0.0).Set(0.0)
        drive.CreateDampingAttr(damping).Set(damping)
        drive.CreateTargetPositionAttr(0.0).Set(0.0)
        drive.CreateTargetVelocityAttr(0.0).Set(0.0)
        drive.CreateMaxForceAttr(max_force).Set(max_force)
        configured.append((name, damping, max_force, role))
finally:
    stage.SetEditTarget(previous_target)

if len(configured) != 6:
    raise RuntimeError(f"Configured {len(configured)} joints, expected 6")

print("=== Measured wheelchair capsule powered-hold gate prepared ===")
for name, damping, max_force, role in sorted(configured):
    print(f"  {name}: Strength=0, Damping={damping:.6f}, maxForce={max_force:.1f} ({role})")
print("wheel capsule count:", capsule_count)
print("prepared root z:", INITIAL_ROOT_Z_M, "m")
print("Caster joints are passive; only the two drive wheels hold zero speed.")
print("Session Layer only. Do not save the stage.")
