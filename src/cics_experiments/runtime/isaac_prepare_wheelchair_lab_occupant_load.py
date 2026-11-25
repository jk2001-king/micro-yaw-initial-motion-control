"""Apply a rigid seated-occupant equivalent to the measured wheelchair base.

The occupant is merged into base_link because it is assumed rigidly seated for
this screening experiment. Mass, combined CoM, and diagonal inertia are all
updated. The four wheel/fork link masses remain unchanged. Session Layer only.
"""

import numpy as np
import omni.timeline
import omni.usd
from pxr import Gf, Sdf, UsdPhysics


BASE_LINK = "/World/wheelchair_lab_screening/base_link"

OCCUPANT_MASS_KG = float(
    globals().get("wheelchair_lab_occupant_mass_kg", 50.0)
)
CONDITION_LABEL = str(
    globals().get("wheelchair_lab_occupant_condition_label", "occupied50_center")
)

# Imported empty-wheelchair base properties from wheelchair_lab_parametric.xacro.
EMPTY_BASE_MASS_KG = 36.0
EMPTY_BASE_COM_M = np.asarray([0.15264, 0.0, 0.10], dtype=float)
EMPTY_BASE_DIAGONAL_INERTIA_KG_M2 = np.asarray(
    [1.3308, 2.0175, 1.8483], dtype=float
)
NONBASE_MASS_KG = 4.0
TOTAL_EMPTY_MASS_KG = EMPTY_BASE_MASS_KG + NONBASE_MASS_KG

# Provisional seated-human equivalent. Coordinates are relative to base_link,
# whose origin is the drive-axle midpoint at drive-wheel center height.
OCCUPANT_COM_M = np.asarray(
    globals().get("wheelchair_lab_occupant_com_m", (0.120, 0.0, 0.500)),
    dtype=float,
)
OCCUPANT_BOX_DIMS_M = np.asarray([0.300, 0.340, 0.700], dtype=float)

# Remaining wheel/fork x moment and effective support locations.
NONBASE_X_MOMENT_KG_M = 2.0 * 0.25 * 0.500 + 2.0 * 0.25 * 0.510
CASTER_SUPPORT_X_M = 0.510

if not 0.0 <= OCCUPANT_MASS_KG <= 100.0:
    raise RuntimeError("wheelchair_lab_occupant_mass_kg must be in [0, 100] kg")
if OCCUPANT_COM_M.shape != (3,) or not np.all(np.isfinite(OCCUPANT_COM_M)):
    raise RuntimeError(
        "wheelchair_lab_occupant_com_m must contain three finite values (x, y, z)"
    )

timeline = omni.timeline.get_timeline_interface()
if timeline.is_playing() or not timeline.is_stopped():
    raise RuntimeError("Press Stop before changing the occupant-load condition")

stage = omni.usd.get_context().get_stage()
base = stage.GetPrimAtPath(BASE_LINK)
if not base.IsValid():
    raise RuntimeError(f"Missing measured-wheelchair base link: {BASE_LINK}")


def _box_diagonal_inertia(mass, dims):
    x, y, z = [float(value) for value in dims]
    return np.asarray(
        [
            mass * (y * y + z * z) / 12.0,
            mass * (x * x + z * z) / 12.0,
            mass * (x * x + y * y) / 12.0,
        ],
        dtype=float,
    )


combined_base_mass = EMPTY_BASE_MASS_KG + OCCUPANT_MASS_KG
combined_base_com = (
    EMPTY_BASE_MASS_KG * EMPTY_BASE_COM_M + OCCUPANT_MASS_KG * OCCUPANT_COM_M
) / combined_base_mass

base_offset = EMPTY_BASE_COM_M - combined_base_com
occupant_offset = OCCUPANT_COM_M - combined_base_com
base_parallel_axis = EMPTY_BASE_MASS_KG * (
    np.dot(base_offset, base_offset) - base_offset * base_offset
)
occupant_parallel_axis = OCCUPANT_MASS_KG * (
    np.dot(occupant_offset, occupant_offset) - occupant_offset * occupant_offset
)
combined_diagonal_inertia = (
    EMPTY_BASE_DIAGONAL_INERTIA_KG_M2
    + base_parallel_axis
    + _box_diagonal_inertia(OCCUPANT_MASS_KG, OCCUPANT_BOX_DIMS_M)
    + occupant_parallel_axis
)

total_mass = TOTAL_EMPTY_MASS_KG + OCCUPANT_MASS_KG
total_x_moment = (
    combined_base_mass * combined_base_com[0] + NONBASE_X_MOMENT_KG_M
)
total_com_x = total_x_moment / total_mass
estimated_caster_share = total_com_x / CASTER_SUPPORT_X_M

previous_target = stage.GetEditTarget()
stage.SetEditTarget(stage.GetSessionLayer())
try:
    mass_api = UsdPhysics.MassAPI.Apply(base)
    mass_api.CreateMassAttr(float(combined_base_mass)).Set(float(combined_base_mass))
    mass_api.CreateCenterOfMassAttr().Set(
        Gf.Vec3f(*[float(value) for value in combined_base_com])
    )
    mass_api.CreateDiagonalInertiaAttr().Set(
        Gf.Vec3f(*[float(value) for value in combined_diagonal_inertia])
    )
    base.CreateAttribute(
        "experiment:occupantMassKg", Sdf.ValueTypeNames.Double
    ).Set(OCCUPANT_MASS_KG)
    base.CreateAttribute(
        "experiment:totalMassKg", Sdf.ValueTypeNames.Double
    ).Set(total_mass)
    base.CreateAttribute(
        "experiment:targetCasterLoadShare", Sdf.ValueTypeNames.Double
    ).Set(estimated_caster_share)
    base.CreateAttribute(
        "experiment:loadShareCondition", Sdf.ValueTypeNames.String
    ).Set(CONDITION_LABEL)
    base.CreateAttribute(
        "experiment:occupantCom", Sdf.ValueTypeNames.Float3
    ).Set(Gf.Vec3f(*[float(value) for value in OCCUPANT_COM_M]))
finally:
    stage.SetEditTarget(previous_target)

# Keep the existing load-gate interface synchronized in the Script Editor.
wheelchair_lab_target_caster_share = float(estimated_caster_share)
wheelchair_lab_load_share_label = CONDITION_LABEL

print("=== Measured wheelchair occupant-load condition prepared ===")
print("condition:", CONDITION_LABEL)
print("empty wheelchair / occupant / total mass:", TOTAL_EMPTY_MASS_KG, OCCUPANT_MASS_KG, total_mass, "kg")
print("provisional occupant CoM relative to base_link:", tuple(OCCUPANT_COM_M), "m")
print("combined base mass:", round(combined_base_mass, 6), "kg")
print("combined base CoM:", tuple(round(float(v), 9) for v in combined_base_com), "m")
print("combined base diagonal inertia:", tuple(round(float(v), 9) for v in combined_diagonal_inertia), "kg m2")
print("estimated total CoM x:", round(float(total_com_x), 9), "m")
print("estimated caster share:", round(float(estimated_caster_share), 6))
print("PROVISIONAL OCCUPANT MODEL — physical calibration is still required.")
print("Session Layer only. Do not save the stage.")
print("Next run isaac_wheelchair_lab_static_contact_load_gate.py while stopped.")
