"""Read the final recorded recipes without importing or starting Isaac Sim."""
import ast
import hashlib
from pathlib import Path
from .paths import package_root, resolve_path

SOURCES = {
    'patient_transport': 'isaac_patient_transport_cics26_lateral_fullmatrix_next.py',
    'wheelchair': 'isaac_wheelchair_cics26_final9_next.py',
}
PROPOSED = {'patient_transport': 'P', 'wheelchair': 'B3H'}
METHODS = ('B0', 'B1', 'B2', 'P', 'B3H')


def definition_tree(path, sdk=False):
    tree = ast.parse(Path(path).read_text(), filename=str(path))
    body = []
    for node in tree.body:
        if isinstance(node, ast.Assign) and any(
                isinstance(target, ast.Name) and target.id == 'timeline' for target in node.targets):
            break
        if not sdk and isinstance(node, (ast.Import, ast.ImportFrom)):
            names = ([v.name for v in node.names] if isinstance(node, ast.Import) else [node.module or ''])
            if any(name.split('.')[0] in ('omni', 'pxr', 'isaacsim') for name in names):
                continue
        body.append(node)
    if not body or not any(isinstance(n, ast.FunctionDef) for n in body):
        raise ValueError('Final scheduler definition boundary not found')
    return ast.Module(body=body, type_ignores=[])


def definitions(platform, sdk=False):
    if platform not in SOURCES:
        raise ValueError('Choose patient_transport or wheelchair')
    path = package_root() / 'runtime' / SOURCES[platform]
    namespace = {'__name__': 'cics_final_recipe', '__file__': str(path)}
    exec(compile(definition_tree(path, sdk), str(path), 'exec'), namespace)
    return namespace


def available():
    return dict(platforms={key: [row[0] for row in definitions(key)['CONDITIONS']] for key in SOURCES},
                methods=[*METHODS, 'proposed'], stages=['check', 'prepare', 'run'],
                platform_fixed_proposed=PROPOSED, default_stage='check',
                default_motion_authority=False)


def select(platform='patient_transport', condition=None, method='proposed', stage='check',
           allow_motion=False):
    recipe = definitions(platform)
    conditions = {row[0]: row for row in recipe['CONDITIONS']}
    condition = condition or ('265center' if platform == 'patient_transport' else '90p0_center')
    if condition not in conditions or method not in (*METHODS, 'proposed'):
        raise ValueError('Choose one registered condition and B0/B1/B2/P/B3H/proposed')
    if stage not in ('check', 'prepare', 'run'):
        raise ValueError('Choose check, prepare or run')
    if stage == 'run' and allow_motion is not True:
        raise ValueError('A new simulated arm requires --allow-motion')
    method = PROPOSED[platform] if method == 'proposed' else method
    row = conditions[condition]
    if platform == 'patient_transport':
        option = next(v for v in recipe['METHODS'] if v[0] == method)
        label = f'cics26_pt_{condition}_lateral_lowbed_frontpos_{method}_v5'
        trace_root = resolve_path('studies/CICS26/results/patient_transport_lab/natural_flip_pilot')
        metadata = dict(mass_kg=row[1], base_com_m=row[2], target_m=0.70,
                        speed_m_s=option[2], initial_caster_degrees=recipe['FRONT_POS'])
        model = resolve_path('isaac/assets/patient_transport_lab/patient_transport_lab.usd')
    else:
        label = recipe['result_label'](condition, method)
        trace_root = resolve_path('studies/CICS26/results/wheelchair_lab/natural_flip_gate')
        metadata = dict(mass_kg=row[1], occupant_mass_kg=row[2], occupant_com_x_m=row[3],
                        target_m=0.90, speed_m_s=0.025 if method == 'B1' else 0.05)
        model = resolve_path('isaac/assets/wheelchair/wheelchair_lab.usd')
    digest = hashlib.sha256((package_root() / 'runtime' / SOURCES[platform]).read_bytes()).hexdigest()
    return dict(schema=1, platform=platform, condition=condition, method=method,
                stage=stage, allow_motion=bool(allow_motion), repeats=3,
                recorded_label=label, model=str(model), recipe=metadata,
                original_result=str(trace_root / label), recipe_sha256=digest,
                safety_and_control_parameters_changed=False,
                original_records_are_not_motion_authority=True)


def dynamic_settings(platform, condition, method):
    """Evaluate the exact recorded call arguments, not a hand-copied new policy."""
    recipe = definitions(platform)
    if method == 'proposed':
        method = PROPOSED[platform]
    row = next(r for r in recipe['CONDITIONS'] if r[0] == condition)
    if method not in METHODS:
        raise ValueError('Unknown method')
    if platform == 'wheelchair':
        return recipe['dynamic_config'](recipe['result_label'](condition, method), method, row[4])
    option = next(r for r in recipe['METHODS'] if r[0] == method)
    scope = dict(recipe, condition=condition, mass=row[1], com=row[2],
                 caster_start=recipe['FRONT_POS'], method=method,
                 label=f'cics26_pt_{condition}_lateral_lowbed_frontpos_{method}_v5',
                 profile=option[1], speed=option[2], pulse=option[3],
                 adaptive=option[4], hold=option[5])
    source = package_root() / 'runtime' / SOURCES[platform]
    matches = [node for node in ast.walk(ast.parse(source.read_text()))
               if isinstance(node, ast.Call) and isinstance(node.func, ast.Name)
               and node.func.id == 'run_script' and len(node.args) == 2
               and isinstance(node.args[1], ast.Dict)
               and 'isaac_patient_transport_lab_lateral_repeat_gate.py' in ast.unparse(node.args[0])]
    if len(matches) != 1:
        raise ValueError('Exactly one final patient-transport dynamic recipe is required')
    return eval(compile(ast.Expression(matches[0].args[1]), str(source), 'eval'), scope)
