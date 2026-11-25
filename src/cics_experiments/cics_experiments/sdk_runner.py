"""Fresh Isaac process: original preparation, fresh static gate, one three-repeat arm."""
import ast
import asyncio
import csv
import hashlib
import json
import os
from pathlib import Path
import sys
import time
import traceback
from .paths import package_root, resolve_path
from .protocol import definitions, dynamic_settings, select


async def confirm_queued_pause(timeline, next_update, maximum_updates=5):
    """Wait for the original gate's queued Pause; never issue a replacement Pause."""
    for _ in range(maximum_updates):
        if not timeline.is_playing():
            return
        await next_update()
    if timeline.is_playing():
        raise RuntimeError('Original gate Pause was not acknowledged after queued updates')


def execute_file(name, extra=None, return_static_verdict=False):
    path = package_root() / 'runtime' / name
    namespace = dict(__name__='cics_selected_runtime', __file__=str(path))
    namespace.update(extra or {})
    tree = ast.parse(path.read_text(), filename=str(path))
    if return_static_verdict:
        # Expose the original local boolean; do not duplicate or change its predicates.
        functions = [n for n in tree.body if isinstance(n, ast.AsyncFunctionDef) and n.name == '_run']
        if len(functions) != 1:
            raise ValueError('Exactly one original static coroutine required')
        functions[0].body.append(ast.Return(value=ast.Name(id='passed', ctx=ast.Load())))
        ast.fix_missing_locations(tree)
    exec(compile(tree, str(path), 'exec'), namespace)
    return namespace


async def perform(spec):
    import omni.kit.app
    import omni.timeline
    import omni.usd
    import carb
    output = Path(spec['output'])
    selection = select(spec['platform'], spec['condition'], spec['method'], spec['stage'], spec['allow_motion'])
    settings = dynamic_settings(spec['platform'], spec['condition'], spec['method'])
    if json.loads(json.dumps(settings)) != spec['settings'] or selection['recipe_sha256'] != spec['recipe_sha256']:
        raise ValueError('Selected final recipe changed before SDK execution')
    os.environ['CICS_RUN_ROOT'] = str(output / 'raw')
    context = omni.usd.get_context()
    timeline = omni.timeline.get_timeline_interface()
    if not timeline.is_stopped():
        raise ValueError('A new stopped SDK process is required')
    ok, error = await context.open_stage_async(spec['model'])
    if not ok:
        raise RuntimeError(str(error))
    app = omni.kit.app.get_app()
    for _ in range(5):
        await app.next_update_async()
    stage = context.get_stage()
    recipe = definitions(spec['platform'], sdk=True)
    row = next(r for r in recipe['CONDITIONS'] if r[0] == spec['condition'])
    if spec['platform'] == 'patient_transport':
        recipe['prepare_common'](stage)
        recipe['prepare_mass_com'](stage, row[1], row[2])
        recipe['prepare_lateral_mode'](stage)
        recipe['prepare_fixture'](stage)
    else:
        recipe['prepare_condition'](stage, row)
    result = dict(platform=spec['platform'], condition=spec['condition'], method=spec['method'],
                  stage=spec['stage'], prepared=True, new_dynamic_runs=0,
                  original_records_changed=False,
                  fixed_timeline_stepping=carb.settings.get_settings().get('/app/player/useFixedTimeStepping'))
    if spec['stage'] == 'prepare':
        result['outcome'] = 'PREPARED_NO_PHYSICS_NO_DRIVE'
        return result
    if spec['stage'] != 'run' or spec['allow_motion'] is not True:
        raise ValueError('Current explicit run / allow-motion consent required')
    if spec['platform'] == 'patient_transport':
        static_label = f'cics26_pt_{spec["condition"]}_lateral_lowbed_static_v5'
        static = execute_file('isaac_patient_transport_lab_static_contact_gate.py',
                              {'patient_transport_lab_contact_label': static_label}, True)
        passed = await static['patient_transport_lab_contact_task']
        static_path = static['OUTPUT']
        if not passed or not recipe['static_passed'](static_path, row[1]):
            raise RuntimeError('Fresh original patient-transport static gate did not pass')
    else:
        static_label = f'cics26_final9_{spec["condition"]}_static'
        static = execute_file('isaac_wheelchair_lab_static_contact_load_gate.py',
                              {'wheelchair_lab_load_share_label': static_label,
                               'wheelchair_lab_target_caster_share': row[5]}, True)
        passed = await static['wheelchair_lab_contact_load_task']
        static_path = static['OUTPUT']
        if not passed:
            raise RuntimeError('Fresh original wheelchair static gate did not pass')
        recipe['validate_static'](static_path, row[1], row[5])
    result['fresh_static_passed'] = True
    timeline.stop()
    for _ in range(5):
        await app.next_update_async()
    if spec['platform'] == 'patient_transport':
        dynamic = execute_file('isaac_patient_transport_lab_lateral_repeat_gate.py', settings)
        await dynamic['patient_transport_lateral_gate_task']
    else:
        recipe['set_caster_damping'](stage)
        dynamic = execute_file('isaac_wheelchair_lab_natural_flip_gate.py', settings)
        await dynamic['wheelchair_lab_natural_flip_gate_task']
    await confirm_queued_pause(timeline, app.next_update_async)
    summary = dynamic['OUTPUT_DIR'] / 'summary.csv'
    with summary.open(newline='') as stream:
        rows = list(csv.DictReader(stream))
    if len(rows) != 3:
        raise RuntimeError('Expected exactly three recorded repeats; no successful retry replacement')
    result.update(outcome='ORIGINAL_ARM_RECORDED', summary=str(summary), new_dynamic_runs=3,
                  flip_completed=sum(r['flip_completed'] == 'True' for r in rows),
                  actual_Pause_confirmed=not timeline.is_playing(),
                  static_return_instrumentation_only=True, no_automatic_retry=True)
    return result


def main():
    spec = json.loads(Path(sys.argv[1]).read_text())
    if spec['stage'] not in ('prepare', 'run'):
        raise ValueError('SDK entry is only for prepare/run')
    if spec['stage'] == 'run' and spec.get('allow_motion') is not True:
        raise ValueError('Explicit current motion consent required')
    from isaacsim import SimulationApp
    app = SimulationApp(dict(headless=spec['headless'],
        extra_args=['--/app/player/useFixedTimeStepping=true']))
    output = Path(spec['output'])
    task = None
    try:
        task = asyncio.ensure_future(perform(spec))
        started = time.monotonic()
        while not task.done():
            app.update()
            if time.monotonic() - started > 3600:
                raise TimeoutError('SDK process outer watchdog; original gate limits remain unchanged')
        result = task.result()
        (output / 'completed.json').write_text(json.dumps(result, indent=2) + '\n')
        print(json.dumps(result, indent=2), flush=True)
    except BaseException as error:
        import omni.timeline
        omni.timeline.get_timeline_interface().pause()
        if task is not None and not task.done():
            task.cancel()
        (output / 'failure.json').write_text(json.dumps(dict(
            error=repr(error), traceback=traceback.format_exc(),
            negative_result_not_relabelled=True, no_automatic_retry=True), indent=2) + '\n')
        raise
    finally:
        app.close()


if __name__ == '__main__':
    main()
