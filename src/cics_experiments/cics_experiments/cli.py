"""Standard ROS executables; default checks never start a simulator or motion."""
import argparse
import ast
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys
from .paths import evidence_root, new_output, package_root, workspace_root
from .protocol import available, dynamic_settings, select


def boolean(value):
    if isinstance(value, bool):
        return value
    if value.lower() in ('true', 'false'):
        return value.lower() == 'true'
    raise argparse.ArgumentTypeError('Use true or false')


def check_main():
    parser = argparse.ArgumentParser(description='Offline CICS readiness; no new motion')
    parser.add_argument('--full', action='store_true', help='Verify the complete original file digests')
    args = parser.parse_args()
    sources = [*(package_root() / 'runtime').glob('*.py'), *(package_root() / 'analysis').glob('*.py')]
    errors = []
    for path in sources:
        try:
            ast.parse(path.read_text())
        except SyntaxError as error:
            errors.append(str(error))
    missing = []
    for platform, conditions in available()['platforms'].items():
        for condition in conditions:
            for method in ('B0', 'B1', 'B2', 'P', 'B3H'):
                spec = select(platform, condition, method)
                folder = Path(spec['original_result'])
                names = (['summary.csv'] + [f'trace_r{r:02d}.csv' for r in (1, 2, 3)]
                         if platform == 'patient_transport' else
                         ['summary.csv'] + [f'natural_flip_r{r:02d}.csv' for r in (1, 2, 3)])
                missing.extend(str(folder / name) for name in names if not (folder / name).is_file())
                if not Path(spec['model']).is_file():
                    missing.append(spec['model'])
    status = dict(workspace=str(workspace_root()), evidence=str(evidence_root()),
                  selected_source_files=len(sources), expected_method_arms=80,
                  expected_recorded_runs=240, missing=sorted(set(missing)),
                  syntax_errors=errors, simulator_started=False, new_trials=0)
    if args.full:
        from .verification import verify_originals
        status['original_preservation'] = verify_originals()
    print(json.dumps(status, indent=2))
    return int(bool(missing or errors or args.full and not status['original_preservation']['PASS']))


def reproduce_main():
    parser = argparse.ArgumentParser(description='Recompute final CICS results without new simulation')
    parser.add_argument('--output')
    parser.add_argument('--figures', action='store_true')
    args = parser.parse_args()
    from .reproduce import export
    print(json.dumps(export(args.output, args.figures), indent=2))


def sdk_python(requested=''):
    options = [requested, os.environ.get('ISAAC_SIM_PYTHON', '')]
    command = shutil.which('isaacsim')
    if command:
        options.append(str(Path(command).parent / 'python'))
    options.append(str(Path.home() / 'miniconda3/envs/isaac_env/bin/python'))
    for value in options:
        if value and Path(value).is_file():
            return str(Path(value).expanduser().absolute())
    raise FileNotFoundError('Pass --sdk-python /path/to/Isaac-Python-3.11')


def sdk_environment():
    env = dict(os.environ)
    for key in ('PYTHONPATH', 'PYTHONHOME', 'AMENT_PREFIX_PATH', 'COLCON_PREFIX_PATH', 'CMAKE_PREFIX_PATH'):
        env.pop(key, None)
    env['PYTHONPATH'] = str(package_root())
    if env.get('LD_LIBRARY_PATH'):
        env['LD_LIBRARY_PATH'] = ':'.join(p for p in env['LD_LIBRARY_PATH'].split(':')
                                        if '/opt/ros/' not in p and '/install/' not in p)
    return env


def sdk_exit_status(returncode, output, spec):
    """Kit fast shutdown can return 0 despite a Python exception; require its receipt."""
    output = Path(output)
    if returncode:
        return returncode
    failure, completed = output / 'failure.json', output / 'completed.json'
    if failure.is_file() or not completed.is_file():
        print('SDK completion not verified; see ' + str(output), file=sys.stderr)
        return 1
    value = json.loads(completed.read_text())
    expected = ('PREPARED_NO_PHYSICS_NO_DRIVE' if spec['stage'] == 'prepare' else 'ORIGINAL_ARM_RECORDED')
    if value.get('outcome') != expected or any(value.get(k) != spec[k]
            for k in ('platform', 'condition', 'method', 'stage')):
        return 1
    if spec['stage'] == 'run' and not (
            value.get('fresh_static_passed') is True and value.get('actual_Pause_confirmed') is True
            and value.get('new_dynamic_runs') == 3):
        return 1
    return 0


def experiment_main():
    parser = argparse.ArgumentParser(description='One CICS platform/condition/method; no automatic campaign')
    parser.add_argument('--list', action='store_true')
    parser.add_argument('--platform', default='patient_transport', choices=['patient_transport', 'wheelchair'])
    parser.add_argument('--condition', nargs='?', const='', default='')
    parser.add_argument('--method', default='proposed', choices=['B0', 'B1', 'B2', 'P', 'B3H', 'proposed'])
    parser.add_argument('--stage', default='check', choices=['check', 'prepare', 'run'])
    parser.add_argument('--allow-motion', nargs='?', const=True, type=boolean, default=False)
    parser.add_argument('--headless', nargs='?', const=True, type=boolean, default=True)
    parser.add_argument('--sdk-python', nargs='?', const='', default='')
    parser.add_argument('--output', nargs='?', const='', default='')
    args = parser.parse_args()
    if args.list:
        print(json.dumps(available(), indent=2))
        return 0
    try:
        spec = select(args.platform, args.condition, args.method, args.stage, args.allow_motion)
    except ValueError as error:
        parser.error(str(error))
    spec['settings'] = dynamic_settings(spec['platform'], spec['condition'], spec['method'])
    if args.stage == 'check':
        print(json.dumps(spec, indent=2))
        return 0
    interpreter = sdk_python(args.sdk_python)
    output = new_output('experiments', args.output or None)
    spec.update(output=str(output), headless=args.headless)
    (output / 'selection.json').write_text(json.dumps(spec, indent=2) + '\n')
    command = [interpreter, '-u', '-m', 'cics_experiments.sdk_runner', str(output / 'selection.json')]
    return sdk_exit_status(subprocess.call(command, cwd=workspace_root(), env=sdk_environment()), output, spec)


def isaac_main():
    parser = argparse.ArgumentParser(description='Open Isaac Sim; no automatic experimental motion')
    parser.add_argument('--executable', default=os.environ.get('ISAAC_SIM_EXECUTABLE', 'isaacsim'))
    parser.add_argument('--dry-run', action='store_true')
    args = parser.parse_args()
    executable = shutil.which(args.executable)
    if not executable:
        candidate = Path(sdk_python()).parent / 'isaacsim'
        if candidate.is_file():
            executable = str(candidate)
    if not executable:
        parser.error('Pass --executable /path/to/isaacsim')
    command = [executable, 'isaacsim.exp.full']
    if args.dry_run:
        print(json.dumps(dict(command=command, automatic_motion=False), indent=2))
        return 0
    return subprocess.call(command, cwd=workspace_root(), env=sdk_environment())
