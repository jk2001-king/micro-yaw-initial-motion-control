import ast
import hashlib
import json
from pathlib import Path
import sys

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from cics_experiments import protocol
from cics_experiments.cli import boolean, sdk_environment
from cics_experiments.paths import evidence_root, new_output, package_root, resolve_path, workspace_root


def test_final_matrix_size_and_platform_fixed_realization():
    value = protocol.available()
    assert len(value['platforms']['patient_transport']) == 9
    assert len(value['platforms']['wheelchair']) == 7
    assert value['platform_fixed_proposed'] == dict(patient_transport='P', wheelchair='B3H')
    assert value['default_stage'] == 'check' and not value['default_motion_authority']


def test_preserved_git_is_checked_separately_from_public_history(tmp_path, monkeypatch):
    from cics_experiments import verification
    archived = tmp_path / 'archive/migration/original_git'
    archived.mkdir(parents=True)
    (tmp_path / '.git').mkdir()
    snapshot = dict(git_dir_inode=archived.stat().st_ino, git_head='original-head')
    calls = []
    def read_head(command, **kwargs):
        calls.append(command)
        return 'original-head\n'
    monkeypatch.setattr(verification.subprocess, 'check_output', read_head)
    result = verification.original_git_status(tmp_path, snapshot)
    assert result['head_preserved'] and result['inode_preserved']
    assert calls[0][1] == '--git-dir=' + str(archived)


def test_changed_original_git_head_is_not_accepted(tmp_path, monkeypatch):
    from cics_experiments import verification
    directory = tmp_path / '.git'
    directory.mkdir()
    snapshot = dict(git_dir_inode=directory.stat().st_ino, git_head='original-head')
    monkeypatch.setattr(verification.subprocess, 'check_output', lambda *a, **kw: 'other-head\n')
    assert not verification.original_git_status(tmp_path, snapshot)['head_preserved']


CASES = [(p, c, m) for p, cs in protocol.available()['platforms'].items()
         for c in cs for m in protocol.METHODS]


@pytest.mark.parametrize('platform,condition,method', CASES)
def test_each_frozen_recipe_and_original_call_arguments(platform, condition, method):
    selected = protocol.select(platform, condition, method)
    settings = protocol.dynamic_settings(platform, condition, method)
    assert selected['repeats'] == 3
    assert selected['recipe']['speed_m_s'] == (0.025 if method == 'B1' else 0.05)
    assert selected['recipe']['target_m'] == (0.70 if platform == 'patient_transport' else 0.90)
    assert not selected['allow_motion']
    original = evidence_root() / 'isaac/scripts' / protocol.SOURCES[platform]
    namespace = {'__file__': str(original), '__name__': 'cics_original_read_only_recipe'}
    exec(compile(protocol.definition_tree(original), str(original), 'exec'), namespace)
    if platform == 'wheelchair':
        row = next(r for r in namespace['CONDITIONS'] if r[0] == condition)
        assert settings == namespace['dynamic_config'](
            namespace['result_label'](condition, method), method, row[4])
    else:
        row = next(r for r in namespace['CONDITIONS'] if r[0] == condition)
        arm = next(r for r in namespace['METHODS'] if r[0] == method)
        scope = dict(namespace, condition=condition, mass=row[1], com=row[2],
                     method=method, caster_start=namespace['FRONT_POS'],
                     label=selected['recorded_label'], profile=arm[1], speed=arm[2],
                     pulse=arm[3], adaptive=arm[4], hold=arm[5])
        calls = [n for n in ast.walk(ast.parse(original.read_text()))
                 if isinstance(n, ast.Call) and isinstance(n.func, ast.Name)
                 and n.func.id == 'run_script' and len(n.args) == 2
                 and 'isaac_patient_transport_lab_lateral_repeat_gate.py' in ast.unparse(n.args[0])]
        assert settings == eval(compile(ast.Expression(calls[0].args[1]), str(original), 'eval'), scope)


@pytest.mark.parametrize('platform', ['patient_transport', 'wheelchair'])
def test_no_implicit_motion_or_sdk_import(platform):
    with pytest.raises(ValueError, match='allow-motion'):
        protocol.select(platform=platform, stage='run')
    assert 'omni.timeline' not in sys.modules
    assert 'isaacsim' not in sys.modules
    assert protocol.select(platform=platform, stage='prepare')['allow_motion'] is False


@pytest.mark.parametrize('value', ['/etc/passwd', '../other', 'isaac/../../other'])
def test_paths_reject_absolute_or_traversal(value):
    with pytest.raises(ValueError):
        resolve_path(value)


def test_resolved_paths_only_use_current_workspace():
    assert resolve_path('isaac/scripts') == package_root() / 'runtime'
    assert resolve_path('studies/CICS26/results').is_relative_to(evidence_root())
    for platform in protocol.SOURCES:
        assert Path(protocol.select(platform=platform)['model']).is_file()


@pytest.mark.parametrize('name', ['src', 'archive', 'build', 'install', '.git'])
def test_reserved_outputs_cannot_overwrite_source_or_original(name):
    with pytest.raises((ValueError, FileExistsError)):
        new_output('test', workspace_root() / name / 'not-a-valid-output')


def test_existing_output_is_never_reused(tmp_path):
    with pytest.raises(FileExistsError):
        new_output('test', tmp_path)


@pytest.mark.parametrize('value,expected', [('true', True), ('false', False), ('TRUE', True)])
def test_boolean_options(value, expected):
    assert boolean(value) is expected


def test_empty_launch_overrides_are_accepted_without_starting_sdk(monkeypatch, capsys):
    from cics_experiments.cli import experiment_main
    monkeypatch.setattr(sys, 'argv', ['experiment', '--platform', 'wheelchair',
        '--condition', '--method', 'proposed', '--stage', 'check',
        '--allow-motion', 'false', '--sdk-python', '--output'])
    assert experiment_main() == 0
    value = json.loads(capsys.readouterr().out)
    assert value['condition'] == '90p0_center' and value['method'] == 'B3H'
    assert value['stage'] == 'check' and value['allow_motion'] is False


def test_launch_module_imports_and_constructs_without_starting_processes():
    import runpy
    pytest.importorskip('launch')
    module = runpy.run_path(str(package_root() / 'launch/experiment.launch.py'))
    description = module['generate_launch_description']()
    assert len(description.entities) == 10


def test_sdk_environment_separates_ros_binary_paths(monkeypatch):
    monkeypatch.setenv('PYTHONPATH', '/opt/ros/humble/lib/python3.10/site-packages')
    monkeypatch.setenv('LD_LIBRARY_PATH', '/opt/ros/humble/lib:/safe/sdk/lib')
    value = sdk_environment()
    assert value['PYTHONPATH'] == str(package_root())
    assert value['LD_LIBRARY_PATH'] == '/safe/sdk/lib'


def test_original_control_and_metric_functions_are_unchanged():
    records = json.loads((package_root() / 'config/source_manifest.json').read_text())
    assert len(records) == 21
    for record in records:
        original = evidence_root() / record['original']
        active = workspace_root() / record['active']
        assert hashlib.sha256(original.read_bytes()).hexdigest() == record['original_sha256']
        assert hashlib.sha256(active.read_bytes()).hexdigest() == record['active_sha256']
        functions = lambda p: [ast.dump(n, include_attributes=False) for n in ast.parse(p.read_text()).body
                               if isinstance(n, (ast.FunctionDef, ast.AsyncFunctionDef, ast.ClassDef))]
        assert functions(original) == functions(active), record['original']


def test_sdk_static_verdict_instrumentation_does_not_change_predicates():
    from cics_experiments.sdk_runner import execute_file
    tree = ast.parse(Path(sys.modules[execute_file.__module__].__file__).read_text())
    assert 'passed' in ast.unparse(tree)
    assert 'wheelchair_lab_natural_flip_gate_task' in ast.unparse(tree)


def test_pause_request_is_confirmed_after_queued_update_without_reissuing_pause():
    import asyncio
    from cics_experiments.sdk_runner import confirm_queued_pause
    class Timeline:
        playing = True
        def is_playing(self):
            return self.playing
    timeline = Timeline()
    updates = []
    async def process_original_pause():
        updates.append(True)
        timeline.playing = False
    asyncio.run(confirm_queued_pause(timeline, process_original_pause))
    assert updates == [True]


def test_missing_original_pause_still_fails_closed():
    import asyncio
    from cics_experiments.sdk_runner import confirm_queued_pause
    class Timeline:
        def is_playing(self):
            return True
    async def update_without_pause():
        pass
    with pytest.raises(RuntimeError, match='Pause was not acknowledged'):
        asyncio.run(confirm_queued_pause(Timeline(), update_without_pause))


def test_zero_sdk_exit_without_completion_receipt_is_not_success(tmp_path):
    from cics_experiments.cli import sdk_exit_status
    assert sdk_exit_status(0, tmp_path, dict(stage='run')) == 1
    (tmp_path / 'failure.json').write_text('{}')
    assert sdk_exit_status(0, tmp_path, dict(stage='run')) == 1


def test_only_complete_current_sdk_receipt_is_accepted(tmp_path):
    from cics_experiments.cli import sdk_exit_status
    spec = dict(platform='wheelchair', condition='90p0_center', method='B3H', stage='run')
    record = dict(spec, outcome='ORIGINAL_ARM_RECORDED', fresh_static_passed=True,
                  actual_Pause_confirmed=True, new_dynamic_runs=3)
    (tmp_path / 'completed.json').write_text(json.dumps(record))
    assert sdk_exit_status(0, tmp_path, spec) == 0
    record['actual_Pause_confirmed'] = False
    (tmp_path / 'completed.json').write_text(json.dumps(record))
    assert sdk_exit_status(0, tmp_path, spec) == 1


def test_original_tables_recomputed_without_motion(tmp_path):
    from cics_experiments.reproduce import export
    value = export(tmp_path / 'recomputed')
    assert value['outcome'] == 'MATCHED_ORIGINAL_NUMERICAL_RESULTS'
    assert value['recorded_runs'] == dict(wheelchair=105, patient_transport=135)
    assert value['proposed_target_reaches'] == dict(wheelchair='21/21', patient_transport='27/27')
    assert len(value['tables']) == 6 and value['new_simulated_trials'] == 0
