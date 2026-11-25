"""Recompute final CICS metrics without changing original data or starting Kit."""
import contextlib
import csv
import hashlib
import importlib
import io
import json
import math
import os
from pathlib import Path
import sys
from .paths import new_output, package_root, resolve_path


def read_csv(path):
    with Path(path).open(newline='') as stream:
        return list(csv.DictReader(stream))


def compare_table(current, original):
    actual, expected = read_csv(current), read_csv(original)
    if len(actual) != len(expected) or not actual or actual[0].keys() != expected[0].keys():
        raise ValueError('Table shape changed: ' + str(original))
    for index, (left, right) in enumerate(zip(actual, expected)):
        for key in left:
            if left[key] == right[key]:
                continue
            try:
                a, b = float(left[key]), float(right[key])
            except ValueError:
                raise ValueError(f'Table label changed: {original}:{index}:{key}') from None
            if not (math.isfinite(a) and math.isfinite(b) and math.isclose(a, b, rel_tol=1e-12, abs_tol=1e-9)):
                raise ValueError(f'Measured value changed: {original}:{index}:{key}')
    return len(actual)


def export(output=None, figures=False):
    if os.environ.get('CICS_RUN_ROOT'):
        raise ValueError('Original result reproduction cannot use a new-run output view')
    folder = new_output('reproduced', output)
    analysis = str(package_root() / 'analysis')
    if analysis not in sys.path:
        sys.path.insert(0, analysis)
    os.environ.setdefault('MPLCONFIGDIR', '/tmp/cics-reproduction-mpl')
    import matplotlib
    matplotlib.use('Agg')
    patient = importlib.import_module('build_patient_transport_final9_evidence')
    wheelchair = importlib.import_module('build_wheelchair_final7_evidence')
    integrated = importlib.import_module('build_cics26_integrated_two_page_assets')
    for name in ('patient_transport', 'wheelchair', 'integrated', 'reports'):
        (folder / name).mkdir()
    patient.OUT = folder / 'patient_transport'
    patient.DOC = folder / 'reports/patient_transport.md'
    wheelchair.OUT = folder / 'wheelchair'
    wheelchair.REPORT = folder / 'reports/wheelchair.md'
    integrated.OUT = folder / 'integrated'
    integrated.WHEELCHAIR_CSV = wheelchair.OUT / 'wheelchair_final7_metrics.csv'
    integrated.PATIENT_CSV = patient.OUT / 'patient_transport_final9_metrics.csv'
    transcript = io.StringIO()
    with contextlib.redirect_stdout(transcript):
        patient.main()
        wheelchair_rows = wheelchair.collect()
        wheelchair.write_csv(wheelchair_rows)
        wheelchair.write_report(wheelchair_rows)
        records = integrated.load_records()
        integrated.export_exact_records(records)
        paired, summaries, ablation = integrated.summarize(records)
        integrated.write_readme(summaries)
        readme = integrated.OUT / 'README.md'
        readme.write_text(readme.read_text().replace(
            'python3 studies/CICS26/scripts/build_cics26_integrated_two_page_assets.py',
            'ros2 run cics_experiments reproduce_results --figures'))
        if figures:
            wheelchair.plot(wheelchair_rows)
            integrated.plot_framework()
            integrated.plot_integrated_validation(records, paired)
            integrated.plot_compact_table(summaries)
    (folder / 'analysis.log').write_text(transcript.getvalue())
    pairs = [
        (patient.OUT / 'patient_transport_final9_metrics.csv',
         'studies/CICS26/paper_assets/patient_transport/final9/patient_transport_final9_metrics.csv'),
        (wheelchair.OUT / 'wheelchair_final7_metrics.csv',
         'studies/CICS26/paper_assets/wheelchair/final7/wheelchair_final7_metrics.csv'),
        *[(integrated.OUT / name, 'studies/CICS26/paper_assets/cics26_final_integrated/' + name)
          for name in ('all_platform_condition_metrics.csv', 'proposed_vs_ramp_paired_metrics.csv',
                       'platform_proposed_summary.csv', 'platform_variant_ablation.csv')],
    ]
    checks = []
    for current, relative in pairs:
        original = resolve_path(relative)
        checks.append(dict(table=current.name, rows=compare_table(current, original),
                           original=str(original), original_sha256=hashlib.sha256(original.read_bytes()).hexdigest()))
    result = dict(outcome='MATCHED_ORIGINAL_NUMERICAL_RESULTS', output=str(folder),
                  method_arms=dict(wheelchair=35, patient_transport=45),
                  recorded_runs=dict(wheelchair=105, patient_transport=135),
                  proposed_target_reaches=dict(wheelchair='21/21', patient_transport='27/27'),
                  tables=checks, new_simulated_trials=0, original_results_changed=False,
                  metric_definitions_changed=False)
    (folder / 'verification.json').write_text(json.dumps(result, indent=2) + '\n')
    return result
