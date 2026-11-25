"""Relocatable code paths; original experiment bytes remain in the archive."""
from datetime import datetime
from pathlib import Path
import os


def package_root():
    source = Path(__file__).resolve().parents[1]
    if (source / 'runtime').is_dir():
        return source
    from ament_index_python.packages import get_package_share_directory
    return Path(get_package_share_directory('cics_experiments')).resolve()


def workspace_root():
    for parent in (package_root(), *package_root().parents):
        if (parent / 'src/cics_experiments/package.xml').is_file():
            return parent
    raise FileNotFoundError('Build this package inside the CICS source workspace')


def evidence_root():
    return workspace_root() / 'archive/original'


def resolve_path(relative):
    path = Path(relative)
    if path.is_absolute() or '..' in path.parts:
        raise ValueError('A workspace-relative path without traversal is required')
    text = path.as_posix()
    for old, new in (
        ('isaac/scripts', package_root() / 'runtime'),
        ('isaac/assets', package_root() / 'assets'),
        ('studies/CICS26/scripts', package_root() / 'analysis'),
    ):
        if text == old or text.startswith(old + '/'):
            return (new / text[len(old):].lstrip('/')).resolve()
    if text.startswith('studies/CICS26/results/') or text == 'studies/CICS26/results':
        current = os.environ.get('CICS_RUN_ROOT')
        return (Path(current).resolve() if current else evidence_root()) / path
    if text.startswith(('studies/', 'data/', 'results/', 'docs/', 'config/')):
        return evidence_root() / path
    return workspace_root() / path


def new_output(kind, requested=None):
    root = workspace_root()
    output = (Path(requested).expanduser().resolve() if requested else
              root / 'log/cics' / kind / datetime.now().strftime('%Y%m%d_%H%M%S_%f'))
    if output.exists():
        raise FileExistsError('Existing outputs are never reused: ' + str(output))
    for reserved in (root / 'src', root / 'archive', root / 'build', root / 'install', root / '.git'):
        if output == reserved or reserved in output.parents:
            raise ValueError('Output cannot overwrite source, models or original evidence')
    output.mkdir(parents=True, exist_ok=False)
    return output
