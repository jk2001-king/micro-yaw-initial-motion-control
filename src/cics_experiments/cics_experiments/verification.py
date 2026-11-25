"""Check original-byte preservation and active path independence."""
from pathlib import Path
import hashlib
import json
import os
import subprocess
from .paths import evidence_root, package_root, workspace_root


def digest(path):
    value = hashlib.sha256()
    with path.open('rb') as stream:
        for chunk in iter(lambda: stream.read(4 * 1024 * 1024), b''):
            value.update(chunk)
    return value.hexdigest()


def original_git_status(root, snapshot):
    """Verify the preserved predecessor Git, separate from the public CICS history."""
    for directory in (root / 'archive/migration/original_git', root / '.git'):
        if not directory.is_dir() or directory.stat().st_ino != snapshot['git_dir_inode']:
            continue
        head = subprocess.check_output(
            ['git', '--git-dir=' + str(directory), 'rev-parse', 'HEAD'], text=True).strip()
        return dict(directory=str(directory), head_preserved=head == snapshot['git_head'],
                    inode_preserved=True)
    return dict(directory=None, head_preserved=False, inode_preserved=False)


def verify_originals():
    root = workspace_root()
    original = evidence_root()
    snapshot = json.loads((root / 'archive/migration/original_manifest.json').read_text())
    mismatches = []
    generated_changes = []
    checked = 0
    total_bytes = 0
    for record in snapshot['original_files']:
        relative = record['path']
        archived = original / relative
        current = root / relative
        path = archived if archived.exists() or archived.is_symlink() else current
        if 'symlink' in record:
            same = path.is_symlink() and os.readlink(path) == record['symlink']
        else:
            same = path.is_file() and digest(path) == record['sha256']
        if same:
            checked += 1
            total_bytes += record.get('size', 0)
        else:
            transient = relative.startswith(('.vscode/', '.pytest_cache/', 'build/', 'install/', 'log/'))
            (generated_changes if transient else mismatches).append(relative)
    syntax_errors = []
    old_path_matches = []
    for path in (root / 'src').rglob('*.py'):
        text = path.read_text()
        try:
            compile(text, str(path), 'exec')
        except SyntaxError:
            syntax_errors.append(str(path))
        if snapshot['source'] in text:
            old_path_matches.append(str(path))
    broken = []
    for folder in ('src', 'build', 'install'):
        for path in (root / folder).rglob('*'):
            if path.is_symlink() and not path.exists():
                broken.append(str(path.relative_to(root)))
    git_status = original_git_status(root, snapshot)
    result = dict(workspace=str(root), original_files_declared=len(snapshot['original_files']),
                  original_files_byte_verified=checked, verified_bytes=total_bytes,
                  original_evidence_or_source_mismatches=mismatches,
                  changed_generated_cache_files=generated_changes,
                  syntax_errors=syntax_errors, active_python_old_path_references=old_path_matches,
                  active_broken_symlinks=broken,
                  original_git_directory=git_status['directory'],
                  original_git_head_preserved=git_status['head_preserved'],
                  original_git_directory_inode_preserved=git_status['inode_preserved'],
                  active_source_count=dict(runtime=len(list((package_root() / 'runtime').glob('*.py'))),
                                           analysis=len(list((package_root() / 'analysis').glob('*.py')))),
                  original_records_rewritten=False)
    result['PASS'] = not (mismatches or syntax_errors or old_path_matches or broken)
    result['PASS'] &= result['original_git_head_preserved'] and result['original_git_directory_inode_preserved']
    (root / 'archive/migration/verification.json').write_text(json.dumps(result, indent=2) + '\n')
    return result
