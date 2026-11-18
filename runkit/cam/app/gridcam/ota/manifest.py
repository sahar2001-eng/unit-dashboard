# OTA bundle manifest — pure, testable on desktop.
#
# manifest.json (created by host/make_bundle.py):
#   {"version": "1.1.0", "created": "...",
#    "files": [{"path": "gridcam/app.py", "sha256": "...", "size": 1234}, ...]}
#
# validate_tree() re-hashes every installed file against the manifest, so a
# truncated download or partial extract can never be booted.

import hashlib
import json


def load(path):
    """Parse and structurally validate a manifest file. Raises ValueError."""
    with open(path) as f:
        m = json.load(f)
    if not isinstance(m, dict) or 'version' not in m or 'files' not in m:
        raise ValueError('manifest missing version/files')
    for entry in m['files']:
        if not all(k in entry for k in ('path', 'sha256', 'size')):
            raise ValueError('manifest entry missing path/sha256/size')
        p = entry['path']
        if p.startswith('/') or '..' in p:
            raise ValueError('unsafe path in manifest: %s' % p)
    return m


def sha256_file(path, bufsize=1024):
    h = hashlib.sha256()
    with open(path, 'rb') as f:
        while True:
            chunk = f.read(bufsize)
            if not chunk:
                break
            h.update(chunk)
    return ''.join('%02x' % b for b in h.digest())


def validate_tree(root, m):
    """Verify every manifest file exists under root with matching hash/size.

    Returns a list of error strings; empty list = valid.
    """
    import os
    errors = []
    for entry in m['files']:
        path = '%s/%s' % (root, entry['path'])
        try:
            size = os.stat(path)[6]
        except OSError:
            errors.append('missing: %s' % entry['path'])
            continue
        if size != entry['size']:
            errors.append('size mismatch: %s (%d != %d)'
                          % (entry['path'], size, entry['size']))
            continue
        if sha256_file(path) != entry['sha256']:
            errors.append('hash mismatch: %s' % entry['path'])
    return errors


def compile_check(root, m):
    """Syntax-check every .py in the manifest. Returns error strings."""
    errors = []
    for entry in m['files']:
        if not entry['path'].endswith('.py'):
            continue
        path = '%s/%s' % (root, entry['path'])
        try:
            with open(path) as f:
                compile(f.read(), entry['path'], 'exec')
        except SyntaxError as e:
            errors.append('syntax error: %s: %s' % (entry['path'], e))
        except OSError:
            errors.append('unreadable: %s' % entry['path'])
    return errors
