# Application OTA — A/B slot update with automatic rollback.
#
# Bundle format "GCPK1" (built by host/make_bundle.py, no tar dependency):
#     b"GCPK1\n" + b"<manifest_len>\n" + manifest.json bytes
#     + each file's raw bytes, concatenated in manifest order
#
# Flow: download to /flash/ota/bundle.gpk -> unpack into the INACTIVE slot
# -> re-hash every file (manifest.validate_tree) -> compile()-check every
# .py -> write boot.json {"state": "trial"} -> machine.reset().
#
# The trial boot is judged by main.py + HealthService: a crash reverts to
# the previous slot with no network involved; 60 s of health commits it
# (commit_trial(), called from Application.mark_stable()).

import json
import os

from .. import util
from . import manifest

BOOT_JSON = '/flash/boot.json'
SLOTS = ('app_a', 'app_b')
BUNDLE = 'bundle.gpk'
MAGIC = b'GCPK1\n'


def read_boot():
    try:
        with open(BOOT_JSON) as f:
            return json.load(f)
    except (OSError, ValueError):
        return {'active': 'app_a', 'prev': None, 'state': 'good'}


def write_boot(meta):
    with open(BOOT_JSON, 'w') as f:
        json.dump(meta, f)


def commit_trial():
    """Promote a healthy trial slot to permanent. Returns True if committed."""
    meta = read_boot()
    if meta.get('state') != 'trial':
        return False
    meta['state'] = 'good'
    write_boot(meta)
    return True


def _rmtree(path):
    try:
        entries = os.listdir(path)
    except OSError:
        return
    for name in entries:
        child = '%s/%s' % (path, name)
        try:
            os.remove(child)
        except OSError:
            _rmtree(child)
            try:
                os.rmdir(child)
            except OSError:
                pass
    # caller removes/keeps the root as needed


class AppOta:
    """Drives one update cycle. Constructed per request from the HTTP API."""

    def __init__(self, app):
        self.app = app
        self.log = app.log.tagged('ota')
        self._dir = app.config.get('ota.dir')
        util.makedirs(self._dir)

    # -- download -------------------------------------------------------
    async def _download(self, url):
        """Minimal async HTTP GET streamed to flash. Returns bundle path."""
        import asyncio
        proto, _, rest = url.partition('://')
        if proto != 'http':
            raise ValueError('only http:// urls supported')
        host, _, path = rest.partition('/')
        host, _, port = host.partition(':')
        port = int(port or 80)

        reader, writer = await asyncio.open_connection(host, port)
        writer.write(('GET /%s HTTP/1.0\r\nHost: %s\r\n\r\n'
                      % (path, host)).encode())
        await writer.drain()

        status = await reader.readline()
        parts = status.split(b' ')
        if len(parts) < 2 or parts[1] != b'200':
            raise OSError('http status: %s' % status)
        length = None
        while True:
            line = await reader.readline()
            if not line or line in (b'\r\n', b'\n'):
                break
            k, _, v = line.decode().partition(':')
            if k.strip().lower() == 'content-length':
                length = int(v.strip())

        dst = '%s/%s' % (self._dir, BUNDLE)
        got = 0
        with open(dst, 'wb') as f:
            while True:
                chunk = await reader.read(1024)
                if not chunk:
                    break
                f.write(chunk)
                got += len(chunk)
        writer.close()
        await writer.wait_closed()
        if length is not None and got != length:
            raise OSError('truncated download: %d/%d' % (got, length))
        self.log.info('downloaded %d bytes' % got)
        return dst

    # -- unpack ---------------------------------------------------------
    def _unpack(self, bundle_path, slot):
        root = '/flash/' + slot
        _rmtree(root)
        util.makedirs(root)

        with open(bundle_path, 'rb') as f:
            if f.read(len(MAGIC)) != MAGIC:
                raise ValueError('bad bundle magic')
            mlen = int(f.readline().strip())
            m = json.loads(f.read(mlen))
            # persist the parsed manifest for validate_tree + records
            with open('%s/manifest.json' % root, 'w') as mf:
                json.dump(m, mf)
            for entry in m['files']:
                p = entry['path']
                if p.startswith('/') or '..' in p:
                    raise ValueError('unsafe path: %s' % p)
                if '/' in p:
                    util.makedirs('%s/%s' % (root, p.rsplit('/', 1)[0]))
                remaining = entry['size']
                with open('%s/%s' % (root, p), 'wb') as out:
                    while remaining > 0:
                        chunk = f.read(min(1024, remaining))
                        if not chunk:
                            raise ValueError('bundle truncated at %s' % p)
                        out.write(chunk)
                        remaining -= len(chunk)
        return root, m

    # -- install a bundle already on flash (upload or download path) ----
    def install_from_file(self, bundle_path):
        """Unpack + validate + arm trial boot + reset. Returns False on
        rejection (board untouched); resets the board on success."""
        meta = read_boot()
        active = meta.get('active', 'app_a')
        target = SLOTS[1] if active == SLOTS[0] else SLOTS[0]
        self.log.info('OTA install: %s -> slot %s' % (bundle_path, target))

        root, m = self._unpack(bundle_path, target)
        errors = manifest.validate_tree(root, m)
        errors += manifest.compile_check(root, m)
        if errors:
            for e in errors[:10]:
                self.log.error('OTA reject: %s' % e)
            return False

        os.remove(bundle_path)
        write_boot({'active': target, 'prev': active, 'state': 'trial',
                    'version': m.get('version')})
        self.log.info('OTA installed %s to %s, rebooting for trial'
                      % (m.get('version'), target))
        self.app.logsvc.flush()
        import machine
        machine.reset()

    # -- full cycle from a URL ------------------------------------------
    async def update(self, url):
        try:
            bundle = await self._download(url)
            return self.install_from_file(bundle)
        except Exception as e:
            self.log.error('OTA failed: %s' % e)
            self.app.crash_guard.capture(e, context='ota')
            return False
