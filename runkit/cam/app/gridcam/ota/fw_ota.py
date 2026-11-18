# Firmware OTA — special-case path, best effort.
#
# Downloads a firmware image to /flash/ota/firmware.bin, verifies its
# sha256 against the value supplied by the operator, then calls
# machine.bootloader().
#
# CAVEAT (documented in docs/ota.md): whether the AE3/Alif bootloader
# consumes a staged image from /flash is not confirmed. If it does not,
# machine.bootloader() still drops the board into its USB update mode and
# the image must be flashed from a PC (OpenMV IDE / mpremote) — which is
# the fallback procedure for every special case anyway. App code and data
# on /flash are not touched by this module beyond the staged file.

import hashlib

from .. import util


class FwOta:
    def __init__(self, app):
        self.app = app
        self.log = app.log.tagged('fwota')
        self._dir = app.config.get('ota.dir')
        util.makedirs(self._dir)

    async def _download(self, url, dst):
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
        while True:
            line = await reader.readline()
            if not line or line in (b'\r\n', b'\n'):
                break

        h = hashlib.sha256()
        got = 0
        with open(dst, 'wb') as f:
            while True:
                chunk = await reader.read(1024)
                if not chunk:
                    break
                f.write(chunk)
                h.update(chunk)
                got += len(chunk)
        writer.close()
        await writer.wait_closed()
        digest = ''.join('%02x' % b for b in h.digest())
        return got, digest

    async def update(self, url, sha256_hex):
        try:
            dst = '%s/firmware.bin' % self._dir
            self.log.info('fw OTA: downloading %s' % url)
            size, digest = await self._download(url, dst)
            if digest != sha256_hex.lower():
                self.log.error('fw OTA: sha256 mismatch (%s != %s), aborting'
                               % (digest, sha256_hex))
                return False
            self.log.info('fw OTA: image ok (%d bytes), entering bootloader'
                          % size)
            self.app.logsvc.flush()
            import machine
            machine.bootloader()
        except Exception as e:
            self.log.error('fw OTA failed: %s' % e)
            self.app.crash_guard.capture(e, context='fw_ota')
            return False
