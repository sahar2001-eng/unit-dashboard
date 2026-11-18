# Logging: a RAM-first Logger plus a LogService that batches writes to
# rotating files on /flash.
#
# Design rules (flash wear): log lines go to serial immediately and into a
# bounded RAM ring; the file on /flash is only appended every `flush_s`
# seconds (or on demand). Rotation is size-based: app.0.log -> app.1.log ...

import os

from . import stall, util
from .service import Service

DEBUG, INFO, WARN, ERROR = 10, 20, 30, 40
_NAMES = {DEBUG: 'DEBUG', INFO: 'INFO', WARN: 'WARN', ERROR: 'ERROR'}
_LEVELS = {'debug': DEBUG, 'info': INFO, 'warn': WARN, 'error': ERROR}


class _Tagged:
    """Cheap per-service facade over the shared Logger."""

    def __init__(self, logger, tag):
        self._log = logger
        self._tag = tag

    def debug(self, msg):
        self._log.log(DEBUG, self._tag, msg)

    def info(self, msg):
        self._log.log(INFO, self._tag, msg)

    def warn(self, msg):
        self._log.log(WARN, self._tag, msg)

    def error(self, msg):
        self._log.log(ERROR, self._tag, msg)


class Logger:
    """Shared logger: level filter, serial echo, RAM ring, flush queue."""

    def __init__(self, level='info', ring_lines=200):
        self.level = _LEVELS.get(level, INFO)
        self._ring = []
        self._ring_max = ring_lines
        self._pending = []            # lines not yet flushed to flash
        self._pending_max = ring_lines
        self.on_error = None          # LogService hooks flush() here

    def tagged(self, tag):
        return _Tagged(self, tag)

    def set_level(self, level):
        self.level = _LEVELS.get(level, self.level)

    def log(self, lvl, tag, msg):
        if lvl < self.level:
            return
        line = '%9.3f %-5s %-8s %s' % (
            util.uptime_s(), _NAMES.get(lvl, '?'), tag, msg)
        print(line)
        self._ring.append(line)
        if len(self._ring) > self._ring_max:
            self._ring.pop(0)
        self._pending.append(line)
        if len(self._pending) > self._pending_max:
            self._pending.pop(0)      # drop oldest rather than grow
        if lvl >= ERROR and self.on_error is not None:
            try:
                self.on_error()       # errors must survive an imminent crash
            except Exception:
                pass

    def tail(self, n=50):
        return self._ring[-n:]

    def take_pending(self):
        lines, self._pending = self._pending, []
        return lines


class LogService(Service):
    """Flushes the Logger's pending lines to rotating files on /flash."""

    name = "logsvc"

    def setup(self):
        cfg = self.app.config.section('log')
        self._dir = cfg['dir']
        self._file_size = cfg['file_size']
        self._file_count = cfg['file_count']
        self._flush_s = cfg['flush_s']
        self._chunk = cfg.get('flush_chunk_lines', 16)
        self._bytes_written = 0        # wear observability (telemetry)
        self._flushes = 0
        util.makedirs(self._dir)
        self.app.log.set_level(cfg['level'])
        if cfg.get('flush_on_error'):
            self.app.log.on_error = self.flush

    def _path(self, i):
        return '%s/app.%d.log' % (self._dir, i)

    def _rotate_if_needed(self):
        if util.file_size(self._path(0)) < self._file_size:
            return
        try:
            os.remove(self._path(self._file_count - 1))
        except OSError:
            pass
        for i in range(self._file_count - 2, -1, -1):
            try:
                os.rename(self._path(i), self._path(i + 1))
            except OSError:
                pass

    def _append(self, lines):
        try:
            with open(self._path(0), 'a') as f:
                for line in lines:
                    f.write(line)
                    f.write('\n')
                    self._bytes_written += len(line) + 1
        except OSError:
            pass                      # a full/broken fs must not kill the app

    def flush(self):
        """Synchronous flush: shutdown and flush_on_error, where the line
        must reach flash before anything else runs."""
        lines = self.app.log.take_pending()
        if not lines:
            return
        self._rotate_if_needed()
        self._append(lines)
        self._flushes += 1

    async def flush_async(self, chunk_lines=None):
        """Periodic flush, in chunks with a yield between them.

        Flash writes do not await, and this loop is also the one that feeds
        the STM32 (there is no _thread on this port). One burst used to hold
        it for the whole write; now the longest hold is one chunk. Lines are
        taken from the Logger up front, so a line logged mid-flush waits for
        the next one -- and a sync flush_on_error landing between two chunks
        is appended between them, so the file order can interleave there.
        """
        import asyncio
        n = chunk_lines or self._chunk
        lines = self.app.log.take_pending()
        if not lines:
            return
        with stall.section(self.app, 'log rotate'):
            self._rotate_if_needed()
        for i in range(0, len(lines), n):
            if i:
                await asyncio.sleep(0)
            with stall.section(self.app, 'log write'):
                self._append(lines[i:i + n])
        self._flushes += 1

    async def run(self):
        import asyncio
        while True:
            await asyncio.sleep(self._flush_s)
            await self.flush_async()
            self.heartbeat()

    def stop(self):
        self.flush()

    def list_files(self):
        out = []
        for i in range(self._file_count):
            size = util.file_size(self._path(i))
            if size >= 0:
                out.append({'name': 'app.%d.log' % i, 'size': size})
        return out

    def read_file(self, name):
        if '/' in name or '..' in name:
            return None
        try:
            with open('%s/%s' % (self._dir, name)) as f:
                return f.read()
        except OSError:
            return None

    def stats(self):
        return {'ring': len(self.app.log.tail(10000)),
                'flushes': self._flushes,
                'bytes_written': self._bytes_written,
                'files': self.list_files()}
