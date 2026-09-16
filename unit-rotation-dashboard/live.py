#!/usr/bin/env python3
"""Live sources for the UNIT dashboard, and the recorder.

Two feeds from the gridcam camera, deliberately on two different links:

  data   USB serial (115200). The camera prints one CSV row per frame --
             frame,tier,theta_deg,x_mm,y_mm,n_c,n_r,mad_h,mad_v,fps
         -- at the vision rate (~114 fps). Reading it costs the camera
         nothing: it is printing anyway. These are the same numbers the
         STM32 receives. NOT /telemetry, which is a snapshot and is
         documented to drop the vision loop from 118 to 85 fps when polled.

  video  HTTP MJPEG from the camera's own web server (GET /stream on :8080)
         over WiFi. USB cannot carry video while the app runs. The camera
         draws the overlay and compresses JPEGs ONLY while a client is
         connected, so StreamReader connects only while someone is watching
         or a recording is running, and the dashboard is the camera's one
         client -- browsers get frames from our proxy, never from the camera.

Both feeds are stamped with the host clock on arrival. That is the alignment
a recording gets: the JPEG stream carries no frame id, so video and data are
aligned to a few milliseconds, not to the exact frame.
"""

import collections
import csv
import json
import os
import subprocess
import threading
import time
import urllib.request
import uuid

import serial
from serial.tools import list_ports

OPENMV_VID, OPENMV_PID = 0x37C5, 0x16E3
CSV_FIELDS = ['frame', 'tier', 'theta_deg', 'x_mm', 'y_mm',
              'n_c', 'n_r', 'mad_h', 'mad_v', 'fps']


# ------------------------------------------------------------------ parsing
def parse_row(line):
    """One CSV row -> dict, or None for anything else on the port.

    The camera's log lines ("  545.875 WARN  ntp ...") and the CSV header
    share the serial port with the data rows, so anything that is not
    exactly ten fields with the right types is skipped, not raised on.
    A blank tier means Algorithm 1 did not produce a fix this frame: that
    is the case during a commanded turn and also when there is no grid in
    view -- the CSV cannot tell those apart, so neither can we.
    """
    p = line.split(',')
    if len(p) != 10:
        return None
    try:
        frame = int(p[0])
        n_c, n_r = int(p[5]), int(p[6])
        fps = float(p[9])
        def f(s):
            # vision_service._fmt() writes '' for an absent value, NOT '--'
            # (that is overlay.py's formatter, a different thing). An empty
            # field is the NORMAL way X is reported mid-cell and the way
            # everything reads during a turn, so treating it as a parse
            # failure throws away most of the stream.
            return None if s == '' else float(s)
        theta, x, y, mad_h, mad_v = f(p[2]), f(p[3]), f(p[4]), f(p[7]), f(p[8])
    except ValueError:
        return None
    return {'frame': frame, 'tier': p[1] or 'none',
            'theta': theta, 'x': x, 'y': y,
            'n_c': n_c, 'n_r': n_r, 'mad_h': mad_h, 'mad_v': mad_v,
            'fps': fps}


def find_openmv_port(hint=''):
    """Resolve the camera by USB ID, never by port number -- the numbering
    follows plug order and the ESP32 (303a:1001) can land on either."""
    if hint:
        return hint
    found = []
    for p in list_ports.comports():
        if p.vid == OPENMV_VID and p.pid == OPENMV_PID:
            found.append(p.device)
    # macOS lists both /dev/cu.* and /dev/tty.*; cu is the one to open
    found.sort(key=lambda d: (not d.startswith('/dev/cu.'), d))
    return found[0] if found else None


# ------------------------------------------------------------- serial data
class SerialReader(threading.Thread):
    def __init__(self, port_hint='', baud=115200, history=30000):
        super().__init__(daemon=True, name='serial')
        self.port_hint = port_hint
        self.baud = baud
        self.rows = collections.deque(maxlen=history)   # (seq, host_t, wall_t, row)
        self.seq = 0
        self.state = 'searching'
        self.port = None
        self.error = ''
        self.skipped = 0
        self.reconnects = 0
        self.last_t = 0.0
        self._subs = []
        self._lock = threading.Lock()
        self._stop = threading.Event()

    def subscribe(self, fn):
        with self._lock:
            self._subs.append(fn)

    def unsubscribe(self, fn):
        with self._lock:
            if fn in self._subs:
                self._subs.remove(fn)

    def reconfigure(self, port_hint):
        # No-op when nothing changed. This is called on every settings save,
        # and dropping a healthy connection to re-open the same port loses
        # data for a second for no reason.
        if port_hint == self.port_hint:
            return
        self.port_hint = port_hint
        self._stop.set()            # drop the current connection; run() reopens

    def since(self, seq):
        with self._lock:
            return [r for r in self.rows if r[0] > seq]

    def fps_estimate(self):
        now = time.monotonic()
        with self._lock:
            n = sum(1 for r in self.rows if now - r[1] <= 1.0)
        return n

    LIVE_S = 2.0

    def status(self):
        age = (time.monotonic() - self.last_t) if self.last_t else None
        # Rows arriving IS connected, whatever a flag says. A state flag set
        # by one thread and cleared by another drifts out of date; data in
        # the last LIVE_S seconds cannot.
        state = 'connected' if (age is not None and age < self.LIVE_S) else self.state
        return {'state': state, 'port': self.port, 'error': self.error,
                'rows': self.seq, 'skipped': self.skipped,
                'reconnects': self.reconnects,
                'fps': self.fps_estimate(), 'age_s': age}

    def run(self):
        while True:
            self._stop.clear()
            port = find_openmv_port(self.port_hint)
            if not port:
                self.state, self.port = 'searching', None
                time.sleep(1.0)
                continue
            try:
                ser = serial.Serial(port, self.baud, timeout=1)
            except Exception as e:                       # noqa: BLE001
                self.state, self.port, self.error = 'error', port, str(e)
                time.sleep(1.5)
                continue
            if self.seq:                 # not the first connection of the run
                self.reconnects += 1
            self.state, self.port, self.error = 'connected', port, ''
            try:
                while not self._stop.is_set():
                    raw = ser.readline()
                    if not raw:
                        continue
                    host_t, wall_t = time.monotonic(), time.time()
                    row = parse_row(raw.decode('latin-1').strip())
                    if row is None:
                        self.skipped += 1
                        continue
                    with self._lock:
                        self.seq += 1
                        rec = (self.seq, host_t, wall_t, row)
                        self.rows.append(rec)
                        subs = list(self._subs)
                    self.last_t = host_t
                    for fn in subs:
                        try:
                            fn(host_t, wall_t, row)
                        except Exception:                # noqa: BLE001
                            pass
            except Exception as e:                       # noqa: BLE001
                self.state, self.error = 'disconnected', str(e)
            finally:
                try:
                    ser.close()
                except Exception:                        # noqa: BLE001
                    pass
            time.sleep(1.0)


# ------------------------------------------------------------ MJPEG video
class StreamReader(threading.Thread):
    """The camera's ONE client. Holds the latest JPEG; connects only while
    `viewers > 0` so an idle dashboard costs the camera nothing."""

    def __init__(self, base_url):
        super().__init__(daemon=True, name='stream')
        self.base_url = base_url.rstrip('/')
        self.jpeg = None
        self.jpeg_t = 0.0
        self.jpeg_n = 0
        self.state = 'idle'
        self.error = ''
        # Viewers are LEASES, not a counter. A proxy generator that the web
        # server abandons after a failed write (the browser went away) is
        # never finalised, so a plain counter leaks one per page load and the
        # camera reader can never go idle. A lease must be refreshed every
        # second; one not refreshed within LEASE_S is simply not a viewer.
        self._leases = {}          # token -> (last_touch, pinned)
        self._next_token = 1
        self._times = collections.deque(maxlen=120)
        self._subs = []
        self._lock = threading.Lock()
        self._bump = threading.Event()   # new frame arrived

    def subscribe(self, fn):
        with self._lock:
            self._subs.append(fn)

    def unsubscribe(self, fn):
        with self._lock:
            if fn in self._subs:
                self._subs.remove(fn)

    LEASE_S = 3.0

    def acquire(self, pinned=False):
        """Returns a token. Call touch(token) at least every LEASE_S while
        the viewer is alive, unless pinned (the recorder), and release() it
        when done -- release is a courtesy; expiry is the guarantee."""
        with self._lock:
            tok = self._next_token
            self._next_token += 1
            self._leases[tok] = (time.monotonic(), pinned)
            return tok

    def touch(self, tok):
        with self._lock:
            if tok in self._leases:
                self._leases[tok] = (time.monotonic(), self._leases[tok][1])

    def release(self, tok):
        with self._lock:
            self._leases.pop(tok, None)

    @property
    def viewers(self):
        now = time.monotonic()
        with self._lock:
            dead = [t for t, (seen, pinned) in self._leases.items()
                    if not pinned and now - seen > self.LEASE_S]
            for t in dead:
                del self._leases[t]
            return len(self._leases)

    def reconfigure(self, base_url):
        # Same no-op rule as the serial reader, and for a sharper reason: if
        # the URL is unchanged the run loop never re-enters its outer loop,
        # so a 'reconnecting' set here was never cleared and the pill said
        # "reconnecting" for ever while frames kept arriving.
        base_url = base_url.rstrip('/')
        if base_url == self.base_url:
            return
        self.base_url = base_url
        self.state = 'reconnecting'

    def wait_frame(self, last_n, timeout=1.0):
        """Block until a frame newer than last_n exists; returns (n, jpeg)."""
        if self.jpeg_n > last_n:
            return self.jpeg_n, self.jpeg
        self._bump.clear()
        self._bump.wait(timeout)
        return self.jpeg_n, self.jpeg

    def fps_estimate(self):
        now = time.monotonic()
        return sum(1 for t in self._times if now - t <= 1.0)

    LIVE_S = 2.0

    def status(self):
        age = (time.monotonic() - self.jpeg_t) if self.jpeg_t else None
        state = 'connected' if (age is not None and age < self.LIVE_S) else self.state
        return {'state': state, 'url': self.base_url, 'error': self.error,
                'viewers': self.viewers, 'frames': self.jpeg_n,
                'fps': self.fps_estimate(), 'age_s': age}

    def run(self):
        while True:
            if self.viewers <= 0:
                self.state = 'idle'
                time.sleep(0.25)
                continue
            url = self.base_url + '/stream'
            try:
                resp = urllib.request.urlopen(url, timeout=6)
            except Exception as e:                       # noqa: BLE001
                self.state, self.error = 'unreachable', str(e)
                time.sleep(2.0)
                continue
            self.state, self.error = 'connected', ''
            base = self.base_url
            try:
                while self.viewers > 0 and self.base_url == base:
                    line = resp.readline()
                    if not line:
                        raise EOFError('stream ended')
                    if not line.startswith(b'--'):
                        continue
                    headers = {}
                    while True:
                        h = resp.readline()
                        if h in (b'\r\n', b'\n', b''):
                            break
                        k, _, v = h.decode('latin-1').partition(':')
                        headers[k.strip().lower()] = v.strip()
                    n = int(headers.get('content-length', '0') or 0)
                    if n <= 0:
                        continue
                    buf = bytearray()
                    while len(buf) < n:
                        chunk = resp.read(n - len(buf))
                        if not chunk:
                            raise EOFError('short frame')
                        buf += chunk
                    host_t, wall_t = time.monotonic(), time.time()
                    jpeg = bytes(buf)
                    with self._lock:
                        self.jpeg, self.jpeg_t = jpeg, host_t
                        self.jpeg_n += 1
                        self._times.append(host_t)
                        subs = list(self._subs)
                    self._bump.set()
                    for fn in subs:
                        try:
                            fn(host_t, wall_t, jpeg)
                        except Exception:                # noqa: BLE001
                            pass
            except Exception as e:                       # noqa: BLE001
                self.state, self.error = 'disconnected', str(e)
            finally:
                try:
                    resp.close()
                except Exception:                        # noqa: BLE001
                    pass
            time.sleep(0.5)


# --------------------------------------------------------------- recording
def serial_reconnects(reader):
    return getattr(reader, 'reconnects', 0)


def _ffmpeg():
    import imageio_ffmpeg
    return imageio_ffmpeg.get_ffmpeg_exe()


class Recorder:
    """One session at a time: data rows and JPEG frames straight to disk as
    they arrive (no RAM growth), then the JPEG file is encoded to H.264 when
    the recording stops, so a browser can play it back."""

    def __init__(self, data_dir, serial_reader, stream_reader):
        self.data_dir = data_dir
        self.ser = serial_reader
        self.cam = stream_reader
        self.lock = threading.Lock()
        self.active = None          # dict while recording
        self.encoding = {}          # sid -> 'encoding' | 'done' | 'error: ...'

    # -- lifecycle ---------------------------------------------------------
    def start(self, name):
        with self.lock:
            if self.active:
                return None, 'already recording'
            sid = time.strftime('%Y%m%d-%H%M%S-') + uuid.uuid4().hex[:6]
            d = os.path.join(self.data_dir, sid)
            os.makedirs(d, exist_ok=True)
            t0 = time.monotonic()
            fcsv = open(os.path.join(d, 'frames.csv'), 'w', newline='')
            w = csv.writer(fcsv)
            w.writerow(['t_s', 'wall_time'] + CSV_FIELDS)
            fidx = open(os.path.join(d, 'video_index.csv'), 'w', newline='')
            wi = csv.writer(fidx)
            wi.writerow(['video_frame', 't_s', 'wall_time'])
            fmj = open(os.path.join(d, 'video.mjpeg'), 'wb')
            a = {'id': sid, 'dir': d, 'name': name or sid, 't0': t0,
                 'wall0': time.time(), 'rows': 0, 'frames': 0,
                 'tiers': collections.Counter(),
                 # A recording that silently stops collecting looks exactly
                 # like a short one. Measured once (2026-09-11): 204 rows in
                 # the first 1.8 s of an 8 s recording, cause unidentified
                 # and not reproduced in three later trials. So the gaps are
                 # recorded rather than assumed absent.
                 'last_row_t': None, 'max_gap_s': 0.0, 'gaps': 0,
                 'reconnects0': serial_reconnects(self.ser),
                 'fcsv': fcsv, 'w': w, 'fidx': fidx, 'wi': wi, 'fmj': fmj,
                 'last_frame_t': None}
            self.active = a
            self.write_meta(sid, {'id': sid, 'name': a['name'], 'kind': 'live',
                                  'created': a['wall0'], 'state': 'recording',
                                  'notes': ''})
            self.ser.subscribe(self._on_row)
            self.cam.subscribe(self._on_frame)
            a['lease'] = self.cam.acquire(pinned=True)
            return sid, None

    def stop(self):
        with self.lock:
            a = self.active
            if not a:
                return None, 'not recording'
            self.active = None
        self.ser.unsubscribe(self._on_row)
        self.cam.unsubscribe(self._on_frame)
        self.cam.release(a['lease'])
        for k in ('fcsv', 'fidx', 'fmj'):
            a[k].close()
        duration = time.monotonic() - a['t0']
        meta = self.read_meta(a['id']) or {'id': a['id'], 'name': a['name']}
        meta.update({'state': 'encoding' if a['frames'] else 'done',
                     'duration_s': round(duration, 3),
                     'rows': a['rows'], 'frames': a['frames'],
                     'data_fps': round(a['rows'] / duration, 1) if duration else 0,
                     'video_fps': round(a['frames'] / duration, 1) if duration else 0,
                     'video': bool(a['frames']),
                     'gaps': a['gaps'],
                     'max_gap_s': round(a['max_gap_s'], 3),
                     'reconnects': serial_reconnects(self.ser) - a['reconnects0'],
                     'tiers': dict(a['tiers'])})
        self.write_meta(a['id'], meta)
        if a['frames']:
            self.encoding[a['id']] = 'encoding'
            threading.Thread(target=self._encode, args=(a, duration),
                             daemon=True).start()
        else:
            try:
                os.remove(os.path.join(a['dir'], 'video.mjpeg'))
            except OSError:
                pass
        return a['id'], None

    def status(self):
        with self.lock:
            a = self.active
            if not a:
                return None
            return {'id': a['id'], 'name': a['name'],
                    'elapsed_s': round(time.monotonic() - a['t0'], 1),
                    'rows': a['rows'], 'frames': a['frames']}

    # -- feed hooks (called from the reader threads) -----------------------
    def _on_row(self, host_t, wall_t, row):
        with self.lock:
            a = self.active
            if not a:
                return
            a['w'].writerow(['%.4f' % (host_t - a['t0']), '%.3f' % wall_t]
                            + [row[k] if row[k] is not None else ''
                               for k in ('frame', 'tier', 'theta', 'x', 'y',
                                         'n_c', 'n_r', 'mad_h', 'mad_v', 'fps')])
            if a['last_row_t'] is not None:
                gap = host_t - a['last_row_t']
                if gap > a['max_gap_s']:
                    a['max_gap_s'] = gap
                if gap > 0.5:
                    a['gaps'] += 1
            a['last_row_t'] = host_t
            a['rows'] += 1
            a['tiers'][row['tier']] += 1

    def _on_frame(self, host_t, wall_t, jpeg):
        with self.lock:
            a = self.active
            if not a:
                return
            a['fmj'].write(jpeg)
            a['wi'].writerow([a['frames'], '%.4f' % (host_t - a['t0']),
                              '%.3f' % wall_t])
            a['frames'] += 1
            a['last_frame_t'] = host_t

    # -- encode ------------------------------------------------------------
    def _encode(self, a, duration):
        sid, d = a['id'], a['dir']
        src = os.path.join(d, 'video.mjpeg')
        dst = os.path.join(d, 'video.mp4')
        # constant input rate = frames over wall time, so playback length
        # matches the real duration (the mjpeg file carries no timestamps)
        rate = max(1.0, a['frames'] / duration) if duration > 0 else 30.0
        cmd = [_ffmpeg(), '-y', '-loglevel', 'error',
               '-f', 'mjpeg', '-framerate', '%.3f' % rate, '-i', src,
               '-c:v', 'libx264', '-pix_fmt', 'yuv420p', '-preset', 'veryfast',
               '-movflags', '+faststart', dst]
        try:
            r = subprocess.run(cmd, capture_output=True, text=True, timeout=1800)
            if r.returncode != 0:
                raise RuntimeError(r.stderr.strip()[-400:] or 'ffmpeg failed')
            os.remove(src)
            self.encoding[sid] = 'done'
            state, err = 'done', None
        except Exception as e:                           # noqa: BLE001
            self.encoding[sid] = 'error'
            state, err = 'error', str(e)
        meta = self.read_meta(sid) or {'id': sid}
        meta['state'] = state
        meta['video_rate'] = round(rate, 3)
        if err:
            meta['error'] = err
        self.write_meta(sid, meta)

    # -- meta --------------------------------------------------------------
    def read_meta(self, sid):
        p = os.path.join(self.data_dir, sid, 'meta.json')
        if not os.path.exists(p):
            return None
        with open(p) as f:
            return json.load(f)

    def write_meta(self, sid, meta):
        with open(os.path.join(self.data_dir, sid, 'meta.json'), 'w') as f:
            json.dump(meta, f, indent=2)
