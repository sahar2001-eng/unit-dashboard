# The REAL gridcam camera loop (VisionService.run() with the real config,
# TurnService, UART link and its V-command dispatch) over the frames of one
# raw clip, under the unix MicroPython. Only the sensor, the UART wire and
# the log are fakes. Run by engine.py inside the run kit's container:
#
#   micropython -X heapsize=64M mp_run.py <job.json>
#
# job: app, t2lib, pcx64, raw, n (frames), tile, clock_us,
#      cmds [[frame, "V 20 1"], ...]  (as if the STM32 sent them),
#      set {"config.key": value}, out
# Out: one JSON line per published frame:
#   {"ck": clip frame, "frame": frame_id, "tile", "tier", "suspect", "x_mm",
#    "y_mm", "theta_deg", "code", "q", "why", "mode", "heading" (turns
#    only), "ops": [...]}
# "ops" are the camera's own overlay draw calls for that frame (overlay.draw
# on a recording image), then a last {"summary": ...} line.

import sys
import json
import time

with open(sys.argv[1]) as _f:
    JOB = json.load(_f)
W, H = 320, 200
FRAME = W * H

sys.path.insert(0, JOB['app'])
sys.path.append(JOB['pcx64'])


class _Ulab:            # camera.py imports ulab; the loop never uses it
    numpy = None


class Stop(Exception):
    pass


class Wire:
    sent = 0

    def __init__(self, *a, **kw):
        pass

    def write(self, data):
        Wire.sent += 1
        return len(data)

    def read(self, n=-1):
        return None

    def any(self):
        return 0


class _Machine:
    UART = Wire


sys.modules['ulab'] = _Ulab
sys.modules['machine'] = _Machine

from gridcam.vision import tile2 as t2mod     # noqa: E402
t2mod._lib_dir = lambda: JOB['t2lib']


class Log:
    def __init__(self):
        self.lines = []

    def info(self, m):
        self.lines.append(('info', m))

    def warn(self, m):
        self.lines.append(('warn', m))

    def error(self, m):
        self.lines.append(('error', m))

    def tagged(self, name):
        return self


class Bus:
    def __init__(self, log):
        self._subs = {}
        self._log = log

    def subscribe(self, topic, cb):
        self._subs.setdefault(topic, []).append(cb)

    def publish(self, topic, payload):
        for cb in self._subs.get(topic, ()):
            try:
                cb(payload)
            except Exception as e:
                self._log.error('bus %s handler: %r' % (topic, e))


class RecImg:
    """Takes the overlay's draw calls instead of drawing them."""

    def __init__(self, ops):
        self.ops = ops

    def draw_line(self, xy, color=255, thickness=1):
        self.ops.append(['line', list(xy), color])

    def draw_cross(self, xy, color=255, size=5, thickness=1):
        self.ops.append(['cross', list(xy), color, size])

    def draw_string(self, xy, text, color=255, scale=1, **kw):
        self.ops.append(['text', list(xy), text, color, scale])

    def draw_rectangle(self, r, color=255, thickness=1, fill=False):
        self.ops.append(['rect', list(r), color, 1 if fill else 0])


def main():
    from gridcam import util
    from gridcam.config import ConfigService
    from gridcam.vision import camera as cmod
    from gridcam.vision import overlay
    import gridcam.vision.vision_service as vs
    from gridcam.vision.turn_service import TurnService
    from gridcam.comms.uart_link import UartLinkService
    import rawgray

    clk = [0]
    step_us = JOB.get('clock_us', 33333)
    util.ticks_ms = lambda: clk[0] // 1000
    util.ticks_us = lambda: clk[0]
    total = JOB['n']

    ops = []
    real_draw = overlay.draw

    def rec_draw(img, res, w, h, scale=1.0, lines=True, text=True):
        del ops[:]
        real_draw(RecImg(ops), res, w, h, scale, lines, text)
        raise RuntimeError('no JPEG here: the PC draws the picture')
    vs.overlay.draw = rec_draw

    class Cam:
        @staticmethod
        def otsu(img, scale, bins=False):
            return cmod.Camera.otsu(img, scale, bins)

        def __init__(self, *a, **kw):
            self.k = 0
            self.f = open(JOB['raw'], 'rb')
            self.buf = bytearray(FRAME)
            self.fr = cmod.GrayFrame(rawgray)
            self.light = (None, None)

        def setup(self):
            pass

        def readback(self):
            return {}

        def set_light(self, e=None, g=None):
            if (e, g) == self.light:
                return False
            self.light = (e, g)
            return True

        def apply_settings(self):
            pass

        def snapshot(self):
            if self.k >= total:
                raise Stop()
            if self.f.readinto(self.buf) != FRAME:
                raise Stop()
            self.cur = self.k
            self.k += 1
            clk[0] += step_us
            self.fr.set(self.buf, W, H)
            return self.fr

    class App:
        pass

    log = Log()
    app = App()
    app.log = log
    app.bus = Bus(log)
    app.ticks_ms = util.ticks_ms
    app.camled = None
    app.config = ConfigService(app, path=JOB['out'] + '.cfg.json')
    app.config.setup()
    app.config.set('vision.print_csv', False)
    app.config.set('uart.fast_tx', False)
    app.config.set('vision.tile', JOB.get('tile', 1))
    # the stream overlay on every frame, at full size (display only)
    app.config.set('http.mjpeg_divisor', 1)
    app.config.set('http.mjpeg_scale', 1.0)
    app.config.set('http.overlay_lines', True)
    app.config.set('http.overlay_text', True)
    for key in JOB.get('set', {}):
        app.config.set(key, JOB['set'][key])

    vs.Camera = Cam
    app.vision = vs.VisionService(app)
    app.vision.setup()
    app.uart = UartLinkService(app)
    app.uart.setup()
    app.turn = TurnService(app)
    app.turn.setup()
    svc = app.vision
    svc.stream_clients = 1          # as if the page were watching

    cmds = {}
    for c in JOB.get('cmds', []):
        cmds.setdefault(int(c[0]), []).append(c[1])
    fout = open(JOB['out'], 'w')
    cam = svc._camera
    count = [0]

    def on_frame(r):
        t = svc.tile() if hasattr(svc, 'tile') else JOB.get('tile', 1)
        d = {'ck': cam.cur, 'frame': r.frame_id, 'tile': t, 'tier': r.tier,
             'suspect': 1 if r.suspect else 0, 'x_mm': r.x_mm,
             'y_mm': r.y_mm, 'theta_deg': r.theta_deg,
             'code': r.junction_code, 'q': r.q, 'why': r.q_why,
             'mode': app.turn.state_name(), 'ops': list(ops)}
        if d['mode'] != 'DRIVE':
            d['heading'] = app.turn.heading()[0]   # the turn angle (0xA8 phi)
        del ops[:]
        fout.write(json.dumps(d))
        fout.write('\n')
        count[0] += 1
    app.bus.subscribe('frame', on_frame)

    import asyncio

    async def host():
        done = -1
        while True:
            k = cam.k
            if k != done:
                done = k
                for line in cmds.get(k, ()):
                    app.bus.publish('uart_cmd', line.split(' '))
            await asyncio.sleep(0)

    async def run():
        asyncio.create_task(host())
        await asyncio.sleep(0)
        await svc.run()

    t0 = time.ticks_ms()
    try:
        asyncio.run(run())
    except Stop:
        pass
    fout.close()
    st = svc.stats()
    summary = {'frames_in': cam.k, 'frames_out': count[0], 'tile': st.get('tile'),
               'tile_err': st.get('tile_err'), 'wall_ms': time.ticks_ms() - t0,
               'errors': [x[1] for x in log.lines if x[0] == 'error'],
               'warnings': [x[1] for x in log.lines if x[0] == 'warn'],
               'turn': app.turn.stats()}
    with open(JOB['out'], 'a') as f:
        f.write(json.dumps({'summary': summary}))
        f.write('\n')
    print('frames in %d out %d' % (cam.k, count[0]))


main()
