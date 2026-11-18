# VisionService — the application's hot path.
#
# Loop: snapshot -> Otsu (every N frames) -> GridTracker.analyse on the raw
# framebuffer -> publish FrameResult on the app bus (UART link, telemetry and
# the MJPEG streamer subscribe there). The overlay is drawn only while a
# stream client is connected, so the annotation cost is zero in normal
# operation. Two awaits per frame give network tasks CPU.
#
# fps is measured top-of-loop to top-of-loop, NOT with time.clock() — see
# the note in run(); clock.fps() ignores the tail of the loop body.
#
# TWO TILES (1.32.0): vision.tile 1 = tape grid (pipeline.GridTracker), 2 =
# UNIT16 hole grid (tile2.Tile2Tracker). Both are ready from setup (Tile 2's
# code is imported first, before the heap fragments) and each keeps its own
# tracker and quality scorer. A switch is a request (request_tile) that the
# loop applies between frames, held while a turn runs or the camera is lent
# out; after it no x/y goes out until the new tile's first good junction.

from .. import util
from ..service import Service
from . import overlay, pipeline
from . import tile2 as t2mod
from . import quality as qmod
from .points import PointStats
from .camera import Camera
from .quality import why_code

# the one-shot mark crash_note() leaves after a vision crash on Tile 2
# (crash.CrashGuard.mark); the next boot reads and removes it
T2_CRASH_MARK = 'vision_tile2.mark'

# capture timeouts in a row before the loop lets one crash (and reboot)
CAM_TRIES = 5


class FrameResult:
    """One frame's measurements. Slotted to avoid per-frame dict churn.

    x_mm: SIGNED distance to the nearer junction, about -30 to +30 mm.
          None means either "between junctions" (the dead zone mid-cell,
          a valid reading) or that a computed value was refused as
          physically impossible — `suspect` distinguishes the two.
    y_mm: junction / bar offset from the frame centre. theta_deg: bar angle.
    q: quality 0..100 (vision/quality.py), None while turning or when the
       score is off; q_why: the reason when q is low ('' when fine).
    prec_*: 1-sigma statistical precision of this frame's fit.
    """

    __slots__ = ('frame_id', 'tier', 'theta_deg', 'theta_split', 'x_mm',
                 'y_mm', 'node', 'n_c', 'n_r', 'mad_h', 'mad_v', 'suspect',
                 'fps', 'ts_ms', 'q', 'q_why', 'q_img', 'q_line', 'q_rec',
                 'q_code', 'q_flags',
                 'prec_x_mm', 'prec_y_mm', 'prec_th_deg', 'junction_code')

    def __init__(self):
        self.frame_id = 0
        self.tier = None
        self.theta_deg = None
        # How far the two band fits disagreed about the angle, degrees. Only
        # the junction tier fits both bands, so it is None elsewhere. A large
        # value is a frame whose angle should not be steered on -- distinct
        # from having no angle at all.
        self.theta_split = None
        self.x_mm = None
        self.y_mm = None
        self.node = None
        self.junction_code = None   # Tile 2: the code junction's name ('13E')
        self.n_c = 0
        self.n_r = 0
        self.mad_h = None
        self.mad_v = None
        self.suspect = False
        self.fps = 0.0
        self.ts_ms = 0
        self.q = None
        self.q_why = ''
        self.q_img = None
        self.q_line = None
        self.q_rec = None
        self.q_code = 0                       # quality.WHY_CODES index
        self.q_flags = 0                      # bit0 learning, bit1 low alarm
        self.prec_x_mm = None
        self.prec_y_mm = None
        self.prec_th_deg = None


def _num(v):
    return isinstance(v, (int, float)) and not isinstance(v, bool)


def _pos(v):
    """v when it is a number above 0, else None (= keep the Tile 1 value)."""
    return v if _num(v) and v > 0 else None


def _fmt(v):
    return ('%.3f' % v) if v is not None else ''


def _m(v, fmt='%.3f'):
    return '-' if v is None else fmt % v


def frame_meta(frame_i, ts_ms, res, fps, tile=None):
    """One line per streamed picture: what the camera measured on it.
    'key=value' pairs, space-separated; an absent value is '-', never 0.
    New keys go at the END only (1.32.0: tile, code), so older readers
    that split on spaces keep working. Without `tile`, a result with a
    Tile 2 code slot counts as Tile 2."""
    code = res.get('t2_code')
    if tile is None:
        tile = 2 if 't2_code' in res else 1
    return ('frame=%d ts_ms=%d tier=%s%s x=%s y=%s th=%s fps=%.1f'
            ' tile=%d code=%s' % (
                frame_i, ts_ms, res.get('tier') or '-',
                '!' if res.get('suspect') else '', _m(res.get('x_mm')),
                _m(res.get('y_mm')), _m(res.get('theta_deg')), fps,
                tile, code or '-'))


class VisionService(Service):
    name = "vision"
    critical = True

    CSV_HEADER = 'frame,tier,theta_deg,x_mm,y_mm,n_c,n_r,mad_h,mad_v,fps'

    def setup(self):
        cfg = self.app.config
        # Tile 2's code FIRST, on both tiles: u16c and the Pipe's ~26 KB are
        # asked for before anything else here takes heap, so a switch hours
        # later never needs a big block (bench DESIGN.md: import u16c first).
        self._tile_err = ''
        self._t2_lib = None                   # (u16pipe, code words, names)
        self._t2_why = ''                     # why Tile 2 cannot run
        self._trk = [None, None, None]        # tracker per tile, built once
        self._tile = 0                        # no tile yet (_build_tile2)
        self._tracker = None
        self._load_tile2()

        self._pipe_cfg = cfg.section('pipeline')
        self._print_csv = cfg.get('vision.print_csv', True)
        self._mm_per_px = 80.0 / cfg.get('camera.pixels_per_80mm')
        self._t2_mm = cfg.get('tile2.mm_per_px', 0.220)
        # vision.tile: 1 tape grid, 2 UNIT16 hole grid. Saved, so the camera
        # boots on the last tile, also after a vision crash on Tile 2: never
        # Tile 1 instead (on a Tile 2 floor it sends a wrong y to the STM32).
        # tile_err says it crashed; main.py's safe mode stops a crash loop.
        # The crash mark is read (and removed) on every boot, so it only
        # ever speaks about the boot just before this one.
        crashed2 = self._crashed_on_tile2()
        t = cfg.get('vision.tile', 1)
        if type(t) is not int or t not in (1, 2):
            self.log.warn('vision.tile %r ignored: use 1 or 2' % (t,))
            t = 1
        elif t == 2 and crashed2:
            self._tile_err = ('vision crashed on tile 2 last boot: still on '
                              'tile 2')
            self.log.error(self._tile_err)
        self._tile = t
        self._tile_cfg_seen = cfg.get('vision.tile', 1)
        self._tile_want = 0                   # tile asked for, 0 = none
        self._tile_switches = 0
        self._tile_switch_ms = None           # how long the last switch took
        self._fix_hold = False                # no x/y until a good junction
        self._t2_errors = 0                   # Tile 2 frame errors
        self._t2_logged = False

        self._camera = Camera(cfg, self.log)
        # the tile's own light, before the camera's first latch
        e, g, _lv, _pct = self._tile_light(t)
        set_light = getattr(self._camera, 'set_light', None)
        if set_light is not None:
            set_light(e, g)
        self._camera.setup()
        self._tracker = None                  # the active tile's (needs W/H)
        self._th = 0                          # cached Otsu threshold

        self._result = FrameResult()          # reused every frame
        self._frame_i = 0
        # Frames since the active tile started (0 at boot, as _frame_i, and
        # 0 again on a switch). The Otsu and light rhythms count these, so a
        # tile switched back to runs exactly as a fresh boot on it would.
        self._tile_fi = 0
        self._fps = 0.0
        self._busy_ms = 0.0                   # EMA of per-frame work time
        self._frames_rejected = 0             # frames with zero candidates
        self._reject_ema = 0.0                # recent rejection ratio
        self._last_valid_ms = util.ticks_ms()
        self._no_scene = False
        self._no_scene_after_s = cfg.get('vision.no_scene_after_s', 10)
        self._gc_every = cfg.get('vision.gc_every', 30)
        self.stream_clients = 0               # managed by net/stream.py
        self.points = PointStats()            # scan points per frame
        self.last_jpeg = None                 # latest compressed frame
        self.last_meta = ''                   # ... and its frame's measurement
        self.reject_jpeg = None               # latest frame with NO result
        self._reject_ms = 0
        self.paused = False                   # apriltag VGA switch parks us
        self._tap = None                      # one-shot raw-frame consumer
        self._cam_dirty = False               # camera.* config changed
        self._cam_fails = 0                   # capture timeouts in a row
        self._cam_timeouts = 0                # ... in all, for telemetry
        self._jpeg_divisor = cfg.get('http.mjpeg_divisor', 2)
        self._jpeg_quality = cfg.get('http.mjpeg_quality', 50)
        self._jpeg_scale = cfg.get('http.mjpeg_scale', 0.5)
        self._ov_lines = cfg.get('http.overlay_lines', True)
        self._ov_text = cfg.get('http.overlay_text', True)

        self._quality = None                  # the active tile's scorer
        self._qual = [None, None, None]       # scorer per tile
        self._light_due = 0                   # _tile_fi of the next light read
        self._setup_quality()

        self._c_kernels = None                # vision.c_kernels last applied
        self._set_kernels(cfg.get('vision.c_kernels', True))

        self._profile = cfg.get('vision.profile', False)
        self._prof = {}                       # stage -> EMA microseconds
        self._prof_acc = {}                   # reused dict handed to analyse()
        self._prof_cam_end = 0
        self._last_top = None                 # previous top-of-loop stamp
        # Loop-stall monitor (gridcam/stall.py): ms stamp taken just before
        # this loop yields to the other tasks, None when the next resume is
        # not a frame boundary (first frame, paused).
        self._stall = getattr(self.app, 'stall', None)
        self._yield_ms = None
        self._period_us = 0.0                 # EMA of the true frame period
        cfg.on_change(self._on_config)

        if self._print_csv:
            print(self._csv_header())

    def _setup_quality(self):
        """(Re)build the quality scorers from config, one per tile. A rebuild
        forgets what it learned about "normal"; it re-learns in a few
        seconds of driving."""
        q = self.app.config.section('quality')
        self._set_qkernels(q.get('c_kernels', True))
        self._qual[1] = self._make_quality(1)
        self._qual[2] = self._make_quality(2)
        self._use_quality()

    def _make_quality(self, tile):
        """The scorer for `tile`, or None when the score is off.

        Tile 1: quality.Quality, which reads Tile 1's tracker buffers.
        Tile 2: the same class with quality.tile2 laid over the section
        (quality.make_quality); the loop calls its frame_sig(trk.signals(),
        trk.mm) instead of frame(). While it is None (score off), Tile 2
        runs no Otsu at all (the Pipe makes its own threshold)."""
        return qmod.make_quality(tile, self.app.config.section('quality'))

    def _use_quality(self):
        """Point the loop at the active tile's scorer."""
        self._quality = self._qual[self._tile]
        q = self.app.config.section('quality')
        self._q_csv = bool(q.get('csv', False)) and self._quality is not None
        self._light_due = 0

    def quality(self):
        return self._quality

    def _csv_header(self):
        return self.CSV_HEADER + (',q,why' if self._q_csv else '')

    def _on_config(self, path, value):
        def touched(section):
            """Did this change actually reach `section`?

            set() reports a dotted path; patch() reports path=None with the
            merged overlay as `value`. Treating path=None as "everything
            changed" made ANY patch mark the camera dirty, costing a ~1 s
            re-latch of the vision loop for an unrelated key (observed while
            tuning the mjpeg knobs over HTTP).
            """
            if path is None:
                return isinstance(value, dict) and section in value
            return path.startswith(section + '.')

        if touched('camera'):
            # applied by the loop via the full latch sequence — a bare
            # auto_exposure() on a live pipeline is silently dropped
            self._cam_dirty = True
            # pixels_per_80mm is consumed here, not by the camera, so it
            # would otherwise be the cache-and-forget trap below
            self._mm_per_px = 80.0 / self.app.config.get('camera.pixels_per_80mm')
        if touched('pipeline'):
            # Tile 1's tunables only: Tile 2 keeps its tracker and state
            self._pipe_cfg = self.app.config.section('pipeline')
            self._trk[1] = None               # rebuilt with the new tunables
            if self._tile == 1:
                self._tracker = None
                self._th = 0
        if touched('tile2'):
            self._on_tile2_config()
        if touched('vision'):
            # re-read EVERY vision.* setting, not just the one that prompted
            # this. setup() caches them in attributes, so anything missed here
            # is silently unchangeable at runtime (POST /config appears to
            # succeed and does nothing) — print_csv was exactly that bug.
            cfg = self.app.config
            was = self._profile
            self._profile = cfg.get('vision.profile', False)
            self._print_csv = cfg.get('vision.print_csv', True)
            self._no_scene_after_s = cfg.get('vision.no_scene_after_s', 10)
            self._gc_every = cfg.get('vision.gc_every', 30)
            self._set_kernels(cfg.get('vision.c_kernels', True))
            if self._profile != was:          # stale EMAs would be misleading
                self._prof = {}               # (the period EMA is unaffected)
            # vision.tile: only a CHANGE of the setting asks for a switch, so
            # a write to another vision.* key never undoes a refused one
            t = cfg.get('vision.tile', 1)
            if t != self._tile_cfg_seen:
                self._tile_cfg_seen = t
                ok, why = self.request_tile(t)
                if not ok:
                    self.log.warn('vision.tile %r refused: %s' % (t, why))
                    self._tile_cfg_back()
        if touched('quality'):
            self._setup_quality()
        if touched('http'):
            # same cache-and-forget trap as vision.* above: these are read
            # once in setup(), so without this the mjpeg knobs cannot be
            # tuned at runtime — which is exactly when you want them
            cfg = self.app.config
            self._jpeg_divisor = cfg.get('http.mjpeg_divisor', 2)
            self._jpeg_quality = cfg.get('http.mjpeg_quality', 50)
            self._jpeg_scale = cfg.get('http.mjpeg_scale', 0.5)
            self._ov_lines = cfg.get('http.overlay_lines', True)
            self._ov_text = cfg.get('http.overlay_text', True)

    def camera(self):
        return self._camera

    def _turn(self):
        # lazily, like the apriltag service: build order puts turn after
        # vision, and the vision loop must not care whether it exists.
        return getattr(self.app, 'turn', None)

    def tracker(self):
        return self._tracker

    def tap_frame(self, callback):
        """Register a one-shot consumer of the next raw frame. The callback
        runs synchronously inside the vision loop (may cost one frame) with
        (img, w, h) BEFORE the overlay is drawn."""
        self._tap = callback

    def _turning_result(self, turn):
        """A result dict for a frame where Algorithm 1 did not run.

        Everything Algorithm 1 owns is None — during a turn there is no
        meaningful X, Y or bar angle, and publishing the pre-turn values
        would be worse than publishing nothing.
        """
        phi, _omega = turn.heading()
        res = {'tier': None, 'status': 'turning',
               'theta_deg': None, 'x_mm': None, 'y_mm': None,
               'theta_split': None, 'node': None,
               'line_x': None, 'line_y': None, 'suspect': False,
               'n_c': 0, 'n_r': 0, 'tc': 0, 'tr': 0,
               'mad_h': None, 'mad_v': None,
               'threshold': self._th,
               'centre': (self._tracker.W * 0.5, self._tracker.H * 0.5)}
        if self._tile == 2:
            # the Tile 2 slot, so the overlay and stream meta read it as
            # Tile 2 (Tile 1's dict stays as it was)
            res['t2_code'] = None
        return res

    def _fill_result(self, res):
        r = self._result
        r.frame_id = self._frame_i
        r.tier = res['tier']
        r.theta_deg = res['theta_deg']
        r.theta_split = res['theta_split']
        r.x_mm = res['x_mm']
        r.y_mm = res['y_mm']
        r.node = res['node']
        r.junction_code = res.get('t2_code')
        r.n_c = res['n_c']
        r.n_r = res['n_r']
        r.mad_h = res['mad_h']
        r.mad_v = res['mad_v']
        r.suspect = res['suspect']
        r.fps = self._fps
        r.ts_ms = util.ticks_ms()
        q = self._quality
        if q is not None and q.q is not None:
            r.q, r.q_why = q.q, q.why
            r.q_img, r.q_line, r.q_rec = q._img_now, q.q_line, q.q_rec
            r.q_code = why_code(q.why)
            r.q_flags = ((1 if q.clean_frames < q.cfg['warm_frames'] else 0)
                         | (2 if q._low else 0))
            r.prec_x_mm, r.prec_y_mm = q.prec_x_mm, q.prec_y_mm
            r.prec_th_deg = q.prec_th_deg
        else:
            r.q, r.q_why = None, ''
            r.q_img = r.q_line = r.q_rec = None
            r.q_code = r.q_flags = 0
            r.prec_x_mm = r.prec_y_mm = r.prec_th_deg = None

        # scene health: a frame with no run candidates in EITHER scan is
        # "rejected" (dark/flat image — lens covered, board face-down...).
        #
        # NOT tc/tr any more: since v1_2 those are anchored to
        # nominal_thick_px whenever the histogram peak is implausible, so
        # they are never 0 and would report a covered lens as healthy. The
        # counts are safe again because v1_2 gave row_scan a maxlen, so a
        # whole-row run on a flat frame at threshold 0 is now rejected by
        # width instead of being counted (verified at th = 0, 20, 39, 41).
        rejected = (res['n_c'] == 0 and res['n_r'] == 0
                    and res['status'] != 'turning')
        if rejected:
            self._frames_rejected += 1
        else:
            self._last_valid_ms = r.ts_ms
        self._reject_ema = 0.98 * self._reject_ema + (0.02 if rejected else 0.0)
        blind_s = util.ticks_diff(r.ts_ms, self._last_valid_ms) / 1000.0
        no_scene = blind_s > self._no_scene_after_s
        if no_scene and not self._no_scene:
            self.log.warn('NO SCENE: no usable frame for %.0f s — is the '
                          'camera covered or in the dark?' % blind_s)
        elif self._no_scene and not no_scene:
            self.log.info('scene recovered')
        self._no_scene = no_scene
        return r

    def _set_jpeg(self, jpeg, meta):
        """The stream picture and the measurement of THAT frame, together:
        net/stream.py sends the meta as the part's X-Gridcam header."""
        self.last_meta = meta
        self.last_jpeg = jpeg

    def tile(self):
        """The tile this loop reads now."""
        return self._tile

    def tile_pending(self):
        """True while a switch is asked for and not applied yet. A V 20
        turn is refused meanwhile (turn_service)."""
        return self._tile_want != 0

    # ------------------------------------------------------------ tiles

    def _load_tile2(self):
        """Import Tile 2's code and make its Pipe now (setup, both tiles).
        A failure leaves Tile 2 unavailable, with the reason in tile_err."""
        try:
            self._t2_lib = t2mod._load()
        except Exception as e:
            self._t2_unavailable(e)
            return
        self._build_tile2()

    def _build_tile2(self):
        """A new Tile 2 tracker from the tile2.* settings; True when made.
        The new one is made BEFORE the old one goes, so a value the Pipe
        refuses (its own range checks) keeps the running tracker
        (_on_tile2_config then puts the old values back). Only when there
        is no room for two Pipes does the old one go first. The frame size
        is checked on the first frame it reads (_tile2_tracker)."""
        lib = self._t2_lib[0]
        w, h = getattr(lib, 'FRAME_W', 320), getattr(lib, 'FRAME_H', 200)
        c = self.app.config.section('tile2')
        try:
            try:
                trk = t2mod.Tile2Tracker(w, h, c, self._t2_lib)
            except MemoryError:
                import gc
                self._trk[2] = None           # no room for two: old goes
                if self._tile == 2:
                    self._tracker = None
                gc.collect()
                trk = t2mod.Tile2Tracker(w, h, c, self._t2_lib)
        except Exception as e:                # a bad tile2.* value
            if not isinstance(self._trk[2], t2mod.Tile2Tracker):
                self._trk[2] = None
                self._t2_unavailable(e)
            else:
                self._tile_err = ('tile2.* refused, running values kept: %s'
                                  % (str(e) or e.__class__.__name__))
                self.log.warn(self._tile_err)
            return False
        self._trk[2] = trk
        self._t2_why = ''
        return True

    def _t2_unavailable(self, e):
        self._t2_why = str(e) or e.__class__.__name__
        self._tile_err = 'tile 2 unavailable: %s' % self._t2_why
        self.log.warn(self._tile_err)

    def _crashed_on_tile2(self):
        """Did the boot just before this one end in a vision crash on Tile
        2? Read from the one-shot mark crash_note() leaves, which this
        removes, so an old crash report can never answer for a later boot
        (a crash in setup, for example, writes no service report)."""
        guard = getattr(self.app, 'crash_guard', None)
        if guard is None:
            return False
        try:
            mark = guard.take_mark(T2_CRASH_MARK)
        except Exception:
            return False
        counters = getattr(self.app, 'counters', None)
        return mark and (counters is None or counters.crashes > 0)

    def crash_note(self):
        """Added to a crash report's context. app._supervise calls it only
        when this service crashed; on Tile 2 it also leaves the one-shot
        mark the next boot reads (setup: tile_err says so)."""
        if self._tile == 2:
            guard = getattr(self.app, 'crash_guard', None)
            if guard is not None:
                guard.mark(T2_CRASH_MARK)
        return 'tile=%d' % self._tile

    def _tile_cfg_back(self):
        """A refused or dropped switch is not saved: vision.tile goes back
        to the tile asked for or running, so the next boot never starts on
        a tile the camera refused (the page button's rule too)."""
        back = self._tile_want or self._tile
        self._tile_cfg_seen = back
        if self.app.config.get('vision.tile', 1) != back:
            self.app.config.set('vision.tile', back, persist=True)

    def request_tile(self, t):
        """Ask for tile `t` (1 or 2). Returns (ok, reason). Applied by the
        loop between frames; held while a turn runs or the camera is lent
        out. Does not write the config: patch vision.tile for that (which
        calls this)."""
        if type(t) is not int or t not in (1, 2):   # not True, not 1.0
            return False, 'use 1 or 2'
        if t == 2 and self._t2_why:
            return False, 'tile 2 unavailable: %s' % self._t2_why
        if t == self._tile:
            self._tile_want = 0               # also cancels a pending switch
            return True, 'already on tile %d' % t
        self._tile_want = t
        turn = self._turn()
        if turn is not None and turn.busy():
            return True, 'pending: turning'
        if self.paused:
            return True, 'pending: camera lent out'
        return True, 'pending'

    def _apply_tile(self):
        """The pending switch, at the top of the loop (between frames)."""
        turn = self._turn()
        if turn is not None and turn.busy():
            return                            # never mid-turn; try next frame
        t0 = util.ticks_ms()
        t, was = self._tile_want, self._tile
        self._tile_want = 0
        if t == 2 and self._t2_why:           # broke after the request
            self.log.warn('tile 2 switch dropped: %s' % self._t2_why)
            self._tile_cfg_back()
            return
        self._tile = t
        trk = self._trk[t]
        if trk is not None:
            trk.reset()
        self._tracker = None                  # this frame: _tracker_for (W/H)
        if self._qual[t] is not None:
            self._qual[t].reset()
        self._use_quality()
        self._th = 0
        self._tile_fi = 0                     # Otsu rhythm from here
        self.points.reset()
        self._reject_ema = 0.0
        self._last_valid_ms = t0
        self._no_scene = False
        self._fix_hold = True                 # no x/y until a good junction
        self._t2_logged = False
        self._prof = {}                       # the old tile's stage costs
        if turn is not None:
            turn.tile_switched()
        self._apply_light(t)
        if self._cam_dirty:                   # the tile's exposure/gain
            self._cam_dirty = False
            self._latch_camera()
        self._tile_switches += 1
        self._tile_switch_ms = util.ticks_diff(util.ticks_ms(), t0)
        self.log.info('tile %d (was %d), switched in %d ms'
                      % (t, was, self._tile_switch_ms))
        if self._print_csv:
            print('tile %d' % t)
            print(self._csv_header())
        self._last_top = None                 # a switch is not a frame period

    def _tile_light(self, t):
        """(exposure_us, gain_db, led level, LED % tuple) saved for tile t;
        None = keep the camera.* / camled.* value. Tile 1 is all None."""
        if t != 2:
            return None, None, None, None
        c = self.app.config.section('tile2')
        leds = c.get('leds', [])
        pct = None
        # 'V 5' writes are not type-checked: anything odd means "keep"
        if isinstance(leds, list) and len(leds) == 6 and all(
                _num(p) for p in leds):
            pct = tuple(int(p) for p in leds)
        return (_pos(c.get('exposure_us', 0)), _pos(c.get('gain_db', 0)),
                _pos(c.get('led_level', 0)), pct)

    def tile_led(self):
        """(level, LED %) for the active tile, None = camled.* (camled)."""
        _e, _g, lv, pct = self._tile_light(self._tile)
        return lv, pct

    def _apply_light(self, t):
        """Tile t's saved light: the camera re-latches (by the loop, ~1 s)
        only when exposure or gain really change; the lamp line goes out
        only on a change."""
        e, g, lv, pct = self._tile_light(t)
        set_light = getattr(self._camera, 'set_light', None)
        if set_light is not None and set_light(e, g):
            self._cam_dirty = True
        led = getattr(self.app, 'camled', None)
        if led is not None:
            try:
                led.set_tile_light(lv, pct)
            except Exception as e:
                self.log.error('tile light (lamp) failed: %s' % e)

    def _cam_timeout(self, e):
        """A passing 'Frame capture has timed out' costs a frame, not a
        reboot (camera.py retries the same at boot). True = go on; False =
        let it crash: another error, or CAM_TRIES timeouts in a row."""
        if 'timed out' not in str(e) or self._cam_fails >= CAM_TRIES - 1:
            return False
        self._cam_fails += 1
        self._cam_timeouts += 1
        self.log.warn('camera: %s (%d in a row), going on'
                      % (e, self._cam_fails))
        self._last_top = None                 # not a frame period
        return True

    def _latch_camera(self):
        try:
            self._camera.apply_settings()     # blocking latch dance (~1 s)
        except RuntimeError as e:
            if not self._cam_timeout(e):
                raise
            self._cam_dirty = True            # latch again on a later frame
            return
        rb = self._camera.readback()
        self.log.info('camera settings applied: %s' % rb)
        self._last_top = None

    def _on_tile2_config(self):
        """tile2.* changed: a new Pipe only when its own settings changed
        (made here, between frames), the light at once when Tile 2 is on.
        A value the new Pipe refuses keeps the running one and its values
        go back into the config (saved), so neither this run nor the next
        boot is blind; with no running Pipe, Tile 2 is unavailable with
        the reason until a good value comes."""
        c = self.app.config.section('tile2')
        self._t2_mm = c.get('mm_per_px', 0.220)
        if self._t2_lib is not None:
            try:
                new = t2mod.pipe_args(c)
            except Exception:
                new = None
            old = self._trk[2]
            if new != getattr(old, 'args', None):
                if self._build_tile2():
                    if self._tile == 2:
                        self._tracker = None  # checked again on the next frame
                    if (self._tile_err.startswith('tile 2 unavailable')
                            or self._tile_err.startswith('tile2.* refused')):
                        self._tile_err = ''
                    self.log.info('tile 2 tracker rebuilt (tile2.* changed)')
                elif self._trk[2] is old and old is not None:
                    self._tile2_cfg_back(old.args)
        if self._tile == 2:
            self._apply_light(2)

    def _tile2_cfg_back(self, args):
        """The running Pipe's settings back into tile2.* (saved). The
        listener this fires sees them equal to the Pipe's: no rebuild."""
        try:
            self.app.config.patch({'tile2': dict(zip(t2mod.PIPE_KEYS, args))},
                                  persist=True)
        except Exception as e:
            self.log.error('tile2.* not put back: %s' % e)

    def _tracker_for(self, w, h):
        """The active tile's tracker, kept per tile. Tile 1's is made on the
        first frame whatever the tile, so a switch to it is instant."""
        if self._trk[1] is None:
            self._trk[1] = pipeline.GridTracker(w, h, self._pipe_cfg)
            self.log.info('grid tracker %dx%d, %.4f mm/px'
                          % (w, h, self._mm_per_px))
        if self._tile == 2:
            self._trk[2] = self._tile2_tracker(w, h)
        return self._trk[self._tile]

    def _tile2_tracker(self, w, h):
        """Tile 2's tracker for a w x h frame: the one made at setup, or a
        new one after a tile2.* change; DeadTracker when it cannot run
        (never Tile 1 instead: on a Tile 2 floor that steers on noise)."""
        trk = self._trk[2]
        if isinstance(trk, t2mod.DeadTracker):
            return trk
        try:
            if self._t2_why:
                raise RuntimeError(self._t2_why)
            if trk is None:
                raise RuntimeError('no tracker')
            if (w, h) != (trk.W, trk.H):
                raise ValueError('Tile 2 needs a %dx%d frame (sensor gives '
                                 '%dx%d)' % (trk.W, trk.H, w, h))
            self.log.info('tile 2 (UNIT16) tracker %dx%d' % (w, h))
            return trk
        except Exception as e:
            self._t2_why = str(e) or e.__class__.__name__
            self._tile_err = 'tile 2 unavailable: %s' % self._t2_why
            self.log.error('tile 2 cannot load (%s): reading nothing'
                           % self._t2_why)
            return t2mod.DeadTracker(w, h)

    def _t2_fail(self, e):
        """A Tile 2 error inside the Pipe step. Never leaves the loop, never
        touches Tile 1's kernels: counted, logged once, the Pipe starts
        clean, and the frame reads as nothing."""
        self._t2_note(e, 'frame')
        try:
            self._tracker.reset()
        except Exception:
            pass

    def _t2_note(self, e, where):
        """Count and log (once) a Tile 2 error. On its own (where =
        'quality') for a score error: the Pipe keeps its state, since the
        score never changes what navigation measures."""
        self._t2_errors += 1
        self._tile_err = 'tile 2 %s error: %s' % (where, str(e) or
                                                  e.__class__.__name__)
        if not self._t2_logged:
            self._t2_logged = True
            self.log.error(self._tile_err + ' (counted in tile2_errors)')

    def _set_kernels(self, on):
        """Apply vision.c_kernels: the scan/fit kernels as C (t1c) or viper.
        Only on a change, so a config write to another vision.* key does not
        undo the fallback after a C error."""
        on = bool(on)
        if on == self._c_kernels:
            return
        self._c_kernels = on
        got = pipeline.use_kernels(on)
        if got == 'c':
            self.log.info('kernels: C (t1c)')
        elif on:
            self.log.warn('kernels: viper, t1c refused: %s'
                          % pipeline.KERNELS_ERR)
        else:
            self.log.info('kernels: viper')

    def _set_qkernels(self, on):
        """Apply quality.c_kernels (the score's pixel loops as C or viper),
        only on a change, so other quality.* writes keep a C-error fallback."""
        on = bool(on)
        if on == getattr(self, '_q_c', None):
            return
        self._q_c = on
        got = qmod.use_kernels(on)
        if got == 'c':
            self.log.info('quality kernels: C (t1c)')
        elif on:
            self.log.warn('quality kernels: viper, t1c refused: %s'
                          % qmod.KERNELS_ERR)
        else:
            self.log.info('quality kernels: viper')

    def _ema(self, key, us):
        """Exponential moving average of a stage cost, in microseconds."""
        p = self._prof
        p[key] = 0.9 * p.get(key, us) + 0.1 * us

    def _mark(self, key, t0):
        """EMA the time since t0 into `key`; return a fresh stamp."""
        now = util.ticks_us()
        self._ema(key, util.ticks_diff(now, t0))
        return now

    def profile(self):
        """Per-stage CPU breakdown, or None when profiling is disabled.

        `cam`, `work` and `other` are disjoint and sum to the frame period:
          cam   — blocked inside snapshot() waiting on the sensor
          work  — this loop's own CPU (the sub-stages below break it down)
          other — every OTHER asyncio task (httpd, uart, mdns, health, ...)
        The remaining keys are sub-buckets of `work`, so they sum to `work`
        and NOT to 100%:
          otsu/analyse/quality/stream/post/gc  sum to `work`
          scan/fit                     sum to `analyse` (Tile 1 only)
        (`stream` covers the frame tap, overlay draw and JPEG compress — it
        is ~0 unless a viewer is connected. `otsu` is charged only on the
        frames that recompute it -- on Tile 2 only with a quality score;
        `gc` only on the frames that collect. A tile switch starts the
        buckets again, so none of them is the other tile's.)
        """
        p = self._prof
        period = self._period_us              # always measured, not profiled
        if not self._profile or period <= 0.0:
            return None
        out = {'period_us': round(period, 1),
               'fps': round(1000000.0 / period, 2)}
        other = period - p.get('cam', 0.0) - p.get('work', 0.0)
        for k in ('cam', 'work', 'otsu', 'analyse', 'scan', 'fit',
                  'quality', 'stream', 'post', 'gc'):
            if k in p:
                out[k] = {'us': round(p[k], 1),
                          'pct': round(100.0 * p[k] / period, 1)}
        out['other'] = {'us': round(other, 1),
                        'pct': round(100.0 * other / period, 1)}
        return out

    def _print_row(self, r):
        row = '%d,%s,%s,%s,%s,%d,%d,%s,%s,%.1f' % (
            r.frame_id, r.tier or '',
            _fmt(r.theta_deg), _fmt(r.x_mm), _fmt(r.y_mm),
            r.n_c, r.n_r, _fmt(r.mad_h), _fmt(r.mad_v),
            r.fps)
        if self._q_csv:
            row += ',%s,%s' % ('' if r.q is None else r.q, r.q_why)
        print(row)

    async def run(self):
        import asyncio
        import gc

        while True:
            await asyncio.sleep(0)            # yield before the blocking grab
            # Time the other tasks just held the loop: the end-of-frame
            # yield and the one above, back to back. No frame goes to the
            # STM32 while they run, so this is the camera-side view of an
            # STM32 link drop.
            gap_ms = 0
            if self._yield_ms is not None and self._stall is not None:
                now_ms = util.ticks_ms()
                gap_ms = util.ticks_diff(now_ms, self._yield_ms)
                self._stall.note_gap(self._yield_ms, now_ms)
            self._yield_ms = None
            if self.paused:                   # camera lent out (VGA apriltag)
                await asyncio.sleep(0.1)
                self.heartbeat()
                self._last_top = None         # pause is not a frame period
                continue
            if self._tile_want:               # a tile switch, between frames
                self._apply_tile()
            if self._cam_dirty:               # rare: user tuning via config
                # the ~1 s latch is held like a tile switch: never inside a
                # turn (Algorithm 2 would see one 1 s step), latched on the
                # first frame the turn machine is quiet again
                turn = self._turn()
                if turn is None or not turn.busy():
                    self._cam_dirty = False
                    self._latch_camera()

            # True frame period, measured top-of-loop to top-of-loop.
            #
            # NOT time.clock(): measured on fw v5.0, clock.fps() averages only
            # the tick()->fps() interval and silently drops everything after
            # the fps() call — for this loop that is post + gc + both awaits,
            # about 19% of the period, so it over-reported fps by ~23%.
            # One ticks_us() call per frame is cheaper than being wrong.
            t_top = util.ticks_us()
            if self._last_top is not None:
                dt = util.ticks_diff(t_top, self._last_top)
                if self._stall is not None:
                    self._stall.note_self(dt // 1000 - gap_ms)
                if 0 < dt < 2000000:          # ignore restarts/long stalls
                    self._period_us = (0.9 * self._period_us + 0.1 * dt
                                       if self._period_us else float(dt))
                    self._fps = 1000000.0 / self._period_us
            self._last_top = t_top

            # Profiling (off by default): sub-stage timers only — the period
            # above is always measured.
            prof = self._prof_acc if self._profile else None
            if prof is not None:
                prof.clear()

            try:
                img = self._camera.snapshot() # blocks ~1 frame period
            except RuntimeError as e:
                if not self._cam_timeout(e):
                    raise
                self.heartbeat()
                continue
            self._cam_fails = 0
            t_work = util.ticks_ms()          # camera wait is NOT cpu work
            if prof is not None:
                t_us = util.ticks_us()
                self._ema('cam', util.ticks_diff(t_us, t_top))
                self._prof_cam_end = t_us
            w, h = img.width(), img.height()
            if self._tracker is None:
                # W/H come from the sensor (320x200 here — never hardcode)
                self._tracker = self._tracker_for(w, h)
                self._th = 0
            trk = self._tracker
            tile2 = self._tile == 2

            cfg = self._pipe_cfg
            buf = img.bytearray()
            turn = self._turn()
            if turn is not None:
                turn.ensure(w, h)
                turn.poll(self._frame_i, util.ticks_ms())

            # THE GATE: while the robot is turning, Algorithm 1 is skipped
            # entirely. The grid sweeps past during a turn, so its fit would
            # be meaningless; Algorithm 2 measures the rotation instead.
            if turn is not None and not turn.alg1_active():
                if tile2:
                    # Tile 2: the frame's one Pipe.step (it tracks through
                    # the turn) gives the angle; no edges = skip the frame
                    try:
                        ang = trk.turn_step(buf)
                    except Exception as e:
                        self._t2_fail(e)
                        ang = None
                    if ang is not None:
                        turn.turn_frame(buf, util.ticks_ms(), ang)
                else:
                    turn.turn_frame(buf, util.ticks_ms())
                # Only a real turn reports. The rotation layer below runs the
                # same analysis while DRIVING and must stay silent, or the
                # link carries two frames per analysed frame.
                turn.report()
                res = self._turning_result(turn)
                if self._quality is not None:
                    self._quality.skip()      # no Algorithm 1 -> no score
                res['q'] = None
                if prof is not None:
                    t_us = self._mark('analyse', t_us)
            else:
                if turn is not None:
                    if turn.take_reseed():
                        # Tile 1: the band and last_X from before the turn
                        # describe a world that has rotated 90 degrees away.
                        # Tile 2: nothing, the Pipe tracked through it.
                        trk.reseed()
                        self._th = 0
                    if turn.force_full_scan():
                        trk.force_full()

                every = cfg['otsu_every']
                qs = self._quality
                q_bad = False
                if tile2:
                    # Tile 2's Pipe makes its own threshold: Otsu there only
                    # feeds the quality score (light and flicker), on the
                    # tracker's own rhythm, and not at all without a scorer.
                    # A score error never resets the Pipe (_t2_note).
                    if qs is not None:
                        try:
                            ran = trk.prepare_quality(img, self._tile_fi, qs,
                                                      Camera.otsu)
                        except Exception as e:
                            self._t2_note(e, 'quality')
                            ran = False
                            q_bad = True
                        self._th = trk.th
                        if prof is not None and ran:
                            t_us = self._mark('otsu', t_us)
                elif every <= 1 or self._th == 0 or self._tile_fi % every == 0:
                    # the tile's own frame count, not the loop's: after a
                    # switch the rhythm is a fresh run's on any frame
                    if qs is not None and self._tile_fi >= self._light_due:
                        # light read rides on the Otsu copy's histogram
                        self._th, bins = Camera.otsu(img, cfg['otsu_scale'],
                                                     True)
                        qs.light(bins, self._th)
                        self._light_due = self._tile_fi + qs.light_every
                    else:
                        self._th = Camera.otsu(img, cfg['otsu_scale'])
                    if qs is not None:
                        qs.otsu(self._th)
                    if prof is not None:
                        t_us = self._mark('otsu', t_us)

                t2_bad = False
                try:
                    res = trk.analyse(buf, self._th, self._mm_per_px, prof)
                except Exception as e:
                    if tile2:
                        # never out of the loop, never Tile 1's kernels
                        self._t2_fail(e)
                        res = t2mod.empty_result(w, h, self._th, 'tile2-error')
                        t2_bad = True
                    else:
                        if pipeline.KERNELS != 'c':
                            raise
                        # t1c checks every argument and raises where viper
                        # would not. Never lose the frame or the loop for
                        # it: back to viper for good (until vision.c_kernels
                        # is set again), fresh tracker state, same frame
                        # again.
                        pipeline.use_kernels(False)
                        pipeline.KERNELS_ERR = 'C kernel failed: %s' % e
                        self.log.error('C kernel failed (%s): now on viper'
                                       % e)
                        trk.reset()
                        res = trk.analyse(buf, self._th, self._mm_per_px,
                                          prof)
                self.points.add(res['tier'], res['status'], res['n_c'],
                                res['n_r'], self._pipe_cfg['min_pts'])
                self.points.note(res, self._frame_i)
                if prof is not None:
                    t_us = self._mark('analyse', t_us)

                # Quality score: reads the result and the tracker's own
                # buffers, never changes them.
                if qs is not None and tile2:
                    # Tile 2's scorer reads the Pipe's own numbers (no
                    # pixels, so no C kernel fallback); mm is tile2's
                    if t2_bad or q_bad:
                        qs.skip()             # no Pipe answer or light read
                    else:
                        try:
                            qs.frame_sig(trk.signals(), trk.mm)
                        except Exception as e:
                            self._t2_note(e, 'quality')
                            qs.skip()
                    res['q'], res['q_why'] = qs.q, qs.why
                    ev = qs.take_event()
                    if ev is not None:
                        if ev[0] == 'warn':
                            self.log.warn(ev[1])
                        else:
                            self.log.info(ev[1])
                    if prof is not None:
                        t_us = self._mark('quality', t_us)
                elif qs is not None:
                    try:
                        qs.frame(res, self._tracker, self._mm_per_px, buf)
                    except Exception as e:
                        if qmod.KERNELS != 'c':
                            raise
                        # same rule as the scan kernels: never lose the
                        # frame for a C error -- viper for good, score again
                        qmod.use_kernels(False)
                        qmod.KERNELS_ERR = 'C kernel failed: %s' % e
                        self.log.error('quality C kernel failed (%s): now '
                                       'on viper' % e)
                        qs.frame(res, self._tracker, self._mm_per_px, buf)
                    res['q'], res['q_why'] = qs.q, qs.why
                    ev = qs.take_event()
                    if ev is not None:
                        if ev[0] == 'warn':
                            self.log.warn(ev[1])
                        else:
                            self.log.info(ev[1])
                    if prof is not None:
                        t_us = self._mark('quality', t_us)

                if self._fix_hold:
                    # after a tile switch: no x/y until the new tile's
                    # first good junction (the STM32 may still be counting)
                    if res['tier'] == 'junction' and not res['suspect']:
                        self._fix_hold = False
                    else:
                        res['x_mm'] = None
                        res['y_mm'] = None

                if turn is not None:
                    # Algorithm 1's answer feeds the machine: it is what
                    # detects landing on a junction and what cross-checks the
                    # angle after a turn. Only a trusted tier-1 fix counts.
                    fix = None
                    if res['tier'] == 'junction' and not res['suspect']:
                        fix = (1, res['x_mm'], res['y_mm'], res['theta_deg'])
                    now = util.ticks_ms()
                    turn.drive_result(fix, now)
                    # No rotation layer here any more. Upstream drop v2
                    # (2026-09-11) deleted it: Algorithm 2 runs only while
                    # the host has commanded a turn, never alongside drive.
                    # turn_frame() is called from the TURN branch above.

            tapped = False
            if self._tap is not None:
                tap, self._tap = self._tap, None
                tapped = True
                try:
                    tap(img, w, h)
                except Exception as e:
                    self.log.error('frame tap failed: %s' % e)

            # a tap may consume/compress the framebuffer (apriltag does) —
            # the frame is no longer drawable, skip overlay/stream this once
            #
            # The overlay is drawn ONLY on frames we are about to encode.
            # It used to run on every frame while streaming, so with the
            # default divisor of 2 half the annotation work was drawn and
            # then thrown away (859 us/frame, measured).
            if (not tapped and self.stream_clients > 0
                    and self._frame_i % self._jpeg_divisor == 0):
                try:
                    # Downscale FIRST, then annotate. JPEG cost goes with
                    # pixel count, and a smaller frame is also less for the
                    # WiFi task to push. Drawing before the shrink made the
                    # fixed-size font unreadable at scale 0.5 — and the
                    # readout is the point of the overlay. copy() rather
                    # than an in-place pool: this firmware freezes hard on
                    # some framebuffer geometry changes, no watchdog.
                    s = self._jpeg_scale
                    src = img if s == 1.0 else img.copy(x_scale=s, y_scale=s)
                    if self._ov_lines or self._ov_text:
                        overlay.draw(src, res, w, h, s,
                                     self._ov_lines, self._ov_text)
                    # bytes() COPY is required: .bytearray() is a view
                    # into the camera framebuffer, which the next
                    # snapshot() overwrites before the async stream
                    # handler gets to send it
                    self._set_jpeg(bytes(src.compress(
                        quality=self._jpeg_quality).bytearray()),
                        frame_meta(self._frame_i, util.ticks_ms(), res,
                                   self._fps, self._tile))
                except Exception:
                    self.last_jpeg = None
            if prof is not None:
                t_us = self._mark('stream', t_us)

            # Diagnostic: keep the latest frame Algorithm 1 gave NO result
            # for (GET /shot?reject=1), at most once a second. Compressed in
            # place AFTER every other use of img this frame, and only with no
            # stream client, so it can never touch a frame the stream sends.
            if (self.stream_clients == 0 and res is not None
                    and res.get('tier') is None and 'why' in res
                    and util.ticks_diff(util.ticks_ms(), self._reject_ms) > 1000):
                self._reject_ms = util.ticks_ms()
                try:
                    self.reject_jpeg = bytes(img.compress(
                        quality=self._jpeg_quality).bytearray())
                except Exception:
                    pass

            r = self._fill_result(res)
            if self._print_csv:
                self._print_row(r)
            self.app.bus.publish('frame', r)

            self._frame_i += 1
            self._tile_fi += 1
            self.heartbeat()
            if prof is not None:
                t_us = self._mark('post', t_us)

            # gc every N frames, not every frame: a full collect is
            # milliseconds on MicroPython and the loop allocates almost
            # nothing per iteration (the tracker's arrays are preallocated).
            if self._gc_every <= 1 or self._frame_i % self._gc_every == 0:
                gc.collect()
                if prof is not None:
                    t_us = self._mark('gc', t_us)
            busy = util.ticks_diff(util.ticks_ms(), t_work)
            self._busy_ms = 0.9 * self._busy_ms + 0.1 * busy
            if prof is not None:
                # pipeline stages were accumulated into prof by analyse()
                for k in prof:
                    self._ema(k, prof[k])
                self._ema('work', util.ticks_diff(util.ticks_us(),
                                                  self._prof_cam_end))
            self._yield_ms = util.ticks_ms()
            await asyncio.sleep(0)            # let network tasks run

    def stats(self):
        # cpu estimate: work time per frame x frames per second
        cpu = min(100.0, self._busy_ms * self._fps / 10.0)
        prof = self.profile()
        r = self._result
        t = self._tracker
        return {'frames': self._frame_i, 'fps': self._fps,
                'busy_ms': self._busy_ms, 'cpu_pct': cpu,
                'profile': prof,
                'tile': self._tile,
                'tile_cfg': self.app.config.get('vision.tile', 1),
                'tile_err': self._tile_err,
                'tile_pending': self._tile_want or None,
                'tile_switches': self._tile_switches,
                'tile_switch_ms': self._tile_switch_ms,
                'tile2_errors': self._t2_errors,
                'cam_timeouts': self._cam_timeouts,
                'kernels': pipeline.KERNELS,
                'kernels_err': pipeline.KERNELS_ERR,
                'quality_kernels': qmod.KERNELS,
                'quality_kernels_err': qmod.KERNELS_ERR,
                'camera': self._camera.readback(),
                'frames_rejected': self._frames_rejected,
                'reject_pct': self._reject_ema * 100.0,
                'no_scene': self._no_scene,
                'tier': r.tier, 'x_mm': r.x_mm, 'y_mm': r.y_mm,
                'theta_deg': r.theta_deg,
                'junction_code': r.junction_code,
                'tile2': (t.debug() if t is not None and hasattr(t, 'debug')
                          else None),
                'x_rejected': t.n_rejected if t else 0,
                'track': t.status if t else '',
                'threshold': self._th,
                'quality': (self._quality.stats()
                            if self._quality is not None else None),
                'stream_clients': self.stream_clients,
                'points': self.points.as_dict(
                    {'track': [t.NC, t.NR], 'full': [t.NAC, t.NAR]}
                    if t else None)}

    def healthy(self):
        return True
