# Camera wrapper: firmware v5 `csi` module with legacy `sensor` fallback.
#
# Exposure and gain are locked from config (auto modes off) so detection
# thresholds stay stable frame to frame.
#
# camera.gray (1.31.0): "firmware" = the firmware makes the gray picture
# inside snapshot() (~2.35 ms a frame on the AE3); "green" = the camera runs
# RAW (Bayer) and rawgray.green() makes the gray picture from the green
# pixels (~1.1 ms). snapshot() then returns a GrayFrame instead of the
# camera's image; see GrayFrame below.

from array import array

from ulab import numpy as np


def _import_rawgray():
    """The rawgray .mpy: next to this file in the app slot, else top-level."""
    try:
        from . import rawgray
    except ImportError:
        import rawgray
    return rawgray


RAWGRAY_API = 1                         # rawgray.API this file expects


def gamma_lut():
    """The firmware's gamma table, rebuilt (ports/alif isp_reset:
    imlib_update_gamma_table(-0.2, 1.0, 2.2)): green gray gets the same
    brightness curve as the firmware's gray."""
    lut = bytearray(256)
    for i in range(256):
        v = ((i / 255.0) ** (1.0 / 2.2) * 1.0 - 0.2) * 255.0
        v = int(v + 0.5) if v >= 0 else 0
        lut[i] = 255 if v > 255 else v
    return lut


class GrayFrame:
    """The gray picture of camera.gray = "green", in a plain buffer.

    The vision loop only needs width(), height() and bytearray(), and Otsu
    (Camera.otsu sees a GrayFrame and uses rawgray). Anything else (the
    stream's copy/compress, the overlay, AprilTag) goes to a real gray
    image.Image built on the same buffer -- only when it is used, once per
    frame (an in-place op like crop() or compress() changes that image, so
    the next frame gets a fresh one)."""

    def __init__(self, rg):
        self.rg = rg
        self.buf = None
        self.w = 0
        self.h = 0
        self.hist = array('i', [0] * 256)
        self._img = None

    def set(self, buf, w, h):
        self.buf = buf
        self.w = w
        self.h = h
        self._img = None

    def width(self):
        return self.w

    def height(self):
        return self.h

    def bytearray(self):
        return self.buf

    def image(self):
        if self._img is None:
            import image
            fmt = getattr(image, 'GRAYSCALE', None)
            if fmt is None:
                import csi
                fmt = csi.GRAYSCALE
            self._img = image.Image(self.w, self.h, fmt, buffer=self.buf)
        return self._img

    def __getattr__(self, name):
        return getattr(self.image(), name)


class Camera:
    """Owns the image sensor; produces frames and their ndarray views."""

    def __init__(self, config, log):
        self._config = config
        self._cfg = config.section('camera')
        self._log = log
        self._cam = None
        self._legacy = False
        self._awb_on = True             # firmware default after reset()
        self._green = False             # camera.gray == "green" and usable
        self._rg = None
        self._lut = None
        self._frame = None
        self._gbuf = None
        # a tile's own light (vision_service, tile2.exposure_us/gain_db):
        # None = camera.* from config
        self._exp_tile = None
        self._gain_tile = None
        self._latched = None            # (exposure_us, gain_db) last latched

    def set_light(self, exposure_us=None, gain_db=None):
        """Use this exposure/gain instead of camera.*, None = camera.*.
        Takes effect at the next apply_settings(). Returns True when that
        changes what is latched now (the caller then latches, ~1 s)."""
        self._exp_tile = exposure_us
        self._gain_tile = gain_db
        return self._wanted() != self._latched

    def _wanted(self):
        c = self._config.section('camera')
        e = self._exp_tile if self._exp_tile is not None else c['exposure_us']
        g = self._gain_tile if self._gain_tile is not None else c['gain_db']
        return e, g

    def _set_gray(self):
        """camera.gray, boot only: "firmware" (default) or "green". Green
        needs rawgray.mpy (API 1); without it the camera stays on firmware
        gray and says why."""
        want = self._cfg.get('gray', 'firmware')
        self._green = False
        if want == 'green':
            try:
                rg = _import_rawgray()
                api = getattr(rg, 'API', None)
                if api != RAWGRAY_API:
                    raise ValueError('rawgray API %s, need %d'
                                     % (api, RAWGRAY_API))
                self._rg = rg
                self._lut = gamma_lut()
                self._frame = GrayFrame(rg)
                self._green = True
            except Exception as e:
                self._log.warn('camera.gray green refused (%s): firmware '
                               'gray' % (str(e) or e.__class__.__name__))
        elif want != 'firmware':
            self._log.warn('camera.gray %r unknown: firmware gray' % (want,))
        self._log.info('gray: %s' % ('green (raw + rawgray)' if self._green
                                     else 'firmware'))

    def setup(self):
        try:
            import csi
            cam = csi.CSI()
            self._cam = cam
            self._set_gray()
            self._init_csi(cam)
            self._log.info('csi camera: exposure_us=%d gain_db=%.1f'
                           % (cam.exposure_us(), cam.gain_db()))
        except ImportError:
            import sensor
            sensor.reset()
            sensor.set_pixformat(sensor.GRAYSCALE)
            sensor.set_framesize(getattr(sensor, self._cfg['framesize']))
            sensor.set_auto_exposure(False, exposure_us=exposure_us)
            sensor.set_auto_gain(False, gain_db=gain_db)
            sensor.skip_frames(time=200)
            self._legacy = True
            self._log.info('sensor camera: exposure_us=%d gain_db=%.1f'
                           % (sensor.get_exposure_us(), sensor.get_gain_db()))

    def _init_csi(self, cam):
        """Bring the sensor up, retrying a transient capture timeout.

        WHY: 'RuntimeError: Frame capture has timed out' during the settle
        snapshot has been seen at boot on this hardware (crash dumps on
        2026-08-05 and 2026-08-08, different builds). Vision is a critical
        service, so letting that escape means app.run() raises, main.py
        writes a boot dump and calls machine.reset() — a full board reboot,
        and machine.reset() is itself known to wedge USB on this board. A
        transient sensor hiccup must not cost a reboot.

        Each attempt starts from a fresh reset(): framesize() on an already
        live pipeline can hard-freeze the VM, and reset() is what makes it
        safe to call again.
        """
        import csi
        import time
        attempts = self._cfg.get('init_retries', 3)
        delay_ms = self._cfg.get('init_retry_ms', 500)
        for i in range(attempts):
            try:
                cam.reset()
                self._awb_on = True     # reset() restores auto white balance
                cam.pixformat(csi.BAYER if self._green else csi.GRAYSCALE)
                cam.framesize(getattr(csi, self._cfg['framesize']))
                self._set_framebuffers()
                self.apply_settings()
                if i:
                    self._log.warn('camera came up on attempt %d/%d'
                                   % (i + 1, attempts))
                return
            except RuntimeError as e:
                if i == attempts - 1:
                    self._log.error('camera init failed after %d attempts: %s'
                                    % (attempts, e))
                    raise
                self._log.warn('camera init attempt %d/%d failed (%s) — '
                               'resetting and retrying in %d ms'
                               % (i + 1, attempts, e, delay_ms))
                time.sleep_ms(delay_ms)

    def _set_framebuffers(self):
        """camera.framebuffers: capture the next frame while this one is
        processed. Boot only (and after a framesize switch, which resets it):
        never called from apply_settings(), so a camera.* change on a running
        camera does not touch it -- it takes effect at the next restart.

        1 = capture on demand, snapshot() waits for it (8.0 ms in all on 0302).
        2 = the capture keeps running; snapshot() waits for the frame in
            progress (a frame that ends while we are busy is dropped).
        3 = the capture keeps running; snapshot() gets the newest finished
            frame. Each buffer costs 128 KB (the firmware sizes it for RGB565).
        A buffer is never handed out twice: snapshot() returns the previous
        one to the free list, and it comes back only with a new picture."""
        n = self._cfg.get('framebuffers', 0)
        self._fb_cfg = n
        if n not in (1, 2, 3):
            if n:
                self._log.warn('framebuffers %r ignored: use 1, 2 or 3' % (n,))
            return
        try:
            self._cam.framebuffers(n)
        except Exception as e:
            self._log.warn('framebuffers(%d) refused (%s): firmware default'
                           % (n, e))
            return
        self._log.info('framebuffers %d' % n)

    def apply_settings(self):
        """(Re-)latch manual exposure/gain — needed after reset() AND after
        every framesize change.

        IMPORTANT (measured on AE3 fw v5.0): each manual setting needs
        frames to latch — one settle frame first, and two frames after
        auto_exposure BEFORE touching auto_gain, or the exposure value is
        silently discarded (back-to-back calls lose it).

        Also the ONLY reliable way to change settings at runtime: a bare
        auto_exposure() on a live pipeline is silently dropped (measured —
        50 ms/24 dB requests left the image unchanged), so the vision loop
        calls this on camera.* config changes.
        """
        self._cfg = self._config.section('camera')   # re-read live values
        cam = self._cam
        fr = self._cfg.get('framerate', 0)
        if fr:
            # ACCEPTED BUT NOT HONOURED on this sensor (POC measurement: the
            # driver reports 240 back, the applied exposure says 120 fps).
            # Kept because it is harmless and may bite on a later firmware;
            # a failure must not cost the latch sequence below.
            try:
                cam.framerate(fr)
            except Exception as e:
                self._log.warn('framerate(%d) failed (%s)' % (fr, e))
        exposure_us, gain_db = self._wanted()   # camera.*, or the tile's own
        cam.snapshot(time=500)
        cam.auto_exposure(False, exposure_us=exposure_us)
        cam.snapshot()
        cam.snapshot()
        cam.auto_gain(False, gain_db=gain_db)  # min ~3.15 dB
        cam.snapshot()
        cam.snapshot()
        self._latched = (exposure_us, gain_db)
        self._set_awb()

    def _set_awb(self):
        """camera.auto_whitebal. The PAG7936 is raw (Bayer): every snapshot()
        converts to grayscale on the CPU, and with auto white balance on it
        first reads the sensor's RGB statistics over I2C, every frame.
        false = auto_whitebal(False) with NO gains: the read stops and the
        gains it last learned stay in use (a flag, no sensor reconfigure).
        Called only on a change, so the default leaves the camera exactly as
        the firmware brings it up."""
        want = bool(self._cfg.get('auto_whitebal', True))
        if want == self._awb_on:
            return
        try:
            self._cam.auto_whitebal(want)
        except Exception as e:
            self._log.warn('auto_whitebal(%s) refused (%s)' % (want, e))
            return
        self._awb_on = want
        self._log.info('auto_whitebal %s' % ('on' if want else
                                             'off (gains frozen)'))

    def set_framesize(self, name):
        """Temporary resolution switch (e.g. VGA for AprilTag range).
        Caller must own the camera (vision loop paused) and is responsible
        for switching back. Re-latches exposure/gain."""
        if self._legacy:
            import sensor
            sensor.set_framesize(getattr(sensor, name))
            sensor.skip_frames(time=200)
            return
        import csi
        self._cam.framesize(getattr(csi, name))
        self._set_framebuffers()            # framesize() resets the count
        self.apply_settings()

    def snapshot(self):
        if self._legacy:
            import sensor
            return sensor.snapshot()
        img = self._cam.snapshot()
        if not self._green:
            return img
        # camera.gray = "green": the frame is RAW; make the gray picture
        w, h = img.width(), img.height()
        if self._gbuf is None or len(self._gbuf) != w * h:
            self._gbuf = bytearray(w * h)     # once (again after a resize)
        self._rg.green(img.bytearray(), self._gbuf, w, h, self._lut)
        self._frame.set(self._gbuf, w, h)
        return self._frame

    @staticmethod
    def otsu(img, scale, bins=False):
        """Otsu threshold of a downscaled copy of the frame.

        The one place the algorithm touches the image API. The POC measured
        the threshold on a 0.25-scale copy within 1 grey level of the full
        frame at 5x less cost (reference/pipeline_v2-2_method.pdf).

        bins=True also returns the copy's 256-bin histogram (fractions), for
        the light part of the quality score: (th, bins). Same histogram the
        threshold came from, so no second copy is made.
        """
        if isinstance(img, GrayFrame):
            # camera.gray = "green": the same 0.25-scale sample, in C
            step = max(1, int(1.0 / scale + 0.5))
            rg, buf, w, h = img.rg, img.buf, img.w, img.h
            th = int(rg.otsu(buf, w, h, step))
            if not bins:
                return th
            n = int(rg.hist(buf, w, h, step, img.hist))
            return th, [v / n for v in img.hist] if n else [0.0] * 256
        sm = img.copy(x_scale=scale, y_scale=scale)
        hist = sm.get_histogram()
        v = hist.get_threshold().value
        th = int(v() if callable(v) else v)  # attribute on fw 5.0 (measured)
        b = hist.bins() if bins else None
        del sm
        if bins:
            return th, b
        return th

    @staticmethod
    def to_ndarray(img, w, h):
        """Grayscale frame as a 2D uint8 ulab array."""
        try:
            return img.to_ndarray(dtype='B')
        except AttributeError:
            return np.frombuffer(img.bytearray(),
                                 dtype=np.uint8).reshape((h, w))

    def readback(self):
        """Actual sensor values (telemetry) — proves settings latched."""
        try:
            if self._legacy:
                import sensor
                return {'exposure_us': sensor.get_exposure_us(),
                        'gain_db': sensor.get_gain_db()}
            rb = {'exposure_us': self._cam.exposure_us(),
                  'gain_db': self._cam.gain_db()}
            try:
                rb['framebuffers'] = self._cam.framebuffers()
            except Exception:
                pass
            try:                    # what the sensor driver runs at
                rb['framerate'] = self._cam.framerate()
            except Exception:
                pass
            rb['auto_whitebal'] = self._awb_on
            rb['gray'] = 'green' if self._green else 'firmware'
            try:
                # not in "green": the raw camera keeps no colour stats, the
                # driver then reports -inf, and -inf is not valid JSON
                # (/telemetry broke on the AE3, 2026-11-17)
                if not self._green:
                    g = [round(v, 2) for v in self._cam.rgb_gain_db()]
                    if all(-1e9 < v < 1e9 for v in g):
                        rb['rgb_gain_db'] = g
            except Exception:
                pass
            # what the config asks for now; differs from 'framebuffers'
            # until the next restart
            rb['framebuffers_cfg'] = self._config.section('camera').get(
                'framebuffers', 0)
            return rb
        except Exception:
            return {}
