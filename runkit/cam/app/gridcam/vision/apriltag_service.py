# On-demand AprilTag detection (TAG36H11), organized from the manual test
# script reference/apriltag_ae3_led_VGA_AUTO_GAIN.py.
#
# Detection is on demand only (POST /apriltag/detect or the dashboard
# button) — it never runs in the per-frame hot path. Two capture modes
# (config 'apriltag.capture'):
#
#   'current' (default) — taps the next live frame from the vision loop
#       at the navigation resolution (QVGA). Zero interruption; tags must
#       be close enough to resolve.
#   'vga' (experimental) — pauses the vision loop, switches the camera to
#       VGA for range (as the reference script ran), detects, restores.
#       Costs ~0.5-2 s of navigation blindness per call; the camera is
#       always restored via try/finally.
#
# Preprocessing ('apriltag.preprocess'): 'none' (default) detects on the frame
# exactly as captured; 'enhance' first applies the reference script's adaptive
# histogram equalisation + mild unsharp mask, for embossed/low-contrast tags.
# The boxes and the JPEG for the web page are made AFTER detection, so they
# never change what the detector saw.

from .. import util
from ..service import Service


class AprilTagService(Service):
    name = "apriltag"

    def setup(self):
        cfg = self.app.config.section('apriltag')
        self._cfg = cfg
        self.last_tags = []
        self.last_jpeg = None
        self.last_ts_ms = None
        self._detections = 0
        self._busy = False
        self.last_preprocess = None

    # -- shared detection core ------------------------------------------
    def _detect_on(self, img, w, h):
        import image as image_mod
        import math

        cfg = self._cfg
        mode = preprocess_mode(cfg)
        if mode == 'enhance':
            img.histeq(adaptive=True, clip_limit=cfg['histeq_clip'])
            img.gaussian(1, unsharp=True, mul=cfg['unsharp_mul'])
        self.last_preprocess = mode

        tags = img.find_apriltags(
            families=getattr(image_mod, cfg['family']))

        out = []
        for tag in tags:
            x, y, tw, th = tag.rect
            img.draw_rectangle(tag.rect, color=255)
            img.draw_cross((tag.cx, tag.cy), color=255)
            out.append({
                'id': tag.id,
                'cx': tag.cx, 'cy': tag.cy,
                'rect': [x, y, tw, th],
                'rotation_deg': math.degrees(tag.rotation),
                'decision_margin': tag.decision_margin,
            })

        for i, t in enumerate(out):
            img.draw_string((2, 2 + 12 * i),
                            'ID:%d a:%.0f m:%.1f'
                            % (t['id'], t['rotation_deg'],
                               t['decision_margin']),
                            color=255)

        # bytes() COPY — .bytearray() aliases the camera framebuffer
        quality = self.app.config.get('http.mjpeg_quality', 50)
        self.last_jpeg = bytes(img.compress(quality=quality).bytearray())
        self.last_tags = out
        self.last_ts_ms = util.ticks_ms()
        self._detections += 1
        return out

    # -- capture strategies ---------------------------------------------
    async def _detect_current(self):
        """Tap the next live frame from the vision loop."""
        import asyncio
        done = asyncio.Event()
        result = {}

        def tap(img, w, h):
            try:
                result['tags'] = self._detect_on(img, w, h)
            except Exception as e:
                result['error'] = str(e)
            done.set()

        self.app.vision.tap_frame(tap)
        try:
            await asyncio.wait_for(done.wait(), self._cfg['timeout_s'])
        except asyncio.TimeoutError:
            return {'error': 'timeout waiting for a frame'}
        return result

    async def _detect_vga(self):
        """Temporary VGA switch + centre crop = 2x digital zoom.

        Measured on fw v5.0: find_apriltags OVERFLOWS its frame buffer on
        a full VGA (640x400) image, and HVGA hard-faults the sensor
        driver. So: capture at VGA, crop the central 320x200 — same pixel
        count the detector handles at 20 ms, but each far-away tag spans
        2x the pixels (half the field of view). Tag coordinates are
        relative to the crop.
        """
        import asyncio
        vision = self.app.vision
        camera = vision.camera()
        vision.paused = True
        await asyncio.sleep(0.2)          # let the loop park
        result = {}
        try:
            camera.set_framesize('VGA')
            img = camera.snapshot()
            w, h = img.width(), img.height()
            img.crop(roi=(w // 4, h // 4, w // 2, h // 2))
            result['tags'] = self._detect_on(img, img.width(), img.height())
        except Exception as e:
            result['error'] = str(e)
        finally:
            try:
                camera.set_framesize(
                    self.app.config.get('camera.framesize', 'QVGA'))
            except Exception as e:
                self.log.error('framesize restore failed: %s' % e)
            vision.paused = False
        return result

    # -- public API ------------------------------------------------------
    async def detect(self):
        """Run one detection; returns {'tags': [...]} or {'error': ...}."""
        if self._busy:
            return {'error': 'detection already in progress'}
        self._busy = True
        t0 = util.ticks_ms()
        # read live so a runtime config change takes effect without reboot
        self._cfg = self.app.config.section('apriltag')
        try:
            if self._cfg['capture'] == 'vga':
                # DISABLED on fw v5.0: reconfiguring csi framesize on a
                # live pipeline hard-freezes the whole VM (verified
                # 2026-08-05; no hardware WDT exists to recover). The
                # implementation is kept for future firmware — opt in
                # with capture='vga_unsafe' only on a bench setup.
                result = {'error': "capture='vga' is disabled: csi "
                                   "framesize switch freezes fw v5.0 "
                                   "(see CLAUDE.md); use 'current'"}
            elif self._cfg['capture'] == 'vga_unsafe':
                result = await self._detect_vga()
            else:
                result = await self._detect_current()
        finally:
            self._busy = False
        result['elapsed_ms'] = util.ticks_diff(util.ticks_ms(), t0)
        result['preprocess'] = preprocess_mode(self._cfg)
        if 'tags' in result:
            self.log.info('detected %d tag(s) in %d ms'
                          % (len(result['tags']), result['elapsed_ms']))
        else:
            self.log.warn('detection failed: %s' % result.get('error'))
        return result

    def last_result(self):
        age = (util.ticks_diff(util.ticks_ms(), self.last_ts_ms) / 1000.0
               if self.last_ts_ms is not None else None)
        return {'tags': self.last_tags, 'age_s': age,
                'capture': self._cfg['capture'],
                'preprocess': self.last_preprocess}

    def stats(self):
        return {'detections': self._detections,
                'last_tags': len(self.last_tags)}


def preprocess_mode(cfg):
    """'enhance' only when asked for exactly; anything else is the raw frame,
    so a typo in config.json can never start modifying frames."""
    return 'enhance' if cfg.get('preprocess') == 'enhance' else 'none'
