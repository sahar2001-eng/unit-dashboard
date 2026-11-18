# Configuration — every tunable in the application lives in DEFAULTS.
#
# ConfigService overlays /flash/etc/config.json (deep merge) on top of
# DEFAULTS and lets other services read values by dotted path and register
# change listeners. Nothing outside this module hardcodes a tunable.

import json

from .service import Service

CONFIG_PATH = "/flash/etc/config.json"

# The 'pipeline' block is the POC's tunables (reference/scan_fit_fps_live.py,
# upper-case constants) lower-cased: tests/desktop/test_pipeline_sync.py pins
# every value to the POC, so a retune from upstream cannot reach the algorithm
# without also reaching this table. Structural constants (MAXPTS, WMAX, ...)
# live in vision/pipeline.py and are not tunable — see its header.
DEFAULTS = {
    'app': {
        'name': 'gridcam',
    },
    'pipeline': {
        # scan geometry
        'step': 5,              # column/row spacing while tracking
        'acquire_step': 2,      # spacing for the full re-acquire scan
        'min_len': 4,           # shorter runs are noise
        'long_cut': 45,         # longer runs are the OTHER bar (band finder)
        'extra': 320,           # px added either side of the tracked band.
                                # 320 = the frame width, i.e. NO window: the
                                # row scan covers the whole frame every frame.
                                # 8 assumed the vertical bar moves < 8 px per
                                # frame, true only below ~0.17 m/s at ~86 fps
                                # (0.244 mm/px). Above that the bar left the
                                # window, its runs were cut at the edge and X
                                # dropped to 2 frames per junction (robot 0302,
                                # 2026-11-01: 7 of 20 junctions inferred at
                                # 0.7 m/s, 0 with 320). Costs ~5 fps.
        'reacquire_every': 30,  # full-width rescan cadence while locked
        'reacquire_mini_every': 12,  # ... while in mini-junction mode
        'otsu_scale': 0.25,     # Otsu on a quarter-scale copy
        'otsu_every': 3,        # recompute the threshold every N frames
        # fit
        'iters': 3,             # MAD rejection rounds
        'reject': 2.5,          # MAD sigmas
        'wsigma': 30,           # Gaussian width of the junction-local fit, px
        'thick_lo': 0.60,       # accepted run length, x tape thickness
        'thick_hi': 1.35,
        'nominal_thick_px': 29, # real tape width. If the run-length histogram
                                # peak falls outside [n/2, n*2] it is replaced
                                # by this: hist_peak can lock onto a serrated
                                # edge's teeth, and thick_lo/hi would then
                                # filter FOR the teeth.
        'min_pts': 8,
        'coverage_frac': 0.25,  # points needed as a fraction of scan lines
        'max_joint_mad_px': 3.0,  # full-junction fit residual gate
        'max_fit_mad_px': 5.0,    # single-bar "is there a bar?" gate
        # junction validity
        'min_v_span_px': 6,     # a real vertical bar spans several columns
        'thick_match_frac': 0.55,  # same tape: widths must be comparable
        # X: signed against the nearer junction, None in the dead zone
        'junction_spacing_mm': 80.0,
        'x_range_mm': 30.0,     # valid band either side of a junction; beyond
                                # it X is None ("between junctions")
        'x_direction': 1,       # -1 for a camera mounted 180 degrees round,
                                # which swaps leaving/approaching
        'max_step_mm': 30.0,    # physically impossible frame-to-frame jump
    },
    'camera': {
        'exposure_us': 1000,    # 1.30.0 (Sahar, 2026-11-17): 1000 is the
                                # default again. Before: 1500, upstream's v1_2
                                # value. It raised this from
                                # 1000 with the comment "WORKS NOW -- fixed by
                                # the Sept development firmware", which is the
                                # same PAG7936 exposure/gain fix we confirmed
                                # on 5.0.1 (see openmv/CLAUDE.md). At 1000 the
                                # frame was mean 6-10 of 255; light the scene
                                # with the white LEDs before raising it again,
                                # and note 'framerate' caps it at one frame
                                # period (4080 us at 240 fps).
        'gain_db': 24.0,        # the only real brightness control (POC:
                                # mean 42 @12 dB, 71 @18, 106 @24)
        'framerate': 240,       # accepted, not honoured (runs 120); 0 = skip
        'framesize': 'QVGA',    # 320x200 on this sensor
        'pixels_per_80mm': 327.9,   # = 80 / 0.244 mm/px. Upstream drop v2
                                # (2026-09-11) re-measured MM_PER_PIXEL as
                                # 0.244 ("71.5 mm spanned 293 px, twice"),
                                # replacing the 0.223 this was set from — so
                                # every mm we report moved by ~9%. NOT yet
                                # re-measured on OUR lens and mount height.
                                # recalibrate: raw jx at two junctions,
                                # 80 / |jx2 - jx1|. Open issue: one constant
        # 'Frame capture has timed out' during the settle snapshot has been
        # seen at boot. Vision is critical, so an escape would reboot the
        # whole board (main.py -> machine.reset(), which can wedge USB).
        # Retry from a fresh reset() instead.
        # Frame buffers (vision/camera.py _set_framebuffers): 1 = capture on
        # demand, snapshot() waits for the frame; 2 = capture keeps running, wait
        # for the frame in progress; 3 = capture keeps running, get the
        # newest finished frame (+128 KB each). 0 = firmware default.
        # Boot only: a change applies at the next camera restart.
        # Measured on 0302 (1.28.0): 1 = 76 fps, 2 = 105, 3 = 124 (the
        # firmware's own default before this setting existed).
        'framebuffers': 3,
        # Auto white balance (vision/camera.py _set_awb). On (the firmware
        # default) reads the sensor's RGB stats over I2C every frame before
        # the CPU converts the Bayer frame to gray. false = stop that and keep
        # the gains last learned. Live; /telemetry camera.rgb_gain_db.
        # 1.30.0: false by default (Sahar, 2026-11-17): the picture is gray,
        # and it saves ~0.5 ms a frame (blank AE3: cam 3.08 -> 2.59 ms).
        'auto_whitebal': False,
        # Who makes the gray picture (vision/camera.py), BOOT ONLY:
        # "firmware" = the firmware, inside snapshot() (~2.35 ms a frame).
        # "green"    = camera in RAW, rawgray.mpy makes it from the green
        #              pixels (~1.1 ms). Blank AE3 live loop: 189.6 -> 232.5
        #              fps; through the tracker X/Y within 0.05 mm of the
        #              firmware gray on average (2026-11-17). Not yet tested
        #              on a robot, so not the default. auto_whitebal does
        #              nothing in "green" (red/blue are not used).
        'gray': 'firmware',
        'init_retries': 3,
        'init_retry_ms': 500,
    },
    'vision': {
        'print_csv': True,      # CSV row per frame on serial
        'no_scene_after_s': 10, # no usable frame this long -> no_scene flag
        'gc_every': 30,         # gc.collect() every N frames: a full collect
                                # is milliseconds and the loop allocates
                                # almost nothing per frame (POC finding)
        'profile': False,       # per-stage timing breakdown in /telemetry.
                                # Costs a few ticks_us() calls per frame (well
                                # under 1% CPU) but is off by default; flip it
                                # live with POST /config {"vision":
                                # {"profile":true}} — no reboot needed.
        'tile': 1,              # floor: 1 = tape grid, 2 = UNIT16 hole grid
                                # (vision/tile2.py, the bench's Tile 2 code).
                                # Live (1.32.0): a change switches between
                                # frames, held while a turn runs; the camera
                                # boots on the saved tile (also after a
                                # vision crash on Tile 2). Switch only
                                # with the robot stopped: after a switch no
                                # x/y goes out until the first good junction.
        'c_kernels': True,      # scan/fit kernels in C (t1c.mpy from
                                # openmv/bench/native, same results as the
                                # viper ones). false = viper. A missing or
                                # wrong t1c, or a C error, falls back to
                                # viper by itself; /telemetry vision.kernels
                                # says which runs. Live, no reboot needed.
    },
    # Tile 2 (UNIT16) numbers, passed to the bench's u16pipe.Pipe. ALL are
    # the bench's PLACEHOLDERS (openmv/bench main.py T2_*) until measured on
    # the real floor -- see the Tile 2 plan, "To measure". Live: a change
    # makes a new Pipe (its tracking starts again).
    'tile2': {
        'mm_per_px': 0.220,     # mm per lens-corrected px
        'x_direction': 1,       # sign of forward for X
        'x_range_mm': 30.0,     # X is given within this of a code junction
        'spacing_mm': 80.0,     # code junction to code junction
        'lens_k1': 0.15,        # lens correction strength, 0 = off
        # Tile 2's own light, used while Tile 2 is on (1.32.0). 0 or [] =
        # keep the Tile 1 value (camera.exposure_us / camera.gain_db /
        # camled.brightness / camled.leds); switching back to Tile 1 puts
        # those back. Live. The camera re-latches (~1 s, no frames) only
        # when exposure or gain really change.
        'exposure_us': 0,       # camera exposure on Tile 2, us
        'gain_db': 0.0,         # camera gain on Tile 2, dB
        'led_level': 0,         # lamp level on Tile 2, 1..255
        'leds': [],             # LED4..LED9 % on Tile 2: six numbers 0..100
    },
    # Frame quality score (vision/quality.py): q = 0..100 per frame plus the
    # reason when it is low. Scores the pipeline's output; never changes it.
    'quality': {
        # 1.30.0: OFF by default, on demand (Sahar, 2026-11-17). Off = no
        # score and no 0xA9 frame (~0.7 ms score + ~0.7 ms UART a frame).
        # Live, no restart: POST /config
        #   {"patch": {"quality": {"enabled": true}}, "persist": false}
        'enabled': False,
        'c_kernels': True,      # the score's two pixel loops in C (t1c.mpy,
                                # same results as viper). false = viper. A
                                # missing t1c or a C error falls back by
                                # itself; /telemetry vision.quality_kernels.
        'light_every': 15,      # light/contrast measured every N frames
        'recent_frames': 30,    # window for the lost / re-search rates
        'avg_frames': 10,       # window for q_avg (what the low alarm uses)
        'low': 40,              # q_avg under this -> "QUALITY LOW" in the log
        'ok': 60,               # ... back over this -> "quality ok again";
                                # below it a frame's `why` is filled in
        'csv': False,           # add q,why to the serial CSV row
        # learning "normal" per robot: slow running average, clean frames only
        'learn_alpha': 0.005,   # ~200 clean frames (~2 s of driving)
        'learn_lo': 0.6,        # learned value kept within these x the seed,
        'learn_hi': 1.6,        # so bad can never become normal
        'learn_min_q': 0.8,     # a frame teaches only if it scored this well
        'warm_frames': 200,     # until this many clean frames: 'learning'
        # seeds of "normal" (measured on the robot video, see docs/quality.md)
        'normal': {
            'cov_c_j': 0.70,    # points / column scan lines, at a junction
            'cov_r_j': 0.65,    # points / row scan lines, at a junction
            'cov_c_y': 0.80,    # points / column scan lines, between junctions
            'mad': 0.20,        # fit spread, px
            'sharp': 0.68,      # share of runs within +-2 px of the tape width
            'width': 31.0,      # tape width in the picture, px
            'split': 0.5,       # angle between the two bars minus 90, deg
            'contrast': 125.0,  # tape minus floor brightness
            'th_jitter': 0.6,   # Otsu change per update
            'edge_c': 0.22,     # edge steepness across the horizontal bar
            'edge_r': 0.24,     # ... across the vertical bar (0.5 = perfect
                                # step; the video is upscaled 2x, so a real
                                # camera frame should learn higher)
        },
        # scoring limits: score 0 at *_bad, 1 at *_good, straight line between
        'pts_bad': 0.35, 'pts_good': 0.75,   # points, as a share of normal
        'rel_bad': 0.50, 'rel_good': 0.80,   # sharpness/contrast vs normal
        'mad_bad': 3.0, 'mad_good': 1.0,     # px (3.0 = the pipeline's gate)
        'mad_v_bad': 4.0, 'mad_v_good': 2.0, # vertical bar: the lens bends it
                                             # near the frame edge (~2 px)
        'width_bad': 0.35, 'width_good': 0.15,   # off normal, as a share
        'split_bad': 6.0, 'split_good': 3.0,     # deg beyond normal (the lens
                                                 # bends a bar at the frame
                                                 # edge by ~3 deg: normal)
        'contrast_bad': 40, 'contrast_good': 100,
        'dark_bad': 60, 'dark_good': 110,        # tape brightness
        'clip_bad': 15.0, 'clip_good': 3.0,      # % of tape at 250+
        'jitter_bad': 12.0, 'jitter_good': 4.0,  # Otsu change per update
        'lost_rate_bad': 0.30, 'lost_rate_good': 0.05,
        'research_bad': 0.60, 'research_good': 0.20,
        'refused_score': 0.3,   # a frame whose X was refused
        'img_floor': 0.4,       # a bad PICTURE alone pulls q down to 40 only
        # "a crossing bar is in view but no junction was found": at least
        # bar_min_cov of the row lines have their point within +-bar_tol px
        # of the median x, and that x is not within bar_edge x width of the
        # frame edge. Robot video, long_cut 60: missed junctions had ~0.55-0.7
        # of rows on a bar at x 36-76; real between-junction frames had
        # <= 0.48, or the bar at the frame edge.
        'bar_min_cov': 0.45, 'bar_tol': 6, 'bar_edge': 0.1,
        'missed_score': 0.2,
        'sharp_half': 2,        # px either side of the width peak
        'edge_bad': 0.60, 'edge_good': 0.85,     # edge steepness vs normal
        'edge_samples': 8,      # scan points whose edges are measured
        'edge_win': 3,          # px either side of the edge searched
        'sharp_min_runs': 10,   # fewer runs -> no blur/width verdict
        # Tile 2 (UNIT16 hole grid, frame_sig): laid over the keys above
        # for the Tile 2 scorer, one level deep, so this 'normal' replaces
        # Tile 1's (quality.make_quality). The bench's T2_QCFG (openmv/bench
        # tile2.py): normals and limits from one 542-frame clip, still to be
        # checked on AE3 recordings (bench owner item O17).
        'tile2': {
            'normal': {
                'cov_r': 0.45,  # road points kept / strips laid (bench 2.1.0: clean bar points first)
                'cov_x': 0.37,  # cross bar, the same
                'mad': 0.50,    # road fit rms, px
                'split': 0.19,  # |90 - angle between the bars|, deg
                'contrast': 61.5,   # plate - holes, quarter-copy Otsu split
                'th_jitter': 0.6,   # mean |Otsu change| every 3 frames
            },
            'mad_good': 1.4, 'mad_bad': 2.5,       # road rms px; 2.5 = the
                                                   # Pipe's GATE_RMS_TRACK
            'mad_v_good': 1.3, 'mad_v_bad': 2.5,   # cross rms px
            'split_good': 1.5, 'split_bad': 4.0,   # deg; 4.0 = PERP_TOL
            'contrast_good': 35, 'contrast_bad': 15,   # grey levels
            'dark_good': 50, 'dark_bad': 25,       # plate mean
            'unread_frames': 10,    # reason 16 code_unreadable: this many
                                    # frames in a row with a full code block
                                    # in view but no name ...
            'unread_score': 0.5,    # ... then this (a warning: X is there,
                                    # only the name is missing)
        },
    },
    'turn': {
        'enabled': True,
        'report_every': 1,      # send the 0xA8 status every Nth turn frame.
                                # 1 is affordable: 8 bytes is 0.69 ms of
                                # blocking write, against 3.4 ms for a line.
        'fault_recover_ms': 1000,  # FAULT -> DRIVE on our own after this.
                                # Long enough for the STM32 to see FAULT and
                                # recover us itself; 0 = never (old latch).
        'max_turn_ms': 20000,   # a turn nobody ends (no V 20 0) is ended
                                # after this. A 360 takes ~8.4 s. 0 = never.
        'switch_quiet_ms': 1000,  # a tile switch / camera re-latch waits
                                # this long after a turn ends: the STM32's
                                # next sub-turn of a 180/360 comes a tick
                                # after DRIVE (1.32.0, turn_service.busy)
    },
    'uart': {
        'enabled': True,
        # UART1 on the AE3 = board pins P4/P5, which ARE the QWIIC
        # connector's two signal lines (P4 = SCL = TX, P5 = SDA = RX;
        # MCU P0_5/P0_4). machine.UART(1) selects them by default —
        # verified on fw v5.0. Bench check: host/uart_loopback.sh
        'id': 1,                # machine.UART id — set per wiring
        'baud': 115200,         # must match the STM32 main board
        'heartbeat_ms': 500,
        # 'binary' is the 8-byte hot-path frame (protocol.encode_xy); 'ascii'
        # is the readable 39-byte line, useful on a bench terminal but 4.9x
        # the blocking write cost, which comes straight off the frame rate.
        'frame_format': 'binary',
        # Heading (theta) travels inside the measurement frame, so it
        # costs 2 bytes rather than a frame of its own. This is the only
        # knob: degrees of band-fit disagreement above which the angle is
        # sent as absent instead. 0 = no gate -- measure real split values
        # on hardware before inventing a threshold.
        'theta_max_split_deg': 0,
        'stream_enabled': True,  # STM32 can stop/resume with "V 2 <0|1>"
        'divisor': 1,            # send every Nth frame ("V 3 <n>")
        # Frame quality (0xA9, protocol.encode_quality) right after each x/y
        # frame, same seq. 8 more bytes = ~0.69 ms more blocking write per
        # frame. Safe on any STM32: unknown types >= 0x80 are consumed.
        'quality_frame': True,
        # Send each frame's x/y + quality bytes straight into the UART's
        # 32-byte hardware TX FIFO (comms/fifo_tx.py) instead of
        # UART.write(), which waits for the wire (~1.6 ms a frame). Checks
        # the UART registers first and refuses if anything is off; a full
        # FIFO sends that frame the normal way. Live; /telemetry uart.fast_tx.
        # 1.32.0: the 0xA8 turn status and the keep-alives (uptime + IP,
        # heartbeat) go the same way; uart.fast_kinds counts each kind.
        # 1.30.0: on by default (Sahar, 2026-11-17); 0302 ran it with crc 0 /
        # missed 0, blank AE3 post 1.36 -> 0.39 ms (with quality off).
        'fast_tx': True,
        # Keep-alive/uptime frame period. The STM32 calls the link dead
        # after CAM_LINK_ALIVE_MS (2 s), so this must stay well under it.
        'keepalive_ms': 1000,    # send MSG_HEARTBEAT if no frames this long
        'rx_poll_ms': 50,
        # rxbuf must cover a full poll interval of continuous rx, or bytes
        # are lost between polls: 115200 8N1 = 11520 B/s, so 50 ms = 576 B.
        # The driver default of 256 B (~22 ms) is NOT enough.
        'rxbuf': 1024,          # ~89 ms of headroom at 115200
    },
    'wifi': {
        'enabled': True,
        # Ordered list of {'ssid', 'password'}, tried in turn until one
        # joins; the first is tried first. Added to by deploy.sh wifi, the
        # STM32's cam_wifi (V 10) or POST /config. Max 8 (net/wifi.py).
        'networks': [],
        'ssid': '',             # older single-network form; still tried,
        'password': '',         #   after 'networks'. V 10 migrates it.
        'legacy_creds_path': '/flash/etc/wifi.json',  # fallback location
        'connect_timeout_s': 20,
        'retry_max_s': 60,      # reconnect backoff ceiling
        'ap_fallback': True,    # no creds / repeated join failures -> own AP
        'ap_after_failures': 3, # STA attempts before falling back
        'ap_ssid': 'gridcam',   # actual SSID gets a -<uid> suffix
        'ap_password': 'gridcam123',   # WPA2 needs >= 8 chars
    },
    'ntp': {
        # Only built when WiFi is enabled; syncs on 'wifi_up' then every
        # resync_h. Without it the board boots at 1970-01-01 and nothing in
        # the logs can be correlated with off-board events.
        'enabled': True,
        # 'boot'     -- sync once after boot (up to boot_attempts tries), then
        #               never again until the next boot. DEFAULT: settime()
        #               blocks the one asyncio loop that also feeds the STM32,
        #               so every attempt is a risk to the link.
        # 'periodic' -- the old behaviour: every resync_h, retry_s on failure,
        #               for ever.
        # 'off'      -- never.
        # Never attempted in AP mode whatever this says: there is no upstream,
        # so the DNS lookup blocks past timeout_s and fails (2026-10-05: an
        # STM32 link drop every ~67 s). TODO: take the time from the ESP32.
        'mode': 'boot',
        'boot_attempts': 3,     # 'boot' mode gives up after this many failures
        'host': 'pool.ntp.org',
        'timeout_s': 2,         # BLOCKING socket wait — keep well under
                                #   health.beat_deadline_ms (4000)
        'resync_h': 12,         # periodic re-sync interval
        'retry_s': 60,          # backoff after a failed attempt
        # The RTC is kept in UTC; the timezone is applied only when
        # formatting for humans, so a DST change can never make timestamps
        # appear to go backwards.
        'tz': 'Asia/Jerusalem',  # label shown in logs/telemetry
        'tz_offset_min': 120,    # STANDARD offset (IST = UTC+2)
        'tz_dst': 'israel',      # 'israel' (IDT, +1h) or 'none' for a
                                 #   fixed offset. Israel rule: starts the
                                 #   Friday before the last Sunday of March,
                                 #   ends the last Sunday of October.
    },
    'mdns': {
        'enabled': True,
        'hostname': '',         # '' = auto: gridcam-<uid> -> gridcam-xxxx.local
        'instance': '',         # DNS-SD instance name; '' = same as hostname
        'announce_s': 60,       # unsolicited announcement period
    },
    'http': {
        'enabled': True,
        'port': 8080,
        'token': '',            # shared-secret for POST endpoints; '' = open (LAN)
        # Measured on the AE3 (QVGA, encode us/frame and bytes):
        #   scale 0.50 q50 160x100  3338 us  2223 B
        #   scale 0.50 q80 160x100  3662 us  3296 B   <- default
        #   scale 1.00 q50 320x200  8158 us  4941 B
        #   scale 1.00 q70 320x200  8593 us  6289 B
        #   scale 1.00 q90 320x200  9824 us 11089 B
        # JPEG quality is CHEAP (+10% for q50->q80), resolution is
        # EXPENSIVE (+144% for 0.5->1.0). So spend on quality first.
        'mjpeg_quality': 80,
        'mjpeg_divisor': 2,     # stream every Nth frame
        # The overlay is drawn AFTER this downscale (see vision_service), so
        # the fixed-size font stays readable. Raise to 1.0 for a full
        # 320x200 stream at ~2.4x the encode cost.
        'mjpeg_scale': 0.5,     # 0.5 -> 160x100 from a 320x200 frame
        # What the stream overlay draws (display only, never the measurement):
        'overlay_lines': True,  # fitted grid lines, junction + centre crosses
        'overlay_text': True,   # the 'X.. Y.. T.. (tier)' / 'no marker' readout
    },
    'log': {
        'level': 'info',        # debug|info|warn|error
        'dir': '/flash/log',
        'file_size': 32 * 1024, # rotate when app.0.log exceeds this
        'file_count': 4,        # app.0.log .. app.3.log  (hard cap ~128 KB)
        'flush_s': 30,          # background flush period; writes happen ONLY
                                #   when new lines are pending (flash wear)
        'flush_chunk_lines': 16,  # periodic flush writes this many lines,
                                #   then yields: flash writes do not await,
                                #   and the loop also feeds the STM32
        'flush_on_error': True, # ERROR lines hit flash immediately
        'ring_lines': 200,      # RAM ring buffer for /logs?tail= and flushes
    },
    'stall': {
        # Loop-stall monitor (gridcam/stall.py): time the vision loop spends
        # yielded to other tasks. There is no _thread here, so a task that
        # does not await holds the frames to the STM32 hostage; the STM32
        # calls >2 s of silence a link drop.
        'threshold_ms': 100,    # count a gap at or above this
        'warn_ms': 500,         # and log it, with the suspect, at or above this
    },
    'apriltag': {
        'enabled': True,
        'capture': 'current',   # 'current' = live QVGA frame, no nav pause.
                                # 'vga' is REJECTED on fw v5.0: a live
                                # framesize switch hard-freezes the VM and
                                # there is no watchdog to recover
                                # ('vga_unsafe' bypasses, bench only).
        'family': 'TAG36H11',
        # What the detector sees. 'none' (default): the frame exactly as the
        # camera captured it. 'enhance': adaptive histogram equalisation +
        # unsharp mask first -- for embossed / low-contrast tags; it was
        # always on before 1.23. Anything else is treated as 'none'.
        'preprocess': 'none',
        'histeq_clip': 8,       # adaptive histogram equalisation clip limit
        'unsharp_mul': 1.2,     # unsharp-mask strength
        'timeout_s': 5,
    },
    # Camera illumination. The lamp is the SIX WHITE LEDs on the STM32 main
    # board (IS31FL3207), not anything on this module -- the camera owns the
    # setting and pushes it over the UART as "L 5 <level>". Replaces the
    # on-board WS2812 strip as the camera's light source.
    'camled': {
        # BOOT STATE, not a persisted setting. camled reads this from
        # DEFAULTS and actively prunes any stored override, so the lamp always
        # comes up on: a robot that boots dark looks broken, and the reason
        # would be buried in this file. Switching it off is session-only.
        'enabled': True,
        'brightness': 128,      # 0..255, one shared level; THIS persists
        # Each of the six white LEDs (PCB names LED4..LED9) as % of the
        # brightness, 0..100. Sent with it as ONE line, "L 11 <level> <p4>..
        # <p9>". LED5 and LED8 sit over the junction and washed it out at
        # full level (robot 0302, 2026-10-25), hence 35. Must match the
        # STM32's boot values (led.h LED_WHITE_PCT_DEFAULTS).
        'leds': {'led4': 100, 'led5': 35, 'led6': 100,
                 'led7': 100, 'led8': 35, 'led9': 100},
        # Re-push the level this often. The STM32 comes up dark after a
        # reset and cannot ask for the value, so the camera must re-offer
        # it. 0 disables (not recommended -- the lamp then stays off).
        'reassert_s': 10,
    },

    'led': {
        'enabled': True,
        'ok_pin': 'LED_GREEN',      # heartbeat blink while healthy
        'fault_pin': 'LED_RED',     # solid while unhealthy (blink = safe mode)
        # white boot blink = every colour at once; missing colours are skipped
        'boot_pins': ['LED_RED', 'LED_GREEN', 'LED_BLUE'],
        'boot_blink_ms': 100,       # fast blink while services are starting
        'active_low': True,         # AE3 status LEDs light on logic 0 (measured)
        # named colours for manual override (POST /cmd {"cmd":"led"})
        'colors': {'red': 'LED_RED', 'green': 'LED_GREEN', 'blue': 'LED_BLUE'},
        'slow_blink_ms': 1000,      # 'slow' override rate; 'fast' uses blink_ms
                                    # (rounded down to a whole number of ticks)
        'blink_ms': 250,            # heartbeat half-period — independent of
                                    # health.period_s so the blink stays
                                    # visible. Also the health task's wake
                                    # rate, so it sets the idle CPU cost.
    },
    'watchdog': {
        'enabled': True,
        'timeout_ms': 8000,
        'beat_deadline_ms': 4000,   # critical service must beat this often
    },
    'health': {
        'period_s': 2,
        'stable_after_s': 60,   # healthy uptime before mark_stable()
    },
    'ota': {
        'dir': '/flash/ota',
        'keep_crash_files': 8,
    },
}


def prune_overlay(overlay, defaults):
    """Drop overlay entries equal to their default. Keeps the persisted
    file minimal, so firmware default changes are not shadowed by stale
    full-config snapshots. Returns a new dict."""
    out = {}
    for k, v in overlay.items():
        if k in defaults and isinstance(v, dict) and isinstance(defaults[k], dict):
            sub = prune_overlay(v, defaults[k])
            if sub:
                out[k] = sub
        elif k in defaults and defaults[k] == v:
            pass                        # redundant — matches the default
        else:
            out[k] = v
    return out


def deep_merge(base, over):
    """Recursively merge dict `over` into a copy of dict `base`."""
    out = {}
    for k in base:
        out[k] = base[k]
    for k in over:
        v = over[k]
        if k in out and isinstance(out[k], dict) and isinstance(v, dict):
            out[k] = deep_merge(out[k], v)
        else:
            out[k] = v
    return out


def _type_err(want, v):
    """Why `v` cannot replace a default of type `want`, or None if it can.
    bool is checked before int: in Python True IS an int, so an int default
    would otherwise take a checkbox value, and a bool default a 1."""
    if isinstance(want, bool):
        return None if isinstance(v, bool) else 'must be true or false'
    if isinstance(want, int):
        ok = isinstance(v, int) and not isinstance(v, bool)
        return None if ok else 'must be a whole number'
    if isinstance(want, float):
        ok = isinstance(v, (int, float)) and not isinstance(v, bool)
        return None if ok else 'must be a number'
    if isinstance(want, str):
        return None if isinstance(v, str) else 'must be text in quotes'
    if isinstance(want, list):
        return None if isinstance(v, list) else 'must be a list [...]'
    return None


def validate_patch(patch, defaults, current, prefix='', out=None):
    """Reasons `patch` would corrupt the config; [] if it is safe to apply.

    Every key must exist in DEFAULTS with a value of the same type, so a typo
    or a section replaced by a scalar ({"camera": 5}) is refused before it can
    reach flash. A value equal to the current one is never refused: the web
    page sends the whole effective config back, stale keys included."""
    if out is None:
        out = []
    if not isinstance(patch, dict):
        out.append('%s must be a JSON object {...}' % (prefix.rstrip('.')
                                                       or 'the config'))
        return out
    for k in patch:
        v = patch[k]
        name = prefix + str(k)
        cur = current.get(k) if isinstance(current, dict) else None
        # same type too: in Python 1 == True, so a 1 for a bool that is
        # currently true would otherwise pass as 'unchanged'
        if v == cur and type(v) is type(cur):
            continue
        if not isinstance(defaults, dict) or k not in defaults:
            out.append("unknown setting '%s'" % name)
            continue
        want = defaults[k]
        if isinstance(want, dict):
            if not isinstance(v, dict):
                out.append('%s is a section and must be an object {...}' % name)
            else:
                validate_patch(v, want, cur if isinstance(cur, dict) else {},
                               name + '.', out)
            continue
        why = _type_err(want, v)
        if why:
            out.append('%s %s (got %s)' % (name, why, json.dumps(v)[:40]))
    return out


class ConfigService(Service):
    """Layered configuration: DEFAULTS <- /flash/etc/config.json <- runtime."""

    name = "config"

    def __init__(self, app, path=CONFIG_PATH):
        super().__init__(app)
        self._path = path
        self._data = DEFAULTS
        self._overlay = {}
        self._listeners = []

    def setup(self):
        self.reload()

    def reload(self):
        """Re-read the overlay file and rebuild the effective config."""
        try:
            with open(self._path) as f:
                self._overlay = json.load(f)
        except (OSError, ValueError):
            self._overlay = {}
        self._data = deep_merge(DEFAULTS, self._overlay)

    def get(self, path, default=None):
        """Dotted-path lookup, e.g. get('http.port')."""
        node = self._data
        for part in path.split('.'):
            if not isinstance(node, dict) or part not in node:
                return default
            node = node[part]
        return node

    def section(self, name):
        """Whole config section as a dict (e.g. section('pipeline'))."""
        return self._data.get(name, {})

    def set(self, path, value, persist=False):
        """Runtime override by dotted path; optionally persisted to flash."""
        parts = path.split('.')
        node = self._overlay
        for part in parts[:-1]:
            if part not in node or not isinstance(node[part], dict):
                node[part] = {}
            node = node[part]
        node[parts[-1]] = value
        self._overlay = prune_overlay(self._overlay, DEFAULTS)
        self._data = deep_merge(DEFAULTS, self._overlay)
        if persist:
            self.save()
        for cb in self._listeners:
            try:
                cb(path, value)
            except Exception:
                pass

    def patch(self, overlay, persist=False):
        """Merge a nested dict of overrides (from the HTTP API).

        Raises ValueError, listing every reason, when the patch fails
        validate_patch(); nothing is applied or saved then. Returns whether
        the result reached flash (always False without persist)."""
        errors = validate_patch(overlay, DEFAULTS, self._data)
        if errors:
            raise ValueError('; '.join(errors))
        self._overlay = prune_overlay(deep_merge(self._overlay, overlay),
                                      DEFAULTS)
        self._data = deep_merge(DEFAULTS, self._overlay)
        saved = self.save() if persist else False
        for cb in self._listeners:
            try:
                cb(None, overlay)
            except Exception:
                pass
        return saved

    # What a reset keeps: the WiFi login (without it the camera drops into
    # its fallback AP and is no longer at its address), the web token, and
    # the tile: vision.tile says what floor this is, not a tuning value, and
    # Tile 1 code on a Tile 2 floor steers on noise.
    RESET_KEEP = (('wifi',), ('http', 'token'), ('vision', 'tile'))

    def reset_defaults(self):
        """Every setting back to DEFAULTS except RESET_KEEP, saved at once.
        Returns whether the file reached flash. Every listener is told about
        every section, so each service re-reads its own."""
        kept = {}
        for path in self.RESET_KEEP:
            src, dst = self._overlay, kept
            for part in path[:-1]:
                src = src.get(part) if isinstance(src, dict) else None
                dst = dst.setdefault(part, {})
            if isinstance(src, dict) and path[-1] in src:
                dst[path[-1]] = src[path[-1]]
            elif not dst and len(path) > 1:
                kept.pop(path[0], None)
        self._overlay = kept
        self._data = deep_merge(DEFAULTS, self._overlay)
        saved = self.save()
        everything = dict((k, {}) for k in DEFAULTS)
        for cb in self._listeners:
            try:
                cb(None, everything)
            except Exception:
                pass
        return saved

    def save(self):
        """Write the overlay to flash; True on success. Serialised BEFORE the
        file is opened, so a failure there cannot leave it truncated."""
        try:
            text = json.dumps(self._overlay)
            with open(self._path, 'w') as f:
                f.write(text)
            return True
        except (OSError, TypeError, ValueError) as e:
            if self.log:
                self.log.error("config save failed: %s" % e)
            return False

    def on_change(self, callback):
        """callback(path_or_None, value) after every set()/patch()."""
        self._listeners.append(callback)

    def as_dict(self):
        return self._data

    def stats(self):
        return {'overlay_keys': len(self._overlay)}
