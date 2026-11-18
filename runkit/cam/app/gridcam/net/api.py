# Remote-support HTTP API.
#
#   GET  /telemetry           health + per-service stats JSON
#   GET  /config              effective config
#   POST /config              {"patch": {...}, "persist": bool}   [token]
#                             {"reset": true}: every setting to its default,
#                              except wifi.* and http.token; saved at once
#   GET  /logs                list log files; ?tail=N for recent RAM lines
#   GET  /logs/<file>         download a rotated log file
#   GET  /crash               list crash reports
#   GET  /crash/<file>        download one report
#   POST /cmd                 {"cmd": "restart"|"safe_mode"|
#                              "ota_app","url":...|"ota_fw","url":...,
#                              "sha256":...}                      [token]
#                             {"cmd": "led", "color": ..., "mode": ...}
#                              colour: off|red|green|blue|yellow|cyan|
#                                      magenta|white
#                              mode  : on|off|slow|fast|auto
#                              ("auto" returns the LED to the health service)
#                             {"cmd": "points_reset"}  zero /telemetry vision.points
#                             {"cmd": "tile", "tile": 1|2}  switch the floor
#                              tile and save it (only vision.tile is
#                              written). The loop applies it between frames;
#                              it is held while turning or paused. Switch
#                              only when the robot is stopped.
#
# [token] endpoints check the shared secret from config http.token
# (X-Auth-Token header or ?token=) — LAN-grade protection.

import json

from .httpd import send_json, send_response


def register(server, app):
    log = app.log.tagged('api')

    async def telemetry(req):
        await send_json(req.writer, app.telemetry())

    async def get_config(req):
        await send_json(req.writer, app.config.as_dict())

    async def post_config(req):
        body = req.json()
        if isinstance(body, dict) and body.get('reset') is True:
            saved = app.config.reset_defaults()
            log.info('config reset to defaults (wifi, token and tile kept)')
            await send_json(req.writer, {'ok': True, 'persisted': saved})
            return
        if not isinstance(body, dict) or 'patch' not in body:
            await send_json(req.writer, {'error': 'need {"patch": {...}}'}, 400)
            return
        try:
            saved = app.config.patch(body['patch'],
                                     persist=bool(body.get('persist')))
        except ValueError as e:
            # nothing applied, nothing written: the config stays as it was
            errors = str(e).split('; ')
            log.warn('config refused: %s' % errors[0])
            await send_json(req.writer, {'error': 'config refused',
                                         'errors': errors}, 400)
            return
        await send_json(req.writer, {'ok': True, 'persisted': saved})

    async def logs(req):
        name = req.path[len('/logs'):].strip('/')
        if name:
            data = app.logsvc.read_file(name)
            if data is None:
                await send_json(req.writer, {'error': 'no such file'}, 404)
            else:
                await send_response(req.writer, content_type='text/plain',
                                    body=data)
            return
        tail = req.query.get('tail')
        if tail:
            await send_response(req.writer, content_type='text/plain',
                                body='\n'.join(app.log.tail(int(tail))))
        else:
            await send_json(req.writer, {'files': app.logsvc.list_files()})

    async def crash(req):
        name = req.path[len('/crash'):].strip('/')
        if name:
            data = app.crash_guard.read_report(name)
            if data is None:
                await send_json(req.writer, {'error': 'no such report'}, 404)
            else:
                await send_response(req.writer, content_type='text/plain',
                                    body=data)
        else:
            await send_json(req.writer,
                            {'reports': app.crash_guard.list_reports()})

    def _tile_state(resp):
        # what the vision loop runs now, from its own telemetry
        v = getattr(app, 'vision', None)
        st = {}
        if v is not None:
            try:
                st = v.stats()
            except Exception:
                st = {}
        resp['active'] = st.get('tile')
        resp['pending'] = st.get('tile_pending')
        resp['tile_err'] = st.get('tile_err') or ''
        return resp

    def _tile(t):
        """The page's tile button. Writes ONLY vision.tile (persisted), so
        it never touches camera.* (a 1 s latch) the way a full textarea
        Apply does. Returns (http status, reply)."""
        if type(t) is not int or t not in (1, 2):   # not True, not 1.0
            return 400, {'error': 'tile must be 1 or 2'}
        v = getattr(app, 'vision', None)
        reason = ''
        req_tile = getattr(v, 'request_tile', None)
        if req_tile is not None:
            # ask the loop first: a refused tile is not saved
            try:
                ok, reason = req_tile(t)
            except Exception as e:
                ok, reason = False, str(e) or e.__class__.__name__
            if not ok:
                log.warn('tile %d refused: %s' % (t, reason))
                return 409, _tile_state({'error': 'tile refused',
                                         'reason': reason or 'refused',
                                         'tile': t})
        try:
            saved = app.config.patch({'vision': {'tile': t}}, persist=True)
        except ValueError as e:
            return 400, {'error': 'config refused',
                         'errors': str(e).split('; ')}
        log.info('tile %d asked (saved %s)' % (t, saved))
        return 200, _tile_state({'ok': True, 'tile': t, 'persisted': saved,
                                 'reason': reason or ''})

    async def cmd(req):
        import asyncio
        body = req.json() or {}
        c = body.get('cmd')
        log.info('cmd: %s' % c)

        if c == 'restart':
            await send_json(req.writer, {'ok': True, 'action': 'reset'})
            await asyncio.sleep(0.5)
            import machine
            machine.reset()

        elif c == 'safe_mode':
            # force the crash-loop guard to trip on next boot
            counters = app.counters
            counters.crashes = 99
            counters._store()
            await send_json(req.writer, {'ok': True, 'action': 'safe_mode'})
            await asyncio.sleep(0.5)
            import machine
            machine.reset()

        elif c == 'ota_app':
            url = body.get('url')
            if not url:
                await send_json(req.writer, {'error': 'need url'}, 400)
                return
            from ..ota.app_ota import AppOta
            ota = AppOta(app)
            await send_json(req.writer, {'ok': True, 'action': 'ota_app'})
            # run after responding; install() resets the board on success
            asyncio.create_task(ota.update(url))

        elif c == 'ota_fw':
            url = body.get('url')
            sha = body.get('sha256')
            if not url or not sha:
                await send_json(req.writer,
                                {'error': 'need url and sha256'}, 400)
                return
            from ..ota.fw_ota import FwOta
            await send_json(req.writer, {'ok': True, 'action': 'ota_fw'})
            asyncio.create_task(FwOta(app).update(url, sha))

        elif c == 'camled':
            # Camera illumination: the six white LEDs on the STM32 main board.
            # The camera owns the value; the STM32 owns the hardware.
            if app.camled is None:
                await send_json(req.writer,
                                {'error': 'camled unavailable'}, 503)
                return
            try:
                if body.get('action') == 'off':
                    app.camled.off()
                else:
                    app.camled.set(enabled=body.get('enabled'),
                                   brightness=body.get('brightness'))
                resp = {'ok': True}
                resp.update(app.camled.stats())
                await send_json(req.writer, resp)
            except Exception as e:
                await send_json(req.writer, {'error': str(e)}, 500)

        elif c == 'led':
            # Manual status-LED control, bypassing the health heartbeat.
            #   {"cmd":"led","color":"white","mode":"fast"}
            #   {"cmd":"led","mode":"auto"}    -> back to the service
            mode = body.get('mode', 'auto')
            try:
                if mode == 'auto':
                    app.health.clear_led_override()
                else:
                    app.health.set_led_override(body.get('color', 'white'),
                                                mode)
            except ValueError as e:
                await send_json(req.writer, {'error': str(e)}, 400)
                return
            except Exception as e:
                await send_json(req.writer, {'error': str(e)}, 500)
                return
            await send_json(req.writer,
                            {'ok': True, 'led_override': app.health.led_override()})

        elif c == 'identify':
            # blink the (otherwise unused) blue LED so the board can be
            # physically located among several
            async def _blink():
                try:
                    import machine
                    led = machine.LED('LED_BLUE')
                    for _ in range(20):
                        led.on()
                        await asyncio.sleep(0.15)
                        led.off()
                        await asyncio.sleep(0.15)
                except Exception as e:
                    log.warn('identify failed: %s' % e)
            asyncio.create_task(_blink())
            await send_json(req.writer, {'ok': True, 'action': 'blink 6s'})

        elif c == 'points_reset':
            app.vision.points.reset()
            await send_json(req.writer, {'ok': True})

        elif c == 'tile':
            status, resp = _tile(body.get('tile'))
            await send_json(req.writer, resp, status)

        else:
            await send_json(req.writer, {'error': 'unknown cmd'}, 400)

    async def ota_upload(req):
        """Direct bundle upload from the browser (streamed to flash —
        bundles exceed the in-memory body cap, so read from req.reader)."""
        import asyncio
        length = int(req.headers.get('content-length', '0') or '0')
        if length <= 0:
            await send_json(req.writer, {'error': 'empty upload'}, 400)
            return
        from .. import util
        ota_dir = app.config.get('ota.dir')
        util.makedirs(ota_dir)
        dst = '%s/upload.gpk' % ota_dir
        got = len(req.body)
        with open(dst, 'wb') as f:
            if req.body:                       # small uploads were preloaded
                f.write(req.body)
            while got < length:
                chunk = await req.reader.read(min(1024, length - got))
                if not chunk:
                    break
                f.write(chunk)
                got += len(chunk)
        if got != length:
            await send_json(req.writer,
                            {'error': 'truncated: %d/%d' % (got, length)}, 400)
            return
        log.info('ota upload: %d bytes' % got)
        from ..ota.app_ota import AppOta
        ota = AppOta(app)
        await send_json(req.writer, {'ok': True, 'bytes': got,
                                     'action': 'install+reset'})
        await asyncio.sleep(0.5)               # let the response flush
        ok = ota.install_from_file(dst)        # resets the board on success
        if ok is False:
            log.error('uploaded bundle rejected — still on current version')

    async def apriltag_detect(req):
        if app.apriltag is None:
            await send_json(req.writer, {'error': 'apriltag disabled'}, 400)
            return
        result = await app.apriltag.detect()
        await send_json(req.writer, result,
                        200 if 'tags' in result else 500)

    async def apriltag_last(req):
        if app.apriltag is None:
            await send_json(req.writer, {'error': 'apriltag disabled'}, 400)
            return
        await send_json(req.writer, app.apriltag.last_result())

    async def apriltag_image(req):
        jpeg = app.apriltag.last_jpeg if app.apriltag else None
        if jpeg is None:
            await send_json(req.writer, {'error': 'no detection yet'}, 404)
            return
        req.writer.write(b'HTTP/1.1 200 OK\r\n'
                         b'Content-Type: image/jpeg\r\n' +
                         ('Content-Length: %d\r\n\r\n' % len(jpeg)).encode())
        req.writer.write(jpeg)
        await req.writer.drain()

    server.add_route('GET', '/telemetry', telemetry)
    server.add_route('POST', '/ota/upload', ota_upload, protected=True)
    server.add_route('POST', '/apriltag/detect', apriltag_detect)
    server.add_route('GET', '/apriltag/image', apriltag_image)
    server.add_route('GET', '/apriltag', apriltag_last)
    server.add_route('GET', '/config', get_config)
    server.add_route('POST', '/config', post_config, protected=True)
    server.add_route('GET', '/logs', logs)
    server.add_route('GET', '/crash', crash)
    server.add_route('POST', '/cmd', cmd, protected=True)
