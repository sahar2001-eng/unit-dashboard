# MJPEG live stream of the annotated camera view.
#
#   GET /stream    multipart/x-mixed-replace MJPEG (open in a browser)
#   GET /shot      single JPEG snapshot
#   GET /shot?reject=1  the last frame Algorithm 1 gave no result for
#
# The vision loop draws the overlay and compresses frames ONLY while
# `vision.stream_clients > 0`, so streaming cost is zero with no viewer.
# Frame rate is bounded by http.mjpeg_divisor; one or two concurrent
# viewers is the design point.

BOUNDARY = b'gridcamframe'


def register(server, app):
    log = app.log.tagged('stream')
    generation = [0]        # a new viewer supersedes the previous one, so a
                            # half-dead browser connection cannot linger as a
                            # ghost client keeping overlay/compress active

    async def stream(req):
        import asyncio
        vision = app.vision
        writer = req.writer
        writer.write(b'HTTP/1.1 200 OK\r\n'
                     b'Content-Type: multipart/x-mixed-replace; '
                     b'boundary=' + BOUNDARY + b'\r\n'
                     b'Connection: close\r\n\r\n')
        await writer.drain()

        generation[0] += 1
        my_gen = generation[0]
        vision.stream_clients += 1
        log.info('stream client connected (%d)' % vision.stream_clients)
        last = None
        try:
            while generation[0] == my_gen:
                jpeg = vision.last_jpeg
                meta = getattr(vision, 'last_meta', '')
                if jpeg is None or jpeg is last:
                    # 5 ms, not 50: at 50 the loop slept through ~4 frames
                    # after every send and the stream topped out near 20 fps
                    # whatever mjpeg_divisor said. A wake that finds nothing
                    # new costs microseconds on the asyncio loop.
                    await asyncio.sleep(0.005)
                    continue
                last = jpeg
                writer.write(b'--' + BOUNDARY + b'\r\n'
                             b'Content-Type: image/jpeg\r\n' +
                             ('X-Gridcam: %s\r\nContent-Length: %d\r\n\r\n'
                              % (meta, len(jpeg))).encode())
                writer.write(jpeg)
                writer.write(b'\r\n')
                await writer.drain()
        except Exception:
            pass                        # client went away
        finally:
            vision.stream_clients -= 1
            log.info('stream client gone (%d)' % vision.stream_clients)

    async def shot(req):
        import asyncio
        vision = app.vision
        if req.query.get('reject'):
            # the last frame Algorithm 1 gave no result for (vision_service)
            jpeg = vision.reject_jpeg
            if jpeg is None:
                from .httpd import send_json
                await send_json(req.writer, {'error': 'no rejected frame yet'}, 404)
                return
            req.writer.write(b'HTTP/1.1 200 OK\r\nContent-Type: image/jpeg\r\n' +
                             ('Content-Length: %d\r\n\r\n' % len(jpeg)).encode())
            req.writer.write(jpeg)
            await req.writer.drain()
            return
        vision.stream_clients += 1      # ask the loop for one compressed frame
        try:
            jpeg = None
            for _ in range(40):         # up to ~2 s
                await asyncio.sleep(0.05)
                if vision.last_jpeg is not None:
                    jpeg = vision.last_jpeg
                    break
        finally:
            vision.stream_clients -= 1
        if jpeg is None:
            from .httpd import send_json
            await send_json(req.writer, {'error': 'no frame'}, 500)
            return
        writer = req.writer
        writer.write(b'HTTP/1.1 200 OK\r\n'
                     b'Content-Type: image/jpeg\r\n' +
                     ('Content-Length: %d\r\n\r\n' % len(jpeg)).encode())
        writer.write(jpeg)
        await writer.drain()

    server.add_route('GET', '/stream', stream)
    server.add_route('GET', '/shot', shot)
