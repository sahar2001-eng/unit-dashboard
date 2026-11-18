# Minimal asyncio HTTP server for remote support.
#
# No framework: parses "METHOD /path?query HTTP/1.x" + headers + a small
# optional body, dispatches on a prefix route table, and gives handlers a
# thin Request/Response API. Handlers are registered by net/api.py and
# net/stream.py. Single-purpose by design (~LAN, one or two clients).

import json

from ..service import Service

MAX_BODY = 8192


class Request:
    def __init__(self, method, path, query, headers, body, reader, writer):
        self.method = method
        self.path = path
        self.query = query          # dict
        self.headers = headers      # dict, lower-case keys
        self.body = body            # bytes
        self.reader = reader
        self.writer = writer

    def json(self):
        try:
            return json.loads(self.body)
        except ValueError:
            return None


async def send_response(writer, status=200, content_type='application/json',
                        body=b'', headers=None):
    reason = {200: 'OK', 400: 'Bad Request', 401: 'Unauthorized',
              404: 'Not Found', 500: 'Internal Server Error'}.get(status, '')
    if isinstance(body, str):
        body = body.encode()
    writer.write(('HTTP/1.1 %d %s\r\n' % (status, reason)).encode())
    writer.write(('Content-Type: %s\r\n' % content_type).encode())
    writer.write(('Content-Length: %d\r\n' % len(body)).encode())
    writer.write(b'Connection: close\r\n')
    for k, v in (headers or {}).items():
        writer.write(('%s: %s\r\n' % (k, v)).encode())
    writer.write(b'\r\n')
    writer.write(body)
    await writer.drain()


async def send_json(writer, obj, status=200):
    await send_response(writer, status=status, body=json.dumps(obj))


def _parse_query(qs):
    out = {}
    for pair in qs.split('&'):
        if '=' in pair:
            k, v = pair.split('=', 1)
            out[k] = v
        elif pair:
            out[pair] = ''
    return out


class HttpService(Service):
    name = "http"

    def setup(self):
        cfg = self.app.config.section('http')
        self._port = cfg['port']
        self._token = cfg['token']
        self._routes = []           # (method, prefix, handler)
        self._requests = 0
        self._errors = 0

        from . import api, stream, webui
        api.register(self, self.app)
        stream.register(self, self.app)
        webui.register(self, self.app)

    def add_route(self, method, prefix, handler, protected=False):
        self._routes.append((method, prefix, handler, protected))

    def _find_route(self, method, path):
        best = None
        for m, prefix, handler, protected in self._routes:
            if m == method and path.startswith(prefix):
                if best is None or len(prefix) > len(best[0]):
                    best = (prefix, handler, protected)
        return best

    def _authorized(self, req):
        if not self._token:
            return True
        return req.headers.get('x-auth-token') == self._token or \
            req.query.get('token') == self._token

    async def _handle(self, reader, writer):
        self._requests += 1
        try:
            line = await reader.readline()
            parts = line.decode().split()
            if len(parts) < 2:
                return
            method, target = parts[0], parts[1]
            path, _, qs = target.partition('?')

            headers = {}
            while True:
                h = await reader.readline()
                if not h or h == b'\r\n' or h == b'\n':
                    break
                k, _, v = h.decode().partition(':')
                headers[k.strip().lower()] = v.strip()

            body = b''
            length = int(headers.get('content-length', '0') or '0')
            if 0 < length <= MAX_BODY:
                body = await reader.readexactly(length)

            req = Request(method, path, _parse_query(qs), headers, body,
                          reader, writer)
            route = self._find_route(method, path)
            if route is None:
                await send_json(writer, {'error': 'not found'}, 404)
            elif route[2] and not self._authorized(req):
                await send_json(writer, {'error': 'unauthorized'}, 401)
            else:
                await route[1](req)
        except Exception as e:
            self._errors += 1
            self.log.error('request failed: %s' % e)
            try:
                await send_json(writer, {'error': str(e)}, 500)
            except Exception:
                pass
        finally:
            try:
                writer.close()
                await writer.wait_closed()
            except Exception:
                pass

    async def run(self):
        import asyncio
        server = await asyncio.start_server(self._handle, '0.0.0.0',
                                            self._port)
        self.log.info('listening on :%d' % self._port)
        while True:
            await asyncio.sleep(5)
            self.heartbeat()

    def stats(self):
        return {'port': self._port, 'requests': self._requests,
                'errors': self._errors}
