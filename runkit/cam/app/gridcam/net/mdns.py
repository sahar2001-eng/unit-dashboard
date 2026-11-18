# Minimal mDNS / DNS-SD responder — makes the board discoverable as
# <hostname>.local and advertises the dashboard as an _http._tcp service,
# so neighbours on the LAN can find it without knowing its DHCP address.
#
# Pure-Python (no firmware support needed). Answers:
#   A    <hostname>.local                       -> board IP
#   PTR  _http._tcp.local                       -> <instance>._http._tcp.local
#   PTR  _services._dns-sd._udp.local           -> _http._tcp.local
#   SRV/TXT <instance>._http._tcp.local         -> port 8080 + version info
# and multicasts an unsolicited announcement every `announce_s` so passive
# browsers see the module even if group-membership (IGMP join) is not
# supported by the port — sending multicast always works.
#
# Overhead: one UDP socket + ~1.5 kB buffer; CPU only when packets arrive
# (poll sleep 100 ms between reads, never touches the vision hot path).
#
# The packet encode/parse helpers are pure and desktop-tested.

import struct

from ..service import Service

MDNS_ADDR = '224.0.0.251'
MDNS_PORT = 5353

TYPE_A = 1
TYPE_PTR = 12
TYPE_TXT = 16
TYPE_SRV = 33
CLASS_IN = 1
CACHE_FLUSH = 0x8000
TTL = 120


# ------------------------------------------------------------ wire format
def encode_name(name):
    """'a.b.local' -> DNS label sequence."""
    out = b''
    for label in name.split('.'):
        if label:
            e = label.encode()
            out += bytes((len(e),)) + e
    return out + b'\x00'


def parse_name(data, off):
    """Decode a (possibly compressed) name. Returns (name, next_offset)."""
    labels = []
    jumped = False
    next_off = off
    hops = 0
    while True:
        if off >= len(data) or hops > 16:
            break
        length = data[off]
        if length == 0:
            if not jumped:
                next_off = off + 1
            break
        if length & 0xC0 == 0xC0:            # compression pointer
            if not jumped:
                next_off = off + 2
            off = ((length & 0x3F) << 8) | data[off + 1]
            jumped = True
            hops += 1
            continue
        labels.append(data[off + 1:off + 1 + length].decode())
        off += 1 + length
    return '.'.join(labels), next_off


def parse_questions(data):
    """Return list of (name_lowercase, qtype) from a DNS query packet."""
    if len(data) < 12:
        return []
    flags, qdcount = struct.unpack('>HH', data[2:6])
    if flags & 0x8000:                       # a response, not a query
        return []
    out = []
    off = 12
    for _ in range(qdcount):
        try:
            name, off = parse_name(data, off)
            qtype, _qclass = struct.unpack('>HH', data[off:off + 4])
            off += 4
            out.append((name.lower(), qtype))
        except (struct.error, IndexError):
            break
    return out


def record(name, rtype, rdata, ttl=TTL, flush=True):
    cls = CLASS_IN | (CACHE_FLUSH if flush else 0)
    return (encode_name(name) +
            struct.pack('>HHIH', rtype, cls, ttl, len(rdata)) + rdata)


def a_rdata(ip):
    return bytes(int(p) for p in ip.split('.'))


def srv_rdata(port, target):
    return struct.pack('>HHH', 0, 0, port) + encode_name(target)


def txt_rdata(pairs):
    out = b''
    for k, v in pairs:
        item = ('%s=%s' % (k, v)).encode()
        out += bytes((len(item),)) + item
    return out or b'\x00'


def build_response(answers, additionals=()):
    header = struct.pack('>HHHHHH', 0, 0x8400,   # response + authoritative
                         0, len(answers), 0, len(additionals))
    return header + b''.join(answers) + b''.join(additionals)


# ------------------------------------------------------------ service
class MdnsService(Service):
    name = "mdns"

    def setup(self):
        cfg = self.app.config.section('mdns')
        # WifiService already set the system hostname (from the WLAN MAC);
        # reuse it so firmware mDNS and this responder agree on the name
        wifi_name = getattr(self.app.wifi, 'hostname', None) \
            if self.app.wifi else None
        self.hostname = cfg['hostname'] or wifi_name or 'gridcam'
        self.instance = (cfg['instance'] or self.hostname)
        self._announce_s = cfg['announce_s']
        self._port = self.app.config.get('http.port', 8080)
        self._queries = 0
        self._answers = 0
        self._sock = None
        self.log.info('hostname %s.local, service %s._http._tcp.local'
                      % (self.hostname, self.instance))

    # names (lowercase, for matching)
    def _names(self):
        host = self.hostname.lower() + '.local'
        svc = '_http._tcp.local'
        inst = self.instance.lower() + '.' + svc
        return host, svc, inst

    def _records(self, ip):
        host, svc, inst = self._names()
        from .. import VERSION
        a = record(host, TYPE_A, a_rdata(ip))
        srv = record(inst, TYPE_SRV, srv_rdata(self._port, host))
        txt = record(inst, TYPE_TXT, txt_rdata(
            (('version', VERSION), ('path', '/'))))
        ptr = record(svc, TYPE_PTR, encode_name(inst), flush=False)
        meta = record('_services._dns-sd._udp.local', TYPE_PTR,
                      encode_name(svc), flush=False)
        return a, srv, txt, ptr, meta

    def _respond(self, questions, ip):
        a, srv, txt, ptr, meta = self._records(ip)
        host, svc, inst = self._names()
        answers, extra = [], []
        for qname, qtype in questions:
            if qname == host and qtype in (TYPE_A, 255):
                answers.append(a)
            elif qname == svc and qtype in (TYPE_PTR, 255):
                answers.append(ptr)
                extra += [srv, txt, a]
            elif qname == inst and qtype in (TYPE_SRV, TYPE_TXT, 255):
                answers += [srv, txt]
                extra.append(a)
            elif qname == '_services._dns-sd._udp.local' \
                    and qtype in (TYPE_PTR, 255):
                answers.append(meta)
        if not answers:
            return None
        return build_response(answers, extra)

    def _announcement(self, ip):
        a, srv, txt, ptr, meta = self._records(ip)
        return build_response([ptr, srv, txt, a])

    def _open_socket(self, ip):
        import socket
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        except Exception:
            pass
        s.bind(('0.0.0.0', MDNS_PORT))
        # join the multicast group when the port supports it; without it we
        # may miss queries but periodic announcements still reach browsers
        try:
            opt = getattr(socket, 'IP_ADD_MEMBERSHIP', None)
            proto = getattr(socket, 'IPPROTO_IP', 0)
            if opt is not None:
                s.setsockopt(proto, opt,
                             a_rdata(MDNS_ADDR) + a_rdata('0.0.0.0'))
                self.log.info('joined multicast group')
            else:
                self.log.warn('no IP_ADD_MEMBERSHIP — announce-only mode')
        except Exception as e:
            self.log.warn('multicast join failed (%s) — announce-only' % e)
        s.setblocking(False)
        return s

    async def run(self):
        import asyncio
        from .. import util

        while True:                          # wait for the network
            ip = self.app.wifi.ip() if self.app.wifi else None
            if ip:
                break
            await asyncio.sleep(2)
            self.heartbeat()

        try:
            self._sock = self._open_socket(ip)
        except OSError as e:
            # firmware's lwIP mDNS owns UDP 5353 (EADDRINUSE): name
            # resolution of <hostname>.local is handled by the firmware —
            # our DNS-SD advertisement just isn't possible on this port
            self.log.info('port 5353 owned by firmware mDNS (%s) — '
                          'relying on built-in responder for %s.local'
                          % (e, self.hostname))
            import asyncio as _a
            while True:
                await _a.sleep(10)
                self.heartbeat()

        dest = (MDNS_ADDR, MDNS_PORT)
        last_announce = 0

        while True:
            now = util.ticks_ms()
            if util.ticks_diff(now, last_announce) >= self._announce_s * 1000:
                last_announce = now
                ip = (self.app.wifi.ip() if self.app.wifi else None) or ip
                try:
                    self._sock.sendto(self._announcement(ip), dest)
                except OSError:
                    pass
            try:
                data, addr = self._sock.recvfrom(1536)
            except OSError:                  # nothing pending
                await asyncio.sleep(0.1)
                self.heartbeat()
                continue
            questions = parse_questions(data)
            if questions:
                self._queries += 1
                resp = self._respond(questions, ip)
                if resp:
                    self._answers += 1
                    try:
                        self._sock.sendto(resp, dest)
                    except OSError:
                        pass

    def stop(self):
        try:
            if self._sock:
                self._sock.close()
        except Exception:
            pass

    def stats(self):
        return {'hostname': self.hostname + '.local',
                'queries': self._queries, 'answers': self._answers}
