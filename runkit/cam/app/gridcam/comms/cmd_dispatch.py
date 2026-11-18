# Inbound commands from the STM32 main board.
#
# Until now the RX path stopped at `bus.publish('uart_cmd', tokens)` with no
# subscriber anywhere in the app -- lines were CRC-checked and then dropped on
# the floor. This is that missing consumer.
#
# Wire format is the shared ASCII one: "<letter> <sub> [args] <crc>\n", already
# validated and split by protocol.LineDecoder, which strips the CRC token. The
# STM32 is the master; the camera answers.
#
# The V family (vision) is ours:
#
#   V 2 <0|1>    stop / start the x/y stream
#   V 3 <n>      send every Nth frame
#   V 5 <k> <v>  set a config key at run time (vision.tile: V 1 5 if refused)
#   V 6 <seq>    AprilTag: detect on the next frame, answer ONCE (async)
#   V 10 <s> <p> set WiFi credentials, both hex-encoded
#
# Replies follow the same convention as the STM32's own table: an echo of the
# letter with an error field, "V <err> <sub> [data]". Anything unrecognised is
# answered rather than ignored, so a mismatch between the two sides shows up as
# a message instead of silence.

from .. import util
from . import protocol


class CmdDispatch:
    def __init__(self, app, link):
        self.app = app
        self.link = link
        self.log = app.log.tagged('uartcmd')
        self.handled = 0
        self.unknown = 0
        self.errors = 0

    # -- helpers ---------------------------------------------------------
    def _reply(self, payload):
        """Send one CRC'd reply line back to the STM32."""
        try:
            self.link._write(protocol.encode_line(payload.split(' ')))
        except Exception as e:               # never let a reply kill the link
            self.errors += 1
            self.log.error('reply failed: %s' % e)

    def _ok(self, sub, data=''):
        self._reply(('V 0 %s %s' % (sub, data)).strip())

    def _err(self, sub, data=''):
        self._reply(('V 1 %s %s' % (sub, data)).strip())

    # -- entry point -----------------------------------------------------
    def handle(self, tokens):
        """Bus callback for 'uart_cmd'. Must not raise: the bus logs and
        continues, but an exception here would silently drop the command."""
        try:
            self._handle(tokens)
        except Exception as e:
            self.errors += 1
            self.log.error('dispatch %s: %s' % (tokens, e))

    def _handle(self, tokens):
        if len(tokens) < 2:
            return
        letter, sub = tokens[0], tokens[1]
        if letter != 'V':
            # Not addressed to the vision family. Silence is right here: the
            # STM32 mirrors its own traffic on this wire and we must not
            # answer commands meant for something else.
            return

        args = tokens[2:]
        self.handled += 1

        if sub == '2':                                   # stream on/off
            on = bool(int(args[0])) if args else True
            self.link.set_stream(on)
            self.log.info('stream %s by host' % ('on' if on else 'off'))
            self._ok('2', '1' if self.link.streaming() else '0')

        elif sub == '3':                                 # rate divisor
            n = max(1, int(args[0])) if args else 1
            self.app.config.set('uart.divisor', n)
            self._ok('3', str(n))

        elif sub == '5':                                 # set a config key
            if len(args) < 2:
                self._err('5')
                return
            key, val = args[0], _coerce(args[1])
            if key == 'vision.tile' and not self._tile_ok(val):
                return self._err('5')         # refused: nothing saved
            self.app.config.set(key, val, persist=True)
            self._ok('5')

        elif sub == '6':                                 # apriltag, on demand
            # Detection waits for the next frame, and this callback runs inside
            # the UART poll, so it must not block: the detection runs as its own
            # task and answers when done. It used to reply str(last_tags) -- a
            # Python list repr on the wire, which no parser could read.
            try:
                seq = int(args[0]) & 0xFF if args else 0
            except ValueError:
                seq = 0
            tags = getattr(self.app, 'apriltag', None)
            if tags is None:
                self._reply(tag_payload(seq, {'error': 'apriltag disabled'}))
                return
            self._spawn(self._tag_detect(seq, tags))

        elif sub == '10':                                # set wifi credentials
            if not args:
                self._err('10')
                return
            ssid = _unhex(args[0])
            passwd = _unhex(args[1]) if len(args) > 1 else ''
            if not ssid:
                self._err('10')
                return
            # Persisted, not applied: WiFi is not reconfigurable at runtime on
            # this port -- the STA is brought up once in setup() and there is no
            # teardown path. It takes effect at the next boot, which the STM32
            # can force with a power cycle.
            # ADDED to the network list, first in line, not a replacement:
            # the camera moves between sites and should still join the old
            # one. The older single ssid/password pair is folded into the
            # list here and cleared, so there is one place to look.
            from ..net.wifi import known_networks, upsert_network
            nets = upsert_network(
                known_networks(self.app.config.section('wifi')), ssid, passwd)
            self.app.config.patch({'wifi': {'networks': nets, 'ssid': '',
                                            'password': '', 'enabled': True}},
                                  persist=True)
            # Never log the password. The SSID is fine and is what you need to
            # confirm the right network was set.
            self.log.info('wifi "%s" added (%d char password), %d known — '
                          'next boot' % (ssid, len(passwd), len(nets)))
            self._ok('10')

        elif sub == '20':
            # Turn control. The camera measures the rotation; the STM32
            # decides when to start and stop it — "the turn is ended by the
            # host sending DRIVE, never by the camera".
            turn = getattr(self.app, 'turn', None)
            if turn is None:
                return self._err('20', 'no turn service')
            try:
                code = int(args[0])
            except (IndexError, ValueError):
                return self._err('20', 'want 0..3')
            state = turn.command(code)
            if state is None:
                # refused: unknown code, or the machine is not ready
                return self._err('20', turn.state_name())
            # Echo the code, THEN the state. The state is the one at the moment
            # of acceptance, not the one the command produces: the machine acts
            # on the next frame, so replying with the state alone answers
            # "V 20 1" (turn left) with "DRIVE", which reads like a refusal.
            self._ok('20', '%d %s' % (code, state))

        else:
            self.unknown += 1
            self._err(sub)

    def _tile_ok(self, t):
        """'V 5 vision.tile N' asks the vision loop first, like the page's
        tile button: a refused tile is answered V 1 5 and never saved, so
        the next boot cannot start on it. Accepted = applied now or held
        until the turn ends (V 0 5)."""
        req = getattr(getattr(self.app, 'vision', None), 'request_tile', None)
        if req is None:
            return True                       # no loop to ask: as before
        ok, why = req(t)
        if not ok:
            self.log.warn('V 5 vision.tile %r refused: %s' % (t, why))
        return ok

    def _spawn(self, coro):
        import asyncio
        asyncio.create_task(coro)

    async def _tag_detect(self, seq, tags):
        """One detection, one answer -- a failure is an answer too, so the
        STM32 is never left waiting on silence."""
        try:
            result = await tags.detect()
        except Exception as e:
            self.errors += 1
            result = {'error': str(e)}
        self._reply(tag_payload(seq, result))

    def stats(self):
        return {'handled': self.handled, 'unknown': self.unknown,
                'errors': self.errors}


def tag_payload(seq, result):
    """'V 0 6 <seq> <n> <id> <rot_cdeg> <cx> <cy> <margin_milli>'.

    Fixed token count whatever happened: the STM32's table matches on it.
    n = -1 means detection failed (busy, timeout, disabled), n = 0 nothing in
    view; both carry id -1. Only the best tag (highest decision margin) fits:
    the STM32's command line is 50 bytes.
    """
    tags = result.get('tags') if isinstance(result, dict) else None
    if tags is None:
        return 'V 0 6 %d -1 -1 0 0 0 0' % seq
    if not tags:
        return 'V 0 6 %d 0 -1 0 0 0 0' % seq
    best = max(tags, key=lambda t: t['decision_margin'])
    rot = int(round(best['rotation_deg'] * 100)) % 36000
    margin = min(99999999, max(0, int(round(best['decision_margin'] * 1000))))
    return 'V 0 6 %d %d %d %d %d %d %d' % (
        seq, min(len(tags), 99), best['id'], rot,
        min(4095, int(best['cx'])), min(4095, int(best['cy'])), margin)


def _unhex(text):
    """Decode the hex the STM32 sends, or '' if it is not valid hex.

    Credentials are hex-encoded so a space or a non-ASCII byte cannot break the
    space-separated framing or the CRC. It is NOT secrecy: the wire is a short
    cable inside the robot and the value lands in config.json in plaintext.

    A malformed value returns '' and the caller refuses, rather than storing
    half a credential and stranding the camera off the network.
    """
    if not text or (len(text) % 2):
        return ''
    try:
        return bytes(int(text[i:i + 2], 16)
                     for i in range(0, len(text), 2)).decode('utf-8')
    except Exception:
        return ''


def _coerce(text):
    """'12' -> 12, '1.5' -> 1.5, 'true'/'false' -> bool, else the string.

    Config values are typed and the wire is not, so a bare str would quietly
    turn an int setting into text and break whoever reads it back.
    """
    low = text.lower()
    if low in ('true', 'false'):
        return low == 'true'
    try:
        return int(text)
    except ValueError:
        pass
    try:
        return float(text)
    except ValueError:
        return text
