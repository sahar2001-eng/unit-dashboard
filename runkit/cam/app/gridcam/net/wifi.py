# WiFi station management.
#
# Credentials come from the config section 'wifi' in /flash/etc/config.json:
# 'networks' is an ordered list of {ssid, password}, tried in turn until one
# joins (set them with `host/deploy.sh wifi <ssid> <pw>`, the STM32's
# `cam_wifi`, or POST /config). The older single 'ssid'/'password' pair is
# still honoured as one more entry, and /flash/etc/wifi.json as a legacy
# fallback when nothing else is configured.
# Both files live outside the app slots and are never OTA-bundled.
# WiFi is strictly optional: any failure here only degrades remote support,
# never navigation.

import json

from .. import stall
from ..service import Service


MAX_NETWORKS = 8


def known_networks(cfg, legacy=None):
    """Ordered, de-duplicated [{'ssid', 'password'}] from a 'wifi' config
    section: 'networks' first, then the single 'ssid' pair, then `legacy`
    (the old wifi.json). Malformed entries are skipped, never fatal."""
    out = []
    seen = set()

    def add(ssid, password):
        if isinstance(ssid, str) and ssid and ssid not in seen:
            seen.add(ssid)
            out.append({'ssid': ssid, 'password': password or ''})

    for n in cfg.get('networks') or []:
        if isinstance(n, dict):
            add(n.get('ssid'), n.get('password'))
    add(cfg.get('ssid'), cfg.get('password'))
    if isinstance(legacy, dict):
        add(legacy.get('ssid'), legacy.get('password'))
    return out


def upsert_network(networks, ssid, password, limit=MAX_NETWORKS):
    """A new list with `ssid` FIRST (highest priority), replacing any older
    entry of the same name, capped at `limit` (the oldest fall off)."""
    out = [{'ssid': ssid, 'password': password or ''}]
    for n in networks or []:
        if isinstance(n, dict) and n.get('ssid') and n['ssid'] != ssid:
            out.append({'ssid': n['ssid'], 'password': n.get('password', '')})
    return out[:limit]


class WifiService(Service):
    name = "wifi"

    def setup(self):
        cfg = self.app.config.section('wifi')
        self._timeout_s = cfg['connect_timeout_s']
        self._retry_max_s = cfg['retry_max_s']
        self._nets = known_networks(cfg)
        if not self._nets:
            try:
                with open(cfg['legacy_creds_path']) as f:
                    self._nets = known_networks({}, json.load(f))
            except (OSError, ValueError):
                pass
        self._idx = 0                     # next network to try
        self._ssid = None                 # the one we are joined to
        if not self._nets:
            self.log.warn('no wifi credentials configured — wifi idle '
                          '(set with: deploy.sh wifi <ssid> <password>)')

        import network

        # set the system hostname BEFORE creating/activating the WLAN: the
        # firmware's built-in lwIP mDNS responder (it owns UDP 5353)
        # registers <hostname>.local when the interface comes up — setting
        # it later leaves the default (mpy-alif.local) announced forever
        self.hostname = None
        try:
            name = self.app.config.get('mdns.hostname') or ''
            if not name:
                probe = network.WLAN(network.STA_IF)
                mac = probe.config('mac')
                name = 'gridcam-%02x%02x' % (mac[-2], mac[-1])
            network.hostname(name)
            self.hostname = name
            self.log.info('hostname: %s (.local via firmware mDNS)' % name)
        except Exception as e:
            self.log.warn('hostname not set: %s' % e)

        self._wlan = network.WLAN(network.STA_IF)
        self._wlan.active(True)
        self._reconnects = 0
        self._cfg = cfg
        self._ap = None
        self._sta_failures = 0
        if self._nets:
            self.log.info('wifi: %d known network(s): %s' % (
                len(self._nets), ', '.join(n['ssid'] for n in self._nets)))

    def in_ap_mode(self):
        """True once the AP fallback owns the radio: is_connected() then
        answers True, but nothing is upstream (no NTP, no internet)."""
        return self._ap is not None

    def is_connected(self):
        try:
            if self._ap is not None:
                return True
            return self._wlan.isconnected()
        except Exception:
            return False

    def ip(self):
        try:
            if self._ap is not None:
                return self._ap.ifconfig()[0]
            return self._wlan.ifconfig()[0] if self.is_connected() else None
        except Exception:
            return None

    async def _connect_once(self, net):
        import asyncio
        with stall.section(self.app, 'wifi connect'):
            try:
                self._wlan.disconnect()   # abandon a previous failed attempt
            except Exception:
                pass
            self._wlan.connect(net['ssid'], net['password'])
        for _ in range(self._timeout_s * 2):
            if self._wlan.isconnected():
                ip = self._wlan.ifconfig()[0]
                self._ssid = net['ssid']
                self.log.info('=== IP ADDRESS: %s (network "%s") ===' %
                              (ip, net['ssid']))
                self.app.bus.publish('wifi_up', ip)
                return True
            await asyncio.sleep(0.5)
        return False

    def _start_ap(self, reason):
        """Fallback: become an access point so the dashboard stays
        reachable (and a fresh board can be provisioned) without infra."""
        import network
        ssid = self._cfg['ap_ssid']
        try:
            import machine
            uid = machine.unique_id()
            ssid = '%s-%02x%02x' % (ssid, uid[-2], uid[-1])
        except Exception:
            pass
        password = self._cfg['ap_password']
        try:
            self._wlan.active(False)      # STA off; AP owns the radio
        except Exception:
            pass
        ap = network.WLAN(network.AP_IF)
        with stall.section(self.app, 'wifi ap start'):
            ap.active(True)
        try:
            ap.config(essid=ssid, password=password)
        except (TypeError, ValueError):
            ap.config(ssid=ssid, key=password)   # newer port arg names
        self._ap = ap
        ip = ap.ifconfig()[0]
        self.log.info('=== AP MODE (%s): join "%s" pass "%s", '
                      'dashboard http://%s:8080/ ===' %
                      (reason, ssid, password, ip))
        self.app.bus.publish('wifi_up', ip)

    async def run(self):
        import asyncio
        ap_fallback = self._cfg['ap_fallback']

        if not self._nets:
            if not ap_fallback:
                return                    # nothing to do; service stays idle
            self._start_ap('no credentials')
        backoff = 2
        while True:
            if self._ap is not None or self.is_connected():
                backoff = 2
                await asyncio.sleep(5)
                self.heartbeat()
                continue
            net = self._nets[self._idx]
            self.log.info('connecting to %s (%d/%d) ...' % (
                net['ssid'], self._idx + 1, len(self._nets)))
            try:
                ok = await self._connect_once(net)
            except Exception as e:
                self.log.error('connect error: %s' % e)
                ok = False
            if not ok:
                self._reconnects += 1
                self._ssid = None
                # Next network at once; only a whole round of failures
                # counts toward the AP fallback and earns a backoff, so
                # a second network is not held behind the first's retries.
                self._idx = (self._idx + 1) % len(self._nets)
                if self._idx != 0:
                    self.heartbeat()
                    continue
                self._sta_failures += 1
                if ap_fallback and \
                        self._sta_failures >= self._cfg['ap_after_failures']:
                    self.log.warn('STA join failed %d rounds — AP fallback'
                                  % self._sta_failures)
                    try:
                        self._start_ap('join failed x%d'
                                       % self._sta_failures)
                        continue
                    except Exception as e:
                        self.log.error('AP start failed: %s' % e)
                self.log.warn('connect failed, retry in %d s' % backoff)
                await asyncio.sleep(backoff)
                backoff = min(backoff * 2, self._retry_max_s)
            else:
                self._sta_failures = 0
            self.heartbeat()

    def stats(self):
        s = {'connected': self.is_connected(), 'reconnects': self._reconnects,
             'mode': 'ap' if self._ap is not None else 'sta',
             'networks': [n['ssid'] for n in self._nets]}   # never passwords
        if s['connected']:
            s['ip'] = self.ip()
            if self._ap is None:
                s['ssid'] = self._ssid
                try:
                    s['rssi'] = self._wlan.status('rssi')
                except Exception:
                    pass
        return s
