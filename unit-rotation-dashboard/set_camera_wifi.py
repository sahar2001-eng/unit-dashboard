#!/usr/bin/env python3
"""Store WiFi credentials on the gridcam camera, so the dashboard can reach
its video stream.

Run it yourself:  .venv/bin/python set_camera_wifi.py

It asks for the network name and password HERE, in your terminal. The
password is read with getpass (not echoed, not in your shell history) and
goes straight to the camera. It is never printed and never leaves this
machine except to the camera over USB.

On the camera it lands in /flash/etc/config.json in PLAIN TEXT -- that is how
the camera reads it, and it is the same for deploy.sh wifi. Treat the board's
filesystem accordingly.

WiFi is not reconfigurable at run time on this port: the camera brings the
network up once at boot, so the credentials take effect at the NEXT boot.
"""

import getpass
import json
import subprocess
import sys
import os

HERE = os.path.dirname(os.path.abspath(__file__))
MPREMOTE = os.path.join(HERE, ".venv", "bin", "mpremote")
OPENMV_ID = (0x37C5, 0x16E3)


def find_port():
    from serial.tools import list_ports
    hits = [p.device for p in list_ports.comports()
            if (p.vid, p.pid) == OPENMV_ID and p.device.startswith("/dev/cu.")]
    return hits[0] if hits else None


def main():
    if not os.path.exists(MPREMOTE):
        sys.exit("mpremote not found at %s\n  .venv/bin/pip install mpremote" % MPREMOTE)
    port = find_port()
    if not port:
        sys.exit("No OpenMV camera on USB (looking for 37c5:16e3). Plugged in?")
    print("camera on %s" % port)

    ssid = input("WiFi network name (SSID): ").strip()
    if not ssid:
        sys.exit("no SSID given, nothing changed")
    password = getpass.getpass("WiFi password (hidden, not echoed): ")

    # Built as JSON and passed as a literal, so a quote or a backslash in the
    # password cannot break out of the snippet. deploy.sh interpolates it into
    # triple-quoted Python, which a ''' in a password would break.
    creds = json.dumps({"enabled": True, "ssid": ssid, "password": password})
    snippet = (
        "import json, os\n"
        "try: os.mkdir('/flash/etc')\n"
        "except OSError: pass\n"
        "try:\n"
        "    with open('/flash/etc/config.json') as f: cfg = json.load(f)\n"
        "except (OSError, ValueError): cfg = {}\n"
        "cfg['wifi'] = json.loads(%r)\n"
        "with open('/flash/etc/config.json', 'w') as f: json.dump(cfg, f)\n"
        "with open('/flash/etc/config.json') as f: back = json.load(f)\n"
        "w = back.get('wifi', {})\n"
        "print('STORED', w.get('ssid'), 'enabled=%%s' %% w.get('enabled'),\n"
        "      'password_len=%%d' %% len(w.get('password', '')))\n" % creds
    )

    r = subprocess.run([MPREMOTE, "connect", port, "exec", snippet],
                       capture_output=True, text=True)
    out = (r.stdout + r.stderr).strip()
    if r.returncode != 0 or "STORED" not in out:
        print(out)
        sys.exit("\nFailed. If it says 'could not enter raw repl', the camera app owns\n"
                 "the port -- stop the dashboard first, or replug the camera and retry.")

    line = [l for l in out.splitlines() if l.startswith("STORED")][0]
    _, stored_ssid, enabled, plen = line.split(" ", 3)
    print("\nstored on the camera: ssid=%s %s %s" % (stored_ssid, enabled, plen))
    print("  (length only -- the password itself is never printed)")
    print("\nNOW: unplug the camera, wait ~10 s, plug it back in.")
    print("Do NOT soft-reset it -- machine.reset() is documented to wedge USB")
    print("on this board. A physical replug is the reliable way.")
    print("\nThe camera joins the network at boot; that can take ~1 min with")
    print("backoff. Then tell Claude and it will find the address.")


if __name__ == "__main__":
    main()
