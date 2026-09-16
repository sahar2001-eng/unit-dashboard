# UNIT — live grid camera dashboard

Live X / Y / θ from the robot's downward camera, the live video beside it,
and a Record button that saves both under a name you give it.

```
unit-rotation-dashboard/
  app.py        Flask server: pages, API, MJPEG proxy, recordings
  live.py       the two camera feeds and the recorder
  data/<id>/    one folder per recording (see below)
  settings.json camera address, serial port (written by the Settings dialog)
  _old_mock/    the previous offline analysis tool, kept for reference
```

---

## Running it

**First time**

```bash
cd unit-rotation-dashboard
python3 -m venv --system-site-packages .venv
.venv/bin/pip install -r requirements.txt
```

**Every time — just double-click `UNIT Dashboard.app`**

It starts the server if it is not already running, waits for it, and opens the
page. Click it again any time to bring the page back; it checks first and never
starts a second server. Drag it to the Dock for one-click access (the Dock
makes an alias, so the bundle stays in the project folder where it belongs).

If it cannot start, it says why in a dialog rather than failing silently — a
missing `.venv` is the usual cause, and the dialog gives the two commands that
fix it.

**Or from a terminal, if you prefer:**

```bash
cd unit-rotation-dashboard
.venv/bin/python app.py
```

Either way the page is **http://127.0.0.1:5050**. (Not 5000: macOS AirPlay
Receiver sits on that port on every Mac.) `PORT=5051` to move it.

**To stop it**, quit from the terminal with Ctrl+C, or:

```bash
pkill -f "unit-rotation-dashboard.*app.py"
```

Leaving it running is harmless — with nobody watching, it holds no camera
connection at all and just reads the USB data feed.

---

## Where the data comes from — two feeds, two links

| feed | link | what | cost to the camera |
|---|---|---|---|
| **data** | USB serial, 115200 | one CSV row per frame at the vision rate (~114 fps): `frame, tier, theta, x, y, …, fps` — the same numbers the STM32 gets | **none** — it prints them anyway |
| **video** | WiFi, `GET /stream` on port 8080 | MJPEG with the overlay drawn, 160×100 at about half the frame rate | ~3.7 ms per frame, **only while a viewer is connected** |

The serial port is found by USB ID (`37c5:16e3`), never by port number — with
the ESP32 also plugged in the numbering follows plug order. The dashboard is
the camera's **one** video client (that is the camera's design point); your
browser gets frames re-served from it.

Data does not come from `/telemetry`: that is a snapshot, and polling it is
measured to drop the camera from 118 to 85 fps.

### The camera needs to be on your WiFi for video

Out of the box it has no credentials and raises its own access point
(`gridcam-0000` / `gridcam123`, `192.168.4.1`). The data still works over USB;
the video pane says *No video* until the camera is on a network this machine
can reach.

`openmv/host/deploy.sh wifi` does this on Linux, but its port discovery is
Linux-only. On a Mac, run the same thing directly (from the `openmv/` folder,
with the camera on USB; put your own network in):

```bash
.venv/bin/mpremote connect /dev/cu.usbmodem2101 exec "
import json, os
try:
    with open('/flash/etc/config.json') as f: cfg = json.load(f)
except (OSError, ValueError): cfg = {}
w = cfg.setdefault('wifi', {}); w['enabled'] = True; w['ssid'] = 'YOUR_SSID'; w['password'] = 'YOUR_PASSWORD'
try: os.mkdir('/flash/etc')
except OSError: pass
with open('/flash/etc/config.json', 'w') as f: json.dump(cfg, f)
print('wifi credentials stored for', w['ssid'])"
```

Then **unplug the camera, wait ~10 s, plug it back in.** Do not use a soft
reset: on this board `machine.reset()` is documented to wedge USB. The
credentials are stored in plain text on the camera's flash, not anywhere in
this folder.

**Use the IP, not the `.local` name.** Both resolve, but measured on this
network (2026-09-11) `gridcam-6020.local` took **5.4 s** per request against
**0.15 s** for the IP — mDNS resolution here is slow enough to stall the video
proxy. Settings currently holds `http://192.168.1.241:8080`. The address is
DHCP, so if the camera stops answering, check its current IP (below) before
assuming anything is broken.

The camera's address in Settings defaults to `http://gridcam-6020.local:8080`.
Its hostname is in its boot log (`hostname: gridcam-XXXX`); the log is readable
from the `/Volumes/AE3/log/` mass-storage mount while the camera is on USB. If
`.local` does not resolve on your network, use the IP the log prints
(`=== IP ADDRESS: ... ===`).

---

## What the page shows

**X** — mm to the nearer junction, signed: 0 → −30 leaving one, blank across
the middle of the cell, +30 → 0 approaching the next. **Y** — mm off the tape
centreline. **θ** — tape angle in degrees.

**Fix** is which tier of the pipeline produced the frame. *Junction* means both
tapes and their crossing were found (X, Y and θ all valid). *No fix* is what
the camera reports during a commanded turn — and also when there is no grid
in view; the serial data cannot tell those apart. The charts shade those
stretches.

The three charts roll over the last 20 s (Settings → chart window). Blank
values leave gaps rather than drawing zero: zero is a real position.

---

## Recording

Type a name, press **Record**, press **Stop**. Each recording is a folder:

```
data/<id>/
  meta.json          name, notes, timing, tier counts
  frames.csv         every data row: t_s, wall_time, frame, tier, theta, x, y, n_c, n_r, mad_h, mad_v, fps
  video.mp4          H.264, playable in the browser (encoded when you press Stop)
  video_index.csv    t_s of every video frame, so the video and the data line up
```

Rows and frames go to disk as they arrive, so long recordings do not grow in
memory. If the camera was not reachable over WiFi the recording still has all
the data and simply no video.

**Alignment is by host clock, not by frame.** Both feeds are stamped on
arrival at this machine. The camera's JPEG stream carries no frame id, so a
video frame and a data row match to within a few milliseconds, not exactly.
Good enough to see what happened; not a substitute for the data itself. (If
exact matching ever matters, the fix is on the camera: put the frame id in the
MJPEG part headers.)

Opening a recording plays the video with the readout following the playhead;
click on a chart to seek. Rename by editing the title; notes save as you type.
Deleting a recording deletes its folder.

---

## Verified on hardware, 2026-09-11

Camera on WiFi (`sta` mode, RSSI −42), gridcam 1.17.0, a real grid under the lens:

| | |
|---|---|
| data | **113 rows/s** over USB, 0.1% unparsed (the CSV header) |
| video | **14–18 fps** MJPEG through the proxy, 160×100 |
| recording | 12.2 s → 1048 rows at 86/s + 172 frames at 14.1/s, **0 gaps**, 0 reconnects |
| mp4 | decodes with no errors; plays in the browser |
| alignment | the video's burned-in overlay (`X -4.0 Y +2.7 T +1.8`) matched the readout the dashboard picked for that playhead (`−4.0 / +2.7 / +1.77`) |

Note the data rate drops from ~114 to ~86 rows/s while video is streaming —
the camera is compressing JPEGs on the same loop that produces the data. That
is the documented cost of a viewer, not a fault.

## Tuning the video (Settings → Camera stream)

Every frame you watch is compressed **inside the camera's vision loop**, on a
single core (`_thread` does not exist on this port). So video is never free:
frames you watch cost data rate. The controls let you choose the point.

Measured on this rig, 2026-09-11, sole client:

| setting | video fps | data rows/s (82 = no video) |
|---|---|---|
| divisor 4 | ~14 | — |
| divisor 2, q80, 0.5 | 27.5 | 69 (84%) |
| **divisor 1, q80, 0.5** | **46.7** | 61 (74%) |
| divisor 1, q50, 0.25 | 55.3 | 69 (85%) |

Two things that are not obvious:

- **Frame rate is the big lever; picture size barely matters now.** Dropping
  160x100 to 80x50 buys only ~6 fps. (The camera's own config comments say
  resolution is the expensive knob — that was true when the 50 ms poll in
  `net/stream.py` was the real bottleneck, and stopped being true when that
  was fixed to 5 ms.)
- **The ceiling is about 55 fps**, because a streamed frame can only come
  from a vision frame, and the vision rate itself falls as you stream. The
  two meet there.

Changes apply immediately and are **not kept across a camera reboot** unless
you tick *Keep after the camera reboots*, which writes the camera's
`/flash/etc/config.json`.

## When something is off

- **Serial pill says "no camera on USB"** — nothing with USB ID `37c5:16e3` is
  attached. If the camera is there but silent, its app may not be running (it
  prints `gridcam x.y.z starting` at boot; check `/Volumes/AE3/log/app.0.log`).
- **Camera pill says "unreachable"** — the address in Settings is not
  answering on port 8080. Usually the camera is in AP mode (see above) or on a
  different network from this machine.
- **Camera pill says "idle"** — nothing is watching, so the dashboard has
  deliberately let go of the camera. It reconnects when the live pane is open
  or a recording starts.
- **Video is small / blocky** — the camera streams 160×100 by default because
  a 320×200 stream costs it 2.4× the encode time per frame. Raise
  `http.mjpeg_scale` to 1.0 in the camera's config if you want the pixels and
  can spare the frame rate.
