# Grid Localization Analytics — Setup

A local tool for reviewing the robot's grid-localization runs. Upload a clip
from the downward camera and it runs the robot's own camera code over it,
producing an annotated video plus graphs. Everything runs on your own machine — nothing is uploaded anywhere.

You only do the **first-time setup** once. After that, starting it is two lines.

---

## First time only

### 1. Install Python (skip if you already have it)

**Mac** — open Terminal (press ⌘+Space, type "Terminal", Enter) and run:

    python3 --version

If it prints a version number (3.9 or higher), you're set. If it says
"command not found", install from https://www.python.org/downloads/ — download
the macOS installer, run it, then close and reopen Terminal.

**Windows** — open PowerShell (press the Start key, type "PowerShell", Enter)
and run:

    python --version

If it prints a version number (3.9 or higher), you're set. If not, install
from https://www.python.org/downloads/ — download the Windows installer, run
it, and **tick "Add Python to PATH"** on the first screen. Then close and
reopen PowerShell.

### 2. Put the folder somewhere you'll find it

Unzip the folder you were sent (e.g. into Documents or Desktop). Remember where
it is.

### 3. Install the tool's requirements

Point the terminal at the folder, then install. Replace the path with wherever
you actually put it.

**Mac:**

    cd ~/Desktop/unit-rotation-dashboard
    pip3 install -r requirements.txt

**Windows:**

    cd $HOME\Desktop\unit-rotation-dashboard
    pip install -r requirements.txt

Tip: instead of typing the path, type `cd ` (with a space) and then drag the
folder from Finder / File Explorer onto the terminal window — it fills in the
path for you.

This step downloads a few packages and takes a minute. You only do it once.

---

## Every time you want to use it

**Mac:**

    cd ~/Desktop/unit-rotation-dashboard
    python3 app.py

**Windows:**

    cd $HOME\Desktop\unit-rotation-dashboard
    python app.py

You'll see a line like:

    UNIT rotation analytics -> http://127.0.0.1:5000

Open that address in your browser (Chrome or Safari). That's the tool.

**Leave the terminal window open** while you use it. To stop the tool, click
the terminal and press Ctrl+C, or just close the window.

---

## Using it

1. Click **Analyse a video** (or drag a clip onto the drop zone).
2. Pick the **use case** the clip was recorded for (optional, can be set later).
3. If the clip has a turn, enter the **frame the host sent TURN on** (and
   DRIVE, if it was sent). Leave empty for a driving-only clip.
4. Wait while it runs the robot's code over every frame — a progress bar
   shows the frame count.
5. The annotated video and the graphs appear. Click or drag on a graph to
   jump the video to that moment. Click the small **i** on any graph for an
   explanation.
6. **Use cases** in the header lists every case with how many runs it has;
   click one to show only those runs.

Runs are saved in the sidebar and stay there next time you open the tool.
Each run can be renamed, re-tagged, annotated with notes, re-run with a
different turn command, and its video / CSV / summary / terminal output
downloaded.

---

## Common snags

**"Port 5000 is in use"** — something else is using that port (on Mac it's
often AirPlay). Start it on a different port instead:

    Mac:      PORT=5001 python3 app.py
    Windows:  $env:PORT=5001; python app.py

Then open http://127.0.0.1:5001 instead.

**"command not found: cd" or the path is wrong** — you're not pointing at the
right folder. Type `cd ` then drag the folder onto the terminal, as above.

**The page won't load / "access denied"** — try `http://localhost:5000`
instead of `127.0.0.1`, or open it in a private/incognito window (a browser
extension is sometimes the cause).

**Nothing happens after `python app.py`** — make sure you did the one-time
`pip install -r requirements.txt` step first, and that you're inside the
folder (the terminal prompt should show the folder name).

---

## What version am I running?

The bottom-right corner of the page shows the engine version. If a colleague
sends an updated folder, delete your old one, unzip the new one in its place,
and check that number changed.
