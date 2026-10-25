# t2_runner.py -- runs the Tile 2 (UNIT16) camera pipeline over a clip.
#
# This is MicroPython, run by the `micropython` binary next to it, with the
# SAME u16pipe.py and the same C module (u16c, built for this PC) as on the
# AE3. Nothing is re-implemented: the dashboard only draws what this prints.
#
#   micropython t2_runner.py frames.raw N out.jsonl progress.json
#
# frames.raw = N gray frames, 320x200, one after the other (64000 bytes each)
# out.jsonl  = one JSON line per frame (what u16pipe.step() returned)
import sys
import json
from array import array
import u16pipe

raw_path, n, out_path, prog_path = sys.argv[1], int(sys.argv[2]), sys.argv[3], sys.argv[4]

TAB = array('i')
NAMES = []
with open('unit16_codes.csv') as f:
    for ln in f.readlines()[1:]:
        q = ln.strip().split(',')
        if len(q) >= 5:
            TAB.append(int(q[4], 16))
            NAMES.append(q[1] + q[2])

p = u16pipe.Pipe(TAB, NAMES)
FR = bytearray(u16pipe.W * u16pipe.H)


def rnd(v, d=3):
    return None if v is None else round(v, d)


def prog(done):
    with open(prog_path, 'w') as g:
        g.write('{"done": %d, "total": %d}' % (done, n))


src = open(raw_path, 'rb')
out = open(out_path, 'w')
for k in range(n):
    src.readinto(FR)
    r = p.step(FR)
    rec = {
        'k': k, 'ok': 1 if r.get('ok') else 0, 'mode': r.get('mode'),
        'lines': r.get('lines'),
        'X': rnd(r.get('X')), 'Y': rnd(r.get('Y')),
        'Xmm': rnd(r.get('Xmm')), 'Ymm': rnd(r.get('Ymm')),
        'deg': rnd(r.get('deg')), 'th': rnd(r.get('th'), 5),
        'jx': rnd(r.get('jx')), 'jy': rnd(r.get('jy')),
        'code': r.get('code'), 'raw': r.get('code_raw'), 'pres': r.get('pres'),
        'node': r.get('node'), 'xnode': r.get('xnode'),
        'ra': p.rej_angle, 'rp': p.rej_perp, 'rj': p.rej_jump, 'rpl': p.rej_plain,
        'cf': p.conflicts, 'se': p.searches,
    }
    out.write(json.dumps(rec))
    out.write('\n')
    if k % 25 == 0:
        prog(k)
out.close()
prog(n)
