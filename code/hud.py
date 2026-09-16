# hud.py -- live on-camera overlay for the TURN phase.
#
# API NOTE: OpenMV v5 / MicroPython 1.28 takes a TUPLE as the first argument
# of every draw_* call:
#       img.draw_line((x0, y0, x1, y1), color=...)
#       img.draw_rectangle((x, y, w, h), color=..., fill=True)
#       img.draw_cross((x, y), color=..., size=...)
#       img.draw_string((x, y), text, color=...)
# Passing loose ints raises "TypeError: object 'int' isn't a tuple or list"
# on every frame. This matches what v1_1 already does.
#
# Colours match v1_1: white for the X bar, dim grey for the Y bar.

import math

# 0 = nothing, 1 = text only, 2 = text + gauge, 3 = everything (lines too)
HUD_LEVEL = 3

COLOR_X = (255, 255, 255)    # X line -- white, same as v1_1
COLOR_Y = (90, 90, 90)       # Y line -- dim grey, same as v1_1
COLOR_J = (255, 255, 255)
COLOR_TXT = (255, 255, 255)
COLOR_DIM = (90, 90, 90)
COLOR_BG = (0, 0, 0)


def _rot(cx, cy, ux, uy, vx, vy, al, be):
    return int(cx + al * ux + be * vx), int(cy + al * uy + be * vy)


def draw_tape_lines(img, W, H, theta_deg, xline=None, yline=None,
                    junction=None):
    """xline/yline are (a, b) in the rotated frame; junction is (al, be)."""
    if HUD_LEVEL < 3:
        return
    r = math.radians(theta_deg)
    ux, uy = math.cos(r), math.sin(r)
    vx, vy = -uy, ux
    cx, cy = W * 0.5, H * 0.5
    half = W + H

    if xline is not None:
        a, b = xline
        p = _rot(cx, cy, ux, uy, vx, vy, -half, a + b * (-half))
        q = _rot(cx, cy, ux, uy, vx, vy, half, a + b * half)
        img.draw_line((p[0], p[1], q[0], q[1]), color=COLOR_X)

    if yline is not None:
        a, b = yline
        p = _rot(cx, cy, ux, uy, vx, vy, a + b * (-half), -half)
        q = _rot(cx, cy, ux, uy, vx, vy, a + b * half, half)
        img.draw_line((p[0], p[1], q[0], q[1]), color=COLOR_Y)

    if junction is not None:
        jx, jy = _rot(cx, cy, ux, uy, vx, vy, junction[0], junction[1])
        if 0 <= jx < W and 0 <= jy < H:
            img.draw_cross((jx, jy), color=COLOR_J, size=8)


def draw_gauge(img, W, H, heading, rate):
    """Turn gauge, top-right. Grey needle = turn start, white = now."""
    if HUD_LEVEL < 2:
        return
    R = 22
    ox, oy = W - R - 8, R + 8

    img.draw_rectangle((ox - R - 3, oy - R - 3, 2 * R + 6, 2 * R + 6),
                       color=COLOR_BG, fill=True)

    for k in range(4):
        a = math.radians(90 * k)
        s, c = math.sin(a), math.cos(a)
        img.draw_line((int(ox + (R - 4) * s), int(oy - (R - 4) * c),
                       int(ox + R * s), int(oy - R * c)), color=COLOR_DIM)

    span = heading
    if span > 359.0:
        span = 359.0
    elif span < -359.0:
        span = -359.0
    if abs(span) > 1.0:
        rr = R - 9
        n = int(abs(span) / 12) + 2
        if n > 16:
            n = 16
        px = -1
        py = -1
        for i in range(n + 1):
            a = math.radians(span * i / n)
            qx = int(ox + rr * math.sin(a))
            qy = int(oy - rr * math.cos(a))
            if px >= 0:
                img.draw_line((px, py, qx, qy), color=COLOR_X)
            px = qx
            py = qy

    img.draw_line((ox, oy, ox, oy - (R - 5)), color=COLOR_DIM)
    a = math.radians(heading)
    img.draw_line((ox, oy,
                   int(ox + (R - 5) * math.sin(a)),
                   int(oy - (R - 5) * math.cos(a))), color=COLOR_X)

    img.draw_string((ox - 20, oy + R + 2), "%+.1f" % heading, color=COLOR_TXT)


def draw_panel(img, W, H, lines):
    if HUD_LEVEL < 1:
        return
    n = len(lines)
    y0 = H - 4 - 10 * n
    img.draw_rectangle((2, y0 - 2, 150, 10 * n + 4),
                       color=COLOR_BG, fill=True)
    for i in range(n):
        img.draw_string((5, y0 + 10 * i), lines[i], color=COLOR_TXT)


def draw_turn(img, W, H, state_name, theta, phi, omega, tier=0, fit=None):
    """One call for the whole TURN overlay."""
    if HUD_LEVEL < 1:
        return
    if fit is not None:
        draw_tape_lines(img, W, H, theta,
                        fit.get("xline"), fit.get("yline"),
                        fit.get("junction"))
    draw_gauge(img, W, H, phi, omega)
    tier_txt = ("M1 only", "junction", "mini", "one bar")[tier]
    draw_panel(img, W, H, ["turn %+.1f w%+.0f" % (phi, omega),
                           "tape %+.1f (%s)" % (theta, tier_txt),
                           state_name])
