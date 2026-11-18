# On-frame annotation for the live view / MJPEG stream — the POC's overlay.
#
# Firmware v5 drawing API: coordinates are passed as TUPLES
# (img.draw_line((x0, y0, x1, y1)), img.draw_cross((x, y))) — separate int
# arguments raise TypeError.
#
# `scale` exists because the stream encodes a downscaled copy
# (http.mjpeg_scale). Draw on the SMALL image with scaled geometry rather
# than drawing at full size and shrinking afterwards: the font is a fixed 8
# px, so shrinking a full-size annotation by 2x renders the readout
# illegible — which defeats the point of the overlay. Line coordinates take
# the scale; text keeps its native size and its top-left anchor.
#
# `lines` (grid lines + crosses) and `text` (the readout) are switched by
# http.overlay_lines / http.overlay_text. Display only: the measurement is
# never drawn on.

COLOR_X = 255      # X line (horizontal bar) — white
COLOR_Y = 90       # Y line (vertical bar) — dim grey, tellable apart
COLOR_MARK = 255   # cross, ticks, text

# Reason words in the quality strip, as the bench draws them
# (bench quality._STRIP_WORDS); any other reason is shown with spaces.
STRIP_WORDS = {'missed_junction': 'no junction',
               'code_unreadable': 'no code read'}


def _fmt(v):
    return '--' if v is None else '%+.1f' % v


def draw(img, res, w, h, scale=1.0, lines=True, text=True):
    s = scale
    if lines:
        _draw_lines(img, res, w, h, s)
    if text:
        _draw_text(img, res, w, h, s)


def _draw_lines(img, res, w, h, s):
    lx, ly = res['line_x'], res['line_y']
    if lx is not None:
        b, a = lx                                   # y = a + b*x
        img.draw_line((0, int(a * s + 0.5),
                       int((w - 1) * s), int((a + b * (w - 1)) * s + 0.5)),
                      color=COLOR_X)
    if ly is not None:
        b, a = ly                                   # x = a + b*y
        img.draw_line((int(a * s + 0.5), 0,
                       int((a + b * (h - 1)) * s + 0.5), int((h - 1) * s)),
                      color=COLOR_Y)

    node = res['node']
    if node is not None:
        img.draw_cross((int(node[0] * s + 0.5), int(node[1] * s + 0.5)),
                       color=COLOR_MARK, size=8)
        # Tile 2: the junction's code (node number + E/W/S/N), beside it as
        # the bench draws it (tile2.Tile2._draw_lines)
        code = res.get('t2_code')
        if code:
            d = max(2, int(6 * s))          # the bench's +6 px, scaled
            img.draw_string((int(node[0] * s) + d, int(node[1] * s) + d),
                            code, color=COLOR_MARK)

    cx, cy = res['centre']
    img.draw_cross((int(cx * s), int(cy * s)), color=128, size=10)


def _draw_text(img, res, w, h, s):
    tier = res['tier']
    if tier is None:
        if 't2_code' in res:
            # Tile 2: the Pipe's mode tells why; its threshold is not Otsu
            txt = 'no marker  %s' % res['status']
        else:
            txt = 'no marker  otsu%d' % res['threshold']
    else:
        tag = tier
        if res['suspect']:
            tag += '!'
        txt = 'X%s Y%s T%s (%s)' % (_fmt(res['x_mm']), _fmt(res['y_mm']),
                                    _fmt(res['theta_deg']), tag)
    img.draw_string((2, 2), txt, color=COLOR_MARK)

    q = res.get('q')                # quality score, when it is on
    if q is not None:
        draw_quality(img, q, res.get('q_why') or '', int(w * s), int(h * s))


def draw_quality(img, q, why, W, H):
    """Bottom strip: a bar as long as q, a big 'Q96', and the level word and
    the reason beside it.

    The stream is grayscale, so the level is told by the word and the bar
    length, not by colour; the web page adds the colour badge. The font is
    8x10 px, so 'Q96' at scale 2 is 48x20 and the strip is 22 px tall."""
    from .quality import level
    t = 'Q%d' % q
    img.draw_rectangle((0, H - 22, W, 22), color=0, fill=True)
    bar = (W * q) // 100
    if bar > 0:                         # a zero-width rectangle is not drawn
        img.draw_rectangle((0, H - 22, bar, 2), color=255, fill=True)
    img.draw_string((2, H - 20), t, color=255, scale=2)
    x = 6 + 16 * len(t)
    n = (W - x) // 8                    # characters that fit
    img.draw_string((x, H - 20), level(q)[:n], color=230)
    if why:
        w = STRIP_WORDS.get(why) or why.replace('_', ' ')
        img.draw_string((x, H - 10), w[:n], color=170)
