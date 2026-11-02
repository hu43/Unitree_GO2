"""Pure-numpy map styling (runs inside go2_humble container, no scipy/PIL).

Renders <name>.pgm into a styled RGBA PNG:
  - walls bright red, small speckles removed
  - observed free space slightly blue
  - visited (footprint) area a touch lighter than the canvas background
  - unexplored area transparent
"""
import math
import os
import struct
import zlib

import numpy as np

OCC_MAX = 128      # PGM gray < OCC_MAX -> occupied
FREE_MIN = 251     # PGM gray > FREE_MIN -> free (205 = unknown)

# colors (r, g, b, a)
C_WALL = (228, 74, 74, 255)
C_FREE = (34, 64, 94, 255)
C_FOOT = (19, 29, 45, 255)

DENOISE_WIN = 5
DENOISE_MIN = 4
CLOSE_ITER = 1
FOOT_STEPS = 7     # footprint closing radius in cells (7 -> 15x15)


def _shift_or(m, dy, dx):
    return np.roll(np.roll(m, dy, axis=0), dx, axis=1)


def _shift_and(m, dy, dx):
    return np.roll(np.roll(m, dy, axis=0), dx, axis=1)


def _count_window(m, win):
    """count of True in a win x win window centered on each cell."""
    cnt = np.zeros(m.shape, dtype=np.uint8)
    half = win // 2
    for dy in range(-half, half + 1):
        for dx in range(-half, half + 1):
            cnt += _shift_or(m, dy, dx).astype(np.uint8)
    return cnt


def denoise(m, win=DENOISE_WIN, mincnt=DENOISE_MIN):
    """Drop isolated specks: keep a wall cell only if the win-window around
    it holds at least `mincnt` wall cells."""
    return m & (_count_window(m, win) >= mincnt)


def dilate(m, it=1):
    out = m
    for _ in range(it):
        acc = out.copy()
        for dy in (-1, 0, 1):
            for dx in (-1, 0, 1):
                if dy == 0 and dx == 0:
                    continue
                acc |= _shift_or(out, dy, dx)
        out = acc
    return out


def erode(m, it=1):
    out = m
    for _ in range(it):
        acc = out.copy()
        for dy in (-1, 0, 1):
            for dx in (-1, 0, 1):
                if dy == 0 and dx == 0:
                    continue
                acc &= _shift_and(out, dy, dx)
        out = acc
    return out


def closing(m, it=1):
    return erode(dilate(m, it), it)


def fill_holes(m):
    """Flood the outside from the border; whatever is not outside is filled."""
    outside = np.zeros_like(m)
    outside[0, :] = outside[-1, :] = True
    outside[:, 0] = outside[:, -1] = True
    outside &= ~m
    prev = None
    while prev is None or not np.array_equal(prev, outside):
        prev = outside
        grown = outside.copy()
        grown[1:, :] |= outside[:-1, :]
        grown[:-1, :] |= outside[1:, :]
        grown[:, 1:] |= outside[:, :-1]
        grown[:, :-1] |= outside[:, 1:]
        outside = grown & ~m
    return m | ~outside


def footprint(known):
    """Visited-area silhouette: heavy closing + hole filling."""
    st = 2 * FOOT_STEPS + 1
    acc = known.copy()
    # square structuring element via two 1-D passes is not equivalent, so do
    # the full win x win dilation (win is small, this is a one-off render)
    half = FOOT_STEPS
    dil = known.copy()
    for dy in range(-half, half + 1):
        for dx in range(-half, half + 1):
            dil |= _shift_or(known, dy, dx)
    ero = dil.copy()
    for dy in range(-half, half + 1):
        for dx in range(-half, half + 1):
            ero &= _shift_and(dil, dy, dx)
    del acc, st
    return fill_holes(ero)


def style_png(pgm_bytes, w, h):
    """gray pgm bytes -> (png_bytes, w, h) styled RGBA."""
    a = np.frombuffer(pgm_bytes, dtype=np.uint8).reshape(h, w)
    occ = a < OCC_MAX
    free = a > FREE_MIN
    known = occ | free

    occ_d = denoise(occ)
    occ_c = closing(occ_d, CLOSE_ITER)
    free_c = closing(free, 1)
    foot = footprint(known) & ~free_c & ~occ_c

    img = np.zeros((h, w, 4), dtype=np.uint8)
    img[foot] = C_FOOT
    img[free_c] = C_FREE
    img[occ_c] = C_WALL

    raw = bytearray()
    for y in range(h):
        raw.append(0)                       # PNG filter: none
        raw += img[y].tobytes()

    def chunk(tag, payload):
        c = struct.pack(">I", len(payload)) + tag + payload
        return c + struct.pack(">I", zlib.crc32(tag + payload) & 0xFFFFFFFF)

    png = b"\x89PNG\r\n\x1a\n"
    png += chunk(b"IHDR", struct.pack(">IIBBBBB", w, h, 8, 6, 0, 0, 0))
    png += chunk(b"IDAT", zlib.compress(bytes(raw), 6))
    png += chunk(b"IEND", b"")
    return png


def render_map_file(slam_dir, name):
    """Read <slam_dir>/<name>.pgm, return (png_bytes, w, h) or None."""
    path = os.path.join(slam_dir, name + ".pgm")
    try:
        with open(path, "rb") as f:
            data = f.read()
    except OSError:
        return None
    # minimal P5 parser (header may contain comments)
    toks, i = [], 0
    while len(toks) < 4 and i < len(data):
        ch = data[i:i + 1]
        if ch.isspace():
            i += 1
        elif ch == b"#":
            while i < len(data) and data[i:i + 1] != b"\n":
                i += 1
        else:
            j = i
            while j < len(data) and not data[j:j + 1].isspace():
                j += 1
            toks.append(data[i:j])
            i = j
    if len(toks) < 4 or toks[0] != b"P5":
        return None
    w, h = int(toks[1]), int(toks[2])
    i += 1  # single whitespace after maxval
    body = data[i:i + w * h]
    if len(body) < w * h:
        return None
    try:
        png = style_png(body, w, h)
    except Exception:
        return None
    return png, w, h
