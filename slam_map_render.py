#!/usr/bin/env python3
"""对持久地图做墙体增强 + 红墙着色，输出 PNG（供网页直接显示）。

用法:  python3 slam_map_render.py <name> [out.png]
输出:  /home/unitree/hu/go2/slam_maps/<name>_render.png
"""
import json
import os
import struct
import sys
import zlib

import numpy as np

SLAM_DIR = "/home/unitree/hu/go2/slam_maps"


def read_pgm(path):
    with open(path, "rb") as f:
        data = f.read()
    tokens, i = [], 0
    while len(tokens) < 4:
        while i < len(data) and data[i:i + 1].isspace():
            i += 1
        if i < len(data) and data[i:i + 1] == b"#":
            while i < len(data) and data[i:i + 1] != b"\n":
                i += 1
            continue
        s = i
        while i < len(data) and not data[i:i + 1].isspace():
            i += 1
        if s == i:
            break
        tokens.append(data[s:i])
    w, h = int(tokens[1]), int(tokens[2])
    i += 1
    return w, h, np.frombuffer(data[i:i + w * h], dtype=np.uint8).reshape(h, w)


def enhance(occ):
    """去小噪点 -> 闭运算补墙缺口 -> 加粗 1px。"""
    try:
        from scipy import ndimage
    except ImportError:
        return occ
    lab, n = ndimage.label(occ, structure=np.ones((3, 3)))
    if n:
        sizes = np.bincount(lab.ravel())
        keep = np.zeros(n + 1, dtype=bool)
        keep[1:] = sizes[1:] >= 6
        occ = keep[lab]
    occ = ndimage.binary_closing(occ, structure=np.ones((3, 3)), iterations=1)
    occ = ndimage.binary_dilation(occ, structure=np.ones((3, 3)), iterations=1)
    return occ


def render(name):
    pgm = os.path.join(SLAM_DIR, name + ".pgm")
    if not os.path.isfile(pgm):
        raise SystemExit(f"no such map: {name}")
    w, h, arr = read_pgm(pgm)
    occ_raw = arr < 60
    free_raw = arr > 200

    occ = enhance(occ_raw)
    try:
        from scipy import ndimage
        free = ndimage.binary_opening(free_raw, structure=np.ones((3, 3)))
        inner = ndimage.binary_erosion(occ, structure=np.ones((3, 3)), iterations=1)
        edge = occ & ~inner
    except ImportError:
        free = free_raw
        edge = occ
    free = free & ~occ

    out = np.zeros((h, w, 4), dtype=np.uint8)
    out[free] = (28, 40, 60, 170)      # 可通行：深蓝灰半透明
    out[occ] = (200, 60, 60, 230)      # 墙体：红
    out[edge] = (255, 140, 110, 255)   # 墙体轮廓：亮橙

    raw = bytearray()
    for y in range(h):
        raw.append(0)
        raw += out[y].tobytes()

    def chunk(tag, payload):
        c = struct.pack(">I", len(payload)) + tag + payload
        return c + struct.pack(">I", zlib.crc32(tag + payload) & 0xFFFFFFFF)

    png = b"\x89PNG\r\n\x1a\n"
    png += chunk(b"IHDR", struct.pack(">IIBBBBB", w, h, 8, 6, 0, 0, 0))
    png += chunk(b"IDAT", zlib.compress(bytes(raw), 9))
    png += chunk(b"IEND", b"")
    outp = os.path.join(SLAM_DIR, name + "_render.png")
    with open(outp, "wb") as f:
        f.write(png)
    # 同时给一份基础轮廓（只有墙，无地面）便于叠加
    stats = {"name": name, "w": w, "h": h,
             "occ_raw": int(occ_raw.sum()), "occ_enhanced": int(occ.sum()),
             "free": int(free.sum()), "png": outp}
    print(json.dumps(stats))
    return png


if __name__ == "__main__":
    render(sys.argv[1] if len(sys.argv) > 1 else "test")
