#!/usr/bin/env python3
"""视觉/雷达感知：色块检测与视觉伺服、人体跟随、雷达净空。

摄像头帧由主机侧 go2-cam.service 用 VideoClient.GetImageSample() 写进
web/cam/latest.jpg（容器只挂载了 ~/hu/go2，看不到 SDK 与 ~/hu/ai）。
色块和人体检测用容器自带的 cv2 4.5.4，闭环控制放在这里用确定性代码做，
LLM 只负责"决定调用哪个技能"，不参与控制环。
"""

import base64
import logging
import math
import os
import threading
import time

import cv2
import numpy as np

log = logging.getLogger("go2web.perception")

CAM_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "cam")
FRAME_MAX_AGE = 2.5      # 秒；超过就认为相机链路挂了
YOLOX_MODEL = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                           "models", "yolox_s.onnx")
YOLOX_SIZE = 416          # 416 比 640 快约 2.3 倍（Orin CPU 0.26s/帧）
YOLOX_CONF = 0.30
YOLOX_NMS = 0.45
PERSON_CLASS = 0         # COCO person
# Go2 固件对小于约 0.15 m/s 的平移基本不响应（实测 0.1m/s 1.5s 只走 1mm），
# 所以决定要动时给一个最小速度，否则会“只转向不走”。
MIN_VX = 0.20

# 最新一帧 JPEG 缓存在内存里（由 app 的相机线程 5Hz 刷新）；
# 兼容旧的主机侧 cam/latest.jpg 文件，便于排查。
_latest = {"jpeg": None, "ts": 0.0}
_latest_lock = threading.Lock()


def set_frame(jpeg):
    with _latest_lock:
        _latest["jpeg"] = jpeg
        _latest["ts"] = time.time()


def latest_jpeg(max_age=FRAME_MAX_AGE):
    """返回 (jpeg_bytes, age_s)；没有可用帧返回 (None, age)。"""
    with _latest_lock:
        j, ts = _latest["jpeg"], _latest["ts"]
    if j:
        age = time.time() - ts
        if age <= max_age:
            return j, age
        return None, age
    path = os.path.join(CAM_DIR, "latest.jpg")
    if os.path.isfile(path):
        age = time.time() - os.path.getmtime(path)
        if age <= max_age:
            try:
                with open(path, "rb") as f:
                    return f.read(), age
            except OSError:
                pass
        return None, age
    return None, 1e9

# HSV 色相范围（OpenCV: H 0-180）。红色跨 0 需要两段。
COLOR_RANGES = {
    "blue":   [((100, 90, 60), (130, 255, 255))],
    "green":  [((40, 70, 50), (85, 255, 255))],
    "red":    [((0, 90, 60), (10, 255, 255)), ((170, 90, 60), (180, 255, 255))],
    "yellow": [((20, 90, 80), (35, 255, 255))],
    "orange": [((10, 100, 80), (20, 255, 255))],
    "purple": [((130, 60, 50), (165, 255, 255))],
    "cyan":   [((85, 80, 60), (100, 255, 255))],
    "white":  [((0, 0, 180), (180, 40, 255))],
    "black":  [((0, 0, 0), (180, 255, 55))],
}
COLOR_CN = {
    "blue": "蓝色", "green": "绿色", "red": "红色", "yellow": "黄色",
    "orange": "橙色", "purple": "紫色", "cyan": "青色",
    "white": "白色", "black": "黑色",
}


def color_cn(name):
    return COLOR_CN.get((name or "").lower(), name or "")


def load_frame(max_age=FRAME_MAX_AGE, resize_width=640):
    """读最新相机帧，返回 (BGR image, age) 或 (None, age)。"""
    jpeg, age = latest_jpeg(max_age=max_age)
    if jpeg is None:
        return None, age
    img = cv2.imdecode(np.frombuffer(jpeg, np.uint8), cv2.IMREAD_COLOR)
    if img is None:
        return None, age
    if resize_width and img.shape[1] > resize_width:
        scale = resize_width / float(img.shape[1])
        img = cv2.resize(img, (resize_width, int(img.shape[0] * scale)),
                         interpolation=cv2.INTER_AREA)
    return img, age


def frame_b64(max_age=FRAME_MAX_AGE, resize_width=768):
    """给 VLM 用的 base64 JPEG（缩到 768 宽，Jetson 上推理快很多）。"""
    img, age = load_frame(max_age=max_age, resize_width=resize_width)
    if img is None:
        return None, age
    ok, buf = cv2.imencode(".jpg", img, [int(cv2.IMWRITE_JPEG_QUALITY), 82])
    if not ok:
        return None, age
    return base64.b64encode(buf.tobytes()).decode(), age


def detect_color(img, color, min_area_ratio=0.0015):
    """返回最大色块 {cx, cy, w, h, area_ratio, bbox}（归一化），没有则 None。"""
    if img is None:
        return None
    ranges = COLOR_RANGES.get((color or "").lower())
    if not ranges:
        return None
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    mask = None
    for lo, hi in ranges:
        m = cv2.inRange(hsv, np.array(lo, np.uint8), np.array(hi, np.uint8))
        mask = m if mask is None else cv2.bitwise_or(mask, m)
    k = np.ones((5, 5), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, k)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, k)
    cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not cnts:
        return None
    c = max(cnts, key=cv2.contourArea)
    area = float(cv2.contourArea(c))
    H, W = img.shape[:2]
    if area / float(W * H) < min_area_ratio:
        return None
    x, y, w, h = cv2.boundingRect(c)
    return {
        "cx": round((x + w / 2.0) / W, 4),
        "cy": round((y + h / 2.0) / H, 4),
        "w": round(w / float(W), 4),
        "h": round(h / float(H), 4),
        "area_ratio": round(area / float(W * H), 4),
        "bbox": [int(x), int(y), int(w), int(h)],
    }


class YoloPersonDetector:
    """YOLOX-s ONNX（OpenCV Zoo）行人检测。

    机器狗摄像头很低，人靠近时画面里只有腿；HOG 全身检测器在这种视角下
    基本失效。YOLOX 对局部身体、遮挡都更稳。

    预处理/后处理严格按 opencv_zoo 的 demo：letterbox 到 416（灰边 114）、
    BGR->RGB、保持 0-255 不归一化；输出按 stride 解码后再做 NMS。
    """

    STRIDES = (8, 16, 32)

    def __init__(self, path=YOLOX_MODEL, size=YOLOX_SIZE,
                 conf=YOLOX_CONF, nms=YOLOX_NMS):
        self._net = cv2.dnn.readNetFromONNX(path)
        self._net.setPreferableBackend(cv2.dnn.DNN_BACKEND_OPENCV)
        self._net.setPreferableTarget(cv2.dnn.DNN_TARGET_CPU)
        self._size = int(size)
        self._conf = float(conf)
        self._nms = float(nms)
        self._grids, self._strides = self._anchors(self._size)

    @classmethod
    def _anchors(cls, size):
        grids, strides = [], []
        for s in cls.STRIDES:
            hs, ws = size // s, size // s
            xv, yv = np.meshgrid(np.arange(hs), np.arange(ws))
            grid = np.stack((xv, yv), 2).reshape(1, -1, 2).astype(np.float32)
            grids.append(grid)
            strides.append(np.full((1, grid.shape[1], 1), s, dtype=np.float32))
        return np.concatenate(grids, 1), np.concatenate(strides, 1)

    def _letterbox(self, rgb):
        h, w = rgb.shape[:2]
        ratio = min(self._size / float(h), self._size / float(w))
        nw, nh = int(w * ratio), int(h * ratio)
        resized = cv2.resize(rgb, (nw, nh), interpolation=cv2.INTER_LINEAR)
        padded = np.full((self._size, self._size, 3), 114.0, dtype=np.float32)
        padded[:nh, :nw] = resized.astype(np.float32)
        return padded, ratio

    def detect(self, img):
        if img is None:
            return None
        H, W = img.shape[:2]
        rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        padded, ratio = self._letterbox(rgb)
        blob = np.transpose(padded, (2, 0, 1))[np.newaxis].astype(np.float32)
        self._net.setInput(blob)
        out = self._net.forward()
        dets = np.array(out[0], dtype=np.float32)      # (N, 85)
        if dets.ndim != 2 or dets.shape[1] < 6:
            return None
        g = self._grids[0]
        st = self._strides[0]
        dets[:, :2] = (dets[:, :2] + g) * st
        dets[:, 2:4] = np.exp(dets[:, 2:4]) * st
        boxes_cxcywh = dets[:, :4]
        scores_all = dets[:, 4:5] * dets[:, 5:]
        cls = np.argmax(scores_all, axis=1)
        scores = scores_all[np.arange(cls.shape[0]), cls]
        keep = (cls == PERSON_CLASS) & (scores >= self._conf)
        if not np.any(keep):
            return None
        b = boxes_cxcywh[keep] / ratio
        s = scores[keep]
        boxes = [[float(x - w / 2), float(y - h / 2), float(w), float(h)]
                 for x, y, w, h in b]
        idxs = cv2.dnn.NMSBoxes(boxes, s.astype(float).tolist(),
                                self._conf, self._nms)
        if idxs is None or len(idxs) == 0:
            return None
        idxs = np.array(idxs).flatten()
        i = max(idxs, key=lambda k: boxes[k][2] * boxes[k][3])
        x, y, w, h = boxes[i]
        x = max(0.0, min(W - 1.0, x))
        y = max(0.0, min(H - 1.0, y))
        w = min(w, W - x)
        h = min(h, H - y)
        if w <= 1 or h <= 1:
            return None
        return {
            "cx": round((x + w / 2.0) / W, 4),
            "cy": round((y + h / 2.0) / H, 4),
            "w": round(w / W, 4),
            "h": round(h / H, 4),
            "score": round(float(s[i]), 3),
            "detector": "yolox",
        }


class PersonDetector:
    """优先 YOLOX；模型缺失时退回 HOG。"""

    def __init__(self):
        self._yolo = None
        self._hog = None
        if os.path.isfile(YOLOX_MODEL):
            try:
                self._yolo = YoloPersonDetector()
                log.info("person detector: YOLOX (%s)", YOLOX_MODEL)
            except Exception as e:
                log.warning("YOLOX init failed, fallback to HOG: %s", e)
        if self._yolo is None:
            self._hog = cv2.HOGDescriptor()
            self._hog.setSVMDetector(cv2.HOGDescriptor_getDefaultPeopleDetector())
            log.info("person detector: HOG")

    def detect(self, img):
        if img is None:
            return None
        if self._yolo is not None:
            return self._yolo.detect(img)
        H, W = img.shape[:2]
        small = cv2.resize(img, (320, 180), interpolation=cv2.INTER_AREA)
        gray = cv2.cvtColor(small, cv2.COLOR_BGR2GRAY)
        try:
            boxes, weights = self._hog.detectMultiScale(
                gray, winStride=(8, 8), padding=(8, 8), scale=1.08)
        except Exception:
            return None
        if len(boxes) == 0:
            return None
        best = max(zip(boxes, weights.flatten() if hasattr(weights, "flatten") else weights),
                   key=lambda t: t[0][2] * t[0][3])
        (bx, by, bw, bh), score = best
        if float(score) < 0.25:
            return None
        return {
            "cx": round((bx + bw / 2.0) / 320.0, 4),
            "cy": round((by + bh / 2.0) / 180.0, 4),
            "w": round(bw / 320.0, 4),
            "h": round(bh / 180.0, 4),
            "score": round(float(score), 3),
            "detector": "hog",
        }


_person_detector = None


def person_detector():
    global _person_detector
    if _person_detector is None:
        _person_detector = PersonDetector()
    return _person_detector


def _clamp(v, lo, hi):
    return max(lo, min(hi, v))


def _front_blocked(bridge, limit=0.35):
    lidar = getattr(bridge, "lidar", None) or {}
    d = lidar.get("front")
    return d is not None and d < limit


def _stop(bridge):
    try:
        bridge.send_move(0.0, 0.0, 0.0)
        bridge.send_sport(1003)  # SPORT_STOPMOVE
    except Exception:
        pass


async def servo_to_color(bridge, color, max_s=40.0, speed=0.25,
                         stop_area=0.06, cancel=None):
    """走到指定颜色物体前并停下。确定性闭环，LLM 不参与。

    返回 {"reached": bool, "reason": str, "seconds": float, "last": dict}
    """
    import asyncio
    t0 = time.monotonic()
    scan_dir = 1.0
    lost_since = None
    last = None
    frames = 0
    try:
        while time.monotonic() - t0 < max_s:
            if cancel is not None and cancel.is_set():
                return {"reached": False, "reason": "cancelled",
                        "seconds": round(time.monotonic() - t0, 1), "last": last}
            img, age = load_frame()
            if img is None:
                return {"reached": False,
                        "reason": "相机无新帧（age=%.1fs），检查 go2-cam.service" % age,
                        "seconds": round(time.monotonic() - t0, 1), "last": last}
            det = detect_color(img, color)
            frames += 1
            if det is None:
                last = None
                if lost_since is None:
                    lost_since = time.monotonic()
                # 先原地扫视找人；扫了 8s 还看不到就放弃
                if time.monotonic() - lost_since < 8.0:
                    bridge.send_move(0.0, 0.0, 0.45 * scan_dir)
                else:
                    return {"reached": False,
                            "reason": "视野里没有找到%s物体" % color_cn(color),
                            "seconds": round(time.monotonic() - t0, 1), "last": None}
                await asyncio.sleep(0.12)
                continue
            lost_since = None
            last = det
            err_x = (det["cx"] - 0.5) * 2.0            # -1 左 .. +1 右
            vyaw = _clamp(-1.3 * err_x, -0.7, 0.7)
            area = det["area_ratio"]
            if area >= stop_area:
                _stop(bridge)
                return {"reached": True,
                        "reason": "已到%s物体前（占画面 %.1f%%）" % (color_cn(color), area * 100),
                        "seconds": round(time.monotonic() - t0, 1), "last": det}
            if _front_blocked(bridge, 0.35):
                _stop(bridge)
                return {"reached": False, "reason": "正前方 0.35m 内有障碍，已停",
                        "seconds": round(time.monotonic() - t0, 1), "last": det}
            # 没对准就先转，对准了再走；越近越慢（但不低于固件死区）
            if abs(err_x) > 0.30:
                vx = 0.0
            else:
                vx = max(MIN_VX, speed * _clamp(
                    1.0 - area / max(stop_area, 1e-3), 0.25, 1.0))
                if abs(err_x) > 0.15:
                    vx *= 0.6
            bridge.send_move(vx, 0.0, vyaw)
            await asyncio.sleep(0.1)
        return {"reached": False, "reason": "超时（%.0fs）" % max_s,
                "seconds": round(time.monotonic() - t0, 1), "last": last}
    finally:
        _stop(bridge)


async def follow_person(bridge, max_s=60.0, speed=0.3, cancel=None,
                        distance=0.75):
    """跟着画面里最大的人走：视觉居中 + 雷达控距，丢失 1.5s 自动停。"""
    import asyncio
    t0 = time.monotonic()
    det = person_detector()
    last_seen = None
    lost_since = None
    frames = 0
    last = None
    stats = {"fwd": 0, "back": 0, "hold": 0}
    last_log = 0.0
    try:
        while time.monotonic() - t0 < max_s:
            if cancel is not None and cancel.is_set():
                return {"followed": False, "reason": "cancelled",
                        "seconds": round(time.monotonic() - t0, 1),
                        "frames": frames, "last": last}
            img, age = load_frame()
            if img is None:
                return {"followed": False,
                        "reason": "相机无新帧（age=%.1fs）" % age,
                        "seconds": round(time.monotonic() - t0, 1),
                        "frames": frames, "last": last}
            # YOLOX 单帧约 0.2-0.5s，放线程池，别卡住事件循环
            det_now = await asyncio.get_event_loop().run_in_executor(
                None, det.detect, img)
            frames += 1
            if det_now is None:
                if lost_since is None:
                    lost_since = time.monotonic()
                _stop(bridge)
                if time.monotonic() - lost_since > 1.5:
                    return {"followed": last_seen is not None,
                            "reason": "目标丢失超过 1.5s，已停",
                            "seconds": round(time.monotonic() - t0, 1),
                            "frames": frames, "last": last, "vx_stats": stats}
                await asyncio.sleep(0.12)
                continue
            lost_since = None
            last_seen = time.monotonic()
            last = det_now
            err_x = (det_now["cx"] - 0.5) * 2.0
            vyaw = _clamp(-1.2 * err_x, -0.7, 0.7)
            vx = _follow_vx(bridge, det_now, speed, distance)
            if vx > 0.02:
                stats["fwd"] += 1
            elif vx < -0.02:
                stats["back"] += 1
            else:
                stats["hold"] += 1
            if abs(err_x) > 0.6:          # 没对准就别往前冲
                vx = 0.0
            if _front_blocked(bridge, 0.35) and vx > 0:
                vx = 0.0
                vyaw = _clamp(vyaw * 1.5, -0.7, 0.7)
            if time.monotonic() - last_log > 1.0:
                last_log = time.monotonic()
                lid = getattr(bridge, "lidar", None) or {}
                log.info("follow: h=%.2f cx=%.2f front=%.2f narrow=%.2f "
                         "target=%.2f vx=%+.2f vyaw=%+.2f",
                         det_now["h"], det_now["cx"], lid.get("front", 99),
                         lid.get("front_narrow", 99), distance, vx, vyaw)
            # 检测约 3Hz，Move 指令按 10Hz 续发，否则走起来一顿一顿
            t_end = time.monotonic() + 0.3
            while time.monotonic() < t_end:
                if cancel is not None and cancel.is_set():
                    break
                bridge.send_move(vx, 0.0, vyaw)
                await asyncio.sleep(0.1)
        return {"followed": True, "reason": "到达时限 %.0fs，正常结束" % max_s,
                "seconds": round(time.monotonic() - t0, 1),
                "frames": frames, "last": last, "vx_stats": stats,
                "front_m": (getattr(bridge, "lidar", None) or {}).get("front")}
    finally:
        _stop(bridge)


def _follow_vx(bridge, det, speed, distance=0.75):
    """跟随速度：优先用雷达窄前向距离控距（目标 distance 米）。

    雷达已过滤地面点（z<-0.30），窄扇区约 ±20°，正对目标时读数就是到
    对方腿部的距离，比按框高估算准得多。雷达没有读数时才退回框高。
    """
    lidar = getattr(bridge, "lidar", None) or {}
    front = lidar.get("front_narrow")
    if front is None or front >= 90.0:
        front = lidar.get("front")
    if front is not None and front < 90.0:
        if front < 0.45:
            vx = -MIN_VX                    # 快贴上了，直接后退
        elif front < distance - 0.10:
            vx = -max(MIN_VX, min(0.30, 0.8 * (distance - front)))
        elif front <= distance + 0.10:
            vx = 0.0                        # 保持距离
        else:
            vx = max(MIN_VX, min(speed, 0.8 * (front - distance)))
        back = lidar.get("back")
        if vx < 0 and back is not None and back < 0.45:
            vx = 0.0                        # 后方太近，不许后退
        return vx

    # ---- 没有雷达读数：退回框高估算 ----
    h = det.get("h") or 0.0
    h_target = 0.66
    dead = 0.05
    if abs(h - h_target) <= dead:
        return 0.0
    vx = 1.6 * (h_target - h)
    if vx > 0:
        vx = max(MIN_VX, min(speed, vx))
    else:
        vx = -max(MIN_VX, min(0.30, -vx))
    return vx


def lidar_clearances(bridge):
    lidar = dict(getattr(bridge, "lidar", None) or {})
    if not lidar:
        return {"ok": False, "error": "雷达数据未就绪（/utlidar/cloud_base 无数据）"}
    stamp = lidar.get("stamp", 0)
    lidar["age_s"] = round(time.time() - stamp, 2) if stamp else None
    lidar["ok"] = True
    return lidar
