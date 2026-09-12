#!/usr/bin/env python3
"""Go2 web console backend.

- Serves the frontend (static/)
- WebSocket: pushes map/robot state to clients, receives control commands
- Tour state machine: send waypoint goals in sequence, advance on arrival

Lives entirely inside hu/go2/web. Talks to the running SCAN-Planner stack
through ros_bridge.RosBridge over standard ROS2 topics.
"""

import asyncio
import base64
import faulthandler
import json
import logging
import math
import os
import signal
import subprocess
import time
import urllib.request

import aiohttp

# 诊断：每 20s 把所有线程栈 dump 到 threads.log（定位阻塞用）
_fh = open("/home/unitree/hu/go2/threads.log", "w")
faulthandler.dump_traceback_later(20, repeat=True, file=_fh)

import websockets
from websockets.datastructures import Headers
from websockets.http11 import Response as WsResponse

import ros_bridge as rb
import tts

BASE_DIR = os.path.dirname(os.path.abspath(__file__))
CONFIG_PATH = os.path.join(BASE_DIR, "config.json")
STATIC_DIR = os.path.join(BASE_DIR, "static")
UPLOAD_DIR = os.path.join(BASE_DIR, "uploads")

log = logging.getLogger("go2web")

DEFAULT_CONFIG = {
    "host": "0.0.0.0",
    "port": 8080,
    "topics": {
        "map": "/grid_map/occupancy",
        "odom": "/utlidar/robot_odom",
        "goal": "/move_base_simple/goal",
        "cmd_vel": "/cmd_vel",
        "sport_request": "/api/sport/request",
    },
    "map_downsample": 4000,
    "push_hz": 10,
    "reach_threshold": 0.4,
    "max_speed": 0.5,
    "waypoints": [],
}


def load_config():
    if os.path.exists(CONFIG_PATH):
        with open(CONFIG_PATH, "r", encoding="utf-8") as f:
            data = json.load(f)
        merged = dict(DEFAULT_CONFIG)
        merged.update(data)
        return merged
    return dict(DEFAULT_CONFIG)


def save_config(cfg):
    with open(CONFIG_PATH, "w", encoding="utf-8") as f:
        json.dump(cfg, f, ensure_ascii=False, indent=2)


# --------------------------------------------------------------------------
# Tour (waypoint-guided autonomous navigation + turn + action + narration)
# --------------------------------------------------------------------------
class Tour:
    """Guides the robot through waypoints. On reaching a waypoint, runs the
    configured exhibit action AND the voice introduction in parallel, and only
    then advances to the next waypoint."""

    def __init__(self, bridge, cfg):
        self.bridge = bridge
        self.cfg = cfg
        self.active = False
        self.ids = []            # waypoint ids in order
        self.idx = -1
        self.state = "idle"      # idle | traveling | turning | acting | done
        self.intro_text = ""
        self.intro_wp = None
        self._stage_task = None  # current async stage (turn / act+intro)
        self._turn_task = None
        self._yaw_tolerance = math.radians(4.0)
        self._yaw_k = 1.0        # proportional turn gain
        self._max_vyaw = 0.6     # max turn rate

    def start(self, ids):
        if not ids:
            return {"ok": False, "error": "empty waypoint list"}
        self.ids = list(ids)
        self.idx = 0
        self.active = True
        self.state = "traveling"
        self._send_current()
        return {"ok": True, "msg": f"tour started, {len(ids)} points"}

    def stop(self):
        self.active = False
        self.state = "idle"
        self.intro_text = ""
        for task in (self._stage_task, self._turn_task):
            if task:
                task.cancel()
        self._stage_task = None
        self._turn_task = None
        # Halt the robot so the joystick can take over cleanly, then publish a
        # goal at the current position so SCAN-Planner finishes any running
        # trajectory and returns to WAIT_TARGET (releasing /cmd_vel).
        try:
            self.bridge.send_pause(False)
            self.bridge.send_stopmove()
            self.bridge.send_cmd_vel(0, 0, 0)
            pos = self.bridge.snapshot()["position"]
            self.bridge.send_goal(pos[0], pos[1], 0.0)
        except Exception:
            pass
        return {"ok": True, "msg": "tour stopped"}

    def skip_intro(self):
        """Skip the current action+intro stage and advance to the next waypoint."""
        if self.state in ("acting", "introducing") and self._stage_task:
            self._stage_task.cancel()
            self._stage_task = None
            self._advance()
            return {"ok": True, "msg": "stage skipped"}
        return {"ok": False, "error": "not introducing"}

    def _send_current(self):
        if not self.active or self.idx < 0 or self.idx >= len(self.ids):
            return
        wp = self._find(self.ids[self.idx])
        if wp:
            self.bridge.send_goal(wp["x"], wp["y"], wp.get("yaw", 0.0))
            log.info("tour -> waypoint %s (%s)", wp.get("name", wp["id"]), self.ids[self.idx])

    def _find(self, wid):
        for wp in self.cfg.get("waypoints", []):
            if wp["id"] == wid:
                return wp
        return None

    async def tick(self, pos):
        """Advance when the robot reaches the current waypoint, then turn to the
        configured facing angle, then run action + intro in parallel."""
        if not self.active or self.idx < 0 or self.idx >= len(self.ids):
            return
        if self.state == "traveling":
            wp = self._find(self.ids[self.idx])
            if not wp:
                self._advance()
                return
            dx = pos[0] - wp["x"]
            dy = pos[1] - wp["y"]
            if math.hypot(dx, dy) < self.cfg.get("reach_threshold", 0.4):
                self.intro_wp = wp
                self.intro_text = wp.get("intro", "").strip() or tts.DEFAULT_INTRO
                log.info("reached exhibit %s", wp.get("name", wp["id"]))
                if wp.get("yaw_at") is not None and float(wp.get("yaw_at", 0.0)) != 0.0:
                    self.state = "turning"
                    self._turn_task = asyncio.create_task(self._stage_turn(wp))
                else:
                    self.state = "acting"
                    self._stage_task = asyncio.create_task(self._stage_act_intro(wp))

    @staticmethod
    def _wrap_pi(a):
        return (a + math.pi) % (2 * math.pi) - math.pi

    async def _turn_to(self, target_yaw, max_vyaw=0.5, timeout=10.0):
        """Open-loop turn in place to target_yaw.

        Uses the configured turn speed and the angle to compute how long to
        command yaw rotation, instead of waiting for the LIO odometry yaw
        feedback (which updates slowly during in-place turns and caused the
        turn loop to time out without actually rotating).
        """
        self.bridge.send_pause(True)  # stop the bridge overriding the turn
        try:
            yaw0 = self.bridge.snapshot()["yaw"]
            err = self._wrap_pi(target_yaw - yaw0)
            if abs(err) < self._yaw_tolerance:
                log.info("already facing target %.1f deg", math.degrees(target_yaw))
                return
            dur = min(abs(err) / max_vyaw, timeout)
            vyaw = math.copysign(max_vyaw, err)
            log.info("turning %.1f deg at %.2f rad/s for %.1fs",
                     math.degrees(err), vyaw, dur)
            t0 = time.time()
            while time.time() - t0 < dur:
                self.bridge.send_move(0.0, 0.0, vyaw)
                await asyncio.sleep(0.05)
        finally:
            self.bridge.send_move(0.0, 0.0, 0.0)
        log.info("turn finished (target %.1f deg)", math.degrees(target_yaw))

    async def _stage_turn(self, wp):
        try:
            target = math.radians(float(wp.get("yaw_at", 0.0)))
            await self._turn_to(target)
        except asyncio.CancelledError:
            pass
        except Exception as e:
            log.warning("turn error: %s", e)
        if not self.active:
            return
        self._turn_task = None
        self.state = "acting"
        self._stage_task = asyncio.create_task(self._stage_act_intro(wp))

    async def _stage_act_intro(self, wp):
        """Run the exhibit action and the voice introduction in parallel; advance
        only when both have finished."""
        # Pause the bridge (stop its repeated Move commands) so the action is
        # not overridden, and release SCAN-Planner's control by sending a goal
        # at the current position so its closed-loop controller stops /cmd_vel.
        try:
            self.bridge.send_pause(True)
            self.bridge.send_stopmove()
            self.bridge.send_cmd_vel(0, 0, 0)
            pos = self.bridge.snapshot()["position"]
            self.bridge.send_goal(pos[0], pos[1], 0.0)
            await asyncio.sleep(1.0)
        except Exception:
            pass
        # then action + intro in parallel
        try:
            action_name = wp.get("action", "") or ""
            _, action_dur = self.bridge.send_action(action_name)
            # start intro playback concurrently
            intro_task = asyncio.create_task(self._play_intro(wp))
            tasks = [intro_task]
            if action_dur > 0:
                # wait for both: action duration and intro playback
                sleep_task = asyncio.ensure_future(asyncio.sleep(action_dur))
                tasks.append(sleep_task)
                done, pending = await asyncio.wait(tasks, return_when=asyncio.ALL_COMPLETED)
                # keep intro running if it outlasts the action
                for t in pending:
                    await t
            else:
                await intro_task
        except asyncio.CancelledError:
            pass
        except Exception as e:
            log.warning("act+intro error: %s", e)
        finally:
            # resume the bridge so the next leg of the tour can be controlled
            try:
                self.bridge.send_pause(False)
            except Exception:
                pass
            if self.active:
                self._advance()

    async def _play_intro(self, wp):
        try:
            audio = await tts.ensure_audio(wp)
            await tts.play(audio)
        except asyncio.CancelledError:
            pass
        except Exception as e:
            log.warning("intro playback error: %s", e)

    def _advance(self):
        self._stage_task = None
        if not self.active:
            return
        self.idx += 1
        if self.idx >= len(self.ids):
            self.state = "done"
            self.active = False
            self.bridge.send_cmd_vel(0, 0, 0)
            log.info("tour finished")
        else:
            self.state = "traveling"
            self.intro_text = ""
            self._send_current()

    def status(self):
        if not self.active:
            return {"active": False, "idx": -1, "total": len(self.ids),
                    "state": self.state, "intro": ""}
        wp = self._find(self.ids[self.idx]) if self.idx < len(self.ids) else None
        return {
            "active": True,
            "idx": self.idx,
            "total": len(self.ids),
            "state": self.state,
            "current": wp,
            "intro": self.intro_text if self.state in ("acting", "introducing") else "",
        }


# --------------------------------------------------------------------------
# WebSocket handler
# --------------------------------------------------------------------------
# SCAN-Planner launch command (real-robot, 2D Nav Goal mode, bridge on)
SCAN_LAUNCH_CMD = (
    "cd ~/unitree_sdk2-main/example/user/SCAN-Planner && "
    "source setup_scan_planner.sh && "
    "ros2 launch scan_planner real_robot.launch.py navi_mode:=1 bridge_enable:=true"
)
SCAN_LOG_PATH = os.path.join(BASE_DIR, "log", "scan_planner.log")


class WebApp:
    def __init__(self, cfg, bridge):
        self.cfg = cfg
        self.bridge = bridge
        self.tour = Tour(bridge, cfg)
        self.clients = set()
        self.control_mode = "joystick"  # "joystick" (sport Move) or "tour" (SCAN-Planner)
        self.scan_proc = None           # SCAN-Planner launch subprocess
        self.ai_history = []            # AI chat history [{role, content}]
        self.ai_client = None           # websocket that owns the AI session
        self.ai_started = False
        self.ai_busy = False

    async def broadcast_state(self):
        """Push the current map + robot state to all clients."""
        snap = self.bridge.snapshot()
        msg = {
            "type": "state",
            "map": snap["map_points"],
            "map_stamp": snap["map_stamp"],
            "pos": snap["position"],
            "yaw": snap["yaw"],
            "odom_stamp": snap["odom_stamp"],
            "tour": self.tour.status(),
            "control_mode": self.control_mode,
        }
        if self.clients:
            data = json.dumps(msg)
            await asyncio.gather(
                *(c.send(data) for c in list(self.clients)), return_exceptions=True
            )

    async def loop(self):
        hz = self.cfg.get("push_hz", 10)
        while True:
            try:
                await self.tour.tick(self.bridge.snapshot()["position"])
            except Exception:
                pass
            await self.broadcast_state()
            await asyncio.sleep(1.0 / hz)

    # ---- command handling ----
    def handle_message(self, data, websocket=None):
        try:
            msg = json.loads(data)
        except Exception:
            return {"ok": False, "error": "bad json"}
        mtype = msg.get("type")

        if mtype == "cmd_vel":
            # Joystick control -> sport Move channel (like the official remote
            # controller). Independent of /cmd_vel used by SCAN-Planner tours.
            # Taking the joystick always stops an active tour first.
            if self.tour.active:
                self.tour.stop()
            self.control_mode = "joystick"
            vx = float(msg.get("vx", 0.0))
            vy = float(msg.get("vy", 0.0))
            vyaw = float(msg.get("vyaw", 0.0))
            # clamp to max_speed
            m = self.cfg.get("max_speed", 0.5)
            vx = max(-m, min(m, vx))
            vy = max(-m, min(m, vy))
            vyaw = max(-1.2, min(1.2, vyaw))
            self.bridge.send_move(vx, vy, vyaw)
            return {"ok": True}

        if mtype == "goal":
            self.tour.stop()
            self.bridge.send_goal(float(msg["x"]), float(msg["y"]), float(msg.get("yaw", 0.0)))
            return {"ok": True}

        if mtype == "sport":
            action = msg.get("action")
            api = {
                "standup": rb.SPORT_STANDUP,
                "standdown": rb.SPORT_STANDDOWN,
                "damp": rb.SPORT_DAMP,
                "sit": rb.SPORT_SIT,
                "risesit": rb.SPORT_RISESIT,
                "balance": rb.SPORT_BALANCESTAND,
                "stop": rb.SPORT_STOPMOVE,
                "recovery": rb.SPORT_RECOVERYSTAND,
                "heart": rb.SPORT_HEART,
                "freewalk_on": (rb.SPORT_FREEWALK, "true"),
                "freewalk_off": (rb.SPORT_FREEWALK, "false"),
            }.get(action)
            if api is None:
                return {"ok": False, "error": f"unknown action {action}"}
            if isinstance(api, tuple):
                self.bridge.send_sport(api[0], param=api[1])
            else:
                self.bridge.send_sport(api)
            return {"ok": True}

        if mtype == "waypoint":
            return self._handle_waypoint(msg)

        if mtype == "tour":
            return self._handle_tour(msg)

        if mtype == "tts":
            return self._handle_tts(msg)

        if mtype == "scanner":
            return self._handle_scanner(msg)

        if mtype == "emergency":
            return self._handle_emergency()

        if mtype == "ai":
            return self._handle_ai(msg, websocket)

        if mtype == "config":
            return self._handle_config(msg)

        return {"ok": False, "error": f"unknown type {mtype}"}

    # ---- emergency stop: stop everything and damp ----
    def _handle_emergency(self):
        # 1) stop the tour / any running action
        self.tour.stop()
        # 2) stop the joystick
        self.bridge.send_cmd_vel(0, 0, 0)
        self.bridge.send_move(0.0, 0.0, 0.0)
        # 3) shut down SCAN-Planner (kill all planner nodes)
        self._handle_scanner({"op": "stop"})
        # 4) drop into damping mode immediately
        self.bridge.send_damp()
        self.control_mode = "joystick"
        log.warning("EMERGENCY STOP: all stopped, robot in damping mode")
        return {"ok": True, "msg": "急停：已停止所有程序，机器人进入阻尼状态"}

    # ---- AI chat session (Ollama REST API on host: 11434) ----
    OLLAMA_API = "http://127.0.0.1:11434"
    AI_MODEL = "gemma3:4b"

    def _ollama_post(self, path, payload, timeout=120):
        req = urllib.request.Request(
            self.OLLAMA_API + path,
            data=json.dumps(payload).encode(),
            headers={"Content-Type": "application/json"})
        with urllib.request.urlopen(req, timeout=timeout) as r:
            return json.loads(r.read())

    def _ollama_get(self, path, timeout=10):
        req = urllib.request.Request(self.OLLAMA_API + path)
        with urllib.request.urlopen(req, timeout=timeout) as r:
            return json.loads(r.read())

    async def _ai_push(self, payload, client=None):
        target = client or self.ai_client
        if target:
            try:
                await target.send(json.dumps({"type": "ai", **payload}))
            except Exception:
                pass

    async def _ai_check_and_start(self):
        try:
            timeout = aiohttp.ClientTimeout(total=15)
            async with aiohttp.ClientSession(timeout=timeout) as session:
                async with session.get(self.OLLAMA_API + "/api/tags") as r:
                    tags = await r.json()
            models = [m.get("name", "") for m in tags.get("models", [])]
            if not any(m.split(":")[0] == self.AI_MODEL.split(":")[0] for m in models):
                await self._ai_push({
                    "op": "error",
                    "error": f"模型 {self.AI_MODEL} 未下载。请在 Jetson 上运行: ollama pull {self.AI_MODEL}"})
                return
            self.ai_started = True
            await self._ai_push({"op": "started"})
        except Exception as e:
            await self._ai_push({"op": "error", "error": f"ollama 服务不可达: {e}"})

    def _handle_ai(self, msg, websocket):
        op = msg.get("op")
        if op == "start":
            self.ai_client = websocket
            self.ai_history = []
            asyncio.ensure_future(self._ai_check_and_start())
            return {"ok": True}
        if op == "end":
            # 先捕获客户端再清空，否则 ended 消息推不出去
            client = self.ai_client
            self.ai_client = None
            self.ai_started = False
            self.ai_history = []
            asyncio.ensure_future(self._ai_push({"op": "ended"}, client=client))
            return {"ok": True}
        if not self.ai_started:
            return {"ok": False, "error": "AI 会话未启动，请先启动对话"}
        if op == "send":
            asyncio.ensure_future(self._ai_chat(msg.get("text", "")))
            return {"ok": True}
        if op == "image":
            asyncio.ensure_future(
                self._ai_image(msg.get("data", ""), msg.get("name", "camera.jpg")))
            return {"ok": True}
        if op == "video":
            asyncio.ensure_future(
                self._ai_video(msg.get("data", ""), msg.get("name", "camera.mp4")))
            return {"ok": True}
        return {"ok": False, "error": f"unknown ai op {op}"}

    async def _ai_stream(self):
        """向 ollama 发起流式 chat（messages 取 self.ai_history），逐段推给前端。
        返回完整回复文本。"""
        timeout = aiohttp.ClientTimeout(total=300)
        full = ""
        async with aiohttp.ClientSession(timeout=timeout) as session:
            async with session.post(
                    self.OLLAMA_API + "/api/chat",
                    json={"model": self.AI_MODEL,
                          "messages": self.ai_history,
                          "stream": True}) as resp:
                resp.raise_for_status()
                async for raw in resp.content:
                    line = raw.decode(errors="replace").strip()
                    if not line:
                        continue
                    chunk = json.loads(line)
                    piece = chunk.get("message", {}).get("content", "")
                    if piece:
                        full += piece
                        await self._ai_push(
                            {"op": "output", "text": piece, "final": False})
                    if chunk.get("done", False):
                        break
        return full

    async def _ai_chat(self, text):
        if self.ai_busy:
            await self._ai_push({"op": "error", "error": "上一条还在回复中"})
            return
        self.ai_busy = True
        try:
            self.ai_history.append({"role": "user", "content": text})
            full = await self._ai_stream()
            if full.strip():
                self.ai_history.append({"role": "assistant", "content": full.strip()})
            await self._ai_push({"op": "output", "text": "", "final": True})
            log.info("ai chat done (%d chars)", len(full))
        except Exception as e:
            if self.ai_history and self.ai_history[-1].get("role") == "user":
                self.ai_history.pop()
            await self._ai_push({"op": "output", "text": "", "final": True})
            await self._ai_push({"op": "error", "error": str(e)})
            log.error("ai chat failed: %s", e)
        finally:
            self.ai_busy = False

    async def _ai_chat_images(self, prompt, image_paths):
        """图片/视频帧分析：本地图片转 base64，流式交给 ollama 逐段返回。"""
        if self.ai_busy:
            await self._ai_push({"op": "error", "error": "上一条还在回复中"})
            return
        self.ai_busy = True
        try:
            images = []
            for p in image_paths:
                with open(p, "rb") as f:
                    images.append(base64.b64encode(f.read()).decode())
            self.ai_history.append({"role": "user", "content": prompt, "images": images})
            full = await self._ai_stream()
            if full.strip():
                self.ai_history.append({"role": "assistant", "content": full.strip()})
            await self._ai_push({"op": "output", "text": "", "final": True})
            log.info("ai image analysis done (%d chars)", len(full))
        except Exception as e:
            if self.ai_history and self.ai_history[-1].get("images"):
                self.ai_history.pop()
            await self._ai_push({"op": "output", "text": "", "final": True})
            await self._ai_push({"op": "error", "error": str(e)})
            log.error("ai image analysis failed: %s", e)
        finally:
            self.ai_busy = False

    async def _ai_image(self, b64data, name):
        try:
            os.makedirs(UPLOAD_DIR, exist_ok=True)
            ts = int(time.time() * 1000)
            path = os.path.join(UPLOAD_DIR, f"img_{ts}.jpg")
            with open(path, "wb") as f:
                f.write(base64.b64decode(b64data))
            await self._ai_push({"op": "saved"})
            log.info("image saved: %s", path)
            await self._ai_chat_images("请用中文描述这张图片的内容。", [path])
        except Exception as e:
            await self._ai_push({"op": "error", "error": f"图片处理失败: {e}"})

    async def _ai_video(self, b64data, name):
        """Save video, extract 3 key frames with ffmpeg, analyze them together."""
        try:
            os.makedirs(UPLOAD_DIR, exist_ok=True)
            ts = int(time.time() * 1000)
            ext = os.path.splitext(name)[1] or ".mp4"
            vpath = os.path.join(UPLOAD_DIR, f"vid_{ts}{ext}")
            with open(vpath, "wb") as f:
                f.write(base64.b64decode(b64data))
            await self._ai_push({"op": "saved"})
            # duration via ffprobe
            try:
                out = subprocess.run(
                    ["ffprobe", "-v", "quiet", "-show_entries", "format=duration",
                     "-of", "csv=p=0", vpath],
                    capture_output=True, text=True, timeout=15).stdout.strip()
                dur = max(float(out), 1.0)
            except Exception:
                dur = 10.0
            frames = []
            for i, frac in enumerate((0.25, 0.5, 0.75)):
                fp = os.path.join(UPLOAD_DIR, f"frame_{ts}_{i}.jpg")
                t = min(dur * frac, max(dur - 0.1, 0.0))
                subprocess.run(["ffmpeg", "-y", "-loglevel", "quiet",
                                "-ss", str(t), "-i", vpath, "-frames:v", "1", fp],
                               capture_output=True, timeout=30)
                if os.path.isfile(fp):
                    frames.append(fp)
            if not frames:
                await self._ai_push({"op": "error", "error": "视频抽帧失败"})
                return
            log.info("video frames extracted: %s", frames)
            await self._ai_chat_images(
                "这是同一段视频在不同时间点的三帧画面，请用中文描述这段视频的内容。", frames)
        except Exception as e:
            await self._ai_push({"op": "error", "error": f"视频处理失败: {e}"})

    # ---- SCAN-Planner process control ----
    def scan_running(self):
        return self.scan_proc is not None and self.scan_proc.poll() is None

    def _handle_scanner(self, msg):
        op = msg.get("op")
        if op == "start":
            if self.scan_running():
                return {"ok": True, "msg": "SCAN-Planner already running", "running": True}
            try:
                os.makedirs(os.path.dirname(SCAN_LOG_PATH), exist_ok=True)
                logf = open(SCAN_LOG_PATH, "ab")
            except Exception:
                logf = subprocess.DEVNULL
            # start in its own session so the whole launch tree can be killed
            self.scan_proc = subprocess.Popen(
                ["bash", "-c", SCAN_LAUNCH_CMD],
                stdout=logf, stderr=subprocess.STDOUT,
                start_new_session=True,
            )
            log.info("SCAN-Planner launch started (pid %s)", self.scan_proc.pid)
            return {"ok": True, "msg": "SCAN-Planner 启动中", "running": True}

        if op == "stop":
            stopped = False
            if self.scan_proc:
                try:
                    os.killpg(os.getpgid(self.scan_proc.pid), signal.SIGTERM)
                    stopped = True
                except Exception:
                    pass
                self.scan_proc = None
            # fallback: kill any leftover planner nodes
            for pat in ("scan_planner_node", "closed_loop_controller",
                        "odom_tf_broadcaster", "go2_cmd_vel_bridge_node",
                        "real_robot.launch", "scan_planner.*launch"):
                try:
                    subprocess.run(["pkill", "-f", pat],
                                   capture_output=True, timeout=5)
                except Exception:
                    pass
            log.info("SCAN-Planner launch stopped")
            return {"ok": True, "msg": "SCAN-Planner 已停止" if stopped else "已停止(无运行进程)",
                    "running": False}

        if op == "status":
            return {"ok": True, "running": self.scan_running()}

        return {"ok": False, "error": f"unknown scanner op {op}"}

    def _handle_tts(self, msg):
        """Voice actions: preview an exhibit intro, or set a waypoint's intro text."""
        op = msg.get("op")
        if op == "preview":
            wid = int(msg.get("id", 0))
            wp = next((w for w in self.cfg.get("waypoints", []) if w["id"] == wid), None)
            if not wp:
                return {"ok": False, "error": f"no waypoint {wid}"}
            text = (msg.get("text") or wp.get("intro", "")).strip() or tts.DEFAULT_INTRO

            async def _preview():
                try:
                    path = await tts.synthesize(text, os.path.join(
                        tts.AUDIO_DIR, f"preview_{int(time.time())}.mp3"))
                    await tts.play(path)
                except Exception as e:
                    log.warning("preview error: %s", e)
            asyncio.create_task(_preview())
            return {"ok": True, "msg": "preview playing"}

        if op == "set_intro":
            wid = int(msg.get("id", 0))
            for w in self.cfg.get("waypoints", []):
                if w["id"] == wid:
                    w["intro"] = msg.get("intro", "")
                    save_config(self.cfg)
                    return {"ok": True, "waypoints": self.cfg["waypoints"]}
            return {"ok": False, "error": f"no waypoint {wid}"}

        return {"ok": False, "error": f"unknown tts op {op}"}

    def _handle_waypoint(self, msg):
        op = msg.get("op")
        wps = self.cfg.setdefault("waypoints", [])

        if op == "add":  # add by map click
            new_id = max([w["id"] for w in wps], default=0) + 1
            wps.append({
                "id": new_id,
                "name": msg.get("name", f"点{new_id}"),
                "x": float(msg["x"]),
                "y": float(msg["y"]),
                "z": float(msg.get("z", 0.3)),
                "yaw": float(msg.get("yaw", 0.0)),
                "yaw_at": float(msg.get("yaw_at", 0.0)),
                "action": msg.get("action", ""),
            })
            save_config(self.cfg)
            return {"ok": True, "waypoint": wps[-1], "waypoints": wps}

        if op == "record":  # record current robot position
            pos = self.bridge.snapshot()["position"]
            yaw = self.bridge.snapshot()["yaw"]
            new_id = max([w["id"] for w in wps], default=0) + 1
            wps.append({
                "id": new_id,
                "name": msg.get("name", f"点{new_id}"),
                "x": round(pos[0], 3),
                "y": round(pos[1], 3),
                "z": round(pos[2], 3),
                "yaw": round(yaw, 3),
                "yaw_at": 0.0,
                "action": "",
            })
            save_config(self.cfg)
            return {"ok": True, "waypoint": wps[-1], "waypoints": wps}

        if op == "del":
            wid = int(msg["id"])
            self.cfg["waypoints"] = [w for w in wps if w["id"] != wid]
            save_config(self.cfg)
            return {"ok": True, "waypoints": self.cfg["waypoints"]}

        if op == "rename":
            wid = int(msg["id"])
            for w in wps:
                if w["id"] == wid:
                    w["name"] = msg.get("name", w["name"])
            save_config(self.cfg)
            return {"ok": True, "waypoints": wps}

        if op == "set_yaw_at":
            wid = int(msg["id"])
            val = msg.get("yaw_at")
            try:
                val = float(val) if val is not None else 0.0
            except (TypeError, ValueError):
                val = 0.0
            for w in wps:
                if w["id"] == wid:
                    w["yaw_at"] = val
            save_config(self.cfg)
            return {"ok": True, "waypoints": wps}

        if op == "set_action":
            wid = int(msg["id"])
            for w in wps:
                if w["id"] == wid:
                    w["action"] = msg.get("action", "")
            save_config(self.cfg)
            return {"ok": True, "waypoints": wps}

        if op == "reposition":
            # Re-record an existing waypoint at the robot's current position
            # (used when an exhibit spot is unusable and needs re-placing).
            wid = int(msg["id"])
            pos = self.bridge.snapshot()["position"]
            yaw = self.bridge.snapshot()["yaw"]
            if not math.isfinite(pos[0]) and not math.isfinite(pos[1]):
                return {"ok": False, "error": "no valid odometry to record"}
            for w in wps:
                if w["id"] == wid:
                    w["x"] = round(pos[0], 3)
                    w["y"] = round(pos[1], 3)
                    w["z"] = round(pos[2], 3)
                    w["yaw"] = round(yaw, 3)
            save_config(self.cfg)
            return {"ok": True, "waypoints": wps}

        return {"ok": False, "error": f"unknown waypoint op {op}"}

    def _handle_tour(self, msg):
        op = msg.get("op")
        if op == "start":
            res = self.tour.start([int(i) for i in msg.get("ids", [])])
            if res.get("ok"):
                self.control_mode = "tour"
            return res
        if op == "stop":
            res = self.tour.stop()
            self.control_mode = "joystick"
            return res
        if op == "skip":
            return self.tour.skip_intro()
        return {"ok": False, "error": f"unknown tour op {op}"}

    def _handle_config(self, msg):
        key = msg.get("key")
        value = msg.get("value")
        if key in ("max_speed", "reach_threshold"):
            self.cfg[key] = float(value)
            save_config(self.cfg)
            return {"ok": True, key: self.cfg[key]}
        return {"ok": False, "error": f"unknown config key {key}"}


# --------------------------------------------------------------------------
# HTTP static server + entry
# --------------------------------------------------------------------------
CONTENT_TYPES = {
    ".html": "text/html; charset=utf-8",
    ".js": "application/javascript; charset=utf-8",
    ".css": "text/css; charset=utf-8",
    ".png": "image/png",
    ".svg": "image/svg+xml",
    ".json": "application/json; charset=utf-8",
}


async def http_handler(connection, request):
    """Serve static files from static/. Return None for /ws (WebSocket upgrade)."""
    path = request.path
    if path in ("/ws", "/ws/"):
        return None  # let the WebSocket handshake proceed
    # route: "/" -> main panel, "/console" -> tour console, "/ai" -> AI panel
    if path in ("/", ""):
        path = "/index.html"
    elif path in ("/console", "/console/"):
        path = "/console.html"
    elif path in ("/ai", "/ai/"):
        path = "/ai.html"
    # basic path traversal guard
    full = os.path.normpath(os.path.join(STATIC_DIR, path.lstrip("/")))
    if not full.startswith(STATIC_DIR) or not os.path.isfile(full):
        body = b"Not found"
        return WsResponse(404, "Not Found", Headers({"Content-Type": "text/plain"}), body)
    ext = os.path.splitext(full)[1]
    ctype = CONTENT_TYPES.get(ext, "application/octet-stream")
    with open(full, "rb") as f:
        body = f.read()
    return WsResponse(200, "OK", Headers({"Content-Type": ctype}), body)


async def ws_handler(websocket):
    app = WebApp.inst
    app.clients.add(websocket)
    log.info("client connected: %s", websocket.remote_address)
    try:
        # send initial config/waypoints
        await websocket.send(json.dumps({
            "type": "init",
            "config": {
                "max_speed": app.cfg.get("max_speed", 0.5),
                "reach_threshold": app.cfg.get("reach_threshold", 0.4),
            },
            "waypoints": app.cfg.get("waypoints", []),
        }))
        async for raw in websocket:
            resp = app.handle_message(raw, websocket)
            if resp:
                await websocket.send(json.dumps({"type": "ack", **resp}))
    except websockets.exceptions.ConnectionClosed:
        pass
    finally:
        app.clients.discard(websocket)
        if app.ai_client is websocket:
            app.ai_client = None
            app.ai_started = False
        log.info("client disconnected")


async def ai_startup_probe():
    """诊断：启动时立即测 ollama GET 是否在 web 进程内正常工作。"""
    t0 = asyncio.get_event_loop().time()
    try:
        timeout = aiohttp.ClientTimeout(total=15)
        async with aiohttp.ClientSession(timeout=timeout) as session:
            async with session.get("http://127.0.0.1:11434/api/tags") as r:
                data = await r.json()
        log.info("AI_PROBE: OK %.1fs models=%d",
                 asyncio.get_event_loop().time() - t0, len(data.get("models", [])))
    except Exception as e:
        log.error("AI_PROBE: FAILED after %.1fs: %s",
                  asyncio.get_event_loop().time() - t0, e)


async def main():
    cfg = load_config()
    bridge, thread = rb.start_bridge(cfg)
    asyncio.create_task(ai_startup_probe())

    app = WebApp(cfg, bridge)
    WebApp.inst = app

    loop = asyncio.get_event_loop()
    loop.create_task(app.loop())

    host = cfg.get("host", "0.0.0.0")
    port = int(cfg.get("port", 8080))

    async def handler(websocket):
        await ws_handler(websocket)

    server = await websockets.serve(
        handler,
        host,
        port,
        max_size=64 * 1024 * 1024,
        process_request=http_handler,
    )
    log.info("go2 web console listening on http://%s:%d", host, port)
    await server.wait_closed()


if __name__ == "__main__":
    logging.basicConfig(level=logging.INFO,
                        format="%(asctime)s [%(levelname)s] %(name)s: %(message)s")
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        pass
