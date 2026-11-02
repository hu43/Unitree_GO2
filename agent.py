#!/usr/bin/env python3
"""智能 AI 控制：Ollama 工具调用 Agent。

把 DDS 运控、摄像头、激光雷达、导航包装成工具，交给本地 gemma4:e2b
（支持 tools/vision）自主编排。闭环技能（走到色块、跟随人）在
perception.py 里用确定性代码实现，LLM 只负责选工具、看结果、说结论。

安全边界全部写死在 Python 侧：限速、限时、运动看门狗、急停、单任务互斥。
"""

import asyncio
import difflib
import json
import logging
import math
import time

import aiohttp

import perception
import ros_bridge as rb

log = logging.getLogger("go2web.agent")

# 任务里出现这些字眼时，认为用户要机器人“做事”，不允许模型只输出计划
ACTION_WORDS = ("站", "走", "转", "跟", "去", "来", "坐", "趴", "卧", "停",
                "移动", "前进", "后退", "左", "右", "看", "记", "导航", "拿", "过来")

SYSTEM_PROMPT = """你是宇树 Go2 机器狗的控制代理，运行在机器人本地。
用户要求动作时必须调用工具完成，禁止只输出计划，禁止向用户请示。
先 robot_status 看状态；趴下时先 stand_up。
移动用 move/turn（move 必须给 vx，向前为正）；找东西用 find_color/look/walk_to_color；跟人用 follow_person；点位用 goto_waypoint/remember_place；危险时 emergency_stop。
做完用一句中文汇报结果，不要编造工具没有返回的信息。
速度上限 {speed} m/s，单次 move 最长 {dur} 秒，任务总时长 {task} 秒。
"""

TOOLS = [
    {"type": "function", "function": {
        "name": "robot_status",
        "description": "获取机器人状态：位姿、姿态模式、电量、雷达净空、导览/规划器状态、已记录点位。",
        "parameters": {"type": "object", "properties": {}}}},
    {"type": "function", "function": {
        "name": "stand_up",
        "description": "让机器人站立（从趴下/阻尼状态恢复）。",
        "parameters": {"type": "object", "properties": {}}}},
    {"type": "function", "function": {
        "name": "sit",
        "description": "让机器人坐下。",
        "parameters": {"type": "object", "properties": {}}}},
    {"type": "function", "function": {
        "name": "lie_down",
        "description": "卧倒/趴下（StandDown）：四肢收拢趴下，但仍保持关节支撑，和运动模式页的「卧倒」一致。",
        "parameters": {"type": "object", "properties": {}}}},
    {"type": "function", "function": {
        "name": "balance_stand",
        "description": "平衡站立（更稳，适合原地观察）。",
        "parameters": {"type": "object", "properties": {}}}},
    {"type": "function", "function": {
        "name": "stop_move",
        "description": "停止当前移动（急停以外的普通停止）。",
        "parameters": {"type": "object", "properties": {}}}},
    {"type": "function", "function": {
        "name": "move",
        "description": "按给定速度移动一小段时间后自动停下。向前走必须给 vx>0（例如向前一步：vx=0.2, duration_s=1）。",
        "parameters": {"type": "object", "properties": {
            "vx": {"type": "number", "description": "前后速度 m/s，正数向前，范围 ±{speed}"},
            "vy": {"type": "number", "description": "横移速度 m/s，范围 ±0.2"},
            "vyaw": {"type": "number", "description": "转向角速度 rad/s，范围 ±0.8"},
            "duration_s": {"type": "number", "description": "持续秒数，最长 {dur}"},
        }, "required": ["vx", "duration_s"]}}},
    {"type": "function", "function": {
        "name": "turn",
        "description": "原地转向指定角度（正数左转，负数右转）。",
        "parameters": {"type": "object", "properties": {
            "degrees": {"type": "number", "description": "角度，例如 90 或 -45"}},
            "required": ["degrees"]}}},
    {"type": "function", "function": {
        "name": "find_color",
        "description": "在摄像头画面里找指定颜色的物体，返回位置和大小占比。",
        "parameters": {"type": "object", "properties": {
            "color": {"type": "string", "description": "颜色名：blue/green/red/yellow/orange/purple/cyan/white/black"}},
            "required": ["color"]}}},
    {"type": "function", "function": {
        "name": "walk_to_color",
        "description": "自动走到指定颜色物体前并停下（视觉闭环，自动转向和前进）。",
        "parameters": {"type": "object", "properties": {
            "color": {"type": "string", "description": "颜色名，如 blue"},
            "max_seconds": {"type": "number", "description": "最长耗时，默认 40"}},
            "required": ["color"]}}},
    {"type": "function", "function": {
        "name": "follow_person",
        "description": "跟随画面中的人行走，保持距离；人消失 1.5 秒后自动停下。",
        "parameters": {"type": "object", "properties": {
            "max_seconds": {"type": "number", "description": "最长跟随秒数，默认 30，最大 60"}}}}},
    {"type": "function", "function": {
        "name": "look",
        "description": "用摄像头看一眼前方，让视觉模型用中文描述看到的内容。",
        "parameters": {"type": "object", "properties": {
            "question": {"type": "string", "description": "想了解什么，例如“前方有什么障碍物”"}}}}},
    {"type": "function", "function": {
        "name": "lidar_clearances",
        "description": "读取激光雷达前/后/左/右最近障碍距离（米）。",
        "parameters": {"type": "object", "properties": {}}}},
    {"type": "function", "function": {
        "name": "list_waypoints",
        "description": "列出已记录的点位（名称和坐标）。",
        "parameters": {"type": "object", "properties": {}}}},
    {"type": "function", "function": {
        "name": "goto_waypoint",
        "description": "用 SCAN-Planner 自主导航到已记录的点位（会避障）。",
        "parameters": {"type": "object", "properties": {
            "name": {"type": "string", "description": "点位名称，可模糊匹配"}},
            "required": ["name"]}}},
    {"type": "function", "function": {
        "name": "remember_place",
        "description": "把机器人当前位置记录成点位，例如“把这里记成老板办公室”。",
        "parameters": {"type": "object", "properties": {
            "name": {"type": "string", "description": "点位名称"}},
            "required": ["name"]}}},
    {"type": "function", "function": {
        "name": "speak",
        "description": "用机器人扬声器说话（离线 TTS）。",
        "parameters": {"type": "object", "properties": {
            "text": {"type": "string", "description": "要说的话，中文"}},
            "required": ["text"]}}},
    {"type": "function", "function": {
        "name": "emergency_stop",
        "description": "急停：停止导览、规划器、所有运动并让机器人趴下（阻尼）。",
        "parameters": {"type": "object", "properties": {}}}},
]


class RobotAgent:
    """一次只跑一个任务；所有运动都经过限速/限时/看门狗。"""

    def __init__(self, app):
        self.app = app
        self.bridge = app.bridge
        self.cfg = app.cfg
        self._cancel = asyncio.Event()
        self._task = None
        self._owner = None
        self._running = False
        self._motion_active = False
        self._motion_deadline = 0.0
        self._watchdog = None
        self._last = {"task": "", "steps": 0, "final": "", "error": ""}

    # ---- 配置 ----
    @property
    def acfg(self):
        return self.cfg.get("agent") or {}

    def enabled(self):
        return bool((self.cfg.get("ai") or {}).get("enabled", True)) \
            and bool(self.acfg.get("enabled", True))

    def busy(self):
        return self._running

    def status(self):
        return {
            "enabled": self.enabled(),
            "busy": self._running,
            "task": self._last["task"],
            "steps": self._last["steps"],
            "final": self._last["final"],
            "error": self._last["error"],
            "max_speed": self.speed_limit(),
        }

    def speed_limit(self):
        return min(float(self.acfg.get("max_speed", 0.3)),
                   float(self.cfg.get("max_speed", 0.5)))

    # ---- 生命周期 ----
    def start(self, task_text, websocket, push):
        if self._running:
            return {"ok": False, "error": "已有任务在执行，先停止"}
        if not self.enabled():
            return {"ok": False, "error": "AI 或 Agent 总开关未开启"}
        if not task_text.strip():
            return {"ok": False, "error": "任务内容为空"}
        self._owner = websocket
        self._task = asyncio.ensure_future(self._run(task_text.strip(), push))
        return {"ok": True, "task": task_text.strip()}

    def cancel(self):
        self._cancel.set()
        if self._task is not None and not self._task.done():
            self._task.cancel()

    async def _run(self, task_text, push):
        self._running = True
        self._cancel.clear()
        self._last = {"task": task_text, "steps": 0, "final": "", "error": ""}
        t0 = time.monotonic()
        steps = 0
        self._watchdog = asyncio.ensure_future(self._watchdog_loop())
        try:
            push({"op": "started", "task": task_text})
            messages = [
                {"role": "system", "content": SYSTEM_PROMPT.format(
                    speed=self.speed_limit(),
                    dur=self.acfg.get("max_move_s", 3.0),
                    task=self.acfg.get("max_task_s", 180))},
                {"role": "user", "content": task_text},
            ]
            max_steps = int(self.acfg.get("max_steps", 12))
            final = ""
            nudged = 0
            tool_count = 0
            action_like = any(w in task_text for w in ACTION_WORDS)
            for step in range(1, max_steps + 1):
                if self._cancel.is_set():
                    final = "任务已取消"
                    break
                steps = step
                self._last["steps"] = step
                push({"op": "thinking", "step": step, "max": max_steps})
                msg = await self._chat(messages)
                calls = msg.get("tool_calls") or []
                content = (msg.get("content") or "").strip()
                if content:
                    push({"op": "assistant", "text": content})
                if not calls:
                    # 小模型偶尔在第一步只输出计划：要求它立即执行一次。
                    # 已经执行过工具后再出现的纯文本是正常总结，不能再催。
                    if action_like and nudged < 1 and tool_count == 0:
                        nudged += 1
                        log.warning("agent: no tool_calls, nudge (content=%r)",
                                    content[:80])
                        push({"op": "log",
                              "text": "模型只给了计划，已要求它直接执行"})
                        if content:
                            messages.append({"role": "assistant",
                                             "content": content})
                        messages.append({
                            "role": "user",
                            "content": "不要只描述计划。现在立即调用工具执行；"
                                       "需要多个动作就按顺序逐个调用。"})
                        continue
                    final = content or "（模型没有给出文字回复）"
                    break
                messages.append({"role": "assistant", "content": content,
                                 "tool_calls": calls})
                for call in calls:
                    if self._cancel.is_set():
                        break
                    fn = call.get("function") or {}
                    name = fn.get("name", "")
                    raw_args = fn.get("arguments")
                    args = raw_args if isinstance(raw_args, dict) else {}
                    if isinstance(raw_args, str):
                        try:
                            args = json.loads(raw_args) if raw_args.strip() else {}
                        except Exception:
                            args = {}
                    push({"op": "tool", "name": name, "args": args})
                    result = await self._exec(name, args, push)
                    tool_count += 1
                    text = json.dumps(result, ensure_ascii=False, default=str)
                    if len(text) > 1500:
                        text = text[:1500] + "…"
                    push({"op": "result", "name": name, "result": text})
                    messages.append({"role": "tool", "tool_name": name,
                                     "content": text})
            else:
                final = "达到最大步骤数，任务中止"
            self._last["final"] = final
            if final and self.acfg.get("speak_actions", True):
                try:
                    await self.app.voice.speak(final)
                except Exception:
                    pass
            push({"op": "done", "text": final, "steps": steps,
                  "seconds": round(time.monotonic() - t0, 1)})
        except asyncio.CancelledError:
            self._last["error"] = "cancelled"
            push({"op": "error", "error": "任务已取消"})
            raise
        except Exception as e:
            self._last["error"] = str(e)
            push({"op": "error", "error": str(e)})
        finally:
            self._running = False
            self._motion_active = False
            if self._watchdog:
                self._watchdog.cancel()
                self._watchdog = None
            self._stop_motion()

    async def _watchdog_loop(self):
        """运动看门狗：超过该次运动的截止时间仍在动，就强制归零。"""
        while True:
            try:
                if self._motion_active and time.monotonic() > self._motion_deadline:
                    self._motion_active = False
                    self._stop_motion()
                    self.cancel()
                    return
            except Exception:
                pass
            await asyncio.sleep(0.2)

    def _stop_motion(self):
        try:
            self.bridge.send_move(0.0, 0.0, 0.0)
            self.bridge.send_sport(rb.SPORT_STOPMOVE)
        except Exception:
            pass

    async def _wait_standing(self, timeout=8.0):
        """轮询到机器人真正站起来（body_height >= 0.15）或超时。"""
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            if self._cancel.is_set():
                return False
            st = self.bridge.sport_state or {}
            if float(st.get("body_height") or 0.0) >= 0.15:
                return True
            await asyncio.sleep(0.25)
        return False

    async def _ensure_ready(self, push=None):
        """运动前：停导览、释放摇杆锁、必要时站起。

        注意 mode 在站立时也可能一直是 0，判断姿态只认 body_height；
        1001 是阻尼错误码。之前用 mode==0 判断导致站立后被误判为趴下，
        反复重发恢复站立，把正在执行的站起动作打断。
        """
        if self.app.tour.active:
            self.app.tour.stop()
            if push:
                push({"op": "log", "text": "已停止展厅导览"})
        try:
            self.bridge.send_sport_bool(rb.SPORT_SWITCHJOYSTICK, False)
        except Exception:
            pass
        st = self.bridge.sport_state or {}
        height = float(st.get("body_height") or 0.0)
        err = int(st.get("error_code") or 0)
        if height >= 0.15 and err != 1001:
            return True
        if push:
            push({"op": "log", "text": "机器人处于趴下状态，正在站起…"})
        self.bridge.send_sport(rb.SPORT_RECOVERYSTAND)
        ok = await self._wait_standing(8.0)
        if not ok:
            log.warning("agent: stand-up did not complete (height=%s err=%s)",
                        st.get("body_height"), st.get("error_code"))
        return ok

    # ---- Ollama ----
    async def _chat(self, messages):
        model = self.acfg.get("model") or (self.cfg.get("ai") or {}).get(
            "model") or "gemma4:e2b-it-qat"
        payload = {
            "model": model,
            "messages": messages,
            "tools": TOOLS,
            "stream": False,
            "think": False,
            "options": {"temperature": 0.1, "num_ctx": 4096, "num_predict": 512},
        }
        timeout = aiohttp.ClientTimeout(total=180)
        async with aiohttp.ClientSession(timeout=timeout) as s:
            async with s.post(self.app.OLLAMA_API + "/api/chat", json=payload) as r:
                r.raise_for_status()
                data = await r.json()
        return data.get("message") or {}

    async def _vision(self, question, b64):
        model = self.acfg.get("vision_model") or (self.cfg.get("ai") or {}).get(
            "vision_model") or "gemma4:e2b-it-qat"
        payload = {
            "model": model,
            "messages": [{"role": "user",
                          "content": question or "用中文简要描述你看到的场景。",
                          "images": [b64]}],
            "stream": False,
            "think": False,
            "options": {"temperature": 0.2, "num_predict": 256},
        }
        timeout = aiohttp.ClientTimeout(total=120)
        async with aiohttp.ClientSession(timeout=timeout) as s:
            async with s.post(self.app.OLLAMA_API + "/api/chat", json=payload) as r:
                r.raise_for_status()
                data = await r.json()
        return ((data.get("message") or {}).get("content") or "").strip()

    # ---- 工具执行 ----
    async def _exec(self, name, args, push):
        handler = getattr(self, "_tool_" + name, None)
        if handler is None:
            return {"ok": False, "error": "未知工具 %s" % name}
        try:
            return await asyncio.wait_for(handler(args, push),
                                          timeout=self._tool_timeout(name, args))
        except asyncio.TimeoutError:
            self._stop_motion()
            return {"ok": False, "error": "工具 %s 超时" % name}
        except asyncio.CancelledError:
            raise
        except Exception as e:
            self._stop_motion()
            return {"ok": False, "error": "%s 失败: %s" % (name, e)}

    def _tool_timeout(self, name, args):
        if name in ("walk_to_color",):
            return float(args.get("max_seconds") or 40) + 8
        if name in ("follow_person",):
            return float(args.get("max_seconds") or 30) + 8
        if name == "goto_waypoint":
            return 150.0
        if name == "look":
            return 130.0
        if name == "speak":
            return 120.0
        return 30.0

    async def _tool_robot_status(self, args, push):
        snap = self.bridge.snapshot()
        pos, yaw = snap["position"], snap["yaw"]
        wps, frame, _ = self.app.waypoints_view()
        return {
            "ok": True,
            "pose": {"x": round(pos[0], 2), "y": round(pos[1], 2),
                     "yaw_deg": round(math.degrees(yaw), 1), "frame": frame},
            "sport": self.bridge.sport_state or {},
            "battery": self.bridge.battery or {},
            "lidar": self.bridge.lidar or {},
            "tour_active": bool(self.app.tour.active),
            "planner_running": bool(self.app.scan_running()),
            "waypoints": [w.get("name") for w in wps],
            "max_speed": self.speed_limit(),
        }

    async def _tool_stand_up(self, args, push):
        await self._ensure_ready(push)   # 趴下时这里会先恢复站立并等待
        self.bridge.send_sport(rb.SPORT_STANDUP)
        ready = await self._wait_standing(8.0)
        st = self.bridge.sport_state or {}
        return {"ok": bool(ready), "standing": bool(ready),
                "body_height": st.get("body_height"), "mode": st.get("mode"),
                "msg": "已站立" if ready else "站立未完成，请检查机器人是否被遮挡或倾斜"}

    async def _tool_sit(self, args, push):
        self.bridge.send_sport(rb.SPORT_SIT)
        await asyncio.sleep(1.5)
        return {"ok": True, "msg": "已坐下"}

    async def _tool_lie_down(self, args, push):
        """卧倒（StandDown，1005）——与运动模式页的「卧倒」同一个接口。"""
        self._stop_motion()
        self.bridge.send_sport(rb.SPORT_STANDDOWN)
        await asyncio.sleep(2.0)
        return {"ok": True, "msg": "已卧倒（StandDown）"}

    async def _tool_damp(self, args, push):
        """阻尼（1001）：卸力，仅用于急停/异常场景。"""
        self._stop_motion()
        self.bridge.send_damp()
        await asyncio.sleep(0.5)
        return {"ok": True, "msg": "已进入阻尼（卸力）"}

    async def _tool_balance_stand(self, args, push):
        await self._ensure_ready(push)
        self.bridge.send_sport(rb.SPORT_BALANCESTAND)
        await asyncio.sleep(1.5)
        return {"ok": True, "msg": "已平衡站立"}

    async def _tool_stop_move(self, args, push):
        self._stop_motion()
        return {"ok": True, "msg": "已停止移动"}

    async def _tool_move(self, args, push):
        vx = float(args.get("vx", 0.0) or 0.0)
        vy = float(args.get("vy", 0.0) or 0.0)
        vyaw = float(args.get("vyaw", 0.0) or 0.0)
        dur = float(args.get("duration_s", 1.0) or 1.0)
        if abs(vx) < 1e-6 and abs(vy) < 1e-6 and abs(vyaw) < 1e-6:
            return {"ok": False,
                    "error": "vx/vy/vyaw 都是 0，机器人不会移动；"
                             "向前走请传 vx>0（例如 0.2）"}
        ready = await self._ensure_ready(push)
        if not ready:
            return {"ok": False,
                    "error": "机器人没有站起来（body_height 仍低于 0.15），无法移动"}
        await self._move(vx, vy, vyaw, dur)
        st = self.bridge.sport_state or {}
        return {"ok": True, "moved": {"vx": vx, "vy": vy, "vyaw": vyaw,
                                      "seconds": round(dur, 2)},
                "body_height": st.get("body_height")}

    async def _move(self, vx, vy, vyaw, dur):
        limit = self.speed_limit()
        vx = max(-limit, min(limit, vx))
        vy = max(-0.2, min(0.2, vy))
        vyaw = max(-0.8, min(0.8, vyaw))
        # 固件对 <0.15m/s 的平移几乎不响应：要动就给到最小速度
        if 0.02 < abs(vx) < perception.MIN_VX:
            vx = perception.MIN_VX if vx > 0 else -perception.MIN_VX
        if vx < 0:
            back = (getattr(self.bridge, "lidar", None) or {}).get("back")
            if back is not None and back < 0.45:
                vx = 0.0        # 后方太近，不许后退
        dur = max(0.05, min(float(self.acfg.get("max_move_s", 3.0)), dur))
        self._motion_active = True
        self._motion_deadline = time.monotonic() + dur + 1.5
        try:
            end = time.monotonic() + dur
            while time.monotonic() < end:
                if self._cancel.is_set():
                    break
                self.bridge.send_move(vx, vy, vyaw)
                await asyncio.sleep(0.1)
        finally:
            self._motion_active = False
            self._stop_motion()

    async def _tool_turn(self, args, push):
        deg = float(args.get("degrees", 0.0) or 0.0)
        if abs(deg) < 1:
            return {"ok": True, "msg": "角度太小，未动作"}
        ready = await self._ensure_ready(push)
        if not ready:
            return {"ok": False, "error": "机器人没有站起来，无法转向"}
        rate = 0.6
        remain = abs(math.radians(deg))
        sign = 1.0 if deg > 0 else -1.0
        while remain > 0.01 and not self._cancel.is_set():
            chunk = min(remain, rate * float(self.acfg.get("max_move_s", 3.0)) * 0.9)
            await self._move(0.0, 0.0, sign * rate, chunk / rate)
            remain -= chunk
        return {"ok": True, "turned_deg": deg}

    async def _tool_find_color(self, args, push):
        color = str(args.get("color", "")).strip().lower()
        if color not in perception.COLOR_RANGES:
            return {"ok": False, "error": "不支持的颜色 %s，可用: %s" % (
                color, ", ".join(sorted(perception.COLOR_RANGES)))}
        img, age = perception.load_frame()
        if img is None:
            return {"ok": False, "error": "相机无新帧（age=%.1fs），检查 go2-cam.service" % age}
        det = perception.detect_color(img, color)
        self._push_frame(push)
        if det is None:
            return {"ok": True, "found": False, "color": color,
                    "hint": "当前画面没有看到该颜色，可以 turn 或 walk_to_color 让机器人自己找"}
        return {"ok": True, "found": True, "color": color, "detection": det,
                "center_offset": round((det["cx"] - 0.5) * 2.0, 2),
                "size_ratio": det["area_ratio"]}

    async def _tool_walk_to_color(self, args, push):
        color = str(args.get("color", "")).strip().lower()
        if color not in perception.COLOR_RANGES:
            return {"ok": False, "error": "不支持的颜色 %s" % color}
        max_s = min(float(args.get("max_seconds") or 40), 90.0)
        ready = await self._ensure_ready(push)
        if not ready:
            return {"ok": False, "error": "机器人没有站起来，无法行走"}
        self._motion_active = True
        self._motion_deadline = time.monotonic() + max_s + 5
        try:
            res = await perception.servo_to_color(
                self.bridge, color, max_s=max_s,
                speed=self.speed_limit(), cancel=self._cancel)
        finally:
            self._motion_active = False
            self._stop_motion()
        self._push_frame(push)
        res["ok"] = bool(res.get("reached"))
        return res

    async def _tool_follow_person(self, args, push):
        max_s = min(float(args.get("max_seconds") or 30), 60.0)
        ready = await self._ensure_ready(push)
        if not ready:
            return {"ok": False, "error": "机器人没有站起来，无法跟随"}
        self._motion_active = True
        self._motion_deadline = time.monotonic() + max_s + 5
        try:
            res = await perception.follow_person(
                self.bridge, max_s=max_s,
                speed=self.speed_limit(), cancel=self._cancel,
                distance=float(self.acfg.get("follow_distance", 0.75)))
        finally:
            self._motion_active = False
            self._stop_motion()
        res["ok"] = bool(res.get("followed"))
        return res

    async def _tool_look(self, args, push):
        q = args.get("question") or "用中文简要描述你看到的场景，重点说明前方有什么。"
        b64, age = perception.frame_b64()
        if b64 is None:
            return {"ok": False, "error": "相机无新帧（age=%.1fs），检查 go2-cam.service" % age}
        push({"op": "frame", "b64": b64})
        desc = await self._vision(q, b64)
        return {"ok": True, "description": desc, "frame_age_s": round(age, 2)}

    async def _tool_lidar_clearances(self, args, push):
        return perception.lidar_clearances(self.bridge)

    async def _tool_list_waypoints(self, args, push):
        wps, frame, map_name = self.app.waypoints_view()
        return {"ok": True, "frame": frame, "map": map_name,
                "waypoints": [{"name": w.get("name"), "x": w.get("x"),
                               "y": w.get("y"), "yaw": w.get("yaw")}
                              for w in wps]}

    async def _tool_goto_waypoint(self, args, push):
        name = str(args.get("name", "")).strip()
        wps, frame, map_name = self.app.waypoints_view()
        if not wps:
            return {"ok": False, "error": "还没有记录任何点位"}
        names = [str(w.get("name", "")) for w in wps]
        pick = None
        for w in wps:
            if str(w.get("name", "")) == name:
                pick = w
                break
        if pick is None:
            for w in wps:
                if name and name in str(w.get("name", "")):
                    pick = w
                    break
        if pick is None:
            close = difflib.get_close_matches(name, names, n=1, cutoff=0.4)
            if close:
                pick = next(w for w in wps if w.get("name") == close[0])
        if pick is None:
            return {"ok": False, "error": "找不到点位 %s，已有点位: %s" % (
                name, ", ".join(names))}
        x, y, yaw = float(pick["x"]), float(pick["y"]), float(pick.get("yaw", 0.0))
        if frame == "map":
            m = self.bridge.map_to_odom(x, y, yaw)
            if m is None:
                return {"ok": False, "error": "定位未就绪（map→odom TF 缺失）"}
            x, y, yaw = m
        if not self.app.scan_running():
            self.app._handle_scanner({"op": "start"})
            if push:
                push({"op": "log", "text": "SCAN-Planner 未运行，正在启动…"})
            await asyncio.sleep(12.0)
        if self.app.tour.active:
            self.app.tour.stop()
        self.bridge.send_goal(x, y, yaw)
        if push:
            push({"op": "log", "text": "已下发导航目标 %s (%.2f, %.2f)" % (
                pick.get("name"), x, y)})
        threshold = float(self.cfg.get("reach_threshold", 0.4))
        t0 = time.monotonic()
        timeout = 110.0
        while time.monotonic() - t0 < timeout:
            if self._cancel.is_set():
                self._stop_motion()
                return {"ok": False, "error": "导航被取消"}
            pos = self.bridge.snapshot()["position"]
            dist = math.hypot(pos[0] - x, pos[1] - y)
            if dist <= threshold:
                return {"ok": True, "arrived": True,
                        "waypoint": pick.get("name"),
                        "distance_m": round(dist, 2),
                        "seconds": round(time.monotonic() - t0, 1)}
            await asyncio.sleep(0.5)
        self._stop_motion()
        return {"ok": False, "error": "导航超时（%.0fs），可能被障碍物挡住" % timeout}

    async def _tool_remember_place(self, args, push):
        name = str(args.get("name", "")).strip()
        if not name:
            return {"ok": False, "error": "点位名称不能为空"}
        res = self.app._handle_waypoint({"op": "record", "name": name})
        if not res.get("ok"):
            return {"ok": False, "error": res.get("error", "记录失败")}
        return {"ok": True, "waypoint": res.get("waypoint"), "msg": "已记录点位 %s" % name}

    async def _tool_speak(self, args, push):
        text = str(args.get("text", "")).strip()
        if not text:
            return {"ok": False, "error": "没有要说的内容"}
        ok = await self.app.voice.speak(text)
        return {"ok": bool(ok), "spoken": text[:120]}

    async def _tool_emergency_stop(self, args, push):
        self.app._handle_emergency()
        return {"ok": True, "msg": "已急停并进入阻尼状态"}

    def _push_frame(self, push):
        b64, _age = perception.frame_b64()
        if b64:
            push({"op": "frame", "b64": b64})
