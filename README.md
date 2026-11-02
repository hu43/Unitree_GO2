# Go2 展厅导览控制台（Web）

基于 SCAN-Planner 的 Go2 X 真机网页控制台：**手机预设路径点 → 自主避障导航 → 展厅解说导览**。

- **后端**：Python（rclpy + websockets），轻量 WebSocket 桥，对接已运行的 SCAN-Planner 栈
- **前端**：2D 俯视地图 + 虚拟摇杆 + 地图点选 + 展品点位管理 + 导览模式
- **独立环境**：依赖隔离在 `venv/`，不污染系统 Python

## 目录

```
hu/go2/web/
├── app.py           # 后端入口（WebSocket + HTTP + 导览状态机）
├── ros_bridge.py    # ROS2 桥（订阅地图/位置，发布导航/控制）
├── config.json      # 配置（展品点位、速度、话题名）—— 网页可改
├── static/          # 前端（index.html / style.css / app.js）
├── venv/            # 独立 Python 环境（websockets）
├── run_web.sh       # 一键启动
└── README.md
```

## 前置条件（必须）

SCAN-Planner 真机栈正在后台运行（`navi_mode=1` 才能响应 2D Nav Goal）：

```bash
cd ~/unitree_sdk2-main/example/user/SCAN-Planner
source setup_scan_planner.sh
ros2 launch scan_planner real_robot.launch.py navi_mode:=1 bridge_enable:=true
```

## 启动

```bash
cd ~/hu/go2/web
./run_web.sh
```

浏览器（电脑/手机同一局域网）访问 **`http://<本机IP>:8080`**。

页面结构：
- **`/`（主面板）**：Go2 机器狗主面板，显示系统/机器人状态概览，点击「进入展厅导览控制台」跳转
- **`/console`（导览控制台）**：完整的导览控制台（地图/摇杆/展品/语音），顶部有「返回主面板」按钮
（手机需连到与机器人相同的网络，如 Go2 的 WiFi 或同一路由器。）

## 功能

### 🕹️ 运动控制
- 虚拟摇杆：拖动控制前进/转向/横移，松手即停
  - **摇杆走独立 sport 通道**（`/api/sport/request` 的 Move，同官方遥控器 SportClient.Move），**不与 SCAN-Planner 运控冲突**
- 按钮：站立 / 解除锁定 START（RecoveryStand）/ 卧倒（StandDown）/ **急停**
- **⛔ 急停**：一键停止所有——终止导览、关闭 SCAN-Planner 全部进程、停摇杆、**机器人立即进入阻尼状态（Damp）**，无论当前在干什么
- 速度滑块：限制最大速度（`max_speed`）

### ⚙️ SCAN-Planner 系统控制（导览台内）
- **🚀 启动 SCAN-Planner**：一键运行真实机器人导航栈（`navi_mode:=1 bridge_enable:=true`，机器人会站立）
- **⏹ 停止**：终止 SCAN-Planner 全部节点
- 日志：`web/log/scan_planner.log`
- 状态实时显示（运行中/已停止）

### 🔀 控制模式（摇杆 ↔ 导览自动切换）
- **摇杆模式**（默认）：网页摇杆直接控制机器人（sport Move），SCAN-Planner 只做建图，不参与运控
- **导览模式**：点「开始导览」→ 控制权交给 SCAN-Planner（发 2D Nav Goal，closed_loop_controller 经 bridge 控制），**摇杆自动禁用**（页面半透明提示）
- **任意时刻动摇杆**：自动停止导览（发 StopMove + 让 SCAN-Planner 回 WAIT_TARGET）并切回摇杆控制
- 急停按钮在两种模式下都可用

### 🗺️ 地图
- 2D 俯视占据栅格地图（来自 `/grid_map/occupancy`，按高度着色）
- 机器人实时位置 + 朝向箭头
- 拖拽平移、滚轮缩放、一键居中
- **紧凑布局**：地图缩小（桌面 42% 宽、手机顶部小条），**以按键控制为主**，地图只做参考查看

### 📍 展品点位（可配置，存 `config.json`）
两种方式设置点位：
1. **摇杆 + 记录**：把机器人开到展品前 → 点「记录当前点」→ 存真实坐标
2. **地图点选**：切到「加路径点」模式 → 点地图任意位置 → 生成点位

列表可勾选、删除；**每个展品可编辑介绍语音文字**（点开列表项输入文字 + 🔊 试听）；改动自动保存到服务器 `config.json`。

### 🗣️ 语音导览（展厅解说）
- **到达展品自动介绍**：导览中机器人到达展品点 → 自动播放该展品语音介绍 → **介绍完才进入下一个展品**
- **文字 → 语音**：介绍文字用 `edge-tts`（微软中文神经网络语音）实时合成，缓存到 `audio/`（文字改了自动重新合成）
- **试听**：编辑介绍文字后点 🔊 试听，先听效果再保存
- **跳过介绍**：导览中可随时「⏭ 跳过介绍」直接进入下一展品
- **DIY 编排**：展品点位 + 介绍文字自由增删改，完全自定义
- 播放设备：后端所在电脑的扬声器（ffplay 播放到声卡）；展厅建议把导览台电脑接音箱
- 可选自定义音频：展品配置加 `"audio": "文件名.mp3"`（放 `audio/` 目录）则优先播放该文件

### 🔄 转向 + 动作编排（每展品可配）
到达展品后流程：**转向（转到指定朝向）→ 动作 + 语音介绍（同步进行）→ 全部完成才进入下一展品**

每个展品可独立编排：
- **转向角度（°）**：到达后原地旋转到指定朝向（相对地图，如 90° 面向观众）；设 0 = 不转向
- **动作**（参考宇树官方 sports_services 文档）：
  - 👋 打招呼（Hello）/ ❤️ 比心（Heart）/ 💃 舞蹈1（Dance1）/ 🕺 舞蹈2（Dance2）
  - 🧘 伸懒腰（Stretch）/ 🙏 拜年作揖（Scrape）/ ⚖️ 平衡站（BalanceStand）/ 无动作
- 动作与语音介绍**同步执行**，都完成才去下一个
- **自动防冲突**：转向/动作期间，web 端自动暂停 bridge 的 Move 指令（`/go2/motion_pause`）并让 SCAN-Planner 释放控制，完成后恢复——动作不会被运控打断
- 页面展品列表可直接编辑（转向角度输入 + 动作下拉），改动自动保存 `config.json`
- 注：Go2 官方文档没有"握手"和"坐下"动作，因此不提供（不瞎猜）

### 🚶 导览模式（自主避障导航）
1. 在展品列表**勾选**要导览的展品（顺序 = 勾选顺序）
2. 点「开始导览」→ 后端依次向 `/move_base_simple/goal` 发路径点
3. SCAN-Planner 自主避障规划，机器人沿路点行走
4. 到达一个点（距离 < `reach_threshold`）→ 自动发下一个
5. 页面实时显示导览进度 + 当前目标

> 后续可在每个展品点挂接解说（预留：导览到点时前端可触发语音/文案）。

## 🎙️ 离线 AI 语音对话（`/ai` 模块2）

机器人麦克风 → 本地识别 → 本地大模型 → 机器人扬声器，**全程离线**，可在页面顶部用总开关手动开启/关闭。

- **麦克风**：订阅 DDS `/audiosender`（`unitree_go/AudioData`，50Hz、20ms Opus 单声道 16kHz），用系统 `libopus` 解码。不依赖容器里的 ALSA 采集设备（容器的 `/dev/snd` 是启动时快照）。
- **识别**：`sherpa-onnx` + Paraformer 中文模型（`models/sherpa-onnx-paraformer-zh-2023-09-14`）。
- **对话**：本地 Ollama（`ai.model`，默认 `gemma4:e2b-it-qat`），人设可在页面「人设 / 展品知识库」里编辑，也会自动带上各展品的讲解文字。
- **合成**：`sherpa-onnx` VITS Melo 中文模型（`models/vits-melo-tts-zh_en`），复用现有 `ffmpeg -f alsa` 播放链路。默认**离线优先**（`ai.voice.prefer_offline`）：先本地合成，失败才试 edge-tts；edge-tts 失败后进入 5 分钟冷却，试听/导览不再每次白等 10 秒。
- **流式解说**：解说按句切分（≤20 字），首句合成完就开始播放，后面的句子边播边合成；保存解说文字时和启动后都会在后台预合成到缓存（`audio/tts_s_*.wav`），所以试听/导览通常 1-2 秒就出声。Jetson 上 VITS 约 1 倍实时速度，首次遇到全新长文本时首句要等 4-6 秒。
- **音量**：页面顶部「🔊 扬声器音量」滑块 0-100%（平方曲线，保存到 `ai.voice.volume`），对 AI 回复和导览解说都生效。USB 声卡混音器固定 100%；离线合成结果在写盘前归一化到 0.95 峰值，保证每句话都是满量程（模型原始输出峰值只有 0.14-0.39）。需要更响可在 `ai.voice.gain_db` 设 3-6（数字放大 + `alimiter` 限幅，会压缩动态），默认 0。
- **用法**：按住「🎤 按住说话」说话，松开（或静音 0.8s）自动结束；也可以直接打字。
- **总开关**：关闭后后端拒绝所有 AI/语音/Agent 请求，并调用 Ollama `keep_alive:0` 卸载模型（释放约 4GB 内存）；重新打开即恢复。

安装依赖与模型（首次部署）：

```bash
cd ~/hu/go2/web
./install_ai.sh          # 容器内 pip 安装 sherpa-onnx + 从 hf-mirror 下载模型
```

## 🧠 智能 AI 控制（`/ai` 模块3）

自研 Ollama 工具调用 Agent（`agent.py`）：把 DDS 运控、摄像头、激光雷达、导航包装成工具，由本地模型自主编排。LLM 只负责选工具/看结果/说结论，闭环动作在 Python 里确定性执行。

**工具清单**

| 类别 | 工具 |
|---|---|
| 运控 | `robot_status` `stand_up` `sit` `damp` `balance_stand` `stop_move` `move` `turn` |
| 视觉 | `look`（抓帧+VLM 描述）`find_color`（颜色检测）`walk_to_color`（走到色块前）`follow_person`（跟随人） |
| 雷达 | `lidar_clearances`（前后左右最近障碍） |
| 导航 | `list_waypoints` `goto_waypoint`（SCAN-Planner 自主避障）`remember_place` |
| 其它 | `speak` `emergency_stop` |

**示例任务**（页面快捷按钮已内置）

- 走到蓝色方块那里 → `walk_to_color`：HSV 色块检测 + 视觉伺服（对准→前进→面积达标停），丢失 8s 放弃。
- 跟着我走30秒 → `follow_person`：YOLOX 行人检测（`models/yolox_s.onnx`，416 输入，Orin 上约 0.28s/帧），居中转向 + **雷达窄前向距离控距**（默认目标 0.75m，0.65~0.85m 保持、更远前进、更近后退；雷达无读数时退回框高估算），人消失 1.5s 自动停。目标距离可改 `agent.follow_distance`。机器狗视角低，人靠近时画面里只有腿，HOG 全身检测会失效，所以用 YOLOX；模型缺失时自动退回 HOG。
- 看看前方有什么 → `look`：前视相机 + 视觉模型描述。
- 把这里记成老板办公室 → `remember_place`；之后「去老板办公室」会调 `goto_waypoint`（需要 SCAN-Planner 在跑，未运行会自动启动）。
  - Go2 没有机械臂，物理「取东西」做不到，只能到点、观察、播报。

**摄像头**：容器内直接向 `/api/videohub/request` 发 `api_id=1001` 请求，响应 `Response.binary` 即 JPEG，5Hz 缓存在内存（`/api/ai/frame.jpg` 可看缩略图）。不再需要主机侧抓帧进程。

**安全边界（写死在 Python 侧，LLM 无法绕过）**

- 单任务互斥；任务上限 180s，单次 `move` ≤3s，速度上限取 `agent.max_speed` 与全局 `max_speed` 的较小值。
- 运动前自动停导览、释放摇杆锁、趴下时自动站起；所有运动路径 `finally` 归零 + `StopMove`。
- 运动看门狗：超过本次动作截止时间仍在动，立即归零并取消任务。
- 页面「⏹ 停止」「⛔ 急停」都会立即取消 Agent；发起任务的页面断开连接也会取消并归零。

## 配置说明（config.json）

| 键 | 默认 | 说明 |
|---|---|---|
| `host` / `port` | `0.0.0.0` / `8080` | 监听地址（0.0.0.0 = 局域网可访问） |
| `topics` | — | ROS2 话题名（一般不用改） |
| `map_downsample` | 4000 | 地图点降采样上限（流畅度） |
| `push_hz` | 10 | 地图推送频率 |
| `reach_threshold` | 0.4 | 导览到达判定距离（米） |
| `max_speed` | 0.5 | 摇杆最大速度（米/秒） |
| `waypoints` | — | 展品点位数组（网页可增删改） |
| `ai.enabled` | `true` | 离线 AI 总开关（页面顶部可切，关闭时卸载模型） |
| `ai.model` / `ai.vision_model` | `gemma4:e2b-it-qat` | 对话/视觉模型（需支持 tools；`gemma3:4b` 不支持工具调用） |
| `ai.system_prompt` | 空 | 人设，页面可编辑 |
| `ai.inject_waypoints` | `true` | 自动把展品讲解拼进对话上下文 |
| `ai.voice.*` | — | 语音模型名、VAD 门限、静音时长、最长一句话 |
| `agent.max_speed` | 0.3 | Agent 限速（页面滑块可调） |
| `agent.max_steps` | 12 | 工具调用最大步数 |
| `agent.max_move_s` | 3.0 | 单次 move 最长秒数 |
| `agent.follow_distance` | 0.75 | 跟随目标距离（米），雷达控距 |

## 常见问题

- **页面连不上（红色未连接）**：确认 SCAN-Planner 在跑、后端启动了、手机与机器人在同一网络
- **地图空白**：等 SCAN-Planner 建图几秒；确认 `/grid_map/occupancy` 有数据（`ros2 topic hz /grid_map/occupancy`）
- **网页地图点击没反应**：导航点击请使用 `/console` 的「设目标」模式；`/mapview` 是查看器，普通点击不会发送导航目标。确认 `navi_mode=1`，并检查 `/sportmodestate` 与 `/sportmodestate_odom` 都有数据（`ros2 topic hz /sportmodestate`、`ros2 topic hz /sportmodestate_odom`）。两者 DDS 节点须使用同一网络接口配置；规划器日志出现 `no odom` 或 `Ignore RViz goal before receiving initial body pose` 时，目标会被丢弃。
- **导览不动**：确认 SCAN-Planner 是 `navi_mode=1`；检查机器人在 odom 系位置与路径点距离
- **conda 冲突**：`run_web.sh` 已自动剔除 miniconda；若手动运行用 `./venv/bin/python3 app.py`
- **语音一直「没听清」**：先看 `/api/ai/status` 里 `mic_alive` 是否为 true（`/audiosender` 有没有数据）；Go2 内置麦克风增益由固件控制、底噪约 400，靠近机器狗说话效果最好。模型没装时识别会报错，可先打字。
- **机器人不发声**：`/ai/status` 的 `tts_ready` 为 false 时先看 `models/vits-melo-tts-zh_en` 是否齐全（`./install_ai.sh` 可补）；edge-tts 报 403 是微软端点变更，导览解说会自动走离线合成。另外 USB 声卡重新枚举后，容器 `/dev/snd` 快照可能对不上，重启容器或 `sudo systemctl restart go2-web` 可恢复。
- **声音还是小**：硬件混音器已 100%（`amixer -c 1 sget PCM`），合成结果也已归一化；再想更响就调 `ai.voice.gain_db`（3-6，带限幅）并重启服务，或直接用页面顶部的音量滑块。
- **导览控制台「试听」没声音**：默认离线优先，首次合成要 3-5 秒（模型冷加载更久），之后命中缓存约 1-2 秒；`grep "TTS offline\|tts_s_" web.log` 可确认合成路径。若出现 `playback FAILED`，按上一条检查声卡。
- **跟随找不到人**：`perception.py` 优先用 `models/yolox_s.onnx`（约 0.9 置信度能认出局部身体），模型缺失才会退回 HOG（对腿脚基本无效）。检查 `grep "person detector" web.log` 看用的是哪个检测器。
- **Agent 说看不到画面**：`/api/ai/frame.jpg` 能出图说明相机正常；首次调用视觉模型要加载 4.6B 模型，等 1-2 分钟。`grep -i "camera:" web.log` 看抓帧失败计数。
- **Agent 不动**：确认页面总开关和 `agent.enabled` 都开着；任务里是否明确要求移动；`robot_status` 里 `mode=0` 时第一次运动会自动站起，需要 2-3 秒。
- **第一次 AI 回复很慢**：模型冷加载 + Jetson CPU 推理，`gemma4:e2b` 约 5-10 tok/s，属正常。关闭总开关会卸载模型，下次开启重新加载。
