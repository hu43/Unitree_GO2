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
- 按钮：站立 / 坐下 / 蹲下 / 急停（发 `/api/sport/request`）
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

### 🚶 导览模式（自主避障导航）
1. 在展品列表**勾选**要导览的展品（顺序 = 勾选顺序）
2. 点「开始导览」→ 后端依次向 `/move_base_simple/goal` 发路径点
3. SCAN-Planner 自主避障规划，机器人沿路点行走
4. 到达一个点（距离 < `reach_threshold`）→ 自动发下一个
5. 页面实时显示导览进度 + 当前目标

> 后续可在每个展品点挂接解说（预留：导览到点时前端可触发语音/文案）。

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

## 常见问题

- **页面连不上（红色未连接）**：确认 SCAN-Planner 在跑、后端启动了、手机与机器人在同一网络
- **地图空白**：等 SCAN-Planner 建图几秒；确认 `/grid_map/occupancy` 有数据（`ros2 topic hz /grid_map/occupancy`）
- **导览不动**：确认 SCAN-Planner 是 `navi_mode=1`；检查机器人在 odom 系位置与路径点距离
- **conda 冲突**：`run_web.sh` 已自动剔除 miniconda；若手动运行用 `./venv/bin/python3 app.py`
