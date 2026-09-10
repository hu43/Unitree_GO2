# 🐕 Unitree Go2 展厅导览系统

基于 **Unitree Go2 X** 的完全自主展厅导览解决方案：SCAN-Planner 避障规划 + 网页控制台 + 本地大模型 AI（对话、图像/视频分析）。

<div align="center">
  <b>机器人完全自主运行 · 手机网页操控 · 本地多模态 AI</b>
</div>

---

## 🏗️ 系统架构

```
┌──────────── Jetson Orin Nano（机载电脑，Docker Humble 容器）────────────┐
│  SCAN-Planner（避障规划）   go2_cmd_vel_bridge（运控）    Ollama（gemma3:4b）│
│       ▲ /utlidar/* 板载雷达+定位        ▲ /api/sport         网页 AI 后端  │
└───────────────┬────────────────────────┬───────────────────┬───────────┘
                │ DDS（网线直连）          │ sport 指令          │ :8080
         Go2 X 机器人 ◀──────────────────┘        手机/电脑网页 ◀┘（WiFi）
```

- **Go2 X 板载**：L2 级激光雷达 + 定位（`/utlidar/*`），无需外接雷达
- **Jetson 容器**：SCAN-Planner + 运控桥接 + 网页后端 + Ollama 大模型
- **手机/电脑**：网页操控（主面板 → 导览控制台 / AI 控制界面）

## 📂 仓库分支

| 分支 | 内容 |
|---|---|
| **main** | 网页导览系统（最新：含 AI 控制面板，本目录） |
| **scan-planner** | SCAN-Planner ROS2 真机部署案例（ros2-community 分支 + 真机修改） |
| **go2-ai** | AI 控制面板初版存档 |

## 🖥️ 网页功能

### 导览控制台（/console）
- **2D 占据地图**（板载雷达实时建图）+ 机器人位置/朝向
- **虚拟摇杆**运动控制（独立 sport 通道，与规划器互不干扰）
- **展品点位**：摇杆记录 / 地图点选 / 不可用时重录 / 顺序编排（↑↓）
- **自主导览**：按编排顺序避障走位 → 到达展品转向 + 动作 + 语音介绍（同步）→ 下一展品
- **SCAN-Planner 一键启停**
- **急停**：停止一切 + 机器人立即阻尼

### AI 控制界面（/ai）
- **基础 AI 对话**：本地 Ollama gemma3:4b，多轮上下文，`/bye` 或按钮结束
- **图像/视频分析**：📷 拍照 / 📁 导入图片视频 → gemma3:4b 多模态分析（视频自动抽 3 帧），中文回复
- 🏛️ 展厅专用 AI 对话、🧠 智能 AI 控制（占位，开发中）

## 🛠️ 部署

### Jetson 机载（正式运行）
系统已部署为 Docker 容器 + systemd 自启，详见机载 `/home/unitree/hu/go2/README.md`。
- 访问：`http://<Jetson-IP>:8080`
- 重启整套：`sudo systemctl restart go2-tour.service`

### PC 开发
```bash
cd hu/go2/web
python3 -m venv venv --without-pip 2>/dev/null; venv/bin/python get-pip.py  # 或已建好的 venv
venv/bin/pip install websockets edge-tts
# 需要 ROS2 Humble 环境（rclpy）+ unitree_ros2 的 unitree_api 消息包
source /opt/ros/humble/setup.bash && source <unitree_ros2>/install/setup.bash
venv/bin/python app.py
```

### 容器内编译 SCAN-Planner（改代码后）
```bash
docker exec -it go2_humble bash
source /opt/ros/humble/setup.bash
cd /home/unitree/hu/go2
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release \
  --packages-select unitree_api scan_planner_msgs bspline_opt plan_env path_searching traj_utils scan_planner go2_cmd_vel_bridge
```

## ⚙️ 关键参数（config.json）

| 键 | 说明 |
|---|---|
| `max_speed` | 摇杆最大速度（m/s） |
| `reach_threshold` | 导览到达判定距离（m） |
| `waypoints` | 展品点位（x/y/z/yaw/转向角/动作/介绍文字），网页可编辑自动保存 |

## 🙏 致谢

- [SCAN-Planner](https://github.com/wuyi2121/SCAN-Planner)（Han Zheng 等，arXiv:2606.19555）及其 ROS2 社区移植
- [unitree_ros2](https://github.com/unitreerobotics/unitree_ros2) / [unitree_sdk2_python](https://github.com/unitreerobotics/unitree_sdk2_python)
- [Ollama](https://ollama.com) / Google gemma3:4b（本地多模态大模型）
- 亚博智能（largemodel 多模态教程项目）

## ⚖️ License

Apache-2.0（沿用上游；第三方组件见各自许可）
