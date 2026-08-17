<div align="center">
  <h1>SCAN-Planner · 真机部署</h1>
  <h2>Spatial Collision-Aware Local Planning for<br/>Route-Guided Long-Range Quadruped Navigation</h2>
  <p>
    <a href="https://arxiv.org/abs/2606.19555"><img alt="论文" src="https://img.shields.io/badge/Paper-arXiv-b31b1b?logo=arxiv&logoColor=white"/></a>
    <a href="https://wuyi2121.github.io/SCAN-Planner/"><img alt="项目主页" src="https://img.shields.io/badge/Project_Page-Website-4A90E2?logo=googlechrome&logoColor=white"/></a>
  </p>
</div>

SCAN-Planner 是一款面向足式机器人的**空间碰撞感知局部规划器**（Spatial Collision-Aware Local Planning），为上层任务（自主探索、视觉语言导航等）提供稳健的底层规划基础。

本仓库在 [wuyi2121/SCAN-Planner](https://github.com/wuyi2121/SCAN-Planner) 的 ROS 2 社区移植版基础上，完成了 **Unitree Go2 X 真机部署**：板载雷达直连、本地规划器 + 闭环控制器 + SDK2 桥接、录制路点巡航，全程在 Ubuntu 22.04 / ROS 2 Humble 下验证通过。

核心算法、项目设计与原始研究工作归功于 Han Zheng、Zhe Chen、Yiwen Fu、Ming Yang 和 Tong Qin。本仓库的部署适配与工程集成由维护者完成，不代表原作者的官方发布或认可。

---

## 🏗️ 系统架构（真机）

```
┌─────────────── Go2 X 板载（自带） ───────────────┐
│  /utlidar/cloud_deskewed   PointCloud2  odom系点云  │
│  /utlidar/robot_odom       Odometry    odom→base_link│
│  （板载已跑好雷达驱动 + LIO 定位）                   │
└──────────────────────┬──────────────────────────────┘
                       │ ROS 2 DDS（rmw_cyclonedds，网线连接）
        ┌──────────────┴─────────────────┐
        │  本机 PC（Ubuntu 22.04 + Humble）│
        │  ① odom_tf_broadcaster         │  板载里程计 → TF
        │  ② scan_planner_node           │  点云+里程计 → 占据栅格 + B样条轨迹
        │  ③ closed_loop_controller      │  轨迹 → /cmd_vel
        │  ④ go2_cmd_vel_bridge          │  /cmd_vel → /api/sport/request → Go2
        └─────────────────────────────────┘
```

**关键设计**：Go2 X 板载自带雷达与定位，本机只需运行规划器 + 控制器 + 桥接，无需外接雷达或额外 LIO。

---

## ✅ 已实现功能

- **实时建图**：占据栅格地图（RViz 可视化，RELIABLE QoS 适配）
- **模式 1**：2D Nav Goal 手动导航（RViz 点目标即走）
- **模式 2**：关键点录制 + 自动巡航（`keypoint_recorder.py` + waypoints）
- **模式 3**：参考路径跟踪 + 局部避障（订阅 `/initial_path`）
- **安全桥接**：速度限幅 + 指令超时自动停车 + 退出 Damp/StandDown
- **一键环境**：`setup_scan_planner.sh` 自动处理 conda/RMW/unitree_ros2 依赖

---

## 🛠️ 环境要求

| 项目 | 要求 |
|---|---|
| 机器人 | Unitree Go2 X（自带 L2 雷达 + 板载定位） |
| 系统 | Ubuntu 22.04 |
| ROS | ROS 2 Humble（`rmw_cyclonedds_cpp`） |
| 依赖 | `unitree_ros2`（提供 `unitree_api` 消息）、`libarmadillo-dev` |
| 网络 | 本机 ↔ Go2 网线直连（`enp3s0`，`192.168.123.x`） |

## 📦 构建

```bash
cd ~/unitree_sdk2-main/example/user/SCAN-Planner
bash --noprofile --norc -c 'export PATH=/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin; source /opt/ros/humble/setup.bash; source ~/unitree_ros2/install/setup.bash; colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release'
```

> ⚠️ 不要用 conda 的 cmake/python 编译（会污染链接导致 gdal/libcurl 符号错误）。

## 🚀 快速启动

```bash
cd ~/unitree_sdk2-main/example/user/SCAN-Planner
source setup_scan_planner.sh
```

**模式 1：2D Nav Goal 手动导航**
```bash
./run_mode1.sh
```

**模式 2：关键点巡航**（先录制路点）
```bash
ros2 run scan_planner keypoint_recorder.py --odom /utlidar/robot_odom --output /home/jojo/keypoints.yaml
# 按键：a/Enter=记录当前点  l=列表  s=保存  q=保存退出
./run_mode2.sh   # 读取 /home/jojo/keypoints.yaml 巡航
```

**模式 3：参考路径跟踪 + 局部避障**
```bash
./run_mode3.sh
# 另开终端向 /initial_path 发布 nav_msgs/Path
```

## 🎛️ 关键参数

- `bridge_enable`：`false`=干跑（纯观察不动机器人），`true`=真机运动
- `max_vx/max_vy/max_vyaw`：桥接速度限幅（安全）
- `cmd_timeout`：无指令超时自动停车（默认 0.3s）
- `navi_mode`：1=Nav Goal，2=关键点，3=参考路径
- `grid_map.ground_height / body_height`：地图与碰撞高度（爬坡/下楼时调整）
- `cloud_topic`：默认 `/utlidar/cloud_deskewed`（odom 系点云）

详细参数与排障见 [使用教程.md](使用教程.md)。

---

## 📚 致谢与引用

- 规划器框架：**[EGO-Planner](https://github.com/ZJU-FAST-Lab/ego-planner)**、[ROG-Map](https://github.com/hku-mars/ROG-Map)
- 定位参考：**[Elevator-LIO](https://github.com/xiaofan4122/Elevator-LIO)** / FAST-LIO2
- 仿真：**[MARSIM](https://github.com/hku-mars/MARSIM)**、[Mockamap](https://github.com/HKUST-Aerial-Robotics/mockamap)
- ROS 2 移植：基于 [wuyi2121/SCAN-Planner](https://github.com/wuyi2121/SCAN-Planner) 社区 `ros2-community` 分支

```bibtex
@article{zheng2026scan,
  title={SCAN-Planner: Spatial Collision-Aware Local Planning for Route-Guided Long-Range Quadruped Navigation},
  author={Zheng, Han and Chen, Zhe and Fu, Yiwen and Yang, Ming and Qin, Tong},
  journal={arXiv preprint arXiv:2606.19555},
  year={2026}
}
```

## ⚖️ License

Apache License 2.0（本仓库的部署适配沿用上游许可；`src/drivers/` 内第三方组件保留各自许可，见 [NOTICE](NOTICE)）。
