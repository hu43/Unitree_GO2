#!/bin/bash
# 一键启动 SCAN-Planner 模式3（参考路径跟踪 + 局部避障）
# 用法: ./run_mode3.sh
# 说明: 订阅 /initial_path 作为全局参考路径，SCAN-Planner 贴路并局部避障

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

source setup_scan_planner.sh

echo "=============================================="
echo "  SCAN-Planner 模式3：参考路径跟踪 + 局部避障"
echo "  桥接: bridge_enable=true (机器人会动！)"
echo "  输入: 向 /initial_path 发布 nav_msgs/Path"
echo "  急停: 遥控器在手 / Ctrl+C 停车"
echo "=============================================="

ros2 launch scan_planner real_robot.launch.py \
  navi_mode:=3 \
  bridge_enable:=true
