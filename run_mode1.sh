#!/bin/bash
# 一键启动 SCAN-Planner 模式1（2D Nav Goal 手动导航）
# 用法: ./run_mode1.sh
# 说明: 启动后在 RViz 用 2D Nav Goal 点目标

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

source setup_scan_planner.sh

echo "=============================================="
echo "  SCAN-Planner 模式1：2D Nav Goal 导航"
echo "  桥接: bridge_enable=true (机器人会动！)"
echo "  操作: RViz 顶部选 2D Nav Goal 点目标"
echo "  急停: 遥控器在手 / Ctrl+C 停车"
echo "=============================================="

ros2 launch scan_planner real_robot.launch.py \
  navi_mode:=1 \
  bridge_enable:=true
