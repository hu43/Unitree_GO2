#!/bin/bash
# 一键启动 SCAN-Planner 模式2（关键点巡航）
# 用法: ./run_mode2.sh
# 说明: 读取 /home/jojo/keypoints.yaml 里的路点，机器人按顺序巡航
# 修改 keypoints_file 路径可切换路点文件

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

source setup_scan_planner.sh

# 关键点文件（默认用录制的；可改成其他路径）
KEYPOINTS_FILE="${KEYPOINTS_FILE:-/home/jojo/keypoints.yaml}"

if [ ! -f "$KEYPOINTS_FILE" ]; then
  echo "错误: 关键点文件 $KEYPOINTS_FILE 不存在！"
  echo "请先录制: ros2 run scan_planner keypoint_recorder.py --odom /utlidar/robot_odom --output $KEYPOINTS_FILE"
  exit 1
fi

echo "=============================================="
echo "  SCAN-Planner 模式2：关键点巡航"
echo "  路点文件: $KEYPOINTS_FILE"
echo "  桥接: bridge_enable=true (机器人会动！)"
echo "  急停: 遥控器在手 / Ctrl+C 停车"
echo "=============================================="

ros2 launch scan_planner real_robot.launch.py \
  navi_mode:=2 \
  keypoints_file:="$KEYPOINTS_FILE" \
  bridge_enable:=true
