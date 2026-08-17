#!/bin/bash
# Source the SCAN-Planner real-robot environment.
#
# Usage:
#   source ~/unitree_sdk2-main/example/user/SCAN-Planner/setup_scan_planner.sh
#   ros2 launch scan_planner real_robot.launch.py
#
# Order matters: humble first, then unitree_ros2 (provides unitree_api/unitree_go
# messages used by go2_cmd_vel_bridge), then this workspace.

source /opt/ros/humble/setup.bash
source "$HOME/unitree_ros2/install/setup.bash"

# ROS2 talks to the robot's onboard DDS over the robot LAN interface.
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# conda's python3 (3.13) breaks ROS2 python tools (rclpy is built for the
# system python3.10). Push conda's bin behind the system paths so `python3`
# and `ros2 run` resolve to the system interpreter.
if [[ "$PATH" == *"miniconda3"* ]]; then
  export PATH="$(echo "$PATH" | tr ':' '\n' | grep -v 'miniconda3/bin' | tr '\n' ':')"
fi

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$SCRIPT_DIR/install/setup.bash"

echo "[SCAN-Planner] environment ready (RMW: $RMW_IMPLEMENTATION)"

