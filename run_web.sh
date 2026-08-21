#!/bin/bash
# Start the Go2 web console.
# Usage: ./run_web.sh
# Precondition: SCAN-Planner stack is running in navi_mode=1:
#   cd ~/unitree_sdk2-main/example/user/SCAN-Planner && ./run_mode1.sh
#   (or: ros2 launch scan_planner real_robot.launch.py navi_mode:=1 bridge_enable:=true)

set -e
cd "$(dirname "$0")"

# ---- ROS2 environment (needed by rclpy + unitree_api) ----
source /opt/ros/humble/setup.bash
source "$HOME/unitree_ros2/install/setup.bash"
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# ---- drop conda from PATH (its python breaks rclpy) ----
if [[ "$PATH" == *miniconda* ]]; then
  export PATH="$(echo "$PATH" | tr ':' '\n' | grep -v 'miniconda' | tr '\n' ':')"
fi

# ---- isolated venv (provides websockets, sees system rclpy) ----
export VIRTUAL_ENV="$PWD/venv"
export PATH="$PWD/venv/bin:$PATH"

echo "=============================================="
echo "  Go2 展厅导览控制台"
echo "  本机访问:   http://127.0.0.1:8080"
echo "  手机访问(同一 WiFi):"
hostname -I 2>/dev/null | tr ' ' '\n' | grep -v '^$' | while read ip; do
  echo "    http://$ip:8080"
done
echo "  需 SCAN-Planner 在后台运行 (navi_mode=1)"
echo "  Ctrl+C 退出"
echo "=============================================="

exec python3 app.py "$@"
