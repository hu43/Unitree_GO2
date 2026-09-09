#!/bin/bash
# 重启 largemodel model_service (DDS 隔离版: largemodel 只绑 wlan0, 不吃 Go2 内网 eth0 的 DDS 洪水; 其他程序的 eth0 DDS 不受影响)
for p in $(pgrep -f "largemodel_control.launch.py"); do kill -9 $p 2>/dev/null; done
for p in $(pgrep -f "largemodel/lib/largemodel/"); do kill -9 $p 2>/dev/null; done
sleep 2
WLAN_IP=$(ip -4 -o addr show wlan0 | awk "{split(\$4,a,\"/\"); print a[1]}" | head -1)
sed "s/WLAN_IP_PLACEHOLDER/${WLAN_IP:-127.0.0.1}/" /home/unitree/hu/ai/cyclonedds_lm.xml.tpl > /home/unitree/hu/ai/cyclonedds_lm.xml
source /opt/ros/foxy/setup.bash
source /home/unitree/cyclonedds_ws/install/setup.bash
source /home/unitree/yahboom_ws/install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file:///home/unitree/hu/ai/cyclonedds_lm.xml
nohup ros2 launch largemodel largemodel_control.launch.py text_chat_mode:=true > /home/unitree/hu/ai/largemodel.log 2>&1 &
sleep 22
pgrep -f "largemodel/lib/largemodel/" > /dev/null && echo SERVICE-ALIVE || { echo SERVICE-DEAD; tail -5 /home/unitree/hu/ai/largemodel.log; }
