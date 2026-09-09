#!/bin/bash
# text_chat 一键启动 (自动配置DDS环境, 与 largemodel 对话)
source /opt/ros/foxy/setup.bash
source /home/unitree/cyclonedds_ws/install/setup.bash
source /home/unitree/yahboom_ws/install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file:///home/unitree/hu/ai/cyclonedds_lm.xml
exec ros2 run text_chat text_chat
