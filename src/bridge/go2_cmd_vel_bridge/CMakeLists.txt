cmake_minimum_required(VERSION 3.8)
project(go2_cmd_vel_bridge)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(unitree_api REQUIRED)

# ---------------------------------------------------------------------------
# go2_cmd_vel_bridge node (uses the official ROS 2 sport client)
# ---------------------------------------------------------------------------
add_executable(go2_cmd_vel_bridge_node
  src/go2_cmd_vel_bridge_node.cpp
  src/ros2_sport_client.cpp)

target_include_directories(go2_cmd_vel_bridge_node PRIVATE include)

ament_target_dependencies(go2_cmd_vel_bridge_node
  rclcpp geometry_msgs std_msgs unitree_api)

install(TARGETS go2_cmd_vel_bridge_node
  DESTINATION lib/${PROJECT_NAME})

# ---------------------------------------------------------------------------
# TF broadcaster (odom -> base_link) for the onboard odometry
# ---------------------------------------------------------------------------
add_executable(odom_tf_broadcaster src/odom_tf_broadcaster.cpp)

ament_target_dependencies(odom_tf_broadcaster
  rclcpp nav_msgs tf2_ros geometry_msgs)

install(TARGETS odom_tf_broadcaster
  DESTINATION lib/${PROJECT_NAME})

ament_package()
