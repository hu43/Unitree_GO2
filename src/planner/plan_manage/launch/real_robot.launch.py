"""Real-robot bring-up for SCAN-Planner on Go2 X (onboard built-in radar).

Data flow (onboard topics provided by the robot's own ROS2 stack):
  Go2 X onboard:
      /utlidar/cloud_base   (PointCloud2, frame=base_link)
      /utlidar/robot_odom   (Odometry,     odom -> base_link)
  then on this PC:
      scan_planner_node  --/planning/bspline--> closed_loop_controller
          --/cmd_vel--> go2_cmd_vel_bridge_node --SDK2 SportClient::Move--> Go2

Usage:
  ros2 launch scan_planner real_robot.launch.py navi_mode:=1
  (dry-run by default: bridge_enable:=false; set true only when ready)

Optional: set launch_lio:=true to instead run the local L2 driver + Point-LIO
on this PC (for a radar cabled directly to this PC).
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

import os


def _setup(context):
    navi_mode = LaunchConfiguration("navi_mode").perform(context)
    keypoints_file = os.path.expanduser(LaunchConfiguration("keypoints_file").perform(context))
    run_rviz = LaunchConfiguration("rviz").perform(context).lower() in ("1", "true", "yes")
    launch_lio = LaunchConfiguration("launch_lio").perform(context).lower() in ("1", "true", "yes")
    lio_is_real = LaunchConfiguration("lio_is_real").perform(context).lower() in ("1", "true", "yes")

    if navi_mode == "2" and keypoints_file and not os.path.isfile(keypoints_file):
        raise RuntimeError(
            f"keypoints_file '{keypoints_file}' does not exist. "
            f"Record one first with: ros2 run scan_planner keypoint_recorder.py "
            f"--odom /utlidar/robot_odom --output ~/keypoints.yaml"
        )

    body_pose_topic = LaunchConfiguration("body_pose_topic").perform(context)
    sensor_pose_topic = LaunchConfiguration("sensor_pose_topic").perform(context)
    cloud_topic = LaunchConfiguration("cloud_topic").perform(context)
    cmd_vel_topic = LaunchConfiguration("cmd_vel_topic").perform(context)

    planner_overrides = {
        "use_sim_time": False,
        "fsm.navi_mode": int(navi_mode),
        "grid_map.sensor_type": "lidar",
        "grid_map.cloud_is_world": True,
        "grid_map.need_extrinsic": False,
        # Align the planner's world frame with the onboard odometry frame.
        "grid_map.frame_id": "odom",
        "grid_map.sliding_map_frame_id": "sliding_map",
    }

    actions = []

    # ---- optional local L2 driver + Point-LIO (for a radar cabled to this PC) ----
    if launch_lio:
        actions.append(
            Node(
                package="unitree_lidar_ros2",
                executable="unitree_lidar_ros2_node",
                name="unitree_lidar_ros2_node",
                output="screen",
                parameters=[{
                    "initialize_type": int(LaunchConfiguration("lidar_initialize_type").perform(context)),
                    "work_mode": 0,
                    "use_system_timestamp": True,
                    "range_min": 0.0,
                    "range_max": 100.0,
                    "cloud_scan_num": 18,
                    "serial_port": LaunchConfiguration("serial_port").perform(context),
                    "baudrate": 4000000,
                    "lidar_port": int(LaunchConfiguration("lidar_port").perform(context)),
                    "lidar_ip": LaunchConfiguration("lidar_ip").perform(context),
                    "local_port": int(LaunchConfiguration("local_port").perform(context)),
                    "local_ip": LaunchConfiguration("local_ip").perform(context),
                    "cloud_frame": "unilidar_lidar",
                    "cloud_topic": "unilidar/cloud",
                    "imu_frame": "unilidar_imu",
                    "imu_topic": "unilidar/imu",
                }],
            )
        )
        if not lio_is_real:
            actions.append(
                Node(
                    package="point_lio",
                    executable="pointlio_mapping",
                    name="laserMapping",
                    output="screen",
                    parameters=[
                        PathJoinSubstitution([FindPackageShare("point_lio"), "config", "unilidar_l2.yaml"]),
                        {
                            "use_imu_as_input": False,
                            "prop_at_freq_of_imu": True,
                            "check_satu": True,
                            "init_map_size": 10,
                            "point_filter_num": 1,
                            "space_down_sample": True,
                            "filter_size_surf": 0.1,
                            "filter_size_map": 0.1,
                            "cube_side_length": 1000.0,
                            "runtime_pos_log_enable": False,
                        },
                    ],
                )
            )

    # ---- planner ----
    actions.append(
        Node(
            package="scan_planner",
            executable="scan_planner_node",
            name="scan_planner_node",
            output="screen",
            parameters=[PathJoinSubstitution([FindPackageShare("scan_planner"), "config", "planner.yaml"])]
                        + ([keypoints_file] if keypoints_file else [])
                        + [planner_overrides],
            remappings=[
                ("body_pose", body_pose_topic),
                ("sensor_pose", sensor_pose_topic),
                ("cloud", cloud_topic),
                ("depth", "/camera/aligned_depth_to_color/image_raw"),
                ("move_base_simple/goal", "/move_base_simple/goal"),
                ("initial_path", "/initial_path"),
            ],
        )
    )

    # ---- closed-loop controller ----
    actions.append(
        Node(
            package="scan_planner",
            executable="closed_loop_controller",
            name="closed_loop_controller",
            output="screen",
            parameters=[PathJoinSubstitution([FindPackageShare("scan_planner"), "config", "controllers.yaml"]),
                        {"use_sim_time": False}],
            remappings=[
                ("body_pose", body_pose_topic),
                ("cmd_vel", cmd_vel_topic),
            ],
        )
    )

    # ---- odom -> base_link TF (board only sends odometry message, no TF) ----
    actions.append(
        Node(
            package="go2_cmd_vel_bridge",
            executable="odom_tf_broadcaster",
            name="odom_tf_broadcaster",
            output="screen",
            parameters=[{
                "odom_topic": body_pose_topic,
                "odom_frame": "odom",
                "child_frame": "base_link",
                "publish_rate": 50.0,
            }],
        )
    )

    # ---- Go2 SDK2 bridge ----
    actions.append(
        Node(
            package="go2_cmd_vel_bridge",
            executable="go2_cmd_vel_bridge_node",
            name="go2_cmd_vel_bridge",
            output="screen",
            parameters=[{
                "enable": LaunchConfiguration("bridge_enable").perform(context).lower() in ("1", "true", "yes"),
                "cmd_vel_topic": cmd_vel_topic,
                "cmd_timeout": 0.3,
                "control_rate": 50.0,
                "max_vx": 0.8,
                "max_vy": 0.4,
                "max_vyaw": 1.2,
                "stand_mode": 0,
                "stand_down_on_exit": False,
            }],
        )
    )

    if run_rviz:
        actions.append(
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                arguments=["-d", PathJoinSubstitution(
                    [FindPackageShare("scan_planner"), "rviz", "real_robot.rviz"])],
            )
        )

    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("navi_mode", default_value="1"),
        DeclareLaunchArgument("keypoints_file", default_value=""),
        DeclareLaunchArgument("rviz", default_value="true"),
        DeclareLaunchArgument("launch_lio", default_value="false",
                              description="run the local L2 driver + Point-LIO on this PC"),
        DeclareLaunchArgument("lio_is_real", default_value="false",
                              description="true if the local L2 driver already runs on the robot onboard"),
        DeclareLaunchArgument("bridge_enable", default_value="false",
                              description="false = dry-run (no motion); set true only when ready"),
        # Onboard topics (Go2 X built-in radar)
        # cloud_deskewed is in the odom frame (already world-aligned), so we use
        # cloud_is_world=true and sensor_pose = body_pose for the ray origin.
        DeclareLaunchArgument("body_pose_topic", default_value="/utlidar/robot_odom"),
        DeclareLaunchArgument("sensor_pose_topic", default_value="/utlidar/robot_odom"),
        DeclareLaunchArgument("cloud_topic", default_value="/utlidar/cloud_deskewed"),
        DeclareLaunchArgument("cmd_vel_topic", default_value="/cmd_vel"),
        # Local L2 driver wiring (only used when launch_lio:=true)
        DeclareLaunchArgument("serial_port", default_value="/dev/ttyACM0"),
        DeclareLaunchArgument("lidar_initialize_type", default_value="2"),
        DeclareLaunchArgument("lidar_port", default_value="6101"),
        DeclareLaunchArgument("lidar_ip", default_value="192.168.1.62"),
        DeclareLaunchArgument("local_port", default_value="6201"),
        DeclareLaunchArgument("local_ip", default_value="192.168.1.2"),
        OpaqueFunction(function=_setup),
    ])
