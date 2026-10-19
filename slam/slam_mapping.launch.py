"""Start a mapping session: Go2 L2 point cloud -> /scan -> slam_toolbox.

Run inside the go2_humble container with the robot reachable (odom -> base_link
TF from SCAN-Planner's odom_tf_broadcaster must be running):

    ros2 launch /home/unitree/hu/go2/web/slam/slam_mapping.launch.py

Drive the robot around the venue, then save the map with slam_save_map.sh.
"""
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import os

MAP_DIR = "/home/unitree/hu/go2/slam_maps"


def generate_launch_description():
    params = os.path.join(
        get_package_share_directory("slam_toolbox"),
        "config", "mapper_params_online_async.yaml")

    scan = Node(
        package="pointcloud_to_laserscan",
        executable="pointcloud_to_laserscan_node",
        remappings=[("cloud_in", "/utlidar/cloud_base"), ("scan", "/scan")],
        parameters=[{
            "target_frame": "base_link",
            "transform_tolerance": 0.1,
            "min_height": -0.2,
            "max_height": 0.3,
            "angle_min": -3.14159,
            "angle_max": 3.14159,
            "angle_increment": 0.0087,
            "scan_time": 0.1,
            "range_min": 0.3,
            "range_max": 20.0,
            "use_inf": True,
        }],
        output="screen",
    )

    slam = Node(
        package="slam_toolbox",
        executable="async_slam_toolbox_node",
        parameters=[params, {
            "use_sim_time": False,
            "odom_frame": "odom",
            "base_frame": "base_link",
            "scan_topic": "/scan",
            "mode": "mapping",
            "enable_interactive_mode": False,
            "minimum_laser_range": 0.3,
        }],
        output="screen",
    )

    return LaunchDescription([scan, slam])
