"""Start localization against a saved map (no initial pose needed).

    SLAM_MAP_NAME=venue [SLAM_START_POSE='[x,y,yaw]'] ros2 launch \
        /home/unitree/hu/go2/web/slam/slam_localization.launch.py

Loads the serialized pose graph saved earlier and publishes map -> odom.
SLAM_START_POSE is the odom pose where mapping started (recorded by app.py);
slam_toolbox uses it to seed the first scan match at the map origin.
"""
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import json
import os

MAP_DIR = "/home/unitree/hu/go2/slam_maps"
MAP_NAME = os.environ.get("SLAM_MAP_NAME", "venue")
# SLAM_START_POSE 是"狗当前在 map 系里的位姿"（可选先验）。
# 留空 = 不提供先验，让 slam_toolbox 用当前扫描在整张地图上自主定位——
# 这是默认且更稳的做法：狗每次开机停的位置都不同，硬塞 odom 数值会把
# 定位整体拉偏（实测可差 1.3 m）。
_raw = os.environ.get("SLAM_START_POSE", "").strip()
# slam_toolbox localization 模式强制要求 map_start_pose，缺省会直接报错退出；
# 没指定时用 (0,0,0)，即"假定狗就在地图原点/建图起点"。
START_POSE = [0.0, 0.0, 0.0]
if _raw:
    try:
        START_POSE = [float(v) for v in json.loads(_raw)]
    except (ValueError, TypeError):
        pass


def generate_launch_description():
    params = os.path.join(
        get_package_share_directory("slam_toolbox"),
        "config", "mapper_params_localization.yaml")

    # 必须与 slam_mapping.launch.py 完全一致，否则建图/定位的扫描特征不匹配
    scan = Node(
        package="pointcloud_to_laserscan",
        executable="pointcloud_to_laserscan_node",
        remappings=[("cloud_in", "/utlidar/cloud_deskewed"), ("scan", "/scan")],
        parameters=[{
            "target_frame": "base_link",
            "transform_tolerance": 0.2,
            "min_height": -0.6,
            "max_height": 0.6,
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
        executable="localization_slam_toolbox_node",
        parameters=[params, {
            "use_sim_time": False,
            "odom_frame": "odom",
            "base_frame": "base_link",
            "map_frame": "map",
            "scan_topic": "/scan",
            "mode": "localization",
            "enable_interactive_mode": False,
            "minimum_laser_range": 0.3,
            "map_file_name": os.path.join(MAP_DIR, MAP_NAME),
            # 狗当前在 map 系中的位姿（先验）
            "map_start_pose": START_POSE,

            # ---- 定位参数（2026-10-22 实测后回退到原值）----
            # 曾尝试把搜索窗口缩到 0.3m、降低更新频率来压制 map->odom 跳变，
            # 实测无效且更差（跳变 34 次/110s -> 60 次/120s）。原因是跳变并非
            # 来自"搜索范围太大"，而是扫描匹配在特征稀疏的展厅里本身就落在
            # 不同局部极小值上；缩小搜索窗口只是让匹配永远无法收敛。
            # 跳变改由消费侧（ros_bridge.TfFilter）平滑处理，不在这里调参。
            "correlation_search_space_dimension": 1.0,
            "correlation_search_space_resolution": 0.01,
            "correlation_search_space_smear_deviation": 0.1,
            "minimum_time_interval": 0.2,
        }],
        output="screen",
    )

    return LaunchDescription([scan, slam])
