"""Start a mapping session: Go2 L2 point cloud -> /scan -> slam_toolbox.

Run inside the go2_humble container while odom -> base_link TF is available
(SCAN-Planner's odom_tf_broadcaster, or the standalone one SlamManager spawns):

    ros2 launch /home/unitree/hu/go2/web/slam/slam_mapping.launch.py

Drive the robot around the venue, then save with slam_save_map.sh <name>.
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

    # 关键：/utlidar/cloud_base 每帧仅 ~1200 点（且高度带内只有 ~150），
    # 转成 /scan 后墙面碎成放射状点列。cloud_deskewed 是同一雷达的
    # 运动补偿完整点云，每帧 ~11000 点（10 倍密度），建图质量完全不同。
    scan = Node(
        package="pointcloud_to_laserscan",
        executable="pointcloud_to_laserscan_node",
        remappings=[("cloud_in", "/utlidar/cloud_deskewed"), ("scan", "/scan")],
        parameters=[{
            # 输出到 base_link：slam_toolbox 的 base_frame 依赖这条 TF 链
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
        executable="async_slam_toolbox_node",
        parameters=[params, {
            "use_sim_time": False,
            "odom_frame": "odom",
            "base_frame": "base_link",
            "scan_topic": "/scan",
            "mode": "mapping",
            "enable_interactive_mode": False,
            "minimum_laser_range": 0.3,
            # 展厅是千平米级室内场景：放宽闭环搜索半径，否则走远后
            # 回不到起点附近，loop closure 不触发，长走廊会缓慢漂移。
            "loop_search_maximum_distance": 6.0,
            # 每 0.2s 处理一次（步行速度下帧间位移 <10cm，匹配能收敛）。
            "minimum_time_interval": 0.2,
        }],
        output="screen",
    )

    return LaunchDescription([scan, slam])
