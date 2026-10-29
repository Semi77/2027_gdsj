"""启动纯雷达 Cartographer 建图、栅格地图和可选的 RViz2 显示。"""

from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """@brief 读取同目录配置并启动建图节点，rviz 参数决定是否显示窗口。"""
    folder = str(Path(__file__).resolve().parent)
    return LaunchDescription([
        DeclareLaunchArgument("rviz", default_value="true"),
        Node(package="cartographer_ros", executable="cartographer_node",
             name="cartographer_node", output="screen",
             parameters=[{"use_sim_time": False}],
             arguments=["-configuration_directory", folder,
                        "-configuration_basename", "lidar_2d.lua"],
             remappings=[("scan", "/scan")]),
        Node(package="cartographer_ros", executable="cartographer_occupancy_grid_node",
             name="cartographer_occupancy_grid_node", output="screen",
             parameters=[{"use_sim_time": False, "resolution": 0.05,
                          "publish_period_sec": 1.0}]),
        Node(package="rviz2", executable="rviz2", name="rviz2", output="screen",
             arguments=["-d", str(Path(folder) / "mapping.rviz")],
             condition=IfCondition(LaunchConfiguration("rviz"))),
    ])
