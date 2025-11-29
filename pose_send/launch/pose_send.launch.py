#!/usr/bin/env python3
import os
import yaml
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    # 1. 获取 upper 包的参数文件路径
    upper_share = get_package_share_directory("upper")
    upper_params_file = os.path.join(upper_share, "config", "params.yaml")

    # 2. 读取参数
    with open(upper_params_file, "r") as f:
        upper_params = yaml.safe_load(f)

    px4_flag = upper_params.get("px4_flag", 1)
    uart_flag = upper_params.get("uart_flag", 1)

    # 3. 创建 pose_send_node（始终启动）
    pose_send_node = Node(
        package="pose_send",
        executable="pose_send_node",
        name="pose_send_node",
        output="screen",
        parameters=[
            {
                "px4_flag": px4_flag,
                "uart_flag": uart_flag,
            }
        ]
    )

    # 4. 如果 px4_flag 为 1，则包含 px4_launch
    launch_items = [pose_send_node]

    if px4_flag:
        px4_launch_file = os.path.join(
            get_package_share_directory("px4_launch"),
            "launch",
            "px4.launch.py"
        )
        px4_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(px4_launch_file)
        )
        launch_items.append(px4_launch)

    # 5. 返回 LaunchDescription
    return LaunchDescription(launch_items)
