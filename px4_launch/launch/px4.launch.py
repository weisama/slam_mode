import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    # 获取功能包安装路径
    pkg_share = get_package_share_directory('px4_launch')

    # 拼接 YAML 完整路径
    config_yaml = os.path.join(pkg_share, 'config', 'px4_launch.yaml')

    return LaunchDescription([

        Node(
            package='mavros',
            executable='mavros_node',
            output='screen',
            parameters=[config_yaml]
        )
    ])
