from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # 获取功能包共享目录路径
    pkg_path = get_package_share_directory('visualize_delay')
    
    # 创建 RViz2 配置文件路径
    rviz_config_path = os.path.join(pkg_path, 'config', 'visualize_delay.rviz')
    
    # 如果配置文件不存在，创建目录（但不创建文件，让RViz2使用默认配置）
    os.makedirs(os.path.dirname(rviz_config_path), exist_ok=True)
    
    # 检查配置文件是否存在，如果不存在则使用默认参数启动RViz2
    if os.path.exists(rviz_config_path):
        rviz_args = ['-d', rviz_config_path]
    else:
        rviz_args = ['-d']  # 使用默认配置
    
    return LaunchDescription([
        # 可视化延迟节点
        Node(
            package='visualize_delay',
            executable='visualize_delay_node',
            name='visualize_delay',
            output='screen',
            parameters=[{
                'history_size': 500,
                'yaw_scale': 2.0
            }]
        ),
        
        # RViz2 节点
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=rviz_args,
            output='screen'
        )
    ])
