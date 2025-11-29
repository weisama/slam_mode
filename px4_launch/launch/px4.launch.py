# px4_launch/launch/px4.launch.py
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='mavros',
            executable='mavros_node',
            name='mavros',
            namespace='/mavros',
            output='screen',            # 仍想屏幕输出，但只打 WARN/ERROR
            arguments=[
                '--ros-args',
                '--log-level', 'WARN'   # 只显示 WARN 及以上
            ],
            parameters=[{
                'fcu_url': '/dev/ttyS3:921600',
                'gcs_url': '',
                'target_system_id': 1,
                'target_component_id': 1,
                'plugin_allowlist': ['imu', 'vision_pose', 'setpoint_raw'],
                'plugin_denylist': ['*'],
            }],
            remappings=[
                ('~/data',      '/mavros/imu/data'),
                ('~/data_raw',  '/mavros/imu/data_raw'),
                ('~/pose',      '/mavros/vision_pose/pose'),
                ('~/pose_cov',  '/mavros/vision_pose/pose_cov'),
                ('~/local',     '/mavros/setpoint_raw/local'),
                ('~/global',    '/mavros/setpoint_raw/global'),
                ('~/attitude',  '/mavros/setpoint_raw/attitude'),
                ('~/cmd_vel',   '/mavros/setpoint_raw/cmd_vel'),
            ]
        )
    ])
