import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterFile

def launch_setup(context, *args, **kwargs):
    # 获取包路径
    karto_pkg_dir = get_package_share_directory('karto_slam')
    upper_pkg_dir = get_package_share_directory('upper')

    # 启动参数
    mode_launch_arg = LaunchConfiguration('mode').perform(context)
    sync_async_launch_arg = LaunchConfiguration('sync_async').perform(context)
    default_params_file = LaunchConfiguration('params_file')
    default_params_path = default_params_file.perform(context)

    # 读取 upper/config/params.yaml
    upper_params_path = os.path.join(upper_pkg_dir, 'config', 'params.yaml')
    upper_params = {}
    if os.path.exists(upper_params_path):
        with open(upper_params_path, 'r') as f:
            upper_params = yaml.safe_load(f) or {}
        print(f"[INFO] 读取 upper 参数: {upper_params_path}")
    else:
        print(f"[WARN] upper/config/params.yaml 不存在: {upper_params_path}")

    # 读取 my_parameter.yaml
    default_params = {}
    if os.path.exists(default_params_path):
        with open(default_params_path, 'r') as f:
            default_params = yaml.safe_load(f) or {}
        print(f"[INFO] 读取默认参数: {default_params_path}")
    else:
        print(f"[WARN] 默认参数文件不存在: {default_params_path}")

    # 最终参数值与来源
    final_params = {}
    param_sources = {}

    # 关键参数
    key_params = ['mode', 'sync_async', 'scan_topic']
    for p in key_params:
        if p == 'mode':
            if mode_launch_arg:
                final_params[p] = mode_launch_arg
                param_sources[p] = 'launch 参数'
            elif 'mode' in upper_params:
                final_params[p] = upper_params['mode']
                param_sources[p] = 'upper/config/params.yaml'
            else:
                final_params[p] = default_params.get('mode', 'mapping')
                param_sources[p] = 'my_parameter.yaml 或默认值'
        elif p == 'sync_async':
            if sync_async_launch_arg:
                final_params[p] = sync_async_launch_arg
                param_sources[p] = 'launch 参数'
            elif 'sync_async' in upper_params:
                final_params[p] = upper_params['sync_async']
                param_sources[p] = 'upper/config/params.yaml'
            else:
                final_params[p] = default_params.get('sync_async', 'async')
                param_sources[p] = 'my_parameter.yaml 或默认值'
        elif p == 'scan_topic':
            if 'scan_topic' in upper_params:
                final_params[p] = upper_params['scan_topic']
                param_sources[p] = 'upper/config/params.yaml'
            else:
                final_params[p] = default_params.get('scan_topic', 'scan')
                param_sources[p] = 'my_parameter.yaml 或默认值'

    # 打印所有关键参数及来源
    print("\n" + "="*60)
    print("最终 SLAM 配置参数及来源:")
    print("="*60)
    for k, v in final_params.items():
        source = param_sources.get(k, '未知来源')
        print(f"{k:15}: {v}  (来源: {source})")
    print("="*60 + "\n")

    # 打印其他参数（my_parameter.yaml 的额外参数）
    other_params = {k: v for k, v in default_params.items() if k not in final_params}
    if other_params:
        print("[INFO] 其他参数 (来自 my_parameter.yaml, 未被覆盖):")
        for k, v in other_params.items():
            print(f"{k:15}: {v}")
        print("="*60 + "\n")

    # 根据 sync_async 选择可执行文件
    executable_name = 'async_slam_toolbox_node' if final_params['sync_async'] == 'async' else 'sync_slam_toolbox_node'
    print(f"[INFO] 使用 SLAM 节点: {executable_name}")

    # 创建 SLAM Toolbox 节点
    param_file = ParameterFile(default_params_file, allow_substs=True)
    slam_toolbox_node = Node(
        package='slam_toolbox',
        executable=executable_name,
        name='slam_toolbox',
        output='screen',
        parameters=[param_file, {'mode': final_params['mode']}],
        arguments=['--ros-args', '--log-level', 'slam_toolbox:=warn'],
        remappings=[
            ('/odom', '/odom_rf2o'),
            ('/tf', 'tf'),
            ('/tf_static', 'tf_static')
        ]
    )

    return [slam_toolbox_node]


def generate_launch_description():
    karto_pkg_dir = get_package_share_directory('karto_slam')

    # 声明启动参数
    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(karto_pkg_dir, 'config', 'my_parameter.yaml'),
        description='Full path to the ROS2 parameters file to use for the slam_toolbox node'
    )

    declare_mode_cmd = DeclareLaunchArgument(
        'mode',
        default_value='',
        description='SLAM mode: mapping or localization (overrides params.yaml)'
    )

    declare_sync_async_cmd = DeclareLaunchArgument(
        'sync_async',
        default_value='',
        description='Sync/Async mode: sync or async (overrides params.yaml)'
    )

    # 设置 ROS_DOMAIN_ID（可选）
    set_ros_domain_cmd = SetEnvironmentVariable(
        'ROS_DOMAIN_ID',
        '0'
    )

    # 创建启动描述
    ld = LaunchDescription()
    ld.add_action(set_ros_domain_cmd)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_mode_cmd)
    ld.add_action(declare_sync_async_cmd)
    ld.add_action(OpaqueFunction(function=launch_setup))

    return ld
