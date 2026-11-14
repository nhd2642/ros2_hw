import os
import yaml
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    actions = []
    pkg_dir = get_package_share_directory('a1_exploration')
    config_path = os.path.join(pkg_dir, 'config', 'slam_params.yaml')
    # 작업 공간 = <ws>/install/a1_exploration/share/a1_exploration 의 4 단계 위 (~/ros2_hw 고정이면 다른 작업 공간에서 옛 스크립트를 씀)
    ws = os.path.abspath(os.path.join(pkg_dir, '..', '..', '..', '..'))
    relay_path = os.path.join(ws, 'src', 'a1_exploration', 'scripts', 'scan_ready_relay.py')

    with open(config_path, 'r') as f:
        config = yaml.safe_load(f)

    base_params = config['/a1_1/slam_toolbox']['ros__parameters']

    for ns in ['a1_1', 'a1_2']:
        relay = ExecuteProcess(
            cmd=[
                'python3', relay_path,
                '--ros-args',
                '-r', f'__node:=scan_ready_relay_{ns}',
                '-r', f'/tf:=/{ns}/tf',
                '-r', f'/tf_static:=/{ns}/tf_static',
                '-r', f'/a1_1/scan:=/{ns}/scan',
                '-r', f'/a1_1/scan_ready:=/{ns}/scan_ready'
            ],
            output='screen'
        )

        params = dict(base_params)
        params.update({
            'use_sim_time': True,
            'scan_topic': f'/{ns}/scan_ready',
            'transform_timeout': 0.0,
            'min_laser_range': 0.2,
            'max_laser_range': 10.0
        })

        slam = Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            namespace=ns,
            name='slam_toolbox',
            output='screen',
            parameters=[params],
            remappings=[
                ('/tf', f'/{ns}/tf'),
                ('/tf_static', f'/{ns}/tf_static'),
                ('/scan', f'/{ns}/scan_ready'),
                ('/map', f'/{ns}/map'),
                ('/map_metadata', f'/{ns}/map_metadata')
            ]
        )

        actions.extend([relay, slam])

    return LaunchDescription(actions)
