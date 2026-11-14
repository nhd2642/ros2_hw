import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    rl_share = get_package_share_directory('a1_rl')
    gz_share = get_package_share_directory('gazebo_ros')

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(gz_share, 'launch', 'gazebo.launch.py')),
        launch_arguments={
            'world': [rl_share, '/worlds/', LaunchConfiguration('world_name'), '.world'],
            'gui': LaunchConfiguration('gui'),
            'verbose': 'false',
        }.items())

    a1 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(rl_share, 'launch', 'a1_rl.launch.py')),
        launch_arguments={'robot_ns': 'a1_1', 'x': '0.0', 'y': '0.0', 'z': '0.35', 'yaw': '0.0'}.items())

    return LaunchDescription([
        DeclareLaunchArgument('world_name', default_value='flat_1ms'),
        DeclareLaunchArgument('gui', default_value='true'),
        gazebo,
        TimerAction(period=5.0, actions=[a1]),
    ])
