import math
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    """A1 두 대를 평지 월드에 동시에 띄운다 (재난 월드와 같은 시작 자세: a1_1 (0,-1.1) yaw -90도, a1_2 (0,+1.1) yaw +90도).
    스폰 간격: a1_1 5초, a1_2 12초 (한 대씩 컨트롤러가 뜰 시간을 준다)."""
    rl_share = get_package_share_directory('a1_rl')
    gz_share = get_package_share_directory('gazebo_ros')

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(gz_share, 'launch', 'gazebo.launch.py')),
        launch_arguments={
            'world': [rl_share, '/worlds/', LaunchConfiguration('world_name'), '.world'],
            'gui': LaunchConfiguration('gui'),
            'verbose': 'false',
        }.items())

    def a1(ns, y, yaw):
        return IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(rl_share, 'launch', 'a1_rl.launch.py')),
            launch_arguments={'robot_ns': ns, 'x': '0.0', 'y': str(y), 'z': '0.35', 'yaw': '%.5f' % yaw}.items())

    return LaunchDescription([
        DeclareLaunchArgument('world_name', default_value='flat_1ms'),
        DeclareLaunchArgument('gui', default_value='true'),
        gazebo,
        TimerAction(period=5.0, actions=[a1('a1_1', -1.1, -math.pi / 2)]),
        TimerAction(period=12.0, actions=[a1('a1_2', 1.1, math.pi / 2)]),
    ])
