"""A1 한 대를 보행 시험용 월드에 스폰한다.

  ros2 launch a1_champ gait_test.launch.py spawn:=flat            # (0,0)   +X 방향, 평지 시험용
  ros2 launch a1_champ gait_test.launch.py spawn:=ramp            # (0,3)   +X 방향, 경사로 시험용
  ros2 launch a1_champ gait_test.launch.py spawn:=flat gui:=false # 화면 없이 (RTF가 더 잘 나옴)
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def _setup(context, *args, **kwargs):
    a1_share = get_package_share_directory('a1_champ')
    gazebo_ros_share = get_package_share_directory('gazebo_ros')

    spawn = LaunchConfiguration('spawn').perform(context)
    ns = LaunchConfiguration('robot_ns').perform(context)
    gui = LaunchConfiguration('gui').perform(context)
    x, y = (0.0, 0.0) if spawn == 'flat' else (0.0, 3.0)

    world = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(gazebo_ros_share, 'launch', 'gazebo.launch.py')),
        launch_arguments={'world': os.path.join(a1_share, 'worlds', 'gait_test.world'),
                          'gui': gui, 'verbose': 'false',
                          # /clock 1000 Hz (기본 10 Hz 면 CHAMP 보행 궤적이 100 ms 계단이 된다)
                          'params_file': os.path.join(a1_share, 'config', 'gazebo_clock1000.yaml')}.items())

    a1 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(a1_share, 'launch', 'a1.launch.py')),
        launch_arguments={'robot_ns': ns, 'x': str(x), 'y': str(y),
                          'z': '0.28', 'yaw': '0.0'}.items())

    return [world, TimerAction(period=5.0, actions=[a1])]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('spawn', default_value='flat', description='flat 또는 ramp'),
        DeclareLaunchArgument('robot_ns', default_value='a1_1'),
        DeclareLaunchArgument('gui', default_value='true'),
        OpaqueFunction(function=_setup),
    ])
