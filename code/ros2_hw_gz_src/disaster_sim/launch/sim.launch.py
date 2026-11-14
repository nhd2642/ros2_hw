"""시뮬 전체: 월드 + Husky(A200, 실제 구동) + A1 두 대. 위치는 config/mission.yaml (make_world.py 출력) 에서 읽는다.

  ros2 launch disaster_sim sim.launch.py [gui:=false] [start_set:=0]
start_set:=k (1 이상) 이면 세트 k-1 까지 끝난 것처럼 그 주차점에 Husky, 추종 자리에 A1 을 놓는다 (부분 시험용).
"""
import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from comm_relay.mission_logic import start_poses

HUSKY_BRIDGE = ['/husky/cmd_vel@geometry_msgs/msg/Twist]ignition.msgs.Twist',
                '/husky/odom@nav_msgs/msg/Odometry[ignition.msgs.Odometry',
                '/husky/odom/ground_truth@nav_msgs/msg/Odometry[ignition.msgs.Odometry',
                '/husky/imu@sensor_msgs/msg/Imu[ignition.msgs.IMU',
                '/husky/scan@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan']


def _setup(context):
    sim_share = get_package_share_directory('disaster_sim')
    a1_launch = os.path.join(get_package_share_directory('a1_champ'), 'launch', 'a1.launch.py')
    with open(os.path.join(sim_share, 'config', 'mission.yaml')) as f:
        pose = start_poses(yaml.safe_load(f), int(LaunchConfiguration('start_set').perform(context)))

    world = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(sim_share, 'launch', 'world.launch.py')),
        launch_arguments={'gui': LaunchConfiguration('gui')}.items())
    hx, hy, hyaw = (str(v) for v in pose['husky'])
    husky = [Node(package='ros_gz_sim', executable='create', output='screen',
                  arguments=['-name', 'husky', '-file', os.path.join(sim_share, 'models', 'husky', 'model.sdf'),
                             '-x', hx, '-y', hy, '-z', '0.02', '-Y', hyaw]),
             Node(package='ros_gz_bridge', executable='parameter_bridge', name='husky_bridge', output='screen',
                  arguments=HUSKY_BRIDGE)]

    def a1(ns):
        x, y, yaw = (str(v) for v in pose[ns])
        return IncludeLaunchDescription(PythonLaunchDescriptionSource(a1_launch), launch_arguments={
            'robot_ns': ns, 'x': x, 'y': y, 'z': '0.28', 'yaw': yaw}.items())

    return [world, TimerAction(period=4.0, actions=husky),
            TimerAction(period=6.0, actions=[a1('a1_1')]), TimerAction(period=20.0, actions=[a1('a1_2')])]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('start_set', default_value='0'),
        OpaqueFunction(function=_setup),
    ])
