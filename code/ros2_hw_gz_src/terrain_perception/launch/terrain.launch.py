"""깊이 카메라 지형 위험 노드 (두 로봇). sim + 로봇이 뜬 뒤, Nav2 와 같이.
  ros2 launch terrain_perception terrain.launch.py [max_step:=0.04]
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('max_step', default_value='0.04', description='CHAMP swing_height 와 같게'),
        DeclareLaunchArgument('cloud_frame', default_value='auto', description='auto 면 점구름 header 프레임'),
        Node(package='terrain_perception', executable='terrain_hazards', output='screen', remappings=[('/clock', '/clock_slow')],
             parameters=[{'use_sim_time': True,
                          'max_step': LaunchConfiguration('max_step'),
                          'cloud_frame': LaunchConfiguration('cloud_frame')}]),
    ])
