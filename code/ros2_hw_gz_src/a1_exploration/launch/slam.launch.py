import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    pkg_dir = get_package_share_directory('a1_exploration')
    slam_params = os.path.join(pkg_dir, 'config', 'slam_params.yaml')

    slam_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        namespace='a1_1',
        output='screen',
        parameters=[
            slam_params,
            {'use_sim_time': True}
        ],
        remappings=[
            ('/tf', '/a1_1/tf'),
            ('/tf_static', '/a1_1/tf_static')
        ]
    )

    return LaunchDescription([
        slam_node
    ])
