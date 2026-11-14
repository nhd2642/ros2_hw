import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.parameter_descriptions import ParameterValue
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    a1_share = get_package_share_directory('a1_champ')
    rl_share = get_package_share_directory('a1_rl')

    ns = LaunchConfiguration('robot_ns')
    use_sim_time = LaunchConfiguration('use_sim_time')

    urdf_path = os.path.join(a1_share, 'urdf', 'a1.urdf.xacro')

    # 컨트롤러 설정을 /<ns>/ 아래로 감싼다 (a1.launch.py 와 같은 방식)
    ros_control_yaml = RewrittenYaml(
        source_file=os.path.join(rl_share, 'config', 'ros_control_rl.yaml'),
        root_key=ns, param_rewrites={}, convert_types=True)

    robot_description = ParameterValue(Command([
        'xacro ', urdf_path,
        ' robot_ns:=', ns,
        ' ros_control_yaml:=', ros_control_yaml,
        ' effort_state:=true',
    ]), value_type=str)

    tf_remaps = [('/tf', 'tf'), ('/tf_static', 'tf_static')]

    robot_state_publisher = Node(
        package='robot_state_publisher', executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description,
                     'use_sim_time': use_sim_time,
                     'publish_frequency': 200.0,
                     'ignore_timestamp': True}],
        remappings=tf_remaps)

    spawn_entity = Node(
        package='gazebo_ros', executable='spawn_entity.py', output='screen',
        arguments=['-entity', ns,
                   '-topic', ['/', ns, '/robot_description'],
                   '-robot_namespace', ns,
                   '-x', LaunchConfiguration('x'), '-y', LaunchConfiguration('y'),
                   '-z', LaunchConfiguration('z'), '-Y', LaunchConfiguration('yaw')])

    def spawner(name):
        return Node(package='controller_manager', executable='spawner',
                    arguments=[name, '--controller-manager', ['/', ns, '/controller_manager']],
                    output='screen')

    spawn_robot_ctrl = spawner('robot_joint_controller')

    # rl_sim 은 컨트롤러가 뜬 뒤에 시작한다 (패치된 rl_sar: robot_ns 파라미터 사용)
    rl_sim = Node(
        package='rl_sar', executable='rl_sim', output='screen', emulate_tty=True,
        name=['rl_sim_', ns],
        parameters=[{'robot_name': 'a1',
                     'gazebo_model_name': ParameterValue(ns, value_type=str),
                     'robot_ns': ParameterValue(['/', ns], value_type=str)}],
        remappings=[('/cmd_vel', ['/', ns, '/cmd_vel']),
                    ('/imu', ['/', ns, '/imu/data']),
                    ('/joy', ['/', ns, '/joy'])])

    load_joint_states = RegisterEventHandler(OnProcessExit(
        target_action=spawn_entity,
        on_exit=[spawner('joint_states_controller'), spawn_robot_ctrl]))
    start_rl = RegisterEventHandler(OnProcessExit(
        target_action=spawn_robot_ctrl, on_exit=[rl_sim]))

    return LaunchDescription([
        DeclareLaunchArgument('robot_ns', default_value='a1_1'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('x', default_value='0.0'),
        DeclareLaunchArgument('y', default_value='0.0'),
        DeclareLaunchArgument('z', default_value='0.35'),
        DeclareLaunchArgument('yaw', default_value='0.0'),
        GroupAction([
            PushRosNamespace(ns),
            robot_state_publisher,
            spawn_entity,
        ]),
        load_joint_states,
        start_rl,
    ])
