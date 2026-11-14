import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.parameter_descriptions import ParameterValue
from nav2_common.launch import RewrittenYaml


BRIDGE = [  # (ROS 토픽, gz 토픽, ROS 형, gz 형)  {ns} 는 로봇 이름
    ('/{ns}/scan', '/{ns}/scan', 'sensor_msgs/msg/LaserScan', 'ignition.msgs.LaserScan'),
    ('/{ns}/imu/data', '/{ns}/imu/data', 'sensor_msgs/msg/Imu', 'ignition.msgs.IMU'),
    ('/{ns}/front_camera/image_raw', '/{ns}/front_camera/image_raw', 'sensor_msgs/msg/Image', 'ignition.msgs.Image'),
    ('/{ns}/front_camera/camera_info', '/{ns}/front_camera/camera_info', 'sensor_msgs/msg/CameraInfo', 'ignition.msgs.CameraInfo'),
    ('/{ns}/terrain_camera/points', '/{ns}/terrain_camera/depth/points', 'sensor_msgs/msg/PointCloud2', 'ignition.msgs.PointCloudPacked'),
    ('/{ns}/odom/ground_truth', '/{ns}/odom/ground_truth', 'nav_msgs/msg/Odometry', 'ignition.msgs.Odometry'),
]


def _bridge_yaml(ns):
    """ros_gz_bridge config_file (로봇마다 한 파일, /tmp)."""
    import tempfile
    lines = []
    for ros, gz, rt, gt in BRIDGE:
        lines += [f'- ros_topic_name: "{ros.format(ns=ns)}"', f'  gz_topic_name: "{gz.format(ns=ns)}"',
                  f'  ros_type_name: "{rt}"', f'  gz_type_name: "{gt}"', '  direction: GZ_TO_ROS']
    f = tempfile.NamedTemporaryFile('w', prefix=f'gz_bridge_{ns}_', suffix='.yaml', delete=False)
    f.write('\n'.join(lines) + '\n')
    f.close()
    return f.name


def _setup(context):
    # 인자를 include 되는 시점에 문자열로 확정한다. LaunchConfiguration 을 그대로 쓰면 이벤트 핸들러(OnProcessExit)로
    # 나중에 실행되는 spawner/CHAMP 가 그 시점의 값을 읽는데, 로봇 여러 대를 연달아 include 하면 마지막 로봇의
    # robot_ns 로 덮어써져 있어서 앞 로봇의 컨트롤러가 뒤 로봇의 controller_manager 에 붙는다 (동시 스폰 실패 원인).
    ns = LaunchConfiguration('robot_ns').perform(context)
    use_sim_time = LaunchConfiguration('use_sim_time').perform(context).lower() == 'true'
    x, y, z, yaw = (LaunchConfiguration(k).perform(context) for k in ('x', 'y', 'z', 'yaw'))
    soft_start = float(LaunchConfiguration('soft_start').perform(context))

    a1_share = get_package_share_directory('a1_champ')
    champ_base_share = get_package_share_directory('champ_base')

    urdf_path = os.path.join(a1_share, 'urdf', 'a1.urdf.xacro')
    joints_cfg = os.path.join(a1_share, 'config', 'joints.yaml')
    links_cfg = os.path.join(a1_share, 'config', 'links.yaml')
    gait_cfg = os.path.join(a1_share, 'config', 'gait.yaml')

    # 컨트롤러 설정: 노드 이름이 /<ns>/controller_manager 처럼 되도록 root_key로 감싼다 (Nav2 멀티로봇 방식)
    ros_control_yaml = RewrittenYaml(
        source_file=os.path.join(a1_share, 'config', 'ros_control.yaml'),
        root_key=ns, param_rewrites={}, convert_types=True)

    robot_description = Command([
        'xacro ', urdf_path,
        ' robot_ns:=', ns,
        ' ros_control_yaml:=', ros_control_yaml,
    ])

    robot_description_param = ParameterValue(robot_description, value_type=str)

    # 로봇마다 /tf 를 분리: /a1_1/tf, /a1_2/tf (프레임 이름이 같아도 충돌하지 않음)
    tf_remaps = [('/tf', 'tf'), ('/tf_static', 'tf_static')]

    robot_state_publisher = Node(
        package='robot_state_publisher', executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description_param,
                     'use_sim_time': use_sim_time,
                     'publish_frequency': 50.0,
                     'ignore_timestamp': True}],
        remappings=tf_remaps)

    # Gazebo Sim (Fortress): ros_gz_sim create 가 robot_description 토픽을 받아 스폰하고 끝남 (끝나면 컨트롤러 로드)
    spawn_entity = Node(
        package='ros_gz_sim', executable='create', output='screen',
        arguments=['-name', ns, '-topic', '/' + ns + '/robot_description',
                   '-x', x, '-y', y, '-z', z, '-Y', yaw])

    # 센서·정답 위치를 gz -> ROS 로 (Classic 은 플러그인이 ROS 로 바로 냈음). 토픽 이름은 Classic 과 같게
    bridge = Node(
        package='ros_gz_bridge', executable='parameter_bridge', name='sensor_bridge', output='screen',
        parameters=[{'config_file': _bridge_yaml(ns), 'use_sim_time': use_sim_time}])

    def spawner(name):
        # namespace: 로봇 여러 대의 spawner 노드 이름이 겹치지 않게 (대상 controller_manager 는 절대 경로로 지정)
        return Node(package='controller_manager', executable='spawner', namespace=ns,
                    arguments=[name, '--controller-manager', '/' + ns + '/controller_manager'],
                    output='screen')

    effort_spawner = spawner('joint_group_effort_controller')
    load_controllers = RegisterEventHandler(OnProcessExit(
        target_action=spawn_entity,
        on_exit=[spawner('joint_states_controller'), effort_spawner]))

    # CHAMP 는 effort 컨트롤러가 활성화된 '뒤'에 띄운다 + soft start 로 실제 자세에서 천천히 목표 자세로.
    # (open_loop_control JTC 는 활성 전 처진 자세와 명령의 차이를 Kp 로 한 번에 메우며 로봇이 뒤집힐 수 있다.
    #  walk_parameters 기동 성공: 둘 다 없음 19/21, soft start 만 20/23, champ 지연만 8/10, 둘 다 15/15)
    # 이벤트 핸들러로 띄우는 노드에는 GroupAction 의 PushRosNamespace 가 적용되지 않으므로 namespace 를 직접 준다.
    quadruped_controller = Node(
        package='champ_base', executable='quadruped_controller_node', output='screen',
        namespace=ns,
        parameters=[{'use_sim_time': use_sim_time,
                     'gazebo': True,
                     'publish_joint_states': True,
                     'publish_joint_control': True,
                     # 보행 위상(stance)을 foot_contacts 로 -> state_estimation 다리 odom 이 동작 (없으면 /odom 위치가 스폰에 고정)
                     'publish_foot_contacts': True,
                     'foot_contacts_from_gait': True,
                     'joint_controller_topic': 'joint_group_effort_controller/joint_trajectory',
                     'soft_start': soft_start,
                     'urdf': robot_description_param},
                    joints_cfg, links_cfg, gait_cfg],
        remappings=[('cmd_vel/smooth', 'cmd_vel')] + tf_remaps)
    start_champ_after_controllers = RegisterEventHandler(OnProcessExit(
        target_action=effort_spawner, on_exit=[quadruped_controller]))

    state_estimator = Node(
        package='champ_base', executable='state_estimation_node', output='screen',
        parameters=[{'use_sim_time': use_sim_time,
                     'orientation_from_imu': False,
                     # TF 는 /<ns>/tf 로 분리되므로 프레임 이름은 접두사 없이 (odom, base_footprint: ekf 설정과 일치)
                     'frame_prefix': False,
                     'urdf': robot_description_param},
                    joints_cfg, links_cfg, gait_cfg],
        remappings=tf_remaps)

    def ekf(name, yaml_name, out_topic):
        return Node(
            package='robot_localization', executable='ekf_node', name=name, output='screen',
            parameters=[{'base_link_frame': 'base_link', 'use_sim_time': use_sim_time},
                        RewrittenYaml(
                            source_file=os.path.join(champ_base_share, 'config', 'ekf', yaml_name),
                            root_key=ns, param_rewrites={}, convert_types=True)],
            remappings=[('odometry/filtered', out_topic)] + tf_remaps)

    base_to_footprint_ekf = ekf('base_to_footprint_ekf', 'base_to_footprint.yaml', 'odom/local')
    footprint_to_odom_ekf = ekf('footprint_to_odom_ekf', 'footprint_to_odom.yaml', 'odom')

    return [GroupAction([
        PushRosNamespace(ns),
        robot_state_publisher,
        spawn_entity,
        bridge,
        load_controllers,
        start_champ_after_controllers,
        state_estimator,
        base_to_footprint_ekf,
        footprint_to_odom_ekf,
    ])]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('robot_ns', default_value='a1_1'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('x', default_value='0.0'),
        DeclareLaunchArgument('y', default_value='0.0'),
        DeclareLaunchArgument('z', default_value='0.35'),
        DeclareLaunchArgument('yaw', default_value='0.0'),
        DeclareLaunchArgument('soft_start', default_value='3.0',
                              description='CHAMP 기동 시 실제 자세 -> 목표 자세 전환 시간 (s), 0 이면 끔'),
        OpaqueFunction(function=_setup),
    ])
