"""disaster.world 실행 (Gazebo Sim / Fortress). gui:=false 면 서버만 (센서는 DISPLAY 로 렌더링, headless_rendering:=true 면 EGL).

Classic(gazebo_ros) -> Gazebo Sim(ros_gz_sim) 이전 (2026-11-10):
  - 모델 경로: GAZEBO_MODEL_PATH -> IGN_GAZEBO_RESOURCE_PATH (+ GZ_SIM_RESOURCE_PATH)
  - /clock: gazebo_ros 의 publish_rate 설정 대신 ros_gz_bridge 로 gz /clock 을 그대로 넘김 (스텝마다, 1 kHz)
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_prefix
from launch_ros.actions import Node


def _gz(context):
    world = os.path.join(get_package_share_directory('disaster_sim'), 'worlds', LaunchConfiguration('world').perform(context))
    gui = LaunchConfiguration('gui').perform(context).lower() == 'true'
    verbose = LaunchConfiguration('verbose').perform(context).lower() == 'true'
    egl = LaunchConfiguration('headless_rendering').perform(context).lower() == 'true'
    eng = LaunchConfiguration('render_engine').perform(context)
    args = ['-r', '--render-engine', eng] + ([] if gui else ['-s']) + (['--headless-rendering'] if egl else []) + (['-v', '4'] if verbose else []) + [world]
    return [IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': ' '.join(args)}.items())]


def generate_launch_description():
    models = os.path.join(get_package_share_directory('disaster_sim'), 'models')
    paths = [p for p in (os.environ.get('IGN_GAZEBO_RESOURCE_PATH', ''), models) if p]
    # ign_ros2_control 시스템 플러그인 위치 (apt 는 LD_LIBRARY_PATH 로 찾지만 다른 설치에서도 찾게 명시)
    plug = [p for p in (os.environ.get('IGN_GAZEBO_SYSTEM_PLUGIN_PATH', ''),
                        os.path.join(get_package_prefix('ign_ros2_control'), 'lib')) if p]
    return LaunchDescription([
        SetEnvironmentVariable('IGN_GAZEBO_SYSTEM_PLUGIN_PATH', os.pathsep.join(plug)),
        # WSL2(WSLg, D3D12) 에서 ogre2 는 'Ogre::UnimplementedException'(GL3PlusTextureGpu::copyTo) 으로 죽음 (gz-rendering #662,
        # gz-sim #920; 사용자 PC 2026-11-10 확인) -> 기본 ogre(ogre1, Classic 과 같은 엔진). GPU 가 제대로 되는 Linux 면 ogre2 가능
        DeclareLaunchArgument('render_engine', default_value='ogre', description='ogre(기본, WSL 용) 또는 ogre2'),
        DeclareLaunchArgument('headless_rendering', default_value='false',
                              description='true 면 EGL 로 화면 없이 센서 렌더링 (GPU/EGL 필요). false 면 DISPLAY 사용 (WSLg, Xvfb)'),
        # 사용자 PC(WSL) 2026-11-10 lidar_check: 시뮬 서버가 GPU 로 센서를 그리면 2D 라이다 360 점이 전부 range_min(0.15 m) 으로 나옴
        # (화면 유무와 상관없음, 카메라는 정상). 서버만 소프트웨어 렌더링(LIBGL_ALWAYS_SOFTWARE=1)이면 정상 (클라우드와 같은 값).
        # Gazebo 화면·RViz 는 따로 뜨는 프로세스라 GPU 그대로. GPU 가 제대로 되는 Linux 면 software_rendering:=false
        DeclareLaunchArgument('software_rendering', default_value='true', description='true 면 시뮬 서버(센서 렌더링)만 소프트웨어 GL'),
        SetEnvironmentVariable('LIBGL_ALWAYS_SOFTWARE', '1', condition=IfCondition(LaunchConfiguration('software_rendering'))),
        DeclareLaunchArgument('gui', default_value='true', description='false 면 화면 없이 서버만'),
        DeclareLaunchArgument('world', default_value='mission.world', description='worlds/ 안 파일 이름 (원본 맵: disaster.world)'),
        DeclareLaunchArgument('verbose', default_value='false'),
        SetEnvironmentVariable('IGN_GAZEBO_RESOURCE_PATH', os.pathsep.join(paths)),
        SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', os.pathsep.join(paths)),
        OpaqueFunction(function=_gz),
        Node(package='ros_gz_bridge', executable='parameter_bridge', name='clock_bridge', output='screen',
             arguments=['/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock']),
        # 파이썬 노드용 느린 시계 /clock_slow (50 Hz). 각 launch 가 파이썬 노드만 /clock:=/clock_slow 로 리맵
        Node(package='comm_relay', executable='clock_throttle', output='screen'),
    ])
