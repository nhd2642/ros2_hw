"""임무 계층: Husky(임무·주행) + A1 두 대(추종·탐색·복귀·사람 인식). 시뮬·SLAM·Nav2·지형 노드가 뜬 뒤에 띄운다.

실행 순서 (각각 다른 터미널; tools/run_mission.sh 가 같은 순서로 자동 실행):
  1) ros2 launch disaster_sim sim.launch.py                      # 월드 + Husky + A1 두 대 (a1_2 기동까지 약 30 s)
  2) ros2 launch a1_exploration dual_slam.launch.py
  3) ros2 launch a1_exploration nav2_planners.launch.py
     ros2 launch a1_exploration nav2_controllers.launch.py
  4) ros2 launch terrain_perception terrain.launch.py
  5) ros2 launch comm_relay mission.launch.py husky:=false       # A1 쪽 + Husky 주행
  6) ros2 run comm_relay husky_mission                           # Husky 를 여는 노드: 이 터미널에 start / 0 / 1 입력
(husky:=true 면 5 가 husky_mission 도 띄움 -> 명령은 ros2 topic pub --once /husky/cmd std_msgs/msg/String "{data: start}")
start_set:=k 는 sim.launch.py 와 같은 값으로 (세트 k 부터 시작하는 부분 시험).
"""
import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetParameter

from comm_relay.mission_logic import start_poses

# 작업 공간 = <ws>/install/comm_relay/share/comm_relay 의 4 단계 위 (팀원 탐색 스크립트는 src 경로로 실행)
WS = os.path.abspath(os.path.join(get_package_share_directory('comm_relay'), '..', '..', '..', '..'))


def _setup(context):
    k = int(LaunchConfiguration('start_set').perform(context))
    config = os.path.join(get_package_share_directory('disaster_sim'), 'config', 'mission.yaml')
    with open(config) as f:
        cfg = yaml.safe_load(f)
    pose = start_poses(cfg, k)
    robots = list(cfg['slots'])
    d = os.path.join(WS, 'src', 'a1_exploration', 'scripts')

    def script(name, *ros_args):
        cmd = ['python3', os.path.join(d, name), '--ros-args', '-p', 'use_sim_time:=true', '-r', '/clock:=/clock_slow', *ros_args]
        return ExecuteProcess(cmd=cmd, cwd=d, output='screen', name=name.replace('.py', ''))

    spawn_args = [a for ns in robots for a in ('-p', f'spawn.{ns}:={[float(v) for v in pose[ns]]}')]
    def node(exe, **params):      # 파이썬 노드는 느린 시계로 (clock_throttle 설명 참고)
        return Node(package='comm_relay', executable=exe, output='screen', parameters=[params],
                    remappings=[('/clock', '/clock_slow')])

    nodes = [
        node('husky_driver', spawn=[float(v) for v in pose['husky']]),
        node('a1_follower', robots=robots, **{f'slot.{ns}': [float(v) for v in cfg['slots'][ns]] for ns in robots}),
        node('return_relay', output_topic='cmd_vel_return'),
        node('cmd_vel_arbiter'),
        node('frontier_comm_filter'),
        node('nav_executor'),
        node('survivor_detector'),
        script('nav_goal_bridge.py', *spawn_args),
        script('frontier_detector.py'),
        script('goal_assigner.py', '-r', '/frontier_candidates:=/frontier_candidates_comm'),
    ]
    if LaunchConfiguration('husky').perform(context).lower() == 'true':
        nodes.append(node('husky_mission', config=config, start_set=k))
    return nodes


def generate_launch_description():
    return LaunchDescription([
        SetParameter(name='use_sim_time', value=True),
        DeclareLaunchArgument('start_set', default_value='0'),
        DeclareLaunchArgument('husky', default_value='true', description='false 면 husky_mission 은 사용자가 따로 띄움'),
        OpaqueFunction(function=_setup),
    ])
