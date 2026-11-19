import tempfile
import yaml

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    actions = []

    for ns in ['a1_1', 'a1_2']:

        controller_params = {

            'use_sim_time': True,

            'controller_frequency': 10.0,

            'odom_topic': f'/{ns}/odom',

            'min_x_velocity_threshold': 0.001,
            'min_y_velocity_threshold': 0.001,
            'min_theta_velocity_threshold': 0.001,

            'progress_checker_plugin': 'progress_checker',

            'goal_checker_plugins': [
                'general_goal_checker'
            ],

            'controller_plugins': [
                'FollowPath'
            ],

            'progress_checker': {
                'plugin': 'nav2_controller::SimpleProgressChecker',
                'required_movement_radius': 0.3,
                'movement_time_allowance': 10.0
            },

            'general_goal_checker': {
                'plugin': 'nav2_controller::SimpleGoalChecker',
                'xy_goal_tolerance': 0.20,
                'yaw_goal_tolerance': 3.14,   # 탐색은 도착 방향 무관 (2026-11-10 fix10: 도착 시 제자리 정렬 정지 9/15)
                'stateful': True
            },

            # 2026-11-19 N1: DWB -> MPPI (nav2_mppi_controller 1.1.20). 기준선 멈춤의 99 % 가 Nav (DWB 0 명령·진동·제자리 회전).
            # 속도 한도는 DWB 와 같게 (앞 0~0.2 m/s, 후진 없음: N2 에서 후진이 앞뒤 진동·odom 흐름을 키워 기각), 컨트롤러만 바꿈.
            # 비평가 값은 Nav2 기본 설정 그대로, 목표 방향(GoalAngle)·전진 선호(PreferForward)는 뺌 (도착 방향 무관, 후진 없음).
            'FollowPath': {
                'plugin': 'nav2_mppi_controller::MPPIController',
                'time_steps': 40,          # model_dt 0.1 x 40 = 4 s 앞 (0.2 m/s 면 0.8 m, 국소 지도 3x3 m 안)
                'model_dt': 0.1,           # = 1 / controller_frequency
                'batch_size': 1000,        # 기본 2000 의 절반: 시뮬과 로봇 둘이 CPU 를 같이 씀
                'iteration_count': 1,
                'vx_std': 0.2,
                'vy_std': 0.0,
                'wz_std': 0.4,
                'vx_max': 0.2,
                'vx_min': 0.0,
                'vy_max': 0.0,
                'wz_max': 0.5,
                'temperature': 0.3,
                'gamma': 0.015,
                'motion_model': 'DiffDrive',
                'prune_distance': 1.5,     # 국소 지도 반 폭
                'transform_tolerance': 0.4,
                'retry_attempt_limit': 1,
                'visualize': False,
                'critics': ['ConstraintCritic', 'CostCritic', 'GoalCritic', 'PathAlignCritic', 'PathFollowCritic', 'PathAngleCritic'],
                'ConstraintCritic': {'enabled': True, 'cost_power': 1, 'cost_weight': 4.0},
                'CostCritic': {'enabled': True, 'cost_power': 1, 'cost_weight': 3.81, 'critical_cost': 300.0,
                               'consider_footprint': True,       # A1 직사각형 발자국으로 충돌 검사
                               'collision_cost': 1000000.0, 'near_goal_distance': 1.0, 'trajectory_point_step': 2},
                'GoalCritic': {'enabled': True, 'cost_power': 1, 'cost_weight': 5.0, 'threshold_to_consider': 1.4},
                'PathAlignCritic': {'enabled': True, 'cost_power': 1, 'cost_weight': 14.0, 'max_path_occupancy_ratio': 0.05,
                                    'trajectory_point_step': 4, 'threshold_to_consider': 0.5, 'offset_from_furthest': 20,
                                    'use_path_orientations': False},
                'PathFollowCritic': {'enabled': True, 'cost_power': 1, 'cost_weight': 5.0, 'offset_from_furthest': 5,
                                     'threshold_to_consider': 1.4},
                'PathAngleCritic': {'enabled': True, 'cost_power': 1, 'cost_weight': 2.0, 'offset_from_furthest': 4,
                                    'threshold_to_consider': 0.5, 'max_angle_to_furthest': 1.0, 'mode': 0}
            }
        }

        costmap_params = {

            'use_sim_time': True,

            'global_frame': 'odom',
            'robot_base_frame': 'base_footprint',

            'update_frequency': 5.0,
            'publish_frequency': 2.0,

            'transform_tolerance': 0.6,

            'rolling_window': True,

            'width': 3,
            'height': 3,

            'resolution': 0.05,
            'footprint': '[[0.30, 0.16], [0.30, -0.16], [-0.30, -0.16], [-0.30, 0.16]]',   # A1 실제 모양 약 0.6 x 0.32 m (2026-11-10, 전에는 robot_radius 0.23 원 -> Husky 옆 0.5 m 틈 통과 불가)

            'plugins': [
                'obstacle_layer',
                'inflation_layer'
            ],

            'obstacle_layer': {

                'plugin': 'nav2_costmap_2d::ObstacleLayer',

                'enabled': True,

                'observation_sources': 'scan',

                'scan': {
                    'topic': f'/{ns}/scan',
                    'data_type': 'LaserScan',

                    'marking': True,
                    'clearing': True,

                    'obstacle_max_range': 3.0,
                    'raytrace_max_range': 3.5
                }
            },

            'inflation_layer': {

                'plugin': 'nav2_costmap_2d::InflationLayer',

                'inflation_radius': 0.35,
                'cost_scaling_factor': 3.0
            },

            'always_send_full_costmap': True
        }

        params = {
            f'/{ns}/controller_server': {
                'ros__parameters': controller_params
            },

            f'/{ns}/local_costmap/local_costmap': {
                'ros__parameters': costmap_params
            }
        }

        # >>> terrain_patch (terrain_perception terrain_setup.sh, 되돌리기: revert, 지형 층만 끄기: A1_TERRAIN=0)
        try:
            from terrain_perception import nav2_patch as _np
            _np.add_local(params[f'/{ns}/local_costmap/local_costmap']['ros__parameters'], ns)
            _np.tune_controller(params[f'/{ns}/controller_server']['ros__parameters'])
        except ImportError as _e:
            print(f'[terrain_patch] terrain_perception 없음 -> 보정 없이 실행: {_e}')
        # <<< terrain_patch

        with tempfile.NamedTemporaryFile(
            mode='w',
            suffix='.yaml',
            prefix=f'controller_{ns}_',
            delete=False
        ) as tmp:

            yaml.safe_dump(params, tmp)
            params_file = tmp.name

        controller = Node(
            package='nav2_controller',
            executable='controller_server',

            namespace=ns,
            name='controller_server',

            output='screen',

            parameters=[params_file],

            remappings=[
                ('/tf', f'/{ns}/tf'),
                ('/tf_static', f'/{ns}/tf_static'),

                # 중요: 실제 로봇에는 아직 명령을 보내지 않음
                ('cmd_vel', f'/{ns}/cmd_vel_nav_test')
            ]
        )

        lifecycle_manager = Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',

            namespace=ns,
            name='lifecycle_manager_controller',

            output='screen',

            parameters=[{
                'use_sim_time': True,
                'autostart': True,
                'node_names': ['controller_server']
            }]
        )

        actions.extend([
            controller,
            lifecycle_manager
        ])

    return LaunchDescription(actions)
