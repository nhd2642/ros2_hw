import os
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

            'FollowPath': {
                'plugin': 'dwb_core::DWBLocalPlanner',

                'min_vel_x': 0.0,
                'max_vel_x': 0.2,   # 2026-11-10: 0.15 -> 0.2 (walk_check 0.2 m/s 직진 102 %)

                'min_vel_y': 0.0,
                'max_vel_y': 0.0,

                'max_vel_theta': 0.5,

                'min_speed_xy': 0.03,   # 0 명령(정지) 궤적 제외: 선속도 < 0.03 이고 |회전| < 0.05 인 샘플은 무효 (fix12, DWB xy_theta_iterator.isValidSpeed)
                'max_speed_xy': 0.2,   # 2026-11-10: 0.15 -> 0.2 (walk_check 0.2 m/s 직진 102 %)
                'min_speed_theta': 0.05,

                'acc_lim_x': 1.0,
                'acc_lim_y': 0.0,
                'acc_lim_theta': 1.0,

                'decel_lim_x': -1.0,
                'decel_lim_y': 0.0,
                'decel_lim_theta': -1.0,

                'vx_samples': 10,
                'vy_samples': 1,
                'vtheta_samples': 15,

                'sim_time': 1.0,

                'linear_granularity': 0.05,
                'angular_granularity': 0.025,

                'transform_tolerance': 0.4,

                'critics': [
                    'Oscillation',
                    'BaseObstacle',
                    'GoalAlign',
                    'PathAlign',
                    'PathDist',
                    'GoalDist'
                ],

                'BaseObstacle.scale': 0.02,

                'PathAlign.scale': 32.0,
                'PathAlign.forward_point_distance': 0.1,

                'GoalAlign.scale': 24.0,
                'GoalAlign.forward_point_distance': 0.1,

                'PathDist.scale': 32.0,
                'GoalDist.scale': 24.0,

                'RotateToGoal.scale': 32.0,
                'RotateToGoal.slowing_factor': 5.0,
                'RotateToGoal.lookahead_time': -1.0,
                # 0 명령 진단 (2026-11-10, comm_relay tools/dwb_diag.py): A1_DWB_DIAG=1 일 때만 /<ns>/evaluation 발행 (궤적 159 개 점수라 무거움)
                'publish_evaluation': os.environ.get('A1_DWB_DIAG', '0') == '1'
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
