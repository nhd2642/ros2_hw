import tempfile
import yaml

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    actions = []

    for ns in ('a1_1', 'a1_2'):

        parameters = {
            f'/{ns}/planner_server': {
                'ros__parameters': {
                    'use_sim_time': True,
                    'expected_planner_frequency': 1.0,

                    'planner_plugins': ['GridBased'],

                    'GridBased': {
                        'plugin': 'nav2_navfn_planner/NavfnPlanner',
                        'tolerance': 0.25,
                        # A* -> 다익스트라 (2026-11-13): A* 는 출발 칸 비용이 높을 때(사람·벽 0.3 m 안) 'Failed to create a plan from potential
                        # when a legal potential was found' 로 0 점 경로를 냄. 임무 실행의 계획 실패가 전부 이 메시지였고 복귀 실패로 이어졌다
                        'use_astar': False,
                        'allow_unknown': False
                    }
                }
            },

            f'/{ns}/global_costmap/global_costmap': {
                'ros__parameters': {
                    'use_sim_time': True,

                    'global_frame': 'map',
                    'robot_base_frame': 'base_footprint',

                    'update_frequency': 1.0,
                    'publish_frequency': 1.0,
                    'transform_tolerance': 0.6,

                    'resolution': 0.05,
                    'footprint': '[[0.30, 0.16], [0.30, -0.16], [-0.30, -0.16], [-0.30, 0.16]]',   # A1 실제 모양 약 0.6 x 0.32 m (2026-11-10, 전에는 robot_radius 0.23 원 -> Husky 옆 0.5 m 틈 통과 불가)

                    'track_unknown_space': True,
                    'rolling_window': False,

                    'plugins': [
                        'static_layer',
                        'inflation_layer'
                    ],

                    'static_layer': {
                        'plugin': 'nav2_costmap_2d::StaticLayer',

                        'map_topic': f'/{ns}/map',

                        'map_subscribe_transient_local': True,
                        'subscribe_to_updates': False
                    },

                    'inflation_layer': {
                        'plugin': 'nav2_costmap_2d::InflationLayer',

                        'inflation_radius': 0.70,   # fix18: 0.35 -> 0.70
                        'cost_scaling_factor': 3.0
                    },

                    'always_send_full_costmap': True
                }
            }
        }

        # 로봇별 Nav2 파라미터 파일 생성
        # >>> terrain_patch (terrain_perception terrain_setup.sh, 되돌리기: revert, 지형 층만 끄기: A1_TERRAIN=0)
        try:
            from terrain_perception import nav2_patch as _np
            _np.add_global(parameters[f'/{ns}/global_costmap/global_costmap']['ros__parameters'], ns)
            if hasattr(_np, 'tune_planner'):
                _np.tune_planner(parameters[f'/{ns}/planner_server']['ros__parameters'])
        except ImportError as _e:
            print(f'[terrain_patch] terrain_perception 없음 -> 보정 없이 실행: {_e}')
        # <<< terrain_patch

        with tempfile.NamedTemporaryFile(
            mode='w',
            suffix='.yaml',
            prefix=f'nav2_{ns}_',
            delete=False
        ) as tmp:

            yaml.safe_dump(parameters, tmp)
            params_file = tmp.name

        # Nav2 Global Planner
        planner = Node(
            package='nav2_planner',
            executable='planner_server',

            namespace=ns,
            name='planner_server',

            output='screen',

            parameters=[params_file],

            remappings=[
                ('/tf', f'/{ns}/tf'),
                ('/tf_static', f'/{ns}/tf_static')
            ]
        )

        # Planner Lifecycle 관리
        lifecycle_manager = Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',

            namespace=ns,
            name='lifecycle_manager_planner',

            output='screen',

            parameters=[{
                'use_sim_time': True,
                'autostart': True,
                'node_names': ['planner_server'],
                'bond_timeout': 10.0
            }]
        )

        actions.extend([
            planner,
            lifecycle_manager
        ])

    return LaunchDescription(actions)
