"""팀원 nav2 launch 가 import 해서 쓰는 A1 용 Nav2 보정 (terrain_setup.sh 가 launch 에 호출 블록을 넣음).

[지형] 비용지도 파라미터에 지형 위험 층:

  terrain_layer (nav2_costmap_2d::ObstacleLayer, 라이다 obstacle_layer 와 따로)
    terrain       /<ns>/terrain_hazards  marking
    terrain_free  /<ns>/terrain_free     clearing (광선 시작 = 깊이 카메라, 점 = 로봇 둘레 3 m 원 -> 원 안을 지우고 다시 찍음)
  층을 따로 두는 이유: 지형 clearing 광선이 라이다로 찍은 장애물을 지우지 않게.
  전역 비용지도에서는 updateWithMax 라 지형 층이 SLAM 정적 지도(벽)를 지울 수 없다.
그리고 라이다 obstacle_min_range 0.2: 걸을 때 라이다가 자기 몸에 맞아 생기는 0.15~0.16 m 점을 비용지도에 안 찍음
  (2026-11-07 Gazebo 관측, return_relay 도 같은 이유로 거름).
"""

import os

SENSOR_FRAME = 'terrain_camera_link'


def terrain_on():
    """A1_TERRAIN=0 이면 지형 층만 끈다 (비교 실험용). 라이다 최소 거리·진행 검사 보정은 그대로."""
    return os.environ.get('A1_TERRAIN', '1') != '0'


def terrain_layer(ns, sensor_frame=SENSOR_FRAME):
    return {
        'plugin': 'nav2_costmap_2d::ObstacleLayer',
        'enabled': True,
        # 몸 둘레(robot_radius 0.23) 안 지형 위험을 지우지 않음: 지우면 미끄러져 가장자리에 0.23 m 안으로 다가간 순간
        # 가장자리 칸이 비용지도에서 사라져 DWB 가 그대로 감 (2026-11-08 mission: 경사로 서쪽 가장자리 0.16 m 옆에서 넘어짐).
        # 몸에 붙은 오검출은 지형 노드가 로봇 0.35 m 안 새 위험을 버려서 막음.
        'footprint_clearing_enabled': False,
        'observation_sources': 'terrain terrain_free',
        # sensor_frame 을 꼭 줌: 없으면 '센서 위치' = 점구름 프레임(odom / map) 원점 = 스폰 지점이라 obstacle_max_range 가
        # 스폰에서 잰 거리가 됨 -> 스폰에서 3.5 m 넘는 곳의 위험은 비용지도에 안 찍힘
        # (2026-11-08 표 기록: 로봇이 스폰에서 3.8 m 넘게 떨어진 때 위험 0.4 m 안 110 표본 중 치명 칸 있는 것 0 %, 가까울 땐 91~100 %).
        'terrain': {
            'topic': f'/{ns}/terrain_hazards', 'data_type': 'PointCloud2',
            'marking': True, 'clearing': False, 'sensor_frame': sensor_frame,
            'min_obstacle_height': 0.0, 'max_obstacle_height': 1.0,
            'obstacle_max_range': 4.0, 'obstacle_min_range': 0.0,
        },
        'terrain_free': {
            'topic': f'/{ns}/terrain_free', 'data_type': 'PointCloud2',
            'marking': False, 'clearing': True, 'sensor_frame': sensor_frame,
            'min_obstacle_height': 0.0, 'max_obstacle_height': 1.0,
            'raytrace_max_range': 3.5, 'raytrace_min_range': 0.0,
        },
    }


def _insert(costmap, ns):
    plugins = list(costmap.get('plugins', []))
    if 'terrain_layer' in plugins:
        return
    i = plugins.index('inflation_layer') if 'inflation_layer' in plugins else len(plugins)
    plugins.insert(i, 'terrain_layer')
    costmap['plugins'] = plugins
    costmap['terrain_layer'] = terrain_layer(ns)


def add_local(costmap, ns):
    """국소 비용지도(odom) ros__parameters 에 지형 층 + 라이다 최소 거리·높이 범위."""
    if terrain_on():
        _insert(costmap, ns)
    scan = costmap.get('obstacle_layer', {}).get('scan')
    if isinstance(scan, dict):
        scan.setdefault('obstacle_min_range', 0.2)
        # Humble ObstacleLayer 는 관측원별 max_obstacle_height 기본값이 0.0 이라, 안 주면 높이 0 보다 큰 점(라이다 약 0.28 m)을
        # 전부 버림 -> 국소 비용지도에 라이다 장애물이 하나도 안 찍혔음 (2026-11-09 cmcheck: 끝점 2~3 만 개 중 0 %, 지형 층 켬/끔 같음).
        scan.setdefault('min_obstacle_height', 0.0)
        scan.setdefault('max_obstacle_height', 2.0)
    return costmap


def add_global(costmap, ns):
    """전역 비용지도(map) ros__parameters 에 지형 층."""
    if terrain_on():
        _insert(costmap, ns)
    return costmap


def tune_planner(planner):
    """planner_server ros__parameters 보정: NavFn allow_unknown False -> True (A1_PLAN_UNKNOWN=0 이면 그대로).
    왜: goal_assigner 는 두 로봇 지도를 합친 공유 지도로 목표를 고르는데, 각 로봇 플래너는 자기 SLAM 지도만 본다.
    상대 로봇이 본 구역은 내 지도에서 '모름' 이라 allow_unknown False 면 계획이 0 점으로 실패
    (2026-11-08 mission: a1_2 계획 실패 24 / 목표 85, 로봇별 블랙리스트 21, 놀기 40%). 모름을 지나는 경로를 허용하면
    가 보면서 라이다·지형 층이 새로 본 장애물은 국소 비용지도가 막는다(못 지나가면 진행 실패 -> 그 로봇 블랙리스트).
    대가: 모름 속 벽을 지나는 경로가 나올 수 있음 (실제로 막히면 위처럼 실패로 끝남)."""
    if os.environ.get('A1_PLAN_UNKNOWN', '1') == '0':
        return planner
    for name in planner.get('planner_plugins', []):
        pl = planner.get(name)
        if isinstance(pl, dict) and 'navfn' in str(pl.get('plugin', '')).lower():
            pl['allow_unknown'] = True
    return planner


def tune_controller(ctrl):
    """controller_server ros__parameters 보정.
    progress_checker movement_time_allowance 10 -> 20 s: A1(CHAMP) 은 제자리 회전이 약 0.4 rad/s 라 목표가 뒤쪽이면
    돌기만 8 s 가까이 걸리고, Humble SimpleProgressChecker 는 회전을 진행으로 안 쳐서(0.3 m 이동만 봄)
    'Failed to make progress' 로 목표를 버림 (2026-11-07 nav: 북쪽을 보던 a1_2 가 남동쪽 목표에서 실패)."""
    pc = ctrl.get('progress_checker')
    if isinstance(pc, dict) and float(pc.get('movement_time_allowance', 10.0)) < 20.0:
        pc['movement_time_allowance'] = 20.0
    # DWB RotateToGoal 은 목표 xy_goal_tolerance(지정 안 하면 0.25) 안에 들면 전진을 불법으로 하고 제자리 회전만 허용한다.
    # 목표 판정기(general_goal_checker)는 0.20 이라, 0.20~0.25 에서 멈추면 회전만 하다 'Failed to make progress'
    # (2026-11-08 nav_debug Gazebo: 목표 3 개 모두 0.23~0.24 m 남기고 실패, 실제 controller_server + 가짜 로봇으로 재현 0.238 m).
    # 그래서 RotateToGoal 창을 목표 판정기와 같게 맞춘다 (Nav2 기본 설정도 둘이 같음).
    gc_tol = None
    for name in ctrl.get('goal_checker_plugins', []):
        gc = ctrl.get(name)
        if isinstance(gc, dict) and 'xy_goal_tolerance' in gc:
            t = float(gc['xy_goal_tolerance'])
            gc_tol = t if gc_tol is None else min(gc_tol, t)
    if gc_tol is not None:
        for name in ctrl.get('controller_plugins', []):
            fp = ctrl.get(name)
            if isinstance(fp, dict) and 'RotateToGoal' in fp.get('critics', []):
                fp['xy_goal_tolerance'] = min(float(fp.get('xy_goal_tolerance', 0.25)), gc_tol)
    return ctrl
