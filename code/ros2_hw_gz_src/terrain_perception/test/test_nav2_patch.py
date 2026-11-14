from terrain_perception import nav2_patch


def ctrl(**fp):
    return {'goal_checker_plugins': ['general_goal_checker'], 'controller_plugins': ['FollowPath'],
            'progress_checker': {'movement_time_allowance': 10.0},
            'general_goal_checker': {'xy_goal_tolerance': 0.20},
            'FollowPath': dict({'critics': ['RotateToGoal', 'BaseObstacle']}, **fp)}


def test_rotate_window_matches_goal_checker():
    c = nav2_patch.tune_controller(ctrl())
    assert c['FollowPath']['xy_goal_tolerance'] == 0.20          # 없던 값(기본 0.25) -> 판정기와 같게
    assert c['progress_checker']['movement_time_allowance'] == 20.0


def test_smaller_window_kept_and_no_rotate_critic_untouched():
    assert nav2_patch.tune_controller(ctrl(xy_goal_tolerance=0.1))['FollowPath']['xy_goal_tolerance'] == 0.1
    c = ctrl(); c['FollowPath']['critics'] = ['BaseObstacle']
    assert 'xy_goal_tolerance' not in nav2_patch.tune_controller(c)['FollowPath']


def test_terrain_layer_keeps_hazards_under_footprint():
    assert nav2_patch.terrain_layer('a1_1')['footprint_clearing_enabled'] is False


def test_terrain_marking_range_from_camera_not_frame_origin():
    t = nav2_patch.terrain_layer('a1_1')['terrain']
    assert t['sensor_frame'] == 'terrain_camera_link' and t['obstacle_max_range'] >= 3.5


def test_planner_allows_unknown_unless_disabled(monkeypatch):
    def pl():
        return {'planner_plugins': ['GridBased'],
                'GridBased': {'plugin': 'nav2_navfn_planner/NavfnPlanner', 'allow_unknown': False}}
    monkeypatch.delenv('A1_PLAN_UNKNOWN', raising=False)
    assert nav2_patch.tune_planner(pl())['GridBased']['allow_unknown'] is True
    monkeypatch.setenv('A1_PLAN_UNKNOWN', '0')
    assert nav2_patch.tune_planner(pl())['GridBased']['allow_unknown'] is False


def test_local_scan_gets_height_range():
    # 팀원 설정 그대로(높이 범위 없음): Humble 기본 max_obstacle_height 0.0 이면 라이다 점 전부 버려짐 -> 2.0 을 채움
    cm = {'plugins': ['obstacle_layer', 'inflation_layer'],
          'obstacle_layer': {'scan': {'topic': '/a1_1/scan', 'data_type': 'LaserScan'}}}
    s = nav2_patch.add_local(cm, 'a1_1')['obstacle_layer']['scan']
    assert s['max_obstacle_height'] == 2.0 and s['min_obstacle_height'] == 0.0 and s['obstacle_min_range'] == 0.2
    cm2 = {'plugins': ['obstacle_layer'], 'obstacle_layer': {'scan': {'max_obstacle_height': 1.0}}}
    assert nav2_patch.add_local(cm2, 'a1_1')['obstacle_layer']['scan']['max_obstacle_height'] == 1.0   # 팀원이 주면 그대로
