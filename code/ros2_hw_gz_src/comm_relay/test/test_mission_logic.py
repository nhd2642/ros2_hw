import pytest

from comm_relay.mission_logic import DoneWatch, SurvivorTally, in_rect, path_enters_rect, start_poses, zone_dict


def test_done_after_hold_of_zeros():
    w = DoneWatch(hold=30.0, fresh=10.0)
    for t in range(0, 30, 3):
        w.update(float(t), 0)
    assert not w.done(29.0)
    w.update(30.0, 0)
    assert w.done(30.0)
    assert not w.done(45.0)              # 마지막 값이 10 s 넘게 오래됨 (assigner 죽음 등)


def test_done_reset_by_frontier_or_unknown():
    w = DoneWatch(hold=30.0)
    w.update(0.0, 0)
    w.update(20.0, 2)                    # 프런티어 다시 생김
    w.update(21.0, 0)
    assert not w.done(45.0)
    w.update(51.0, 0)
    assert w.done(51.0)
    w.update(52.0, -1)                   # 모름 -> 처음부터
    w.update(53.0, 0)
    assert not w.done(60.0)


def test_done_disabled_and_never_received():
    assert not DoneWatch(hold=0.0).done(100.0)
    assert not DoneWatch(hold=30.0).done(100.0)


def test_rect_zone_and_path_check():
    r = [4.0, 12.0, 1.5, 6.5]
    assert in_rect(r, 8.0, 3.0) and not in_rect(r, 8.0, 1.4)
    assert in_rect(r, 8.0, 1.4, 0.2) and not in_rect(r, 8.0, 1.6, -0.3)
    assert path_enters_rect([(8, 0), (8, 1.6)], r) is None              # 여유 0.15 안
    assert path_enters_rect([(8, 0), (8, 1.7), (8, 3)], r) == (8, 1.7)
    assert path_enters_rect([], r) is None


def test_survivor_tally_merges_and_counts_by_zone():
    t = SurvivorTally(merge=1.5)
    assert t.add(10.6, 5.5) and not t.add(11.2, 5.0)                    # 두 로봇이 같은 사람
    assert t.add(5.5, -5.7)
    assert len(t.people) == 2 and abs(t.people[0][0] - 10.9) < 1e-9
    assert t.count([[4, 12, 1.5, 6.5]]) == 1 and t.count([[4, 12, 1.5, 6.5], [4, 12, -6.5, -1.5]]) == 2
    assert t.count([[16, 24, 1.5, 6.5]]) == 0


def test_start_poses():
    cfg = dict(spawn={'husky': [0, 0, 0], 'a1_1': [-2.4, -0.8, 0]}, slots={'a1_1': [-1.6, -0.8]}, sets=[dict(park=[9.6, 0.0])])
    assert start_poses(cfg, 0) is cfg['spawn']
    assert start_poses(cfg, 1) == {'husky': [9.6, 0.0, 0.0], 'a1_1': [8.0, -0.8, 0.0]}


def test_refine_goal_moves_out_of_inflation_or_drops():
    import numpy as np
    from comm_relay.mission_logic import refine_goal
    c = np.zeros((40, 40), np.uint8)
    c[:, 20:] = 254                  # 벽 (x >= 1.0 m)
    c[:, 17:20] = 200                # 팽창 영역 (x 0.85~1.0)
    x, y, d = refine_goal(c, 0.05, 0.0, 0.0, 0.9, 1.0)
    assert x < 0.85 and abs(y - 1.0) < 0.06 and 0 < d <= 0.5          # 가까운 여유 칸으로
    assert refine_goal(c, 0.05, 0.0, 0.0, 0.5, 1.0) == (0.5, 1.0, 0.0)  # 여유 칸이면 그대로
    assert refine_goal(c, 0.05, 0.0, 0.0, 1.6, 1.0) is None              # 0.5 m 안에 여유 칸 없음 -> 버림
    c[:, :20] = 255                  # 모름(프런티어 쪽)은 갈 수 있는 칸으로 봄
    c[:, 17:20] = 200
    assert refine_goal(c, 0.05, 0.0, 0.0, 0.3, 1.0) == (0.3, 1.0, 0.0)
    assert refine_goal(c, 0.05, 0.0, 0.0, 9.0, 9.0) == (9.0, 9.0, 0.0)  # 비용지도 밖은 판단 안 함


def test_zone_dict_from_mission_state():
    from mission_interfaces.msg import MissionState, Zone
    m = MissionState(phase='EXPLORE', zones=[Zone(robot='a1_1', x_min=16.0, x_max=24.0, y_min=-6.5, y_max=-1.5),
                                             Zone(robot='a1_2', x_min=16.0, x_max=24.0, y_min=1.5, y_max=6.5)])
    z = zone_dict(m.zones)
    assert z == {'a1_1': [16.0, 24.0, -6.5, -1.5], 'a1_2': [16.0, 24.0, 1.5, 6.5]}
    assert in_rect(z['a1_2'], 20.0, 3.0) and not in_rect(z['a1_1'], 20.0, 3.0)
    assert zone_dict(MissionState().zones) == {}
