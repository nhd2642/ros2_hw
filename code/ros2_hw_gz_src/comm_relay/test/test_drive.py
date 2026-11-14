"""drive_logic: 목표 조향·장애물 회피 (ROS 없음)."""
import math

import numpy as np

from comm_relay.drive_logic import Steer, box_center, find_body, free_along, scan_xy


def wall(x, y0, y1, n=40):
    return np.stack([np.full(n, x), np.linspace(y0, y1, n)], axis=1)


def test_scan_xy_drops_self_and_out_of_range():
    r = [0.1, 0.2, 5.0, float('inf'), 12.0]
    pts = scan_xy(r, 0.0, math.pi / 2, 0.1, 10.0, lidar_x=0.3, body=(0.25, 0.3, 0.17))
    assert pts.shape == (2, 2)                       # 0.1 = range_min, inf, 12 > max 는 버림
    assert np.allclose(pts[1], [0.3 - 5.0, 0.0], atol=1e-6)


def test_free_along_band():
    pts = wall(1.0, -0.2, 0.2)
    assert abs(free_along(pts, 0.0, 0.3) - 1.0) < 1e-6
    assert free_along(pts, math.pi / 2, 0.3) == math.inf
    assert free_along(np.zeros((0, 2)), 0.0, 0.3) == math.inf


def test_goes_straight_when_clear_and_stops_at_goal():
    s = Steer(v_max=0.2)
    v, w, st = s.cmd(np.zeros((0, 2)), 0.0, 3.0)
    assert st == 'GO' and abs(v - 0.2) < 1e-9 and w == 0.0
    assert s.cmd(np.zeros((0, 2)), 0.0, 0.1) == (0.0, 0.0, 'ARRIVED')


def test_turns_in_place_when_goal_is_behind():
    v, w, st = Steer(w_max=0.5).cmd(np.zeros((0, 2)), 2.5, 3.0)
    assert v == 0.0 and w == 0.5 and st == 'GO'


def test_avoids_small_wall_toward_open_side():
    s = Steer(v_max=0.2, half=0.37, front=0.45, look=1.2)
    pts = wall(1.0, -0.2, 0.8)                        # 앞 1 m, 왼쪽으로 치우친 벽 -> 오른쪽이 가까운 우회
    s.side = -1.0
    v, w, st = s.cmd(pts, 0.0, 4.0)
    assert st == 'AVOID' and w < 0.0
    s2 = Steer(v_max=0.2, half=0.37, front=0.45, look=1.2)
    s2.side = 1.0                                     # 지난번 왼쪽으로 돌았어도 오른쪽이 더 작은 각으로 트이면 오른쪽
    assert s2.cmd(pts, 0.0, 4.0)[1] < 0.0


def test_blocked_when_boxed_in():
    ring = np.array([[0.6 * math.cos(a), 0.6 * math.sin(a)] for a in np.linspace(-math.pi, math.pi, 90)])
    assert Steer().cmd(ring, 0.0, 3.0) == (0.0, 0.0, 'BLOCKED')


def test_slows_near_obstacle_ahead_while_turning_away():
    s = Steer(v_max=0.2, half=0.3, front=0.3, look=1.5)
    v, w, st = s.cmd(wall(0.5, -0.25, 0.25), 0.0, 4.0)
    assert st == 'AVOID' and v < 0.2


def test_find_body_picks_husky_cluster_not_wall_or_small_robot():
    rear = np.stack([np.full(14, 1.1), np.linspace(-0.33, 0.33, 14)], axis=1)        # 앞 1.1 m 에 Husky 뒷면 (중심 1.6 m)
    facade = np.stack([np.linspace(-1.0, 4.0, 80), np.full(80, 2.3)], axis=1)          # 왼쪽 2.3 m 건물 벽 (길다)
    other = np.array([[1.4, -1.6], [1.45, -1.62], [1.5, -1.6]])                        # 상대 A1 (점 3 개)
    pts = np.concatenate([rear, facade, other])
    c = find_body(pts, (1.6, 0.5))                                                     # 자세가 0.5 m 어긋난 어림
    assert c is not None and abs(c[0] - 1.5) < 0.15 and abs(c[1]) < 0.1                # 평균 1.1 + depth 0.4
    assert find_body(facade, (1.6, 0.9)) is None                                       # 벽만 있으면 못 찾음 (너무 긺)
    assert find_body(np.zeros((0, 2)), (1.6, 0.0)) is None
    small_wall = np.stack([np.full(12, 1.2), np.linspace(-0.2, 0.8, 12)], axis=1)      # 출발점: 앞 1.2 m 작은 벽, Husky 는 그 뒤 (2.4, 0.8)
    assert find_body(small_wall, (2.4, 0.8)) is None                                   # 추정 중심이 gate(0.7 m) 밖 -> 버림


def husky_faces(cx, cy, yaw, side_upto=0.495, rear=True, n=12):
    """원점에서 본 Husky(0.99 x 0.67) 의 뒷면과 원점 쪽 옆면 점 (옆면은 뒤에서 side_upto 까지만 보인다고 침)."""
    pts = []
    if rear:
        pts += [(-0.495, v) for v in np.linspace(-0.335, 0.335, n)]
    co, si = math.cos(yaw), math.sin(yaw)
    side = 0.335 if (-cx * -si + -cy * co) > 0 else -0.335        # 원점이 있는 쪽 옆면
    pts += [(u, side) for u in np.linspace(-0.495, side_upto, n)]
    return np.array([(cx + u * co - v * si, cy + u * si + v * co) for u, v in pts])


def test_box_center_from_rear_and_side_faces():
    for yaw in (0.0, 0.2, -0.3):
        for upto in (0.495, 0.0, -0.3):                    # 옆면 전체 / 절반 / 조금만 보임
            c = box_center(husky_faces(1.6, -0.8, yaw, side_upto=upto), yaw, (0.495, 0.335))
            assert math.hypot(c[0] - 1.6, c[1] + 0.8) < 0.03, (yaw, upto, c)
    c = box_center(husky_faces(0.2, -0.9, 0.0, rear=False), 0.0, (0.495, 0.335))      # 옆에 나란히: 옆면만 보임
    assert math.hypot(c[0] - 0.2, c[1] + 0.9) < 0.03


def test_find_body_with_yaw_is_accurate_where_line_of_sight_guess_is_not():
    pts = husky_faces(1.6, -0.8, 0.0)                      # 추종 자리에서 본 모습: 뒷면 + 옆면 전체
    los = find_body(pts, (1.6, -0.8))
    box = find_body(pts, (1.6, -0.8), yaw=0.0)
    assert box is not None and math.hypot(box[0] - 1.6, box[1] + 0.8) < 0.03
    assert los is None or math.hypot(los[0] - 1.6, los[1] + 0.8) > 0.1          # 시선 방향 어림은 0.1 m 넘게 틀림
