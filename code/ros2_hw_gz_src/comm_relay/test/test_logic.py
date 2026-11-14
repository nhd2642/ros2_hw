"""ROS 없이 도는 로직 시험: 구역 판정, 복귀 상태기계, cmd_vel 중재."""
import math

import pytest

from comm_relay.arbiter_logic import Arbiter
from comm_relay.return_logic import HOLD, NORMAL, RETURNING, STOPPING, WAIT, ReturnLogic, path_hits, scan_points


# ---------- 복귀 ----------
def walk(logic, pts, t0, dt=0.1, hb=True):
    t = t0
    prev = None
    for x, y in pts:
        yaw = 0.0 if prev is None else math.atan2(y - prev[1], x - prev[0])
        if hb:
            logic.on_heartbeat(t)
        logic.on_pose(x, y, yaw, t)
        logic.step(t)
        prev = (x, y)
        t += dt
    return t


def test_wait_until_first_heartbeat():
    lg = ReturnLogic()
    lg.on_pose(0, 0, 0, 0.0)
    for k in range(50):                            # 5 s 동안 하트비트 없음
        assert lg.step(k * 0.1) is None
    assert lg.state == WAIT
    lg.on_heartbeat(5.0)
    lg.step(5.0)
    assert lg.state == NORMAL


def test_return_retraces_and_stops_on_reconnect():
    lg = ReturnLogic()
    path = [(0, 0.02 * i) for i in range(150)]     # 0 -> 2.98 m 북쪽으로
    t = walk(lg, path, 0.0)
    assert lg.state == NORMAL and len(lg.crumbs) > 25
    # 하트비트 끊김
    x, y = path[-1]
    cmds = []
    for k in range(30):
        lg.on_pose(x, y, math.pi / 2, t)
        c = lg.step(t)
        cmds.append(c)
        t += 0.1
    assert lg.state == RETURNING
    # 뒤를 향하므로 먼저 제자리 회전 명령
    assert any(c and c[0] == 0.0 and abs(c[1]) > 0 for c in cmds)
    # 남쪽을 보고 걸어 내려오는 동안 전진 명령
    lg.on_pose(x, y - 0.3, -math.pi / 2, t)
    c = lg.step(t)
    assert c[0] > 0
    # 통신 복구 -> 정지 명령 stop_duration 동안, 그 뒤 NORMAL 이고 명령 없음
    lg.on_heartbeat(t)
    assert lg.step(t) == (0.0, 0.0) and lg.state == STOPPING
    t2 = t + 0.5
    lg.on_heartbeat(t2)
    lg.on_pose(x, y - 0.3, -math.pi / 2, t2)
    assert lg.step(t2) == (0.0, 0.0)
    t3 = t + 1.1
    lg.on_heartbeat(t3)
    lg.on_pose(x, y - 0.3, -math.pi / 2, t3)
    assert lg.step(t3) is None and lg.state == NORMAL


def test_hold_at_home_when_still_disconnected():
    lg = ReturnLogic()
    t = walk(lg, [(0.0, 0.05 * i) for i in range(10)], 0.0)
    # 단절 상태로 지나온 길을 따라 홈까지 내려옴
    t += 1.5
    for k in range(30):
        y = max(0.0, 0.45 - 0.03 * k)
        lg.on_pose(0.0, y, -math.pi / 2, t)
        c = lg.step(t)
        t += 0.1
    assert lg.state == HOLD and c == (0.0, 0.0)


def test_stale_odom_means_stand_still():
    lg = ReturnLogic()
    t = walk(lg, [(0.0, 0.05 * i) for i in range(20)], 0.0)
    c = lg.step(t + 2.0)                           # odom 2 s 없음 + 하트비트 없음
    assert lg.state == RETURNING and c == (0.0, 0.0)


def test_nav_mode_and_fallback():
    lg = ReturnLogic(return_drive='auto')
    t = walk(lg, [(0.0, 0.05 * i) for i in range(30)], 0.0)
    lg.on_pose(0.0, 1.45, math.pi / 2, t + 1.5)
    assert lg.step(t + 1.5, nav_available=True) is None      # Nav2 가 복귀 주행
    assert lg.drive == 'nav'
    path = lg.return_path()
    assert path[-1] == lg.home and len(path) >= 2
    c = lg.step(t + 1.6, nav_available=True, nav_failed=True)  # Nav2 실패 -> 직접 주행
    assert lg.drive == 'direct' and c is not None


def test_odom_jump_breaks_breadcrumbs():
    lg = ReturnLogic()
    t = walk(lg, [(0.0, 0.05 * i) for i in range(20)], 0.0)
    lg.on_heartbeat(t)
    lg.on_pose(3.0, 3.0, 0.0, t)
    assert len(lg.crumbs) == 1


def test_arbiter_priority_timeout_and_idle_zero():
    a = Arbiter([('teleop', 3), ('return', 2), ('nav', 1)], timeout=0.5)
    assert a.select(0.0) == ('idle', (0.0, 0.0, 0.0))
    a.on_cmd('nav', 0.0, (0.1, 0, 0))
    assert a.select(0.1)[0] == 'nav'
    a.on_cmd('return', 0.2, (0.15, 0, 0.3))
    assert a.select(0.3)[0] == 'return'
    a.on_cmd('teleop', 0.3, (0.2, 0, 0))
    assert a.select(0.4)[0] == 'teleop'
    a.on_cmd('return', 0.6, (0.15, 0, 0.3))
    assert a.select(0.85)[0] == 'return'           # teleop 0.5 s 지남 -> 다음 우선순위
    assert a.select(1.2) == ('idle', (0.0, 0.0, 0.0))  # 모두 끊기면 0 명령 (CHAMP 마지막 명령 유지 방지)


# ---------- 복귀 주행 방식 / 길 위 장애물 ----------
def disconnected_far(lg, t):
    """북쪽으로 3 m 걸어온 뒤 단절, 로봇은 남쪽(되짚는 방향)을 봄."""
    t = walk(lg, [(0.0, 0.05 * i) for i in range(60)], t)
    lg.on_pose(0.0, 2.95, -math.pi / 2, t + 1.5)
    return t + 1.5


def test_direct_is_default_even_with_nav():
    lg = ReturnLogic()
    t = disconnected_far(lg, 0.0)
    c = lg.step(t, nav_available=True)
    assert lg.drive == 'direct' and c is not None and c[0] > 0


def test_auto_uses_nav():
    lg = ReturnLogic(return_drive='auto')
    t = disconnected_far(lg, 0.0)
    assert lg.step(t, nav_available=True) is None and lg.drive == 'nav'


def test_blocked_stops_forward_and_resumes():
    lg = ReturnLogic()
    t = disconnected_far(lg, 0.0)
    assert lg.step(t, blocked=lambda: True) == (0.0, 0.0)
    assert any('장애물' in e for _, e in lg.events)
    lg.on_pose(0.0, 2.95, -math.pi / 2, t + 1)
    c = lg.step(t + 1, blocked=lambda: False)
    assert c[0] > 0 and lg.blocked_since is None


def test_rotation_not_blocked():
    lg = ReturnLogic()
    t = walk(lg, [(0.0, 0.05 * i) for i in range(60)], 0.0)
    lg.on_pose(0.0, 2.95, math.pi / 2, t + 1.5)        # 북쪽을 봄 -> 먼저 뒤돌기
    c = lg.step(t + 1.5, blocked=lambda: True)
    assert c[0] == 0.0 and abs(c[1]) > 0


def test_block_handover_to_nav_once():
    lg = ReturnLogic(block_handover=10.0)
    t = disconnected_far(lg, 0.0)
    for k in range(120):                               # 12 s 막힘
        lg.on_pose(0.0, 2.95, -math.pi / 2, t + 0.1 * k)
        c = lg.step(t + 0.1 * k, nav_available=True, blocked=lambda: True)
    assert lg.drive == 'nav' and c is None
    t2 = t + 12.1
    lg.on_pose(0.0, 2.95, -math.pi / 2, t2)
    lg.step(t2, nav_available=True, nav_failed=True)  # Nav2 실패 -> direct, 다시 넘기지 않음
    for k in range(150):
        lg.on_pose(0.0, 2.95, -math.pi / 2, t2 + 0.1 * (k + 1))
        c = lg.step(t2 + 0.1 * (k + 1), nav_available=True, blocked=lambda: True)
    assert lg.drive == 'direct' and c == (0.0, 0.0)


def test_no_handover_by_default():
    # 기본(block_handover 0): Nav2 가 살아 있어도 넘기지 않고 계속 기다린다
    lg = ReturnLogic()
    t = walk(lg, [(0.0, 0.1 * i) for i in range(11)], 0.0)
    lg.on_heartbeat(t - 5.0)
    lg.step(t, nav_available=True)
    for k in range(300):
        lg.on_pose(0.0, 1.0, -math.pi / 2, t + 0.1 * k)
        out = lg.step(t + 0.1 * k, nav_available=True, blocked=lambda: True)
        assert out == (0.0, 0.0)
    assert lg.drive == 'direct'


def test_no_handover_without_nav():
    lg = ReturnLogic()
    t = disconnected_far(lg, 0.0)
    for k in range(150):
        lg.on_pose(0.0, 2.95, -math.pi / 2, t + 0.1 * k)
        c = lg.step(t + 0.1 * k, nav_available=False, blocked=lambda: True)
    assert lg.drive == 'direct' and c == (0.0, 0.0)


def scan_with(points, n=360):
    """로봇 기준 (x, y) 점들만 보이는 360 개 레이 스캔 (나머지는 inf)."""
    r = [float('inf')] * n
    inc = 2 * math.pi / n
    for x, y in points:
        i = int(round((math.atan2(y, x) + math.pi) / inc)) % n
        r[i] = min(r[i], math.hypot(x, y))
    return r, -math.pi, inc


STRAIGHT = [(0.0, 0.0), (0.5, 0.0)]


def test_path_hits_straight():
    assert path_hits([(0.3, y / 100) for y in range(-5, 6)], STRAIGHT, 0.15) >= 3      # 정면 상자
    assert path_hits([(x / 100, 0.25) for x in range(-50, 60, 2)], STRAIGHT, 0.15) == 0   # 옆 벽 0.25 m
    assert path_hits([(0.75, 0.0)], STRAIGHT, 0.15) == 0                               # 0.5 m + 반폭 밖
    assert path_hits([(-0.2, 0.0), (0.02, 0.1)], STRAIGHT, 0.15) == 0                  # 뒤/옆은 안 셈


def test_path_hits_follows_bend():
    # 지나온 길이 0.2 m 앞에서 왼쪽으로 꺾임 (L 자). 꺾인 뒤 길 위 상자는 잡고,
    # 꺾지 않고 곧장 갔으면 있었을 정면 벽(x 0.45)은 안 잡는다 (직선 띠였으면 잡았을 오검출)
    bend = [(0.0, 0.0), (0.2, 0.0), (0.2, 0.3)]
    assert path_hits([(0.2 + d / 100, 0.25) for d in range(-5, 6)], bend, 0.15) >= 3
    assert path_hits([(0.45, y / 100) for y in range(-10, 11, 2)], bend, 0.15) == 0
    assert path_hits([(0.45, y / 100) for y in range(-10, 11, 2)], STRAIGHT, 0.15) >= 3


def test_scan_points_to_odom():
    r, a0, inc = scan_with([(0.3, 0.0)])
    pts = scan_points(r, a0, inc, 0.12, 10.0, 1.0, 2.0, math.pi / 2, 1.0)    # 북쪽을 보는 로봇 앞 0.3 m
    assert len(pts) == 1 and abs(pts[0][0] - 1.0) < 0.01 and abs(pts[0][1] - 2.3) < 0.01
    assert scan_points(r, a0, inc, 0.12, 10.0, 0, 0, 0, 0.2) == []               # max_r 밖은 버림


def test_scan_points_drop_self_and_min_range():
    # Gazebo 에서 걸을 때 나온 것: 방위 +-35°, 0.15 m (= range_min) -> 버림. 몸 상자 안 0.2 m 점도 버림
    r, a0, inc = scan_with([(0.15 * math.cos(math.radians(35)), 0.15 * math.sin(math.radians(35))),
                            (0.20, -0.05), (0.40, 0.0), (0.0, 0.30)])
    pts = scan_points(r, a0, inc, 0.15, 10.0, 0.0, 0.0, 0.0, 1.0)
    assert len(pts) == 2                                                          # 정면 0.4 m, 왼쪽 0.3 m 만
    assert any(abs(px - 0.40) < 0.01 for px, py in pts) and any(abs(py - 0.30) < 0.01 for px, py in pts)
    # 몸 상자 끄면 0.2 m 점은 살아남고, range_min 값은 그래도 버림
    assert len(scan_points(r, a0, inc, 0.15, 10.0, 0.0, 0.0, 0.0, 1.0, self_box=None)) == 3


def test_upcoming_is_retrace_polyline():
    lg = ReturnLogic()
    t = walk(lg, [(0.0, 0.05 * i) for i in range(41)] + [(0.05 * i, 2.0) for i in range(1, 21)], 0.0)
    lg.on_pose(1.0, 2.0, math.pi, t)                    # 동쪽 끝, 서쪽을 봄
    up = lg.upcoming(0.5)
    L = sum(math.hypot(b[0] - a[0], b[1] - a[1]) for a, b in zip(up, up[1:]))
    assert up[0] == (1.0, 2.0) and abs(L - 0.5) < 1e-6
    assert all(abs(y - 2.0) < 1e-6 and x < 1.0 + 1e-9 for x, y in up)   # 지나온 길(y=2) 위로 서쪽


def test_no_record_means_hold_in_place_and_home_is_explore_start():
    """탐색 단계가 아니면(record False) 길을 쌓지 않아 끊겨도 그 자리에 서고, 탐색 시작 뒤 끊기면 시작 자리로 돌아온다."""
    r = ReturnLogic(comm_timeout=1.0)
    r.record = False
    r.on_heartbeat(0.0)
    for i in range(30):                      # 이동 단계: 3 m 걸음
        r.on_pose(0.1 * i, 0.0, 0.0, 0.1 * i)
        r.on_heartbeat(0.1 * i)
        r.step(0.1 * i)
    assert len(r.crumbs) == 0 and abs(r.home[0] - 2.9) < 1e-6
    r.record = True                          # 탐색 시작: 홈 = (2.9, 0)
    for i in range(30, 50):
        r.on_pose(0.1 * i, 0.0, 0.0, 0.1 * i)
        r.on_heartbeat(0.1 * i)
        r.step(0.1 * i)
    assert abs(r.home[0] - 2.9) < 1e-6 and len(r.crumbs) > 10
    r.on_pose(4.9, 0.0, 0.0, 7.0)
    assert r.step(7.0) is not None and r.state == RETURNING     # 하트비트 2 s 없음
    assert r.target()[0] <= 4.9
