"""a1_exploration/scripts/shared_map_merger.py: 기준 좌표 회전 (R1 방향 보정) 계산."""
import math
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..', 'a1_exploration', 'scripts'))
from shared_map_merger import anchor_rotated, heading_angle, rotate  # noqa: E402


def shared(anchor, p):
    x, y = rotate(p[0], p[1], anchor[2])
    return x + anchor[0], y + anchor[1]


def test_anchor_rotated_keeps_robot_point():
    a = (10.0, -2.0, 0.0)
    robot = (40.0, 3.0)                       # 로봇 map 위치
    b = anchor_rotated(a, robot, math.radians(2.0))
    assert all(math.isclose(u, v, abs_tol=1e-9) for u, v in zip(shared(a, robot), shared(b, robot)))
    assert math.isclose(b[2], math.radians(2.0))
    far = (robot[0] + 17.0, robot[1])         # 17 m 떨어진 점은 2° 만큼 돌아 약 0.59 m 옮겨짐
    moved = math.dist(shared(a, far), shared(b, far))
    assert math.isclose(moved, 17.0 * 2 * math.sin(math.radians(1.0)), rel_tol=1e-6)


def test_heading_angle_mean_and_sign():
    s = [(0.2 * i, math.radians(1.5), 0.0) for i in range(10)]          # map->odom +1.5°, 안 돔
    assert math.isclose(heading_angle(s), math.radians(-1.5), abs_tol=1e-9)


def test_heading_angle_skips_turning_samples():
    s = [(0.2 * i, math.radians(1.0), 0.0) for i in range(6)]           # 안 돌 때 +1.0°
    s += [(1.2 + 0.2 * i, math.radians(5.0), 0.1 * (i + 1)) for i in range(6)]   # 0.5 rad/s 로 돌 때 +5°
    assert math.isclose(heading_angle(s), math.radians(-1.0), abs_tol=1e-9)


def test_heading_angle_none_when_too_few():
    assert heading_angle([(0.0, 0.0, 0.0), (0.2, 0.0, 0.0)]) is None
    assert heading_angle([]) is None


def test_heading_angle_wraps_near_pi():
    s = [(0.2 * i, math.pi - 0.01 if i % 2 else -math.pi + 0.01, 0.0) for i in range(10)]
    assert math.isclose(abs(heading_angle(s)), math.pi, abs_tol=0.02)
