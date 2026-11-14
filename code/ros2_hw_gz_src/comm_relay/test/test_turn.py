"""nav_executor 먼저 돌기 판단 (경로 방향, 각도 차). rclpy 가 있어야 import 됨."""
import math

import pytest

pytest.importorskip('rclpy')
from geometry_msgs.msg import PoseStamped  # noqa: E402
from nav_msgs.msg import Path  # noqa: E402

from comm_relay.nav_executor import angle_diff, path_heading  # noqa: E402


def mk(pts):
    p = Path()
    for x, y in pts:
        q = PoseStamped()
        q.pose.position.x, q.pose.position.y = float(x), float(y)
        p.poses.append(q)
    return p


def test_path_heading_samples_ahead():
    # 처음엔 북쪽 0.1 m 꺾였다가 서쪽으로: 0.5 m 지점 방향 (대략 서북서)
    h = path_heading(mk([(0, 0), (0, 0.1)] + [(-0.1 * i, 0.1) for i in range(1, 10)]))
    assert math.degrees(h) == pytest.approx(180 - math.degrees(math.atan2(0.1, 0.5)), abs=1)


def test_path_heading_short_and_degenerate():
    assert path_heading(mk([(0, 0), (0.2, 0)])) == pytest.approx(0.0)      # 0.5 m 보다 짧으면 끝 점
    assert path_heading(mk([(0, 0), (0.01, 0)])) is None
    assert path_heading(mk([(0, 0)])) is None


def test_angle_diff_wraps():
    assert angle_diff(math.radians(170), math.radians(-170)) == pytest.approx(math.radians(-20))
    assert abs(angle_diff(math.pi, 0.0)) == pytest.approx(math.pi)
