"""tools/costmap_diag.py: 계획 실패 순간 전역 비용지도 판정."""
import os
import sys

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'tools'))
from costmap_diag import diag, text  # noqa: E402

RES = 0.05


def room():
    """4 x 4 m 방 (벽 254), 오른쪽 벽에 1 m 문, 문 밖 2 m 복도. 원점 (0, 0)."""
    c = np.zeros((80, 120), np.uint8)
    c[0, :80] = c[-1, :80] = 254
    c[:, 0] = 254
    c[:, 79] = 254
    c[30:50, 79] = 0           # 문 y 1.5~2.5
    return c


def test_open_room_reaches_spawn_outside():
    o = diag(room(), RES, 0.0, 0.0, 2.0, 2.0, spawn=(5.0, 2.0))
    assert o['robot_cost'] == 0 and o['ring'] == 0.0
    assert o['spawn_reach'] is True and o['reach_m2'] > 15


def test_closed_door_isolates_room():
    c = room()
    c[30:50, 75:80] = 253      # 문이 내접 비용으로 닫힘
    o = diag(c, RES, 0.0, 0.0, 2.0, 2.0, spawn=(5.0, 2.0))
    assert o['spawn_reach'] is False and o['reach_m2'] < 16


def test_self_hit_ring_encloses_robot():
    c = room()
    yy, xx = np.mgrid[:80, :120]
    d = np.hypot(xx - 40, yy - 40) * RES
    c[(d >= 0.15) & (d <= 0.2)] = 254   # 로봇 둘레 0.15~0.2 m 고리 (몸에 맞은 라이다 점)
    o = diag(c, RES, 0.0, 0.0, 2.0, 2.0, spawn=(5.0, 2.0))
    assert o['ring'] > 0.2 and o['spawn_reach'] is False and o['reach_m2'] < 0.2
    assert '스폰 닿음 False' in text(o)


def test_unknown_is_passable_and_outside_reported():
    c = np.full((40, 40), 255, np.uint8)
    assert diag(c, RES, 0.0, 0.0, 1.0, 1.0, spawn=(0.2, 0.2))['spawn_reach'] is True
    assert diag(c, RES, 0.0, 0.0, 5.0, 5.0)['inside'] is False
