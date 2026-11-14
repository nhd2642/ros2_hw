"""위험 판정 시험. 점은 카메라에서 광선을 쏴서 만든다(가림 포함) — 1차 시험은 모든 점을
다 넣어서 가림을 빼먹었고, Gazebo 에서 경사로 옆 가장자리를 못 잡았다 (2026-11-08)."""
import math

import numpy as np

from terrain_perception.grid import FREE, HAZARD, RollingState, classify_frame, hazards

TAN10 = math.tan(math.radians(10))


def ramp_h(x, y):
    """disaster.world 남쪽 경사로를 x 방향으로: 폭 1.9 m (|y| <= 0.95), x 0.6~1.6 오르막(0 -> 0.176),
    1.6~2.6 내리막. 바닥 0."""
    on = np.abs(y) <= 0.95
    up = on & (x >= 0.6) & (x < 1.6)
    dn = on & (x >= 1.6) & (x <= 2.6)
    return np.where(up, TAN10 * (x - 0.6), np.where(dn, TAN10 * (2.6 - x), 0.0))


def box_h(x, y):
    return np.where((np.abs(x - 1.0) < 0.15) & (np.abs(y) < 0.15), 0.10, 0.0)


def raycast(hfun, cam, yaw, pitch=math.radians(40), hfov=1.518, w=160, h=120, max_t=2.5, step=0.005):
    """핀홀 카메라(앞 x, 아래로 pitch)에서 높이장으로 광선. 반환 (N, 3) 맞은 점."""
    fx = (w / 2) / math.tan(hfov / 2)
    u, v = np.meshgrid(np.arange(w) - w / 2 + 0.5, np.arange(h) - h / 2 + 0.5)
    d = np.stack([np.ones_like(u) * fx, -u, -v], -1).reshape(-1, 3)            # 몸 기준: x 앞, y 왼쪽, z 위
    d /= np.linalg.norm(d, axis=1, keepdims=True)
    cp, sp, cy, sy = math.cos(pitch), math.sin(pitch), math.cos(yaw), math.sin(yaw)
    Rp = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])                     # 앞을 아래로
    Rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    d = d @ (Rz @ Rp).T
    hit = np.full(len(d), np.nan)
    prev_above = np.ones(len(d), bool)
    for t in np.arange(0.1, max_t, step):
        p = cam + t * d
        below = p[:, 2] <= hfun(p[:, 0], p[:, 1])
        new = below & np.isnan(hit)
        hit[new] = t
        if not np.isnan(hit).any():
            break
    ok = ~np.isnan(hit)
    return cam + hit[ok, None] * d[ok]


def classify(pts, cam, robot_xy=None):
    ij, st = classify_frame(pts, cam_xy=cam[:2], cam_z=cam[2])
    c = (ij + 0.5) * 0.05
    return c[st == HAZARD], c[st == FREE]


def count(xy, pred):
    return int(pred(xy[:, 0], xy[:, 1]).sum()) if len(xy) else 0


def test_flat_floor_no_hazard():
    cam = np.array([0.18, 0.0, 0.27])
    hz, fr = classify(raycast(lambda x, y: 0 * x, cam, 0.0), cam, (0.0, 0.0))
    assert len(hz) == 0 and len(fr) > 300


def test_ramp_from_centerline_surface_and_far_crest_not_hazard():
    # 경사로 정면 가운데서: 표면(10°)·먼 꼭대기는 위험 아님 (먼 오르막의 성긴 표본, 꼭대기 너머 그늘 무시)
    cam = np.array([0.18, 0.0, 0.27])
    hz, fr = classify(raycast(ramp_h, cam, 0.0), cam, (0.0, 0.0))
    assert count(hz, lambda x, y: (np.abs(y) < 0.7) & (x > 0.5) & (x < 2.6)) == 0
    assert count(fr, lambda x, y: (np.abs(y) < 0.7) & (x > 0.7) & (x < 1.5)) > 50


def test_on_ramp_facing_side_edge_sees_drop():
    # 2026-11-07 넘어진 상황: 경사로 위(x 1.1, 높이 0.09)에서 옆(+y) 가장자리를 봄. 가장자리 바깥 바닥은
    # 일부 가려지지만 (4) 가림 너머 낙차로 가장자리를 위험으로
    x0 = 1.1
    cam = np.array([x0, 0.45 + 0.16, ramp_h(np.array([x0]), np.array([0.45]))[0] + 0.27])
    pts = raycast(ramp_h, cam, math.pi / 2)
    hz, _ = classify(pts, cam, (x0, 0.45))
    assert count(hz, lambda x, y: (y > 0.85) & (y < 1.05) & (x > 0.8) & (x < 1.6)) >= 5
    assert count(hz, lambda x, y: (y > 0.6) & (y < 0.85)) == 0                 # 가장자리 안쪽 표면은 아님


def test_side_edge_hidden_floor_without_rule4_is_missed():
    # 대조: (4) 를 끄면(bridge 한 칸) 같은 장면에서 가장자리 위험이 줄어든다 (1차 구현과 같은 판정)
    x0 = 1.1
    cam = np.array([x0, 0.61, ramp_h(np.array([x0]), np.array([0.45]))[0] + 0.27])
    pts = raycast(ramp_h, cam, math.pi / 2)
    edge = lambda x, y: (y > 0.85) & (y < 1.05)  # noqa: E731
    ij, st = classify_frame(pts, cam_xy=cam[:2], cam_z=cam[2])
    ij0, st0 = classify_frame(pts, cam_xy=cam[:2], cam_z=cam[2], bridge=0.05)
    c, c0 = (ij + 0.5) * 0.05, (ij0 + 0.5) * 0.05
    assert count(c[st == HAZARD], edge) > count(c0[st0 == HAZARD], edge)


def test_from_floor_side_sees_ramp_face():
    # 바닥(옆)에서 경사로 옆면을 봄: 보이는 수직면 -> 위험
    cam = np.array([1.3, 1.6, 0.27])
    hz, _ = classify(raycast(ramp_h, cam, -math.pi / 2), cam, (1.3, 1.76))
    assert count(hz, lambda x, y: (y > 0.85) & (y < 1.05) & (x > 0.9) & (x < 1.8)) >= 5


def test_box_and_low_bump():
    cam = np.array([0.18, 0.0, 0.27])
    hz, _ = classify(raycast(box_h, cam, 0.0), cam, (0.0, 0.0))
    assert count(hz, lambda x, y: (np.abs(x - 1.0) < 0.25) & (np.abs(y) < 0.25)) >= 3
    assert count(hz, lambda x, y: (x < 0.7) | (x > 1.6)) == 0
    bump = lambda x, y: np.where((np.abs(x - 1.0) < 0.15) & (np.abs(y) < 0.3), 0.03, 0.0)  # noqa: E731
    hz2, _ = classify(raycast(bump, cam, 0.0), cam, (0.0, 0.0))
    assert len(hz2) == 0                                                       # 3 cm < swing_height 4 cm


def test_memory_moves_and_forgets():
    cam = np.array([0.18, 0.0, 0.27])
    ij, st = classify_frame(raycast(box_h, cam, 0.0), cam_xy=cam[:2], cam_z=cam[2])
    rs = RollingState(cell=0.05, size=4.0, forget=10.0)
    rs.update(ij, st, 0.0, 0.0, 0.0)
    hz0 = rs.hazards_xy(1.0, 0.0, 0.0, 3.0)
    rs.update(np.zeros((0, 2), np.int64), np.zeros(0, np.int8), 1.0, 1.2, 0.0)   # 로봇 1.2 m 이동 -> 격자 이동
    hz1 = rs.hazards_xy(2.0, 0.0, 0.0, 3.0)
    assert len(hz0) >= 3 and set(map(tuple, np.round(hz1, 3))) == set(map(tuple, np.round(hz0, 3)))
    assert len(rs.hazards_xy(20.0, 0.0, 0.0, 3.0)) == 0
    # 같은 칸을 나중에 빈 칸으로 보면 위험이 지워짐 (상자 치움)
    rs.update(ij, np.full(len(st), FREE, np.int8), 3.0, 0.0, 0.0)
    assert len(rs.hazards_xy(3.5, 0.0, 0.0, 3.0)) == 0


def wall_box_h(x, y):
    """x 1.2~1.3 높이 1.0 벽 (카메라는 밑동만 봄) + 벽에서 0.5 m 앞 낮은 상자 (x 0.6~0.7, |y|<0.15, 0.10 m)."""
    wall = (x >= 1.2) & (x <= 1.3)
    box = (x >= 0.6) & (x <= 0.7) & (np.abs(y) < 0.15)
    return np.where(wall, 1.0, np.where(box, 0.10, 0.0))


def _wall_scene(lidar_t=0.0):
    cam = np.array([0.18, 0.0, 0.27])
    ij, st = classify_frame(raycast(wall_box_h, cam, 0.0), cam_xy=cam[:2], cam_z=cam[2])
    rs = RollingState(cell=0.05, size=6.0, forget=10.0)
    rs.update(ij, st, 0.0, 0.0, 0.0)
    ys = np.arange(-1.0, 1.0, 0.02)                     # 라이다(0.3 m)는 벽 앞면만 맞힘, 상자(0.10 m)는 못 봄
    rs.mark_lidar(np.stack([np.full_like(ys, 1.2), ys], 1), lidar_t, 0.0, 0.0)
    return rs


def test_lidar_suppresses_wall_base_keeps_low_box():
    # 2026-11-09 record5: 지형 위험 85~100% 가 벽 0.3 m 안 -> 로봇 둘레 치명으로 갇힘
    rs = _wall_scene()
    wall = lambda x, y: x > 0.95                         # noqa: E731
    box = lambda x, y: (x > 0.45) & (x < 0.85) & (np.abs(y) < 0.3)  # noqa: E731
    raw = rs.hazards_xy(1.0, 0.0, 0.0, 3.0)
    assert count(raw, wall) >= 5 and count(raw, box) >= 3            # 억제 전: 벽 밑동도 위험
    out = rs.hazards_xy(1.0, 0.0, 0.0, 3.0, lidar_clear=0.25)
    assert count(out, wall) == 0 and rs.suppressed >= 5
    assert count(out, box) == count(raw, box)                         # 라이다가 못 보는 낮은 상자는 그대로


def test_lidar_suppression_forgets_and_follows_grid_shift():
    rs = _wall_scene(lidar_t=0.0)
    wall = lambda x, y: x > 0.95                         # noqa: E731
    rs.update(np.zeros((0, 2), np.int64), np.zeros(0, np.int8), 1.0, 1.4, 0.0)   # 격자 이동
    assert count(rs.hazards_xy(1.5, 0.0, 0.0, 3.0, lidar_clear=0.25), wall) == 0
    rs2 = _wall_scene(lidar_t=-9.5)                     # 라이다 기록이 잊힐 만큼 오래됨 (위험은 t=0)
    assert count(rs2.hazards_xy(1.0, 0.0, 0.0, 3.0, lidar_clear=0.25), wall) >= 5


def test_pad_hazards_widens_band_and_keeps_zero_pad():
    from terrain_perception.grid import pad_hazards
    one = np.array([[1.025, 2.025]])
    assert np.allclose(pad_hazards(one, 0.0), one)
    out = pad_hazards(one, 0.15)
    d = np.hypot(out[:, 0] - 1.025, out[:, 1] - 2.025)
    assert len(out) == 9 and abs(d.max() - 0.15) < 0.04
    two = pad_hazards(np.array([[0.025, 0.025], [0.075, 0.025]]), 0.05)
    assert len(two) == len(np.unique(two, axis=0))
    assert len(pad_hazards(np.zeros((0, 2)), 0.15)) == 0
