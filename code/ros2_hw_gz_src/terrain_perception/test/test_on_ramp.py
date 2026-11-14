"""경사로 위에서 몸이 기운 채(오르막 -10°) 보는 경우 — 2026-11-08 Gazebo 오검출 재현 시험.
노드 계산 그대로: 점을 몸 기준으로 바꾼 뒤 IMU pitch 로 돌리고, 높이는 EKF 처럼 2D (경사로 높이 모름)."""
import math

import numpy as np

from terrain_perception.grid import HAZARD, classify_frame
from terrain_perception.terrain_node import R_rpy
from test_grid import ramp_h

CAM_T, PITCH, BASE_Z = np.array([0.16, 0.0, 0.06]), math.radians(40), 0.21


def raycast_R(hfun, cam, R, hfov=1.518, w=160, h=120, max_t=2.0, step=0.005):
    fx = (w / 2) / math.tan(hfov / 2)
    u, v = np.meshgrid(np.arange(w) - w / 2 + 0.5, np.arange(h) - h / 2 + 0.5)
    d = np.stack([np.ones_like(u) * fx, -u, -v], -1).reshape(-1, 3)
    d /= np.linalg.norm(d, axis=1, keepdims=True)
    d = d @ R.T
    hit = np.full(len(d), np.nan)
    for t in np.arange(0.1, max_t, step):
        p = cam + t * d
        hit[(p[:, 2] <= hfun(p[:, 0], p[:, 1])) & np.isnan(hit)] = t
        if not np.isnan(hit).any():
            break
    ok = ~np.isnan(hit)
    return cam + hit[ok, None] * d[ok]


def node_view(x0, y0, yaw, body_pitch_deg, body_roll_deg=0.0, imu_err_deg=0.0):
    g = ramp_h(np.array([x0]), np.array([y0]))[0]
    Rb = R_rpy(math.radians(body_roll_deg), math.radians(body_pitch_deg), yaw)
    base = np.array([x0, y0, g + BASE_Z])
    pw = raycast_R(ramp_h, base + Rb @ CAM_T, Rb @ R_rpy(0, PITCH, 0))
    pb = (pw - base) @ Rb
    Rn = R_rpy(math.radians(body_roll_deg), math.radians(body_pitch_deg + imu_err_deg), yaw)
    off = np.array([x0, y0, BASE_Z])
    c = Rn @ CAM_T + off
    ij, st = classify_frame(pb @ Rn.T + off, cam_xy=c[:2], cam_z=c[2])
    return ((ij + 0.5) * 0.05)[st == HAZARD]


def test_climbing_no_surface_hazard():
    for x0, bp in ((0.7, -5), (0.9, -10), (1.2, -10), (1.4, -10)):
        for err in (0, 3, -3):
            hz = node_view(x0, 0.0, 0.0, bp, imu_err_deg=err)
            surf = hz[(np.abs(hz[:, 1]) < 0.6) & (hz[:, 0] > 0.65) & (hz[:, 0] < 2.55)]
            assert len(surf) == 0, (x0, bp, err, np.round(surf[:4], 2).tolist())


def test_on_ramp_turned_to_side_edge():
    # 경사로 위 (1.1, 0.45) 에서 +y 를 봄: 몸은 옆으로 10° 기움(roll). 가장자리 위험, 안쪽 표면 없음
    hz = node_view(1.1, 0.45, math.pi / 2, 0.0, body_roll_deg=10.0)
    assert ((hz[:, 1] > 0.85) & (hz[:, 1] < 1.05)).sum() >= 3
    assert ((hz[:, 1] > 0.55) & (hz[:, 1] < 0.85)).sum() == 0
