"""목표점까지 2D 라이다로 장애물을 피해 가는 조향 (ROS 없음, 단위 시험 대상). Husky 주행과 A1 추종이 같이 쓴다.

방법: 목표 방위에서 가까운 순서로 후보 방향을 훑어, 몸 폭 띠(half) 안에 라이다 점이 need 거리까지 없는 첫 방향으로 간다.
  - 고른 방향이 몸 방향과 turn_first 넘게 다르면 제자리 회전 (A1 은 옆걸음이 거의 안 나가고 Husky 는 스키드 조향).
  - 속도는 고른 방향의 트인 거리와 남은 거리에 비례해 줄인다. 트인 방향이 없으면 선다 (0, 0).
  - 좌우 후보가 비슷할 때 흔들리지 않게 지난번 고른 쪽을 먼저 본다.
좌표: 로봇 몸 기준 (x 앞, y 왼쪽). 라이다가 몸 중심보다 lidar_x 앞에 있으면 그만큼 옮긴다.
"""
import math

import numpy as np


def scan_xy(ranges, angle_min, inc, rmin, rmax, lidar_x=0.0, body=(0.0, 0.0, 0.0)):
    """LaserScan -> 몸 기준 점 (N, 2). 범위 밖 값과 몸 상자 body = (앞, 뒤, 반폭) 안 점(자기 몸)은 버린다."""
    r = np.asarray(ranges, dtype=float)
    a = angle_min + inc * np.arange(len(r))
    ok = np.isfinite(r) & (r > rmin + 0.01) & (r < rmax)
    x, y = r[ok] * np.cos(a[ok]) + lidar_x, r[ok] * np.sin(a[ok])
    front, back, side = body
    keep = ~((x < front) & (x > -back) & (np.abs(y) < side))
    return np.stack([x[keep], y[keep]], axis=1)


def free_along(pts, heading, half):
    """heading 방향 폭 2*half 띠에서 가장 가까운 점까지 거리 (없으면 inf)."""
    if len(pts) == 0:
        return math.inf
    c, s = math.cos(heading), math.sin(heading)
    along = pts[:, 0] * c + pts[:, 1] * s
    m = (along > 0.0) & (np.abs(pts[:, 1] * c - pts[:, 0] * s) < half)
    return float(along[m].min()) if m.any() else math.inf


def box_center(c, yaw, half):
    """방향 yaw, 반크기 half = (반길이, 반폭) 인 상자의 보이는 면에 찍힌 점들 c (N, 2) -> 상자 중심 (x, y). 관측자는 원점.

    상자 축마다 따로: 점들이 그 축으로 1.2 * 반크기 넘게 퍼져 있으면 그 방향 면 전체가 보이는 것이므로 양 끝의 가운데,
    아니면 보이는 것은 관측자 쪽 면 하나이므로 그 면(관측자에게 가까운 끝)에서 반크기만큼 더 먼 곳.
    """
    co, si = math.cos(yaw), math.sin(yaw)
    out = []
    for a, h in ((c[:, 0] * co + c[:, 1] * si, half[0]), (-c[:, 0] * si + c[:, 1] * co, half[1])):
        if np.ptp(a) >= 1.2 * h:
            out.append((a.min() + a.max()) / 2)
        else:
            out.append(a.min() + h if a.mean() > 0 else a.max() - h)
    return out[0] * co - out[1] * si, out[0] * si + out[1] * co


def find_body(pts, guess, yaw=None, half=(0.495, 0.335), radius=1.3, link=0.25, max_extent=1.5, min_points=4, depth=0.4, gate=0.7):
    """guess (몸 기준 x, y) 근처에서 다른 로봇 몸(Husky)으로 보이는 라이다 점 묶음을 찾아 그 중심을 추정한다. 없으면 None.

    guess 에서 radius 안 점들을 link 거리로 이어 묶고, 크기가 max_extent 이하·점 min_points 이상인 묶음마다 중심을 추정해
    guess 에서 gate 안인 것 중 가장 가까운 것. gate: 출발점의 작은 벽(1 m)을 Husky 로 잡아 a1_1 이 안 떠났음 (추정 중심이 0.9 m 어긋남).
    중심 추정: 그 로봇의 방향 yaw (내 몸 기준) 를 알면 box_center (Husky 0.99 x 0.67 m 상자 맞춤), 모르면 묶음 평균에서 시선 방향으로 depth.
      시선 방향 어림은 옆면이 많이 보일 때 0.5 m 까지 틀려, 세트 도착 때 자세 보정이 a1_1 을 0.47 m 잘못 옮겼다 (2026-11-14).
    왜: SLAM 자세는 도로 40 m 를 가며 1 m 까지 어긋났다 (2026-11-12). 추종은 지도 좌표가 아니라 눈앞의 Husky 를 기준으로 해야 한다.
    """
    if len(pts) == 0:
        return None
    near = pts[np.hypot(pts[:, 0] - guess[0], pts[:, 1] - guess[1]) < radius]
    if len(near) < min_points:
        return None
    near = near[np.argsort(np.arctan2(near[:, 1], near[:, 0]))]            # 방위 순서 = 스캔 순서
    cuts = np.flatnonzero(np.hypot(*np.diff(near, axis=0).T) > link) + 1
    best = None
    for c in np.split(near, cuts):
        if len(c) < min_points or np.ptp(c, axis=0).max() > max_extent:
            continue
        if yaw is not None:
            est = box_center(c, yaw, half)
        else:
            m = c.mean(axis=0)
            r = math.hypot(m[0], m[1])
            if r < 1e-6:
                continue
            est = (m[0] * (1 + depth / r), m[1] * (1 + depth / r))
        d = math.hypot(est[0] - guess[0], est[1] - guess[1])
        if d <= gate and (best is None or d < best[0]):
            best = (d, est)
    return best[1] if best else None


class Steer:
    def __init__(self, v_max=0.2, w_max=0.5, half=0.3, front=0.3, look=1.5, tol=0.25, turn_first=0.6,
                 k_w=1.2, sweep=1.75, step=0.175, v_min=0.0):
        self.v_max, self.w_max, self.half, self.front = v_max, w_max, half, front
        self.look, self.tol, self.turn_first, self.k_w, self.v_min = look, tol, turn_first, k_w, v_min
        self.offsets = [k * step for k in range(int(sweep / step) + 1)]
        self.side = 1.0            # 지난번 돌아간 쪽 (+ 왼쪽)

    def cmd(self, pts, bearing, dist):
        """pts: 몸 기준 장애물 점, bearing/dist: 목표의 몸 기준 방위·거리. 돌려줌 (v, w, 상태 'ARRIVED'|'GO'|'AVOID'|'BLOCKED')."""
        if dist < self.tol:
            return 0.0, 0.0, 'ARRIVED'
        need = min(dist, self.look) + self.front
        for off in self.offsets:
            for sgn in ((self.side, -self.side) if off else (1.0,)):
                h = bearing + sgn * off
                free = free_along(pts, h, self.half)
                if free < need:
                    continue
                if off:
                    self.side = sgn
                h = math.atan2(math.sin(h), math.cos(h))
                w = max(-self.w_max, min(self.w_max, self.k_w * h))
                if abs(h) > self.turn_first:
                    return 0.0, math.copysign(self.w_max, h), 'AVOID' if off else 'GO'
                gap = free_along(pts, 0.0, self.half) - self.front     # 지금 몸 방향으로 남은 여유
                v = max(self.v_min, self.v_max * min(1.0, dist / 0.5, gap / 0.5)) if gap > 0 else 0.0
                return v, w, 'AVOID' if off else 'GO'
        return 0.0, 0.0, 'BLOCKED'
