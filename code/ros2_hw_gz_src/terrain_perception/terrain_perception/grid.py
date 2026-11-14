"""깊이 점들 -> 발 디딜 수 없는 칸(위험) / 디딜 수 있는 칸. ROS 의존 없음 (단위 시험용).

왜: 라이다 평면(지면 약 0.30 m)보다 낮은 것 — 경사로 옆 가장자리, 낮은 잔해, 턱 — 은 SLAM·Nav2 가 못 본다.
    (2026-11-07 a1_1 이 경사로 옆으로 내려가다 넘어짐). 특정 월드 표시 대신 센서로 판단한다.

판정은 한 프레임 안에서만 (classify_frame): 칸(cell) 높이 h = 그 칸 점들의 최대 z, ext = 칸 안 높이 폭.
  프레임끼리 높이를 합치지 않는 이유: EKF odom 은 2D 라 경사로에 오르면 로봇 높이가 0 으로 남고
  (경사로 위 0.1~0.18 m), 몸통 기울기도 IMU 보정 잔차가 남아서 다른 시각에 본 칸끼리 비교하면 가짜 턱이 생긴다.
  판정 결과(위험/빈 칸)만 RollingState 에 '가장 최근 관측 우선'으로 기억한다.
위험 (max_step 기본 0.04 m = CHAMP swing_height):
  (1) 칸 안 높이 폭 ext > max_step                       수직면, 턱 앞면
  (2) 8 이웃과 높이 차 > max_step                          턱, 보이는 낭떠러지 (양쪽 칸)
  (3) 2 칸 떨어진 칸과 기울기 > max_slope                  오를 수 없는 경사
  (4) 가림 너머 낙차: 카메라에서 멀어지는 방향(60° 안)으로, 바로 옆이 안 보이는 칸이고 bridge 거리 안에서
      처음 보이는 칸이 max_step 보다 낮으면 높은 쪽 칸을 위험으로. 위에서 보면 경사로 옆 가장자리 바로 바깥
      바닥은 가장자리에 가려 안 보인다. (1)~(3) 은 양쪽이 다 보여야 해서 못 잡음.
      카메라 쪽 방향은 보지 않는다: 그쪽 빈칸은 먼 거리의 성긴 표본이지 가림이 아니다.
      bridge(0.6 m) 보다 먼 그늘(멀리서 본 경사로 꼭대기 너머 내리막)은 판단하지 않고, 가까이 가서 보면 판정.
      카메라가 bridge_min_angle(25°) 넘게 내려다보는 칸만: 비스듬히(얕게) 보면 행 사이가 몇 칸씩 벌어져, 멀어지는
      쪽 완만한 내리막(경사로 꼭대기 너머 10°)의 행 사이 빈칸이 가림처럼 보임 (2026-11-08 Gazebo: 경사로 위에서
      몸이 10° 들리자 내리막 표면 39칸 오검출). 내려다보는 각이 내리막(10°)보다 충분히 크면 그런 빈칸이 안 생긴다.
      평지에서 카메라 0.27 m 면 약 0.55 m 안. 옆 가장자리는 그쪽으로 몸을 돌리면 0.3~0.5 m 앞(28~40°)이라 든다.
      20° 로 두면 경사로 위에서 비스듬히(15°) 오를 때 꼭대기 너머 내리막에 1~3칸 오검출 (시뮬 조합 108 개 중 10).
      한계: 가파르게 내려다보는 곳에서 그늘 너머 0.6 m 안에 보이는 내리막(평균 약 4° 넘게)은 위험으로 볼 수 있다(보수적).
경사로 가운데 정면에서는 옆 가장자리 바깥 바닥이 시야(87°)나 그늘에 들어 위험이 안 잡힐 수 있다 —
  그 방향으로 걷는 동안은 위험하지 않고, 가장자리 쪽으로 몸을 돌리면 (4) 로 잡힌다 (시험 test_on_ramp_*).
"""
import math

import numpy as np

DIRS = ((1, 0), (-1, 0), (0, 1), (0, -1), (1, 1), (1, -1), (-1, 1), (-1, -1))


def _shift(a, di, dj, fill=np.nan):
    """out[i, j] = a[i + di, j + dj] (범위 밖은 fill)."""
    out = np.full_like(a, fill)
    n, m = a.shape
    i0, i1 = max(0, -di), min(n, n - di)
    j0, j1 = max(0, -dj), min(m, m - dj)
    if i0 < i1 and j0 < j1:
        out[i0:i1, j0:j1] = a[i0 + di:i1 + di, j0 + dj:j1 + dj]
    return out


def hazards(H, ext, cell, max_step=0.04, max_slope_deg=25.0, bridge=0.6, bridge_min_angle=25.0, cam_ij=None, cam_z=None):
    """H: 칸 높이 (nan = 모름), ext: 칸 안 높이 폭, cam_ij: 카메라 위치 (배열 칸 좌표, 실수).
    반환 bool 배열 (위험). (4) 는 카메라에서 멀어지는 방향(각도 60° 안)으로만 본다: 가림은 높은 칸의
    뒤쪽에만 생긴다. 카메라 쪽으로 보이는 빈칸은 먼 거리의 성긴 표본이지 가림이 아니다
    (그쪽까지 보면 먼 오르막 경사로 표면이 가짜 위험이 됨)."""
    known = ~np.isnan(H)
    hz = known & (np.nan_to_num(ext) > max_step)
    tan_s = math.tan(math.radians(max_slope_deg))
    K = max(2, int(round(bridge / cell)))
    if cam_ij is not None:
        ii, jj = np.indices(H.shape)
        vi, vj = ii - cam_ij[0], jj - cam_ij[1]
        vn = np.hypot(vi, vj) + 1e-9
    with np.errstate(invalid='ignore'):
        for di, dj in DIRS:
            hz |= np.abs(H - _shift(H, di, dj)) > max_step                                       # (2)
            hz |= np.abs(H - _shift(H, 2 * di, 2 * dj)) > tan_s * cell * 2 * math.hypot(di, dj)   # (3)
            first = np.full_like(H, np.nan)                                                      # (4)
            for k in range(2, K + 1):
                first = np.where(np.isnan(first), _shift(H, k * di, k * dj), first)
            drop = _shift(~known, di, dj, fill=False) & ((H - first) > max_step)
            if cam_ij is not None:
                drop &= (vi * di + vj * dj) / (vn * math.hypot(di, dj)) > 0.5
                if cam_z is not None:
                    drop &= np.degrees(np.arctan2(cam_z - H, vn * cell)) >= bridge_min_angle
            hz |= drop
    return hz & known


UNKNOWN, FREE, HAZARD = 0, 1, 2


def classify_frame(pts, cam_xy=None, cam_z=None, cell=0.05, min_points=2, **kw):
    """pts: (N, 3) odom 점 (한 프레임), cam_xy / cam_z: 같은 좌표의 카메라 위치.
    반환 (ij (M, 2) 전역 칸 번호 floor(x / cell), state (M,) FREE/HAZARD)."""
    if len(pts) == 0:
        return np.zeros((0, 2), np.int64), np.zeros(0, np.int8)
    ij = np.floor(pts[:, :2] / cell).astype(np.int64)
    lo = ij.min(axis=0) - 1
    rel = ij - lo
    w, h = rel.max(axis=0) + 2
    key = rel[:, 0] * h + rel[:, 1]
    cnt = np.bincount(key, minlength=w * h)
    zmax = np.full(w * h, -np.inf)
    zmin = np.full(w * h, np.inf)
    np.maximum.at(zmax, key, pts[:, 2])
    np.minimum.at(zmin, key, pts[:, 2])
    ok = cnt >= min_points
    H = np.where(ok, zmax, np.nan).reshape(w, h)
    ext = np.where(ok, zmax - zmin, 0.0).reshape(w, h)
    cam_ij = None if cam_xy is None else (cam_xy[0] / cell - lo[0] - 0.5, cam_xy[1] / cell - lo[1] - 0.5)
    hz = hazards(H, ext, cell, cam_ij=cam_ij, cam_z=cam_z, **kw)
    ii, jj = np.nonzero(~np.isnan(H))
    return np.stack([ii + lo[0], jj + lo[1]], 1), np.where(hz[ii, jj], HAZARD, FREE).astype(np.int8)


class RollingState:
    """판정 결과를 odom 격자(로봇 주변 size x size m)에 기억. 가장 최근 관측이 이기고 forget 초 지나면 잊음.

    라이다 억제 (lidar_clear > 0): 라이다(평면 약 0.3 m)가 맞힌 칸 lidar_clear 안의 위험은 내보내지 않는다.
      왜: 카메라(0.27 m, 40° 아래)는 벽 윗부분을 못 봐서 벽·Husky 밑동을 0.1~0.2 m 턱으로 보고 위험으로 찍는다.
      2026-11-09 record5 2회: 지형 위험의 85~100% 가 SLAM 지도 벽 0.3 m 안, 멈춤 9건 모두 로봇 0.45 m 안 치명 66~121칸.
      벽은 라이다 층이 이미 막으니 지형 층이 할 일은 라이다보다 낮은 것(경사로 가장자리, 낮은 잔해)뿐이다.
      라이다는 relay 가 '기울 때 바닥 맞은 빔'을 지운 스캔을 쓴다 (경사로 위에서 바닥 점이 진짜 가장자리를 지우지 않게).
      한계: 벽 lidar_clear 안에 붙은 낮은 위험은 놓친다 (그 자리는 로봇 반경 0.23 m 때문에 어차피 거의 못 지나감)."""

    def __init__(self, cell=0.05, size=10.0, forget=30.0):
        self.cell, self.forget = cell, forget
        self.n = int(round(size / cell))
        self.i0 = self.j0 = None
        self.state = np.zeros((self.n, self.n), np.int8)
        self.t = np.full((self.n, self.n), -np.inf)
        self.lidar_t = np.full((self.n, self.n), -np.inf)
        self.suppressed = 0

    def _recenter(self, cx, cy):
        ci, cj = int(math.floor(cx / self.cell)), int(math.floor(cy / self.cell))
        q = self.n // 4
        if self.i0 is not None and q < ci - self.i0 < 3 * q and q < cj - self.j0 < 3 * q:
            return
        ni0, nj0 = ci - self.n // 2, cj - self.n // 2
        if self.i0 is not None:
            di, dj = ni0 - self.i0, nj0 - self.j0
            self.state = _shift(self.state, di, dj, fill=UNKNOWN)
            self.t = _shift(self.t, di, dj, fill=-np.inf)
            self.lidar_t = _shift(self.lidar_t, di, dj, fill=-np.inf)
        self.i0, self.j0 = ni0, nj0

    def update(self, ij, state, t, cx, cy):
        self._recenter(cx, cy)
        a, b = ij[:, 0] - self.i0, ij[:, 1] - self.j0
        m = (a >= 0) & (a < self.n) & (b >= 0) & (b < self.n)
        self.state[a[m], b[m]] = state[m]
        self.t[a[m], b[m]] = t

    def mark_lidar(self, xy, t, cx, cy):
        """xy: (N, 2) 라이다 맞은 점 (odom)."""
        self._recenter(cx, cy)
        ij = np.floor(np.asarray(xy) / self.cell).astype(np.int64).reshape(-1, 2)
        a, b = ij[:, 0] - self.i0, ij[:, 1] - self.j0
        m = (a >= 0) & (a < self.n) & (b >= 0) & (b < self.n)
        self.lidar_t[a[m], b[m]] = t

    def hazards_xy(self, t, cx, cy, radius, lidar_clear=0.0):
        if self.i0 is None:
            return np.zeros((0, 2))
        hz = ((t - self.t) <= self.forget) & (self.state == HAZARD)
        if lidar_clear > 0:
            lid = (t - self.lidar_t) <= self.forget
            near = np.zeros_like(lid)
            k = int(math.ceil(lidar_clear / self.cell))
            for di in range(-k, k + 1):
                for dj in range(-k, k + 1):
                    if di * di + dj * dj <= (lidar_clear / self.cell) ** 2:
                        near |= _shift(lid, di, dj, fill=False)
            self.suppressed = int((hz & near).sum())
            hz &= ~near
        ii, jj = np.nonzero(hz)
        x = (ii + self.i0 + 0.5) * self.cell
        y = (jj + self.j0 + 0.5) * self.cell
        near = (x - cx) ** 2 + (y - cy) ** 2 <= radius * radius
        return np.stack([x[near], y[near]], 1)


def pad_hazards(xy, pad, cell=0.05):
    """위험 칸 중심들(N x 2)에 pad m 떨어진 8 방향 점을 더해 칸 격자로 맞추고 중복 제거 (2026-11-11 fix16).
    벽보다 위험(경사로 옆 낭떠러지)에서 더 멀리 떨어지게 하려고 비용지도 치명 띠를 pad 만큼 넓힌다.
    근거: 넘어짐 2 번 모두 경사로 오르는 중 가장자리 위험 칸 0.19~0.33 m 옆을 지나다 옆으로 떨어짐(fix11, fix15)."""
    xy = np.asarray(xy, dtype=float).reshape(-1, 2)
    if pad <= 0 or len(xy) == 0:
        return xy
    d = pad / math.sqrt(2.0)
    offs = np.array([[0, 0], [pad, 0], [-pad, 0], [0, pad], [0, -pad], [d, d], [d, -d], [-d, d], [-d, -d]])
    pts = (xy[:, None, :] + offs[None, :, :]).reshape(-1, 2)
    ij = np.unique(np.floor(pts / cell).astype(np.int64), axis=0)
    return (ij + 0.5) * cell
