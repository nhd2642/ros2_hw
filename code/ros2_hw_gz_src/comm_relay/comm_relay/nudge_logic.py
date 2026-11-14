"""같은 자리 계획 실패 뒤 '옮겨 보기' 판단 (ROS 없음, 단위 시험용).

왜: 2026-11-09 긴 실행(record7b, 900 s)에서 a1_2 가 벽 0.27 m 옆에 선 채 534 s 동안 목표 61 개가 모두 0.01 s 만에 계획 실패.
  국소 비용지도(odom)에서는 0.45 m 안 치명 0 칸인데, a1_2 SLAM 지도 위에서는 로봇이 벽선 위에 있었음 (map->odom 어긋남).
  slam_toolbox 는 0.2 m 넘게 움직여야 스캔을 넣으므로 서 있으면 어긋남이 안 고쳐짐 -> 계획 실패 -> 안 움직임 교착.
"""
import math


class PlanFailStreak:
    """같은 자리(radius 안)에서 연속 계획 실패 수. 움직이면 다시 셈."""

    def __init__(self, radius=0.1, need=3):
        self.radius, self.need = radius, need
        self.xy, self.n = None, 0

    def fail(self, xy):
        """계획 실패 하나. 옮겨 보기를 시작할 때면 True (그때 다시 0 부터)."""
        if self.xy is None or math.hypot(xy[0] - self.xy[0], xy[1] - self.xy[1]) > self.radius:
            self.xy, self.n = xy, 0
        self.n += 1
        if self.n >= self.need:
            self.xy, self.n = None, 0
            return True
        return False

    def ok(self):
        self.xy, self.n = None, 0


def free_heading(data, width, height, res, ox, oy, x, y, max_d=0.6, n=16, blocked=99):
    """국소 비용지도(행 우선 data)에서 (x, y) 부터 n 방향으로 max_d 까지 광선을 쏴 막힘(>= blocked 또는 지도 밖) 전까지
    가장 멀리 가는 방향의 단위벡터와 그 거리. 모름(-1)은 빈 칸으로 봄 (SLAM 갱신이 목적이라 안 가본 쪽도 됨)."""
    step = res / 2

    def ray(c, s):
        d = 0.0
        while d + step <= max_d:
            px, py = x + (d + step) * c, y + (d + step) * s
            i, j = int((px - ox) / res), int((py - oy) / res)
            if not (0 <= i < width and 0 <= j < height) or data[j * width + i] >= blocked:
                break
            d += step
        return d

    vs = [(math.cos(2 * math.pi * k / n), math.sin(2 * math.pi * k / n)) for k in range(n)]
    ds = [ray(c, s) for c, s in vs]
    best = max(ds)
    top = [v for v, d in zip(vs, ds) if d >= best - 1e-9]
    # 가장 멀리 가는 방향이 여럿이면(한쪽 벽만 있을 때 반원 전체) 그 평균 = 장애물에서 멀어지는 쪽.
    # 단 트인 쪽이 둘로 갈라지면(복도 양 끝) 평균이 벽을 가리킬 수 있어, 평균 방향도 트였는지 확인하고 아니면 하나를 고름.
    mx, my = sum(v[0] for v in top), sum(v[1] for v in top)
    m = math.hypot(mx, my)
    if m > 1e-6 and ray(mx / m, my / m) >= 0.9 * best:
        return (mx / m, my / m), best
    if m < 1e-6:
        return top[0], best
    return max(top, key=lambda v: v[0] * mx + v[1] * my), best


def body_cmd(v, yaw, speed=0.08, wz=0.5, axis_tol_deg=45.0):
    """odom 방향 단위벡터 v 로 가기 위한 (vx, wz). 옆걸음(vy)은 안 씀: CHAMP 옆걸음은 Gazebo 에서 거의 안 나감
    (2026-11-09 record8b: 옆걸음 위주 옮겨 보기 3 번 실제/odom 이동 0.05~0.24, 앞뒤 위주 1 번 0.99 — 다리 odom 은 계획대로
    움직였다고 셈). 몸 앞뒤 축이 v 와 axis_tol 안이면 그 축으로 걷고(뒤로 걷기 포함), 아니면 가까운 축 쪽으로 제자리 회전."""
    th = math.atan2(-math.sin(yaw) * v[0] + math.cos(yaw) * v[1], math.cos(yaw) * v[0] + math.sin(yaw) * v[1])  # 몸 기준 각
    back = abs(th) > math.pi / 2
    err = th - math.copysign(math.pi, th) if back else th          # 가까운 축(앞 또는 뒤)과의 차
    if abs(err) <= math.radians(axis_tol_deg):
        return (-speed if back else speed), 0.0
    return 0.0, math.copysign(wz, err)


class ZeroCmdWatch:
    """Nav2 명령(속도·회전)이 모두 0 에 가까운 채 hold 초 이어졌는지.

    왜: 2026-11-09 record12 에서 실행기는 FOLLOWING 인데 Nav2 명령이 0 인 채 서 있는 구간이 3 s 이상 구간 합 로봇당 11~101 s
    (경사로 북서 모서리 등). 계획은 성공해서 옮겨 보기(계획 연속 실패 조건)는 안 걸리고, 목표 시간 제한 90 s 까지 기다림.
    """

    def __init__(self, eps=0.02, hold=8.0):
        self.eps, self.hold = eps, hold
        self.last_move = 0.0

    def msg(self, now, vx, wz):
        if abs(vx) >= self.eps or abs(wz) >= self.eps:
            self.last_move = now

    def stalled(self, now, since):
        """since = 이 추종을 시작한 시각. 그 뒤로(또는 마지막 움직임 뒤로) hold 초 동안 0 명령이면 True."""
        return self.hold > 0 and now - max(self.last_move, since) >= self.hold

    def zero_for(self, now, since):
        return now - max(self.last_move, since)


def cell_cost(data, width, height, res, ox, oy, x, y):
    """국소 비용지도(행 우선 data)에서 (x, y) 칸 비용. 지도 밖이면 None. (fix19: 옮겨 보기를 비용 낮은 곳까지)"""
    i, j = math.floor((x - ox) / res), math.floor((y - oy) / res)
    if not (0 <= i < width and 0 <= j < height):
        return None
    return data[j * width + i]
