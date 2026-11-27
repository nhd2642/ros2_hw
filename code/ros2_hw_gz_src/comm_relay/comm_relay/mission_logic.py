"""탐색 완료 판단 · 경로 구역 검사 · Husky 통신 스위치 명령 (ROS 없음, 단위 시험용). record14.

사용자 목표 (2026-11-09): 각 A1 이 자기 구역을 다 탐색하면 원위치(스폰)로 복귀, Husky 와의 통신 불량은 위치(범위)가 아니라
사용자 조작(on/off)으로 정함, 두 로봇은 Husky 를 사이에 두고 자기 구역만 탐색.
"""


class DoneWatch:
    """goal_assigner 가 3 s 마다 내는 /<ns>/frontier_options (그 로봇이 갈 수 있는 자기 구역 프런티어 수) 로 탐색 완료 판단.

    -1 = 그 주기에 로봇 둘레에서 BFS 시작 칸을 못 찾음(벽에 붙음 등) -> 모름, 0 으로 치지 않음.
    완료 = 마지막 hold 초 동안 받은 값이 전부 0 이고, 그 구간을 메울 만큼 오래 받았고, 마지막 값이 fresh 초 안.
    """

    def __init__(self, hold=30.0, fresh=10.0):
        self.hold, self.fresh = hold, fresh
        self.zero_since = None
        self.last_t = None

    def update(self, t, n):
        if n < 0:                      # 모름: 판단을 처음부터
            self.zero_since = None
        elif n == 0:
            if self.zero_since is None:
                self.zero_since = t
        else:
            self.zero_since = None
        self.last_t = t

    def done(self, now):
        return (self.hold > 0 and self.zero_since is not None and self.last_t is not None
                and now - self.last_t <= self.fresh and now - self.zero_since >= self.hold)


def start_poses(cfg, k):
    """세트 k 를 시작할 때의 자세 {husky, a1_1, a1_2: [x, y, yaw]} (cfg = mission.yaml). k = 0 은 처음 스폰,
    k >= 1 은 세트 k-1 주차점에 Husky, 그 뒤 추종 자리에 A1 (부분 시험용 — sim.launch.py 와 mission.launch.py 가 같이 씀)."""
    if k <= 0:
        return cfg['spawn']
    px, py = cfg['sets'][k - 1]['park']
    return {'husky': [px, py, 0.0], **{ns: [px + dx, py + dy, 0.0] for ns, (dx, dy) in cfg['slots'].items()}}


def zone_dict(zones):
    """MissionState.zones (Zone[]) -> {로봇: [x0, x1, y0, y1]} (월드 m)."""
    return {z.robot: [z.x_min, z.x_max, z.y_min, z.y_max] for z in zones}


def in_rect(rect, x, y, grow=0.0):
    """rect = [x0, x1, y0, y1] (월드 m). grow > 0 이면 키우고 < 0 이면 줄여서 판정."""
    return rect[0] - grow <= x <= rect[1] + grow and rect[2] - grow <= y <= rect[3] + grow


def path_enters_rect(xy_world, rect, margin=0.15):
    """경로 점이 rect(다른 로봇 구역) 안으로 margin 넘게 들어가면 그 첫 점, 아니면 None.

    왜: record13 2 회차에서 a1_2 목표는 전부 자기 구역이었는데 Nav2 경로가 상대 구역으로 돌아감. 목표 칸 검사는 경로를 안 봄.
    """
    for x, y in xy_world:
        if in_rect(rect, x, y, -margin):
            return x, y
    return None


class SurvivorTally:
    """A1 들이 보낸 사람 위치(월드)를 merge m 안이면 같은 사람으로 묶어 센다 (두 로봇이 같은 사람을 따로 보고해도 한 명)."""

    def __init__(self, merge=1.5):
        self.merge, self.people = merge, []          # [x, y, 보고 수]

    def add(self, x, y):
        """새 사람이면 True."""
        for p in self.people:
            if (p[0] - x) ** 2 + (p[1] - y) ** 2 <= self.merge ** 2:
                p[0], p[1], p[2] = (p[0] * p[2] + x) / (p[2] + 1), (p[1] * p[2] + y) / (p[2] + 1), p[2] + 1
                return False
        self.people.append([x, y, 1])
        return True

    def count(self, rects, grow=0.5):
        """rects 중 하나(+grow) 안에 있는 사람 수."""
        return sum(1 for x, y, _ in self.people if any(in_rect(r, x, y, grow) for r in rects))


def refine_goal(cost, res, ox, oy, gx, gy, max_cost=127, radius=0.5):
    """Nav2 전역 비용지도(원시 0~255) 기준 목표 보정. record17 (2026-11-10).

    목표 칸 비용이 max_cost 이하이거나 모름(255, 프런티어 쪽)이면 그대로. 아니면 radius m 안에서 그런 칸 중 가장 가까운 칸으로
    옮김. 없으면 None (그 목표는 버림).
    왜: record15/16 에서 a1_2 가 Husky 모서리 옆 목표로 갔다가 출발 칸이 팽창 영역(비용 197~253)이라 계획이 계속 실패·끼임.
    Husky 좌표를 박는 대신 비용지도로 판단 -> 다른 맵의 좁은 틈·장애물 옆 목표에도 똑같이 적용됨.
    돌려줌 (x, y, 옮긴 거리 m) 또는 None. 비용지도 밖 목표는 그대로 (판단 못 함).
    """
    import numpy as np
    h, w = cost.shape
    c, r = int((gx - ox) / res), int((gy - oy) / res)
    if not (0 <= r < h and 0 <= c < w):
        return gx, gy, 0.0
    ok = (cost <= max_cost) | (cost == 255)
    if ok[r, c]:
        return gx, gy, 0.0
    k = int(np.ceil(radius / res))
    r0, r1, c0, c1 = max(0, r - k), min(h, r + k + 1), max(0, c - k), min(w, c + k + 1)
    rr, cc = np.mgrid[r0:r1, c0:c1]
    d = np.hypot(rr - r, cc - c) * res
    m = ok[r0:r1, c0:c1] & (d <= radius)
    if not m.any():
        return None
    i = np.argmin(np.where(m, d, np.inf))
    rr, cc, d = rr.flat[i], cc.flat[i], d.flat[i]
    return ox + (cc + 0.5) * res, oy + (rr + 0.5) * res, float(d)
