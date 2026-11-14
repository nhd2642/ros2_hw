"""복귀 판단·경로 추종 로직 (ROS 의존 없음, 단위 시험 대상).

상태
  WAIT      : 하트비트(/<ns>/comm_status)나 odom 을 아직 한 번도 못 받음 -> 아무 명령도 안 냄
              (팀원 원본: 노드 시작 1 s 뒤 los_checker 가 없으면 바로 복귀 시작)
  NORMAL    : 통신 정상. 지나온 위치를 breadcrumb 로 쌓는다
  RETURNING : 하트비트를 comm_timeout 동안 못 받음 -> breadcrumb 를 역순으로 되짚어 복귀 (즉시 복귀 정책)
  HOLD      : breadcrumb 를 다 되짚고 홈(탐색 시작 위치)에 왔는데도 여전히 단절 -> 정지 명령 유지
  STOPPING  : 복귀 중 통신 복구 -> stop_duration 동안 정지 명령 후 NORMAL
              (CHAMP 는 cmd_vel 타임아웃이 없어 마지막 명령을 계속 따른다. 팀원 원본은 발행만 멈췄음)

복귀 주행 (2026-11-06 사용자 결정):
  기본 'direct' = 이 로직이 지나온 길을 직접 몬다 (Gazebo 에서 뒤돌기 포함 확실히 동작).
  'auto' = Nav2 실행기가 살아 있으면 Nav2 FollowPath 에 맡김 (옵션. 뒤돌 때 Nav2 진행 검사에 걸리는 문제 있음).
  길 위 장애물: 걸을 때만, 앞으로 되짚을 breadcrumb 꺾은선의 띠(폭 +-0.15 m, 앞 0.5 m) 안에 라이다 점이
  연속 2 스캔 잡히면 멈춘다. 라이다 range_min 값과 몸 상자(앞 0.25 / 뒤 0.30 / 좌우 0.17 m) 안 점은 버린다. 그 띠는 로봇 몸이 실제로 지나간 자리라 그때는 비어 있었다 -> 지금 점이 있으면 새 장애물.
  옆 벽은 몸 반폭보다 멀리 있었으므로 띠 밖. 제자리 회전은 막지 않는다. 막히면 그 자리에서 기다린다.
  block_handover > 0 이면 그 초 넘게 막혔을 때 Nav2 가 살아 있으면 넘긴다 (기본 0 = 끔, 2026-11-07 사용자 결정:
  같은 막힌 길을 FollowPath 하므로 이득이 작고 Gazebo 미검증). 한 번 실패하면 이번 복귀에서는 다시 넘기지 않는다.
좌표: breadcrumb 와 추종은 로봇 자신의 odom 프레임(/<ns>/odom, EKF) 하나로만 한다.
      통신 판정(월드 좌표, ground truth)과 섞지 않는다 (팀원 원본은 두 좌표계를 한 콜백에 섞었음).
      홈 = 이번 세트 탐색을 시작한 자리 (record 가 False 인 동안의 마지막 위치. 노드가 /mission/state 로 record 를 정함).
"""
import math
from collections import deque

WAIT, NORMAL, RETURNING, HOLD, STOPPING = 'WAIT', 'NORMAL', 'RETURNING', 'HOLD', 'STOPPING'


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def path_hits(points, path, half_width, start=0.05, collect=False):
    """points: 장애물 점 [(x, y)] (odom). path: [(x, y)] 첫 점 = 로봇 위치, 이어서 되짚을 breadcrumb.
    경로 꺾은선에서 half_width 안에 있는 점 수. 로봇 옆/뒤(첫 구간에서 앞으로 start 미만)는 세지 않는다.
    breadcrumb 는 로봇 몸이 실제로 지나간 자리라 그때는 비어 있었다. 지금 그 띠 안에 점이 있으면
    지나간 뒤에 생긴 것(상자, 무너진 것, 다른 로봇)이다. 옆 벽은 몸 반폭보다 멀리 있었으므로 띠 밖."""
    out = []
    if len(path) < 2:
        return out if collect else 0
    segs = []
    for (ax, ay), (bx, by) in zip(path, path[1:]):
        dx, dy = bx - ax, by - ay
        L = math.hypot(dx, dy)
        if L > 1e-6:
            segs.append((ax, ay, dx / L, dy / L, L))
    if not segs:
        return out if collect else 0
    hw2 = half_width * half_width
    n = 0
    for px, py in points:
        for i, (ax, ay, ux, uy, L) in enumerate(segs):
            vx, vy = px - ax, py - ay
            along = vx * ux + vy * uy
            if i == 0 and along < start:
                break                      # 로봇 옆·뒤: 이 점은 세지 않음
            a = min(max(along, 0.0), L)
            ex, ey = vx - a * ux, vy - a * uy
            if ex * ex + ey * ey <= hw2:
                n += 1
                if collect:
                    out.append((px, py))
                break
    return out if collect else n


SELF_BOX = (0.25, 0.30, 0.17)   # 라이다 기준 몸 상자: 앞 0.25, 뒤 0.30, 좌우 0.17 m (A1 몸통 +-0.134, 엉덩이 x +-0.183, 발 앞 끝)


def scan_points(ranges, angle_min, angle_inc, range_min, range_max, x, y, yaw, max_r, self_box=SELF_BOX):
    """LaserScan 을 odom 좌표 점으로 (라이다 = 몸통 중앙 위, 앞 방향 = 로봇 x.
    a1.urdf.xacro lidar_joint xyz 0 0 0.09 rpy 0). max_r 보다 먼 점은 버린다.
    버리는 점: range_min(+1 cm) 값 — 광선 시작점(0.15 m)이 이미 무언가 안에 있을 때 나오는 값,
    몸 상자 안 점 — 로봇 자신(걸을 때 몸통·다리가 라이다 평면에 걸림). Gazebo 에서 걸을 때만
    방위 +-30~40°, 0.15 m 점이 2~4 스캔 나와 0.1~0.4 s 오정지를 일으킴 (2026-11-07 obstacle 시험)."""
    pts = []
    f, b, h = self_box if self_box else (0.0, 0.0, -1.0)
    for i, r in enumerate(ranges):
        if not (range_min + 0.01 < r <= min(range_max, max_r)):
            continue                          # inf/nan 은 비교가 거짓이라 여기서 빠진다
        a = angle_min + i * angle_inc         # 로봇 기준 방위
        lx, ly = r * math.cos(a), r * math.sin(a)
        if -b <= lx <= f and abs(ly) <= h:
            continue
        c, s_ = math.cos(yaw), math.sin(yaw)
        pts.append((x + c * lx - s_ * ly, y + s_ * lx + c * ly))
    return pts


class ReturnLogic:
    def __init__(self, comm_timeout=1.0, odom_timeout=0.5, spacing=0.10, reach_tol=0.25,
                 max_breadcrumbs=5000, vx=0.15, wz_turn=0.4, wz_max=0.4, k_heading=1.0,
                 turn_threshold=0.35, stop_duration=1.0, jump_threshold=0.5, path_points=300,
                 return_drive='direct', block_handover=0.0):
        self.comm_timeout = comm_timeout
        self.odom_timeout = odom_timeout
        self.spacing = spacing
        self.reach_tol = reach_tol
        self.vx = vx
        self.wz_turn = wz_turn
        self.wz_max = wz_max
        self.k_heading = k_heading
        self.turn_threshold = turn_threshold
        self.stop_duration = stop_duration
        self.jump_threshold = jump_threshold
        self.path_points = path_points
        self.return_drive = return_drive
        self.block_handover = block_handover
        self.blocked_since = None
        self.nav_gave_up = False      # 이번 복귀에서 Nav2 가 실패했으면 다시 넘기지 않음
        self.block_note = ''          # 노드가 막힘 판단 때 채우는 설명 (위치, 잡힌 점) -> 사건 문장에 붙임

        self.state = WAIT
        self.last_hb = None
        self.pose = None              # (x, y, yaw)
        self.pose_t = None
        self.home = None
        self.record = True            # False (탐색 단계 아님): 길을 쌓지 않고 홈 = 지금 자리 -> 끊기면 그 자리에 섬
        self.crumbs = deque(maxlen=max_breadcrumbs)
        self.t_state = 0.0
        self.drive = 'direct'         # RETURNING 중 누가 모나: 'direct'(이 노드) / 'nav'(nav_executor)
        self.events = []              # (t, 문자열) 로그용, 노드가 비운다

    # ---------- 입력 ----------
    def comm_ok(self, t):
        return self.last_hb is not None and t - self.last_hb <= self.comm_timeout

    def on_heartbeat(self, t):
        self.last_hb = t

    def on_pose(self, x, y, yaw, t):
        if self.pose is not None:
            jump = math.hypot(x - self.pose[0], y - self.pose[1])
            if jump > self.jump_threshold:
                # EKF odom 은 순간이동하지 않는다. 튀면 breadcrumb 를 끊어서 이어 붙이지 않는다.
                self._event(t, f'odom 위치가 {jump:.2f} m 튐 -> breadcrumb 이어붙이지 않음')
                self.crumbs.clear()
        self.pose = (x, y, yaw)
        self.pose_t = t
        if self.home is None:
            self.home = (x, y)
        if not self.record:
            self.crumbs.clear()
            self.home = (x, y)
        elif self.state == NORMAL and self.comm_ok(t):
            if not self.crumbs or math.hypot(x - self.crumbs[-1][0], y - self.crumbs[-1][1]) >= self.spacing:
                self.crumbs.append((x, y))

    # ---------- 상태 전이 + 명령 ----------
    def _event(self, t, s):
        self.events.append((t, s))

    def _set(self, t, state, why=''):
        if state != self.state:
            self._event(t, f'{self.state} -> {state}' + (f' ({why})' if why else ''))
            self.state = state
            self.t_state = t

    def _prune(self):
        """도달한 breadcrumb 를 뒤에서부터 지운다."""
        x, y, _ = self.pose
        while self.crumbs and math.hypot(self.crumbs[-1][0] - x, self.crumbs[-1][1] - y) < self.reach_tol:
            self.crumbs.pop()

    def target(self):
        if self.crumbs:
            return self.crumbs[-1]
        return self.home

    def return_path(self):
        """복귀할 경로 (현재 위치에서 가까운 순서): 남은 breadcrumb 역순 + 홈."""
        pts = list(reversed(self.crumbs))[:self.path_points]
        if len(pts) < self.path_points and self.home is not None:
            if not pts or math.hypot(pts[-1][0] - self.home[0], pts[-1][1] - self.home[1]) > 1e-3:
                pts.append(self.home)
        return pts

    def upcoming(self, length):
        """앞으로 되짚을 길: 현재 위치 -> 남은 breadcrumb 역순 -> 홈, 누적 length m 까지 (odom)."""
        x, y, _ = self.pose
        pts, acc = [(x, y)], 0.0
        for qx, qy in self.return_path():
            d = math.hypot(qx - pts[-1][0], qy - pts[-1][1])
            if d < 1e-6:
                continue
            if acc + d >= length:
                f = (length - acc) / d
                pts.append((pts[-1][0] + f * (qx - pts[-1][0]), pts[-1][1] + f * (qy - pts[-1][1])))
                break
            pts.append((qx, qy))
            acc += d
        return pts

    def _start_return(self, t, nav_available, why):
        self.drive = 'nav' if (self.return_drive == 'auto' and nav_available) else 'direct'
        self.blocked_since = None
        self.nav_gave_up = False
        self._set(t, RETURNING, f'{why}, 복귀 주행={self.drive}')

    def step(self, t, nav_available=False, nav_failed=False, blocked=None):
        """반환: (vx, wz) 를 cmd_vel_return 으로 발행, None 이면 발행하지 않음.
        blocked: blocked() -> bool. 앞으로 걸을 때 되짚을 길 띠 안 장애물 여부 (없으면 검사 안 함).
                 노드가 upcoming() 과 최근 스캔으로 판단한다."""
        if self.state == WAIT:
            if self.last_hb is not None and self.pose is not None:
                self._set(t, NORMAL, '첫 하트비트와 odom 수신')
            return None

        ok = self.comm_ok(t)
        if self.state == NORMAL:
            if not ok:
                self._start_return(t, nav_available, f'하트비트 {t - self.last_hb:.1f} s 없음')
            else:
                return None

        if self.state == STOPPING:
            if not ok:
                self._start_return(t, nav_available, '정지 중 다시 단절')
            elif t - self.t_state >= self.stop_duration:
                self._set(t, NORMAL, '정지 완료')
                return None
            else:
                return (0.0, 0.0)

        if self.state in (RETURNING, HOLD) and ok:
            self._set(t, STOPPING, '통신 복구')
            return (0.0, 0.0)

        # 여기부터 단절 상태 (RETURNING / HOLD)
        if self.pose is None or t - self.pose_t > self.odom_timeout:
            return (0.0, 0.0)            # 위치를 모르면 서 있는다
        self._prune()
        tx, ty = self.target()
        x, y, yaw = self.pose
        dist = math.hypot(tx - x, ty - y)

        if self.state == HOLD:
            return (0.0, 0.0)
        if not self.crumbs and dist < self.reach_tol:
            self._set(t, HOLD, '홈(탐색 시작 위치) 도착, 여전히 단절')
            return (0.0, 0.0)

        if self.drive == 'nav':
            if nav_failed or not nav_available:
                self.drive = 'direct'
                self.nav_gave_up = True
                self.blocked_since = None
                self._event(t, '복귀 주행 nav -> direct (nav_executor ' +
                            ('실패 보고' if nav_failed else '응답 없음') + ')')
            else:
                return None              # nav_executor 가 Nav2 FollowPath 로 복귀 경로를 따른다

        err = wrap(math.atan2(ty - y, tx - x) - yaw)
        if abs(err) > self.turn_threshold:
            return (0.0, math.copysign(self.wz_turn, err))      # 제자리 회전은 막지 않는다
        if blocked is not None and blocked():
            if self.blocked_since is None:
                self.blocked_since = t
                self._event(t, f'길 위 장애물 -> 정지 (목표점까지 {dist:.2f} m{self.block_note})')
            elif (self.block_handover > 0 and t - self.blocked_since > self.block_handover
                  and nav_available and not self.nav_gave_up):
                self.drive = 'nav'
                self._event(t, f'{self.block_handover:.0f} s 넘게 막힘 -> Nav2 에 넘겨 돌아가게 함')
                return None
            return (0.0, 0.0)
        if self.blocked_since is not None:
            self._event(t, f'장애물 사라짐 -> 계속 ({t - self.blocked_since:.1f} s 정지)')
            self.blocked_since = None
        wz = max(-self.wz_max, min(self.wz_max, self.k_heading * err))
        return (self.vx, wz)
