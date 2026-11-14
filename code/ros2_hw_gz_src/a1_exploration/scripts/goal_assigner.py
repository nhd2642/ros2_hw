from collections import deque
import json
import math

import numpy as np
import rclpy
from geometry_msgs.msg import PoseArray, PoseStamped
from nav_msgs.msg import OccupancyGrid, Path
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from std_msgs.msg import Int32, String
from visualization_msgs.msg import Marker, MarkerArray

ROBOTS = ('a1_1', 'a1_2')


def inflate(mask, r):
    if r < 1:
        return mask.copy()
    h, w = mask.shape
    padded = np.pad(mask, r, constant_values=False)
    out = np.zeros_like(mask)
    for dy in range(-r, r + 1):
        for dx in range(-r, r + 1):
            if dx * dx + dy * dy <= r * r:
                out |= padded[r+dy:r+dy+h, r+dx:r+dx+w]
    return out


def bfs(walkable, start):
    h, w = walkable.shape
    dist = np.full(h*w, -1, dtype=np.int32)
    prev = np.full(h*w, -1, dtype=np.int32)
    q = deque([start])
    dist[start] = 0
    while q:
        pos = q.popleft()
        row, col = divmod(pos, w)
        for dr, dc in ((1, 0), (-1, 0), (0, 1), (0, -1)):
            rr, cc = row + dr, col + dc
            if 0 <= rr < h and 0 <= cc < w and walkable[rr, cc]:
                new = rr*w + cc
                if dist[new] < 0:
                    dist[new], prev[new] = dist[pos] + 1, pos
                    q.append(new)
    return dist, prev


class GoalAssigner(Node):
    def __init__(self):
        super().__init__('multi_robot_goal_assigner')
        qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL
        )
        self.grid = None
        self.candidates = None
        self.poses = {}

        self.create_subscription(
            OccupancyGrid, '/shared_map',
            lambda m: setattr(self, 'grid', m), qos
        )
        self.create_subscription(
            PoseArray, '/frontier_candidates',
            lambda m: setattr(self, 'candidates', m), 10
        )
        for ns in ROBOTS:
            self.create_subscription(
                PoseStamped, f'/{ns}/pose_shared',
                lambda m, n=ns: self.poses.__setitem__(n, m), 10
            )

        self.goals = {
            n: self.create_publisher(
                PoseStamped, f'/{n}/exploration_goal', 10
            )
            for n in ROBOTS
        }
        self.paths = {
            n: self.create_publisher(
                Path, f'/{n}/candidate_path', 10
            )
            for n in ROBOTS
        }
        self.markers = self.create_publisher(
            MarkerArray, '/assigned_goal_markers', 10
        )

        self.last_log = None
        # >>> comm_relay assigner_setup.sh: 로봇별 블랙리스트 + Nav2 와 맞춘 장애물 여유
        self.clearance = float(self.declare_parameter('obstacle_clearance', 0.15).value)
        self.robot_bl_radius = float(self.declare_parameter('robot_blacklist_radius', 0.9).value)
        # 120 -> 60 s (2026-11-13): nav_executor 는 목표 없이 90 s 면 탐색을 끝내므로, 실패한 프런티어가 그보다 오래 제외되면
        # 한 번 실패로 그 방을 못 보고 끝났다 (세트 1 남쪽 안쪽 방·사람 미발견). 60 s 뒤 다시 주고, 같은 곳 3 번 실패는 실행기가 막는다.
        self.robot_bl_ttl = float(self.declare_parameter('robot_blacklist_ttl', 60.0).value)
        self.robot_bl = {n: [] for n in ROBOTS}
        # 구역 분담: husky_mission 이 /mission/state 로 로봇별 구역 사각형 [x0, x1, y0, y1] (월드 = shared_map) 을 내림.
        # 탐색 단계(EXPLORE)에만 목표를 낸다. 두 로봇 구역이 같으면(공동 건물) 한 건물을 같이 나눠 탐색.
        self.rects = {}
        self.active = False
        # 목표 칸은 구역 가장자리에서 zone_goal_margin 더 안쪽 (2026-11-09 record11: 경계 근처 목표로 상대 쪽에 넘어가 멈춤)
        self.zone_margin = float(self.declare_parameter('zone_goal_margin', 0.3).value)
        # 두 로봇 목표가 goal_spread m 보다 가까우면 그만큼 벌점 -> 서로 떨어진 프런티어를 골라 겹쳐 보는 구간을 줄임 (공동 탐색)
        self.spread = float(self.declare_parameter('goal_spread', 3.0).value)
        # 목표 칸은 장애물에서 goal_clearance 이상 (걷는 길 여유 obstacle_clearance 0.15 와 따로). 0.15 만 띄우면 사람·기둥 뒤 그림자
        # 프런티어를 보러 0.4 m 까지 다가가 사람 발판에 걸리고, 그 자리(팽창 영역 안)에서는 복귀 계획이 실패했음 (2026-11-12 공동 건물).
        # 그만큼 못 다가가는 프런티어(물체 뒤 좁은 그림자)는 후보에서 빠진다 = 다시 보러 가지 않음.
        self.goal_clearance = float(self.declare_parameter('goal_clearance', 0.40).value)
        for n in ROBOTS:
            self.create_subscription(
                PoseStamped, f'/{n}/exploration_blacklist',
                lambda m, n=n: self.robot_bl[n].append((m.pose.position.x, m.pose.position.y, self.get_clock().now().nanoseconds * 1e-9)), 10
            )
        self.create_subscription(
            String, '/mission/state', self.on_mission,
            QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        )
        # 로봇별 '갈 수 있는 자기 구역 프런티어 수' (record14): nav_executor 가 0 이 30 s 이어지면 탐색 완료 -> 원위치 복귀.
        # -1 = 그 주기 로봇 둘레 BFS 시작 칸 없음(모름)
        self.opt_pub = {n: self.create_publisher(Int32, f'/{n}/frontier_options', 10) for n in ROBOTS}
        self.min_reach = float(self.declare_parameter('min_reach_area', 3.0).value)
        # <<< comm_relay assigner_setup.sh
        self.create_timer(3.0, self.process)

    def robot_blocked(self, ns, x, y):
        now = self.get_clock().now().nanoseconds * 1e-9
        self.robot_bl[ns] = [b for b in self.robot_bl[ns] if now - b[2] <= self.robot_bl_ttl]
        return any(math.hypot(x - bx, y - by) <= self.robot_bl_radius for bx, by, _ in self.robot_bl[ns])

    def on_mission(self, msg):
        d = json.loads(msg.data)
        self.active = d.get('phase') == 'EXPLORE'
        self.rects = d.get('zones', {})

    def in_zone(self, ns, x, y, margin=0.0):
        r = self.rects.get(ns)
        return r is not None and r[0] + margin <= x <= r[1] - margin and r[2] + margin <= y <= r[3] - margin

    def log(self, message):
        if message != self.last_log:
            self.get_logger().info(message)
            self.last_log = message

    def process(self):
        if not self.active:
            self.log('탐색 단계 아님 -> 목표 배정 쉼')
            return
        if self.grid is None or self.candidates is None or any(
                ns not in self.poses for ns in ROBOTS):
            self.log(
                'Waiting for /shared_map, '
                '/frontier_candidates, both /pose_shared'
            )
            return

        msg = self.grid
        info = msg.info
        h, w, res = info.height, info.width, info.resolution

        if not h or not w or res <= 0 or len(msg.data) != h*w:
            self.log('Invalid shared map')
            return

        frame = msg.header.frame_id

        if self.candidates.header.frame_id != frame or any(
                p.header.frame_id != frame
                for p in self.poses.values()):
            self.log('Frame mismatch: all inputs must be in shared_map')
            return

        if not self.candidates.poses:
            self.log('No Frontier candidates')
            for n in ROBOTS:
                self.opt_pub[n].publish(Int32(data=0))
            self.publish_markers([])
            return

        cells = np.asarray(msg.data).reshape(h, w)

        # 알려진 자유공간만 이동 가능
        walk_all = (cells >= 0) & (cells <= 20)

        # 장애물 주변 15cm 여유
        walk_all &= ~inflate(cells >= 50, math.ceil(self.clearance/res))

        # 목표 위치는 미탐색 영역에서 10cm 이상 떨어지도록
        safe_goal = walk_all & ~inflate(
            cells == -1, math.ceil(0.10/res)
        ) & ~inflate(cells >= 50, math.ceil(self.goal_clearance/res))

        q = info.origin.orientation
        yaw = math.atan2(
            2*(q.w*q.z + q.x*q.y),
            1-2*(q.y*q.y + q.z*q.z)
        )
        co, si = math.cos(yaw), math.sin(yaw)
        ox = info.origin.position.x
        oy = info.origin.position.y

        def to_grid(x, y):
            dx, dy = x-ox, y-oy
            return (
                int(round((co*dx + si*dy)/res - 0.5)),
                int(round((-si*dx + co*dy)/res - 0.5))
            )

        def to_world(index):
            r, c = divmod(index, w)
            x, y = (c+0.5)*res, (r+0.5)*res
            return ox+co*x-si*y, oy+si*x+co*y

        solutions = {}
        self.reach = {}   # comm_relay: 블랙리스트 포함 닿는 자기 구역 프런티어 수
        self.reach_area = {}   # comm_relay: 로봇에서 BFS 로 닿는 넓이 m²

        for ns in ROBOTS:
            p = self.poses[ns].pose.position
            col, row = to_grid(p.x, p.y)

            # BFS 범위 = 자기 구역 + 로봇 자리 (+1 m). 도로·다른 세트까지 퍼지면 지도가 커질수록 느려짐
            walk = walk_all
            r = self.rects.get(ns)
            if r is not None:
                c0, r0 = to_grid(min(r[0], p.x) - 1.0, min(r[2], p.y) - 1.0)
                c1, r1 = to_grid(max(r[1], p.x) + 1.0, max(r[3], p.y) + 1.0)
                walk = np.zeros_like(walk_all)
                walk[max(0, r0):r1 + 1, max(0, c0):c1 + 1] = walk_all[max(0, r0):r1 + 1, max(0, c0):c1 + 1]

            # 로봇 위치가 장애물 셀인 경우
            # 30cm 이내의 안전한 셀 탐색
            radius = math.ceil(0.30/res)

            starts = [
                (dr*dr+dc*dc, (row+dr)*w+col+dc)
                for dr in range(-radius, radius+1)
                for dc in range(-radius, radius+1)
                if dr*dr+dc*dc <= radius*radius
                and 0 <= row+dr < h
                and 0 <= col+dc < w
                and walk[row+dr, col+dc]
            ]

            if not starts:
                solutions[ns] = (None, None, [])
                continue

            start = min(starts)[1]
            dist, prev = bfs(walk, start)
            options = []
            self.reach[ns] = 0
            self.reach_area[ns] = float((dist >= 0).sum()) * res * res

            for fi, frontier in enumerate(self.candidates.poses):
                fx = frontier.position.x
                fy = frontier.position.y
                if not self.in_zone(ns, fx, fy):
                    continue
                blocked = self.robot_blocked(ns, fx, fy)
                fc, fr = to_grid(fx, fy)

                # Frontier 주변에서 안전한 목표 위치 탐색
                rad = math.ceil(0.65/res)
                best = None

                for rr in range(max(0, fr-rad), min(h, fr+rad+1)):
                    for cc in range(max(0, fc-rad), min(w, fc+rad+1)):
                        idx = rr*w+cc

                        if not safe_goal[rr, cc] or dist[idx] < 0:
                            continue

                        gx, gy = to_world(idx)
                        if not self.in_zone(ns, gx, gy, self.zone_margin):
                            continue
                        shift = math.hypot(gx-fx, gy-fy)

                        if shift > 0.65:
                            continue

                        # 경로 거리 + Frontier와의 거리
                        score = dist[idx]*res + 1.2*shift

                        if best is None or score < best[0]:
                            best = (score, fi, idx, fx, fy)

                if best:
                    self.reach[ns] += 1
                    if not blocked:
                        options.append(best)

            if self.reach[ns] == 0 and not self.in_zone(ns, p.x, p.y):
                # 접근 목표 (로봇이 아직 구역 밖일 때만): 구역 안 프런티어는 있는데 그 0.65 m 안에 닿을 수 있는 안전 칸이 하나도 없을 때 (멀리서 문 너머를 보면
                # 라이다 광선이 듬성듬성해 지도가 줄무늬로만 채워짐 — 2026-11-12 공동 건물: 문에서 4 m 떨어진 출발 자리에서 1500 s 동안 목표 0).
                # 닿을 수 있는 안전 칸 중 프런티어에 가장 가까운 칸으로 간다 (구역 밖이어도 됨). 다가가면 지도가 채워져 보통 규칙으로 돌아옴.
                rr, cc = np.nonzero((dist.reshape(h, w) >= 0) & safe_goal)
                for fi, frontier in enumerate(self.candidates.poses if rr.size else ()):
                    fx, fy = frontier.position.x, frontier.position.y
                    if not self.in_zone(ns, fx, fy) or self.robot_blocked(ns, fx, fy):
                        continue
                    lx, ly = (cc + 0.5) * res, (rr + 0.5) * res
                    shift = np.hypot(ox + co * lx - si * ly - fx, oy + si * lx + co * ly - fy)
                    k = int(np.argmin(shift))
                    idx = int(rr[k]) * w + int(cc[k])
                    options.append((dist[idx] * res + 1.2 * float(shift[k]), fi, idx, fx, fy))
                self.reach[ns] = len(options)

            solutions[ns] = (start, prev, options)

        for n in ROBOTS:
            pocket = self.reach_area.get(n, 0.0) < self.min_reach     # 좁은 주머니에 낌 -> 모름
            self.opt_pub[n].publish(Int32(data=-1 if solutions[n][0] is None or pocket else self.reach.get(n, 0)))
        one, two = (solutions[n][2] for n in ROBOTS)
        pairs = []

        # 서로 다른 Frontier, 목표 간 최소 75cm 거리
        for a in one:
            for b in two:
                ax, ay = to_world(a[2])
                bx, by = to_world(b[2])

                gap = math.hypot(ax-bx, ay-by)
                if a[1] != b[1] and gap >= 0.75:
                    pairs.append((a[0]+b[0] + max(0.0, self.spread-gap), a, b))

        selected = {}

        if pairs:
            _, selected[ROBOTS[0]], selected[ROBOTS[1]] = min(
                pairs, key=lambda p: p[0]
            )
        else:
            # 안전한 목표 두 개가 없으면 한 대만 배정
            singles = [
                (p[0], ns, p)
                for ns in ROBOTS
                for p in solutions[ns][2]
            ]

            if singles:
                _, ns, selected[ns] = min(
                    singles, key=lambda p: p[0]
                )

        stamp = self.get_clock().now().to_msg()
        markers = []
        report = []

        for i, ns in enumerate(ROBOTS):
            if ns not in selected:
                continue

            _, frontier_id, index, fx, fy = selected[ns]
            x, y = to_world(index)
            heading = math.atan2(fy-y, fx-x)

            goal = PoseStamped()
            goal.header.frame_id = frame
            goal.header.stamp = stamp
            goal.pose.position.x = x
            goal.pose.position.y = y
            goal.pose.orientation.z = math.sin(heading/2)
            goal.pose.orientation.w = math.cos(heading/2)

            self.goals[ns].publish(goal)

            # 탐색 경로 시각화
            start, prev, _ = solutions[ns]
            seq, cur = [], index

            while cur >= 0:
                seq.append(cur)
                if cur == start:
                    break
                cur = int(prev[cur])

            path = Path()
            path.header = goal.header

            if seq[-1] == start:
                for cell in reversed(seq):
                    point = PoseStamped()
                    point.header = goal.header
                    point.pose.position.x, point.pose.position.y = to_world(cell)
                    point.pose.orientation.w = 1.0
                    path.poses.append(point)

                self.paths[ns].publish(path)

            # 배정된 목표 화살표
            marker = Marker()
            marker.header = goal.header
            marker.ns = 'assigned_goals'
            marker.id = i
            marker.type = Marker.ARROW
            marker.action = Marker.ADD
            marker.pose = goal.pose
            marker.pose.position.z = 0.18
            marker.scale.x = 0.5
            marker.scale.y = 0.16
            marker.scale.z = 0.12

            marker.color.r, marker.color.g, marker.color.b = (
                (0.1, 0.4, 1.0)
                if ns == 'a1_1'
                else (1.0, 0.2, 0.2)
            )
            marker.color.a = 1.0
            markers.append(marker)

            report.append(
                f'{ns} -> F{frontier_id}: ({x:.2f}, {y:.2f})'
            )

        self.publish_markers(markers)

        self.log(
            ' | '.join(report)
            if report
            else 'No safe reachable goal: check map and pose alignment'
        )

    def publish_markers(self, markers):
        clear = Marker()
        clear.action = Marker.DELETEALL

        self.markers.publish(
            MarkerArray(markers=[clear] + markers)
        )


def main():
    rclpy.init()
    node = GoalAssigner()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
