"""로봇별 Nav2 목표 실행기 (탐색 목표 + 통신 복귀 경로).

팀원 Nav2 구성은 planner_server/controller_server 뿐이다(bt_navigator·behavior server·복구 없음).
그래서 '계획 -> 추종 -> 실패 처리'를 여기서 한다.

탐색:  /<ns>/nav_goal_local (map, nav_goal_bridge 출력) -> ComputePathToPose -> FollowPath
  - goal_assigner 가 3 s 마다 목표를 다시 보내므로: 0.5 m 안의 같은 목표는 무시,
    다른 목표는 지금 목표를 min_dwell 이상 따른 뒤에만 바꾼다 (그 전엔 보류했다가 끝나면 실행).
  - 계획 실패/추종 실패(진행 없음)/goal_timeout -> /<ns>/exploration_blacklist (그 로봇만, goal_assigner 가
    그 로봇에게만 근처 프런티어를 안 줌 — assigner_setup.sh). 이전엔 공용으로 보내 다른 로봇 몫까지 지웠음.
  - 목표가 통신 구역 밖(+여유), 도착했는데 같은 목표 반복 -> 공용 /exploration_blacklist (frontier_comm_filter 가
    둘 다에서 뺌).
통신 끊김:  /<ns>/comm_mode 가 NORMAL 이 아니면 탐색 목표를 취소하고 PAUSED. 복귀 주행은 return_relay 가 직접 한다.
먼저 돌기: 계획한 경로의 앞쪽(turn_sample 0.5 m)이 몸 방향과 turn_first(45°) 넘게 다르면 FollowPath 전에
  제자리 회전(turn_speed 0.5 rad/s, /<ns>/cmd_vel_nav_test)으로 turn_done(20°) 안까지 돌린다 (TURNING).
  왜: DWB 는 경로가 뒤쪽이면 제자리 회전보다 정지를 골라 영영 서 있음. Nav2 RotationShimController 가 그걸 하지만
  경로 점을 '지금' 시각으로 base 로 변환하다 TF 가 늦으면(A1 약 0.2~0.3 s) 그 경로에서는 회전을 포기한다 —
  클라우드 재현: TF 0.05 s 지연에도 60° 돌다 멈춤, 0.25 s 면 안 돎 (2026-11-08). 여기서는 TF 없이 /<ns>/odom 방향과
  map 경로 방향을 비교 (SLAM map 은 odom 과 같은 방향에서 시작, 보정 회전은 몇 도라 45°/20° 판단엔 충분).
  경사 위(IMU 기울기 > turn_max_tilt 6°)에서는 먼저 돌기를 안 하고, 도는 중 기울면 멈추고 바로 추종으로 넘김:
  A1 은 10° 경사로 위 제자리 회전에서 옆으로 미끄러짐 (2026-11-08 mission: TURNING 6 s 동안 0.5 m 미끄러져 경사로
  동쪽 가장자리에서 넘어짐, 표 기록으로 확인).
목표 유지: 따라가는 중 다른 목표가 와도, 지금 목표 commit_radius(1.0 m) 안에 프런티어(/frontier_candidates_comm)가
  아직 있으면 바꾸지 않고 보류(끝나면 시작). 팀원 goal_assigner 는 3 s 마다 다시 고르고 실행기는 min_dwell(8 s) 뒤 바꿔서
  도착 전에 목표가 계속 바뀌었음 (2026-11-08 mission 기준선: a1_1 시도 17 개 중 15 개가 '교체', 도착 1). 프런티어 정보가
  5 s 넘게 없으면 예전처럼 min_dwell 만 봄. commit:=false 로 끔.
벗어나기: 경로 추종이 실패했을 때 로봇 중심 escape_radius(0.1 m) 안에 국소 비용지도 치명·내접 칸(>=99)이 있으면
  (중심이 장애물 테두리 안 -> DWB 후보가 전부 불법 'No valid trajectories'), 0.5 m 안 치명 칸(100)들에서 멀어지는 쪽으로
  escape_speed(0.08 m/s)로 이동 — 최대 escape_time(10 s), 중심이 테두리 밖이면 그만 (ESCAPING).
  이동은 옆걸음 없이 앞뒤 축으로 (축이 45° 넘게 틀어져 있으면 먼저 제자리 회전): 2026-11-09 record8b 에서 옆걸음 위주 이동은
  실제/odom 0.05~0.24 (다리 odom 만 움직였다고 셈), 앞뒤 위주는 0.99.
  지형 층은 몸 둘레 위험을 지우지 않아서(가장자리 보호) 벽·Husky 옆에 0.2 m 까지 붙으면 빠져나오지 못했음
  (2026-11-09 mission: a1_2 가 벽 0.20~0.23 m 옆에서 'No valid trajectories' 5 번, 목표 6 개 연속 실패).
옮겨 보기: 같은 자리(0.1 m 안)에서 경로 계획이 nudge_after(3) 번 연속 실패하면, 국소 비용지도에서 가장 트인 방향으로
  nudge_dist(0.3 m) 이동 (같은 ESCAPING 상태·같은 이동 방식, 최대 nudge_time 20 s 벽시계) -> SLAM 이 스캔을 넣어 map->odom 을 다시 맞추게.
  (2026-11-09 900 s 실행: a1_2 가 벽 0.27 m 옆에 선 채 534 s 동안 계획 61 번 실패 — SLAM 지도에서는 로봇이 벽선 위,
  국소 비용지도는 0.45 m 안 치명 0. 서 있으면 slam_toolbox 가 스캔을 안 넣어(0.2 m 이동 조건) 교착.) nudge_after:=0 으로 끔.
임무 단계 (/mission/state, husky_mission): phase 가 EXPLORE·REGROUP 일 때만 탐색 목표를 받는다 (이동 단계 TRANSIT 은 a1_follower 가 몬다).
  세트 번호가 바뀌면 탐색 상태(완료 판정·도착/실패/폐경로 기록·지나온 길)를 처음으로 돌린다.
목표 없음 완료: 탐색 단계에 IDLE 인 채 마지막 추종(또는 세트 시작·통신 복구) 뒤 idle_done(90 s) 동안 한 번도 추종을 시작하지 못하면
  (계획만 하고 거부·실패한 시도는 세지 않는다: 세면 거부되는 목표를 60 s 마다 다시 받아 끝나지 않았음) (남은 프런티어가 전부 블랙리스트·폐경로·구역 검사에
  걸려 갈 수 있는 목표가 없음) 완료로 보고 탐색 시작 위치로 간다. 2026-11-12: a1_2 가 남은 목표 전부 '폐경로 지남' 거부 -> 프런티어 수는
  0 이 아니라 완료 판정도 안 나고 서 있었음.
탐색 완료 -> 탐색 시작 위치 (record14): goal_assigner 의 /<ns>/frontier_options(갈 수 있는 자기 구역 프런티어 수)가 done_after(30 s) 동안
  0 이고 실행기가 IDLE 이면 (또는 phase 가 REGROUP 이 되면) 이번 세트 탐색 시작 위치(/<ns>/home_local, map)로 Nav2 계획·추종
  (실패하면 home_retries 번 다시, 시간 제한 home_timeout). 도착하면 DONE (그 뒤 탐색 목표 무시),
  /<ns>/set_done = 세트 번호 (latched) -> husky_mission. 사건 '홈 시작' / '홈 도착' / '홈 실패'.
접근 목표: 로봇이 아직 자기 구역 밖(도로 위 출발 자리)이면 구역 밖 목표도 받는다 — 문에서 멀어 문 안쪽 지도가 듬성듬성할 때
  goal_assigner 가 프런티어에 가장 가까운 닿을 수 있는 칸을 준다. 구역 안에 들어온 뒤에는 구역 안 목표만.
경로 구역 검사 (record14): 계획한 경로가 다른 로봇 구역(사각형) 안으로 path_zone_margin 넘게 들어가면 추종하지 않고
  그 목표를 이 로봇 블랙리스트에 (record13: 북쪽 목표인데 Nav2 경로가 남쪽으로 돌아 a1_2 자기 구역 71 %). 구역이 같으면(공동 건물) 검사 안 함.
경사 위 벗어나기 (record14): 기울기 > turn_max_tilt 일 때 벗어나기/옮겨 보기가 제자리 회전을 해야 하면 회전하지 않고 끝냄
  (record13 1 회차: 경사로 위 (-0.77, -2.46) 에서 벗어나기 제자리 회전 중 roll 8 -> 45° 로 서쪽 가장자리에서 넘어짐).
넘어짐: /<ns>/imu/data 기울기 > fall_tilt(45°) 면 FALLEN — 목표 취소, 새 목표 무시(블랙리스트 안 함).
  20° 아래로 3 s 서 있으면 IDLE 로 돌아감. (2026-11-07 mission: 넘어진 a1_1 이 계획 실패 블랙리스트를 쏟아내
  공용 프런티어가 15 -> 3 개로 줄어 a1_2 탐색까지 멈춤)
사건 /<ns>/nav_executor/events: '도착 x y' (FollowPath 성공), '실패 이유' (블랙리스트할 때). 시험·모니터용.
상태 하트비트 /<ns>/nav_executor/state 는 FollowPath 서버가 준비됐을 때만 보낸다.
"""
import json
import math

import numpy as np

import rclpy
from rclpy.executors import ExternalShutdownException
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseArray, PoseStamped
from nav2_msgs.action import ComputePathToPose, FollowPath
from nav2_msgs.srv import GetCostmap
from geometry_msgs.msg import Twist
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import Imu
from std_msgs.msg import Int32, String

from comm_relay.mission_logic import DoneWatch, in_rect, path_enters_rect, refine_goal
from comm_relay.nudge_logic import PlanFailStreak, ZeroCmdWatch, body_cmd, cell_cost, free_heading

def sim_now(node):
    """노드 시계(초). use_sim_time 이면 시뮬 시간 — 2026-11-10 벽시계(time.monotonic) 대신 (RTF 0.1~0.3 에서 한도가 시뮬 기준 3~7 배 엄격했음)"""
    return node.get_clock().now().nanoseconds * 1e-9


LATCHED = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                     durability=DurabilityPolicy.TRANSIENT_LOCAL)


def dist(a, b):
    return math.hypot(a.pose.position.x - b.pose.position.x, a.pose.position.y - b.pose.position.y)


def dist_xy(a, b):
    return math.hypot(a[0] - b[0], a[1] - b[1])


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def angle_diff(a, b):
    """a - b 를 (-pi, pi] 로."""
    return math.atan2(math.sin(a - b), math.cos(a - b))


def path_heading(path, sample=0.5):
    """경로 첫 점에서 sample 거리 넘는 첫 점까지의 방향 (짧으면 끝 점까지). 점이 2 개 미만이거나 3 cm 안이면 None."""
    if len(path.poses) < 2:
        return None
    x0, y0 = path.poses[0].pose.position.x, path.poses[0].pose.position.y
    far = None
    for q in path.poses[1:]:
        far = q.pose.position
        if math.hypot(far.x - x0, far.y - y0) >= sample:
            break
    if math.hypot(far.x - x0, far.y - y0) < 0.03:
        return None
    return math.atan2(far.y - y0, far.x - x0)


class Executor:
    def __init__(self, node, ns, p):
        self.node, self.ns, self.p = node, ns, p
        self.log = node.get_logger()
        self.plan_ac = ActionClient(node, ComputePathToPose, f'/{ns}/compute_path_to_pose')
        self.cm_cli = node.create_client(GetCostmap, f'/{ns}/global_costmap/get_costmap')   # 목표 보정 (record17)
        self.follow_ac = ActionClient(node, FollowPath, f'/{ns}/follow_path')
        node.create_subscription(PoseStamped, f'/{ns}/nav_goal_local', self.on_local_goal, 10)
        node.create_subscription(PoseStamped, f'/{ns}/exploration_goal', self.on_shared_goal, 10)
        node.create_subscription(String, f'/{ns}/comm_mode', self.on_mode, LATCHED)
        node.create_subscription(PoseStamped, f'/{ns}/home_local', lambda m: setattr(self, 'home', m), 10)
        node.create_subscription(Imu, f'/{ns}/imu/data', self.on_imu, qos_profile_sensor_data)
        node.create_subscription(Odometry, f'/{ns}/odom', self.on_odom, 10)
        node.create_subscription(OccupancyGrid, f'/{ns}/local_costmap/costmap', lambda m: setattr(self, 'costmap', m),
                                 LATCHED)
        self.pub_cmd = node.create_publisher(Twist, f'/{ns}/cmd_vel_nav_test', 10)   # 먼저 돌기 (컨트롤러와 같은 출력)
        node.create_subscription(Twist, f'/{ns}/cmd_vel_nav_test', lambda m: self.zero.msg(sim_now(self.node), m.linear.x, m.angular.z), 10)
        self.pub_state = node.create_publisher(String, f'/{ns}/nav_executor/state', 10)
        self.pub_event = node.create_publisher(String, f'/{ns}/nav_executor/events', 10)   # '도착 x y' / '실패 이유'
        self.pub_path = node.create_publisher(Path, f'/{ns}/nav_executor/path', LATCHED)
        self.pub_bl = node.create_publisher(PoseStamped, '/exploration_blacklist', 10)          # 공용 (둘 다 제외)
        self.pub_bl_robot = node.create_publisher(PoseStamped, f'/{ns}/exploration_blacklist', 10)  # 이 로봇만
        self.pub_done = node.create_publisher(Int32, f'/{ns}/set_done', LATCHED)                 # 끝낸 세트 번호 (탐색 시작 위치 도착)
        node.create_subscription(Int32, f'/{ns}/frontier_options',
                                 lambda m: self.done_watch.update(sim_now(self.node), m.data), 10)

        self.state = 'IDLE'
        self.mode = None
        self.active = False           # 임무 단계가 탐색(EXPLORE·REGROUP)인가
        self.set_k = None             # 지금 세트 번호
        self.zone, self.others = None, []   # 자기 구역 사각형, 다른 로봇 구역들 (월드 [x0, x1, y0, y1])
        self.home = None              # 이번 세트 탐색 시작 위치 (map PoseStamped, nav_goal_bridge 가 변환)
        self.gen = 0                  # 오래된 콜백 무시용
        self.handle = None            # 현재 FollowPath goal handle
        self.goal = None              # (local, shared, 시작 시각)
        self.pending = None           # (local, shared)
        self.reached = []             # (local x, y, 도착 시각) 최근 도착한 목표
        self.visits = []              # (local x, y) 이번 세트에 도착한 목표 전부 — 같은 자리 3 번이면 그만 감
        self.last_shared = None       # (shared, 받은 시각)
        self.fallen = False
        self.upright_since = None
        self.odom_yaw = None          # (yaw, 받은 시각)
        self.tilt = 0.0               # IMU 기울기 (°)
        self.tilt_over = None         # 기울기 가드: tilt_guard 를 처음 넘은 시각
        self.failed = []              # (local x, y, 실패 시각) 실패한 목표 — 같은 곳(반경 1 m) 3 번 실패하면 이번 세트에는 다시 안 감 (fix13).
        #                               ttl 180 -> 900 s (2026-11-13): 60 s 뒤 재시도와 맞물려 끝없이 되풀이하지 않게 (세트가 바뀌면 비움)
        self.start_map = None         # 마지막 경로 시작(= 그때 로봇) map 좌표 — 추종 실패 자리 기록용 (fix14)
        self.map_off = None           # 마지막 경로 계획 때 (map - odom) 평행이동
        self.fail_spots = []          # 추종 실패한 자리 (map x, y)
        self.closed = []              # 갇힌 자리(같은 자리 추종 실패 2 번): 경로가 closed_radius 안을 지나면 그 목표는 안 감, 이번 세트 내내 (fix14).
        #                               반경 0.8 -> 0.4 (2026-11-12): 0.8 은 폭 1.6 m 문 하나를 통째로 막아 그 너머 목표가 전부 거부됐음
        self.backing = False          # 갇힌 자리에서 빵부스러기로 물러나는 중
        self.nudge_limit = 0.6        # 옮겨 보기 최대 거리 (트인 거리)
        self.crumbs = []              # odom (x, y) 지나온 길, crumb_spacing 마다 — 원위치 계획이 계속 실패하면 되돌아가는 용 (fix13)
        self.odom_xy = None
        self.costmap = None
        self.escape = None            # (gen, 방향 단위벡터 odom, 시작 시각, 시작 odom xy 또는 None) — xy 있으면 옮겨 보기
        self.streak = PlanFailStreak(need=p['nudge_after'])
        self.zero = ZeroCmdWatch(hold=p['zero_cmd_time'])   # FOLLOWING 인데 Nav2 명령 0 이 이어지는지 (record13)
        self.follow_t = 0.0           # 지금 FOLLOWING 시작 시각
        self.resume_t = 0.0           # 세트 탐색을 시작했거나 통신 복구로 다시 시작한 시각 (목표 없음 완료 판정의 기준)
        self.follow_end = 0.0         # 마지막 추종이 끝난 시각 (긴 추종 끝에 실패하면 시작 시각 기준으로는 곧바로 '목표 없음'이 됨)
        self.done_watch = DoneWatch(hold=p['done_after'])
        self.had_goal = False         # 탐색 목표를 한 번이라도 받았는지 (시작 직후 완료 오판 방지)
        self.home_phase = None        # None -> 'HOMING' -> 'DONE'
        self.home_tries = 0
        self.home_next = 0.0
        self.turn = None              # (gen, path, 목표 방향, 시작 시각) TURNING 중

    # ---------- 공통 ----------
    def set_state(self, s, why=''):
        if s != self.state:
            self.log.info(f'[{self.ns}] executor {self.state} -> {s}' + (f' ({why})' if why else ''))
            self.state = s
            self.pub_state.publish(String(data=s))   # 바뀔 때 바로도 (0.5 s 하트비트만이면 짧은 PLANNING 을 기록이 놓침)

    def cancel(self):
        self.gen += 1
        self.backing = False
        if self.handle is not None:
            self.handle.cancel_goal_async()
            self.handle = None

    def blacklist(self, why):
        if self.goal is None:
            return
        shared = self.goal[1]
        if self.home_phase is None:
            p = self.goal[0].pose.position
            self.failed.append((p.x, p.y, sim_now(self.node)))
        self.pub_event.publish(String(data=f'실패 {why}'))
        self.log.warn(f'[{self.ns}] 목표 실패: {why}' +
                      ('' if shared is None else
                       f' -> {self.ns} 블랙리스트 ({shared.pose.position.x:.2f}, {shared.pose.position.y:.2f})'))
        if shared is not None:
            self.pub_bl_robot.publish(shared)    # 계획·추종 실패는 이 로봇 사정 -> 다른 로봇은 계속 갈 수 있게

    # ---------- 입력 ----------
    def on_imu(self, m):
        q = m.orientation
        tilt = math.degrees(math.acos(max(-1.0, min(1.0, 1 - 2 * (q.x * q.x + q.y * q.y)))))
        self.tilt = tilt
        if not self.fallen:
            if tilt > self.p['fall_tilt']:
                self.fallen = True
                self.upright_since = None
                self.cancel()
                self.goal, self.pending = None, None
                self.set_state('FALLEN', f'기울기 {tilt:.0f}° -> 목표 중단, 새 목표 무시 (블랙리스트 안 함)')
        elif tilt < 20.0:
            now = sim_now(self.node)
            if self.upright_since is None:
                self.upright_since = now
            elif now - self.upright_since > 3.0:
                self.fallen = False
                self.set_state('IDLE', '다시 섬')
        else:
            self.upright_since = None

    def on_odom(self, m):
        self.odom_yaw = (yaw_of(m.pose.pose.orientation), sim_now(self.node))
        self.odom_xy = (m.pose.pose.position.x, m.pose.pose.position.y)
        if not self.crumbs or math.hypot(self.odom_xy[0] - self.crumbs[-1][0], self.odom_xy[1] - self.crumbs[-1][1]) >= self.p['crumb_spacing']:
            self.crumbs.append(self.odom_xy)
            del self.crumbs[:-5000]

    def lethal_push(self):
        """(중심 escape_radius 안 치명·내접 칸 수, 0.5 m 안 치명 칸들에서 멀어지는 단위벡터 odom) — 없으면 (0, None)."""
        g, xy = self.costmap, self.odom_xy
        if g is None or xy is None or not g.info.width:
            return 0, None
        w, res = g.info.width, g.info.resolution
        ox, oy = g.info.origin.position.x, g.info.origin.position.y
        near, px, py = 0, 0.0, 0.0
        i0, i1 = max(0, int((xy[0] - 0.5 - ox) / res)), min(w, int((xy[0] + 0.5 - ox) / res) + 1)
        j0, j1 = max(0, int((xy[1] - 0.5 - oy) / res)), min(g.info.height, int((xy[1] + 0.5 - oy) / res) + 1)
        for j in range(j0, j1):
            for i in range(i0, i1):
                c = g.data[j * w + i]
                if c < 99:
                    continue
                dx, dy = xy[0] - (ox + (i + 0.5) * res), xy[1] - (oy + (j + 0.5) * res)
                d = math.hypot(dx, dy)
                if d <= self.p['escape_radius']:
                    near += 1
                if c >= 100 and 1e-3 < d <= 0.5:
                    px, py = px + dx / d ** 2, py + dy / d ** 2
        n = math.hypot(px, py)
        return near, ((px / n, py / n) if n > 1e-6 else None)

    def try_escape(self):
        """추종 실패 뒤: 장애물 테두리 안이면 ESCAPING 시작하고 True."""
        near, v = self.lethal_push()
        if near == 0 or v is None or self.fallen or self.mode != 'NORMAL':
            return False
        self.gen += 1
        self.escape = (self.gen, v, sim_now(self.node), None)
        self.set_state('ESCAPING', f'중심 {self.p["escape_radius"]:.1f} m 안 장애물 테두리 칸 {near} 개 -> '
                                   f'odom ({v[0]:+.2f}, {v[1]:+.2f}) 쪽으로 벗어나기')
        return True

    def try_nudge(self, why=None):
        """같은 자리 계획 실패 연속(또는 why) 뒤: 국소 비용지도에서 가장 트인 쪽으로 옮겨 보기 시작하고 True."""
        g, xy = self.costmap, self.odom_xy
        if g is None or xy is None or self.fallen or self.mode != 'NORMAL':
            return False
        v, d = free_heading(g.data, g.info.width, g.info.height, g.info.resolution,
                            g.info.origin.position.x, g.info.origin.position.y, xy[0], xy[1])
        if v is None or d < 0.15:
            return False
        self.gen += 1
        self.escape = (self.gen, v, sim_now(self.node), xy)
        self.nudge_limit = d - 0.05           # fix19: 트인 거리까지만
        self.set_state('ESCAPING', f'{why or "같은 자리 계획 실패 %d 번" % self.p["nudge_after"]} -> 옮겨 보기 odom ({v[0]:+.2f}, {v[1]:+.2f}) '
                                   f'쪽 (트인 거리 {d:.2f} m)')
        return True

    def robot_cost(self):
        g, xy = self.costmap, self.odom_xy
        if g is None or xy is None:
            return None
        return cell_cost(g.data, g.info.width, g.info.height, g.info.resolution,
                         g.info.origin.position.x, g.info.origin.position.y, xy[0], xy[1])

    def escape_step(self):
        if self.escape is None:
            return
        g, v, t0, xy0 = self.escape
        near, v2 = self.lethal_push()
        el = sim_now(self.node) - t0
        if xy0 is None:       # 벗어나기: 중심이 테두리 밖이면 끝
            done = el > self.p['escape_time'] or near == 0
        else:                 # 옮겨 보기: nudge_dist 움직이면 끝
            moved = dist_xy(self.odom_xy, xy0) if self.odom_xy else 0.0
            # fix19 (2026-11-11): 0.3 m 만 가고 끝내면 팽창 영역 안(로봇 칸 비용 170~253)에 그대로 남아 계획 실패가 반복됨
            # (155419 a1_2 옮겨 보기 4 번 -> 87 % IDLE). nudge_dist 뒤에도 로봇 칸 비용이 nudge_cost 이상이면 트인 거리까지 더 감.
            low = self.robot_cost() is not None and self.robot_cost() < self.p['nudge_cost']
            done = el > self.p['nudge_time'] or moved >= self.nudge_limit or (moved >= self.p['nudge_dist'] and low)
        done = done or g != self.gen or self.state != 'ESCAPING'
        if done or self.odom_yaw is None:
            self.escape = None
            self.pub_cmd.publish(Twist())
            if g == self.gen and self.state == 'ESCAPING':
                what = '벗어나기' if xy0 is None else f'옮겨 보기 ({dist_xy(self.odom_xy, xy0):.2f} m)'
                self.log.info(f'[{self.ns}] {what} 끝 ({el:.1f} s, 중심 테두리 칸 {near} 개)')
                self.finish()
            return
        if xy0 is None:
            v = v2 or v
        tw = Twist()                    # 옆걸음 없이: 앞뒤 축으로 걷거나 그 축을 맞추려 제자리 회전 (nudge_logic.body_cmd)
        tw.linear.x, tw.angular.z = body_cmd(v, self.odom_yaw[0], self.p['escape_speed'], self.p['turn_speed'])
        if tw.linear.x == 0.0 and self.tilt > self.p['turn_max_tilt']:
            # 경사 위 제자리 회전 금지 (record13 넘어짐): 돌지 않고 벗어나기를 끝냄
            self.escape = None
            self.pub_cmd.publish(Twist())
            self.log.warn(f'[{self.ns}] 경사 위 (기울기 {self.tilt:.0f}°) 벗어나기 제자리 회전 안 함 -> 끝')
            self.finish()
            return
        self.pub_cmd.publish(tw)

    def on_shared_goal(self, msg):
        self.last_shared = (msg, sim_now(self.node))

    def on_mission(self, phase, k, zones):
        explore = phase in ('EXPLORE', 'REGROUP')
        if explore and k != self.set_k:           # 새 세트: 탐색 상태를 처음으로
            self.set_k, self.home, self.home_phase, self.home_tries, self.had_goal = k, None, None, 0, False
            self.done_watch = DoneWatch(hold=self.p['done_after'])
            # 목표 없음 완료 타이머는 세트마다 새로: 안 그러면 지난 세트의 마지막 추종 시각 기준이라, 새 세트 첫 계획이 한 번 실패하자마자
            # '목표 없이 90 s' 로 복귀했다 (2026-11-13 공동 건물: a1_2 가 탐색 없이 끝남)
            self.resume_t = sim_now(self.node)
            self.reached, self.failed, self.fail_spots, self.closed, self.crumbs, self.visits = [], [], [], [], [], []
            if self.state == 'DONE':
                self.set_state('IDLE', f'세트 {k} 탐색 시작')
        if self.active and not explore and self.state in ('PLANNING', 'TURNING', 'FOLLOWING', 'ESCAPING'):
            self.cancel()
            self.goal, self.pending = None, None
            self.set_state('IDLE', f'임무 단계 {phase}')
        self.active = explore
        self.zone = zones.get(self.ns)
        self.others = [r for n, r in zones.items() if n != self.ns and r != self.zone]
        if phase == 'REGROUP' and self.home_phase is None:
            self.start_homing('Husky 가 부름 (REGROUP)')

    def start_homing(self, why):
        self.home_phase, self.pending, self.home_next = 'HOMING', None, 0.0
        self.pub_event.publish(String(data='홈 시작'))
        self.log.warn(f'[{self.ns}] {why} -> 탐색 시작 위치로 복귀')
        if self.state in ('PLANNING', 'TURNING', 'FOLLOWING'):
            self.cancel()
            self.finish()

    def on_mode(self, msg):
        prev, self.mode = self.mode, msg.data
        if prev == self.mode:
            return
        if self.fallen:
            return
        if self.mode == 'NORMAL':
            if self.state == 'PAUSED':
                self.set_state('IDLE', '통신 정상')
            self.resume_t = sim_now(self.node)
            if self.home_phase == 'DONE' and self.state == 'IDLE':
                self.set_state('DONE', '탐색 완료 상태 유지')
            return
        # NORMAL 이 아닌 모든 모드: 탐색 중단 (복귀 주행은 return_relay 가 직접)
        if self.state in ('PLANNING', 'TURNING', 'FOLLOWING', 'ESCAPING'):
            self.goal, self.pending = None, None
        self.cancel()
        self.set_state('PAUSED', f'comm_mode={self.mode}')

    def on_local_goal(self, local):
        if not self.active or self.mode != 'NORMAL' or self.fallen or self.home_phase is not None:
            return
        shared = None
        if self.last_shared and sim_now(self.node) - self.last_shared[1] < 1.0:
            shared = self.last_shared[0]
            x, y = shared.pose.position.x, shared.pose.position.y
            # 로봇이 아직 자기 구역 밖이면 구역 밖 목표도 받는다 (goal_assigner 의 접근 목표: 문에서 먼 출발 자리 -> 문 앞)
            off = self.map_off or (0.0, 0.0)
            me = (self.odom_xy[0] + off[0] + x - local.pose.position.x,
                  self.odom_xy[1] + off[1] + y - local.pose.position.y) if self.odom_xy else None
            approaching = self.zone is not None and me is not None and not in_rect(self.zone, me[0], me[1])
            if self.zone is None or not (approaching or in_rect(self.zone, x, y, -self.p['goal_margin'])):
                self.log.warn(f'[{self.ns}] 목표 ({x:.2f}, {y:.2f}) 가 자기 구역 밖(+{self.p["goal_margin"]} m) -> 무시')
                self.pub_bl.publish(shared)
                return
        # 이미 도착한 목표가 다시 오면 그 프런티어는 도착해도 안 지워지는 것 -> 블랙리스트 (같은 자리 무한 반복 방지).
        # 2026-11-07 mission: a1_1 이 0.18 m 앞 목표에 '즉시 도착'을 56 번 반복 (SLAM 이 0.45 m 이동 전엔 지도 갱신 안 함).
        # 도착 직후 reached_grace 초는 지도·공유 지도 갱신(5 s 주기)을 기다리며 무시만 한다.
        now = sim_now(self.node)
        self.reached = [r for r in self.reached if now - r[2] < self.p['reached_ttl']]
        near = [now - r[2] for r in self.reached
                if math.hypot(local.pose.position.x - r[0], local.pose.position.y - r[1]) < self.p['same_goal_tol']]
        if near and self.state not in ('PLANNING', 'TURNING', 'FOLLOWING'):
            if min(near) < self.p['reached_grace']:
                return
            self.log.warn(f'[{self.ns}] 이미 도착한 목표 ({local.pose.position.x:.2f}, {local.pose.position.y:.2f}) '
                          f'{min(near):.0f} s 뒤 다시 옴 (프런티어가 안 지워짐) -> 블랙리스트', throttle_duration_sec=2.0)
            if shared is not None:
                self.pub_bl.publish(shared)
            return
        # 같은 자리(same_goal_tol)에 fail_area_n 번 도착했는데도 그 프런티어가 다시 오면 이번 세트에는 더 안 간다.
        # 물체 뒤처럼 가도 안 지워지는 프런티어를 블랙리스트가 풀릴 때마다 다시 찾아가 세트가 끝나지 않았음
        # (2026-11-13 세트 2 남쪽: 700 s 넘게 같은 자리들을 돌며 도착 18 번)
        if sum(1 for v in self.visits if math.hypot(local.pose.position.x - v[0], local.pose.position.y - v[1])
               < self.p['same_goal_tol']) >= self.p['fail_area_n']:
            self.log.warn(f'[{self.ns}] 목표 ({local.pose.position.x:.2f}, {local.pose.position.y:.2f}) 에 {self.p["fail_area_n"]} 번 '
                          f'도착했지만 프런티어가 남음 -> 이번 세트에는 다시 안 감', throttle_duration_sec=30.0)
            if shared is not None:
                self.pub_bl_robot.publish(shared)
            return
        self.failed = [f for f in self.failed if now - f[2] < self.p['fail_area_ttl']]
        n_fail = sum(1 for f in self.failed
                     if math.hypot(local.pose.position.x - f[0], local.pose.position.y - f[1]) < self.p['fail_area_radius'])
        if n_fail >= self.p['fail_area_n']:
            self.log.warn(f'[{self.ns}] 목표 ({local.pose.position.x:.2f}, {local.pose.position.y:.2f}) 반경 {self.p["fail_area_radius"]:.1f} m 에서 '
                          f'{n_fail} 번 실패 -> 블랙리스트', throttle_duration_sec=2.0)
            if shared is not None:
                self.pub_bl_robot.publish(shared)
            return
        if self.state == 'ESCAPING':          # 벗어난 뒤 시작
            self.pending = (local, shared)
            return
        if self.state in ('PLANNING', 'TURNING', 'FOLLOWING') and self.goal is not None:
            if dist(local, self.goal[0]) < self.p['same_goal_tol']:
                return
            if sim_now(self.node) - self.goal[2] < self.p['min_dwell']:
                self.pending = (local, shared)
                return
            fr = self.frontier_near_goal()
            if fr is not None:
                self.pending = (local, shared)
                self.log.info(f'[{self.ns}] 지금 목표 근처 프런티어 아직 있음 ({fr:.2f} m) -> 목표 유지, 새 목표 보류',
                              throttle_duration_sec=10.0)
                return
            self.log.info(f'[{self.ns}] 새 목표로 교체')
            self.cancel()
        elif self.pending is not None and dist(local, self.pending[0]) < self.p['same_goal_tol']:
            return
        self.start_goal(local, shared)

    def frontier_near_goal(self):
        """목표 유지 조건: 지금 목표(공유 지도 좌표) commit_radius 안 가장 가까운 프런티어 거리, 아니면 None."""
        fr = self.node.frontiers
        if not self.p['commit'] or self.goal is None or self.goal[1] is None or fr is None:
            return None
        pts, t = fr
        if sim_now(self.node) - t > 5.0:
            return None
        gx, gy = self.goal[1].pose.position.x, self.goal[1].pose.position.y
        d = min((math.hypot(x - gx, y - gy) for x, y in pts), default=None)
        return d if d is not None and d < self.p['commit_radius'] else None

    # ---------- 탐색 목표 ----------
    def start_goal(self, local, shared):
        if not self.plan_ac.server_is_ready() or not self.follow_ac.server_is_ready():
            self.log.warn(f'[{self.ns}] Nav2 액션 서버 없음 -> 목표 보류', throttle_duration_sec=5.0)
            self.pending = (local, shared)
            return
        self.gen += 1
        g = self.gen
        self.backing = False
        self.goal, self.pending = (local, shared, sim_now(self.node)), None
        if self.home_phase is None:
            self.had_goal = True
        self.set_state('PLANNING', f'목표 ({local.pose.position.x:.2f}, {local.pose.position.y:.2f}) [map]')
        # record17: 목표가 장애물 팽창 영역 안이면 전역 비용지도로 가까운 여유 칸으로 옮기거나 버림 (refine_goal 설명)
        if self.p['goal_refine'] and self.cm_cli.service_is_ready():
            self.cm_cli.call_async(GetCostmap.Request()).add_done_callback(lambda f: self.on_goal_costmap(g, local, f))
            return
        self.send_plan(g, local)

    def on_goal_costmap(self, g, local, fut):
        if g != self.gen:
            return
        try:
            m = fut.result().map
            md = m.metadata
            cost = np.asarray(m.data, dtype=np.uint8).reshape(md.size_y, md.size_x)
            x, y = local.pose.position.x, local.pose.position.y
            r = refine_goal(cost, md.resolution, md.origin.position.x, md.origin.position.y, x, y,
                            self.p['goal_max_cost'], self.p['goal_refine_radius'])
        except Exception as e:                # 비용지도를 못 읽으면 보정 없이
            self.log.warn(f'[{self.ns}] 목표 보정용 비용지도 실패: {e!r}', throttle_duration_sec=10.0)
            r = (local.pose.position.x, local.pose.position.y, 0.0)
        if r is None:
            self.blacklist(f'목표 주변 {self.p["goal_refine_radius"]} m 안 여유 칸 없음 (장애물 팽창 영역)')
            self.finish()
            return
        if r[2] > 0:
            self.log.info(f'[{self.ns}] 목표가 장애물 팽창 영역 -> {r[2]:.2f} m 옮김 ({r[0]:.2f}, {r[1]:.2f})')
            local.pose.position.x, local.pose.position.y = float(r[0]), float(r[1])
        self.send_plan(g, local)

    def send_plan(self, g, local):
        req = ComputePathToPose.Goal()
        req.goal = local
        req.planner_id = self.p['planner_id']
        req.use_start = False
        self.plan_ac.send_goal_async(req).add_done_callback(lambda f: self.on_plan_accepted(g, f))

    def on_plan_accepted(self, g, fut):
        h = fut.result()
        if g != self.gen:
            return
        if not h.accepted:
            self.blacklist('planner 거절')
            self.finish()
            return
        h.get_result_async().add_done_callback(lambda f: self.on_plan_result(g, f))

    def on_plan_result(self, g, fut):
        if g != self.gen:
            return
        res = fut.result()
        path = res.result.path
        if res.status != GoalStatus.STATUS_SUCCEEDED or len(path.poses) < 2:
            self.blacklist(f'경로 계획 실패 (status {res.status}, {len(path.poses)}점)')
            if self.p['nudge_after'] > 0 and self.odom_xy and self.streak.fail(self.odom_xy) and self.try_nudge():
                return
            self.finish()
            return
        self.streak.ok()
        self.pub_path.publish(path)
        s0 = path.poses[0].pose.position
        self.start_map = (s0.x, s0.y)
        if self.odom_xy is not None:          # map <- odom 평행이동 (경로가 길어도 실패 자리를 map 좌표로 구하려고)
            self.map_off = (s0.x - self.odom_xy[0], s0.y - self.odom_xy[1])
        if self.home_phase is None and self.path_hits_closed(path):
            self.blacklist('폐경로(갇혔던 자리) 지남 -> 안 감')
            self.finish()
            return
        bad = self.path_zone_violation(path)
        if bad is not None:
            self.blacklist(f'경로가 상대 구역으로 감 (월드 {bad[0]:+.2f}, {bad[1]:+.2f})')
            self.finish()
            return
        self.follow_turn_first(path, g)

    def follow_turn_first(self, path, g):
        """경로 앞쪽이 몸 방향과 turn_first 넘게 다르면 먼저 제자리 회전(TURNING), 아니면 바로 추종. 경사 위에서는 회전 생략.
        Nav2 경로와 지나온 길 역추종(물러나기·홈 복귀)이 같이 쓴다 — 역추종은 늘 뒤쪽이라 그냥 보내면 DWB 가 서 있다 실패했음 (2026-11-12)."""
        h = path_heading(path, self.p['turn_sample'])
        if (h is not None and self.odom_yaw is not None and sim_now(self.node) - self.odom_yaw[1] < 1.0
                and abs(angle_diff(h, self.odom_yaw[0])) > self.p['turn_first']):
            if self.tilt > self.p['turn_max_tilt']:
                self.log.info(f'[{self.ns}] 경사 위 (기울기 {self.tilt:.0f}°) -> 먼저 돌기 생략')
                self.send_follow(path, g, 'FOLLOWING')
                return
            self.turn = (g, path, h, sim_now(self.node))
            self.set_state('TURNING', f'경로 방향과 {math.degrees(angle_diff(h, self.odom_yaw[0])):+.0f}° -> 먼저 제자리 회전')
            return
        self.send_follow(path, g, 'FOLLOWING')

    def turn_step(self):
        """TURNING 일 때 10 Hz. 목표 방향 turn_done 안이거나 turn_timeout 지나면 멈추고 FollowPath."""
        if self.turn is None:
            return
        g, path, h, t0 = self.turn
        if g != self.gen or self.state != 'TURNING':   # 취소·교체·넘어짐·복귀
            self.turn = None
            self.pub_cmd.publish(Twist())
            return
        stale = self.odom_yaw is None or sim_now(self.node) - self.odom_yaw[1] > 1.0
        err = 0.0 if stale else angle_diff(h, self.odom_yaw[0])
        timeout = sim_now(self.node) - t0 > self.p['turn_timeout']
        tilted = self.tilt > self.p['turn_max_tilt'] + 2.0
        if stale or timeout or tilted or abs(err) < self.p['turn_done']:
            self.turn = None
            self.pub_cmd.publish(Twist())
            if timeout:
                self.log.warn(f'[{self.ns}] 먼저 돌기 {self.p["turn_timeout"]:.0f} s 초과 (남은 {math.degrees(err):+.0f}°) -> 그대로 추종')
            if tilted:
                self.log.warn(f'[{self.ns}] 도는 중 기울기 {self.tilt:.0f}° -> 돌기 멈추고 추종 (남은 {math.degrees(err):+.0f}°)')
            self.send_follow(path, g, 'FOLLOWING')
            return
        tw = Twist()
        tw.angular.z = math.copysign(self.p['turn_speed'], err)
        self.pub_cmd.publish(tw)

    def send_follow(self, path, g, state):
        goal = FollowPath.Goal()
        goal.path = path
        goal.controller_id = self.p['controller_id']
        goal.goal_checker_id = self.p['goal_checker_id']
        self.set_state(state, f'{len(path.poses)}점 [{path.header.frame_id}]')
        self.follow_t = sim_now(self.node)
        self.follow_ac.send_goal_async(goal).add_done_callback(lambda f: self.on_follow_accepted(g, f))

    def on_follow_accepted(self, g, fut):
        h = fut.result()
        if g != self.gen:
            if h.accepted:
                h.cancel_goal_async()
            return
        if not h.accepted:
            self.on_follow_done(g, GoalStatus.STATUS_ABORTED)
            return
        self.handle = h
        h.get_result_async().add_done_callback(lambda f: self.on_follow_done(g, f.result().status))

    def on_follow_done(self, g, status):
        if g != self.gen:
            return
        self.handle = None
        self.follow_end = sim_now(self.node)
        if self.backing:                       # 물러나기 끝 (성공이든 실패든) -> 다음 목표
            self.backing = False
            self.finish()
            return
        if status == GoalStatus.STATUS_SUCCEEDED and self.home_phase == 'HOMING':
            self.home_phase, self.goal, self.pending = 'DONE', None, None
            self.pub_cmd.publish(Twist())
            self.pub_event.publish(String(data='홈 도착'))
            self.pub_done.publish(Int32(data=self.set_k))
            self.set_state('DONE', '탐색 시작 위치 도착 -> 이번 세트 끝')
            return
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.log.info(f'[{self.ns}] 목표 도착')
            if self.goal is not None:
                p = self.goal[0].pose.position
                self.pub_event.publish(String(data=f'도착 {p.x:.2f} {p.y:.2f}'))
                self.reached.append((p.x, p.y, sim_now(self.node)))
                self.visits.append((p.x, p.y))
        elif status != GoalStatus.STATUS_CANCELED:
            self.blacklist(f'경로 추종 실패 (status {status})')
            if self.note_fail_spot():
                return
            if self.try_escape():
                return
        self.finish()

    def finish(self):
        self.goal = None
        self.set_state('IDLE')
        if self.pending is not None and self.mode == 'NORMAL':
            local, shared = self.pending
            self.start_goal(local, shared)

    # ---------- 주기 ----------
    def tilt_guard(self, now):
        """기울기 > tilt_guard 가 tilt_guard_time 초 이어졌는지. 아래로 내려가면 타이머 리셋."""
        if self.tilt <= self.p['tilt_guard']:
            self.tilt_over = None
            return False
        if self.tilt_over is None:
            self.tilt_over = now
        return now - self.tilt_over >= self.p['tilt_guard_time']

    def tick(self):
        ready = self.follow_ac.server_is_ready()
        if ready:
            self.pub_state.publish(String(data=self.state))
        if self.state in ('PLANNING', 'TURNING', 'FOLLOWING') and self.goal is not None:
            now = sim_now(self.node)
            limit = self.p['home_timeout'] if self.home_phase == 'HOMING' else self.p['goal_timeout']
            if now - self.goal[2] > limit:
                self.cancel()
                self.blacklist(f'{limit:.0f} s 안에 도착 못함')
                self.finish()
            elif self.state == 'FOLLOWING' and self.mode == 'NORMAL' and self.tilt_guard(now):
                # 기울기 가드 (2026-11-10 fix11: a1_1 이 램프 옆면에서 피치 -33°·기울기 41° 로도 계속 추종): 목표 포기, 그 자리에서 멈춤
                self.cancel()
                self.pub_cmd.publish(Twist())
                self.blacklist(f'기울기 {self.tilt:.0f}° 가 {self.p["tilt_guard_time"]:.0f} s 이상 -> 추종 중단')
                self.finish()
            elif self.state == 'FOLLOWING' and self.mode == 'NORMAL' and self.zero.stalled(now, self.follow_t):
                # 0 명령 감시: 추종 중 Nav2 명령이 hold 초 동안 0 -> 목표 포기(로봇별 블랙리스트) + 벗어나기/옮겨 보기
                n = self.zero.zero_for(now, self.follow_t)
                self.cancel()
                self.log.warn(f'[{self.ns}] 0 명령 감시: FOLLOWING 중 Nav2 명령 0 이 {n:.0f} s 이어짐')
                self.blacklist('0 명령 감시')
                if not (self.try_escape() or self.try_nudge(f'Nav2 명령 0 {n:.0f} s')):
                    self.finish()
        elif self.state == 'IDLE' and self.mode == 'NORMAL' and not self.fallen and self.home_phase == 'HOMING':
            if sim_now(self.node) >= self.home_next:
                self.go_home()
        elif (self.active and self.state == 'IDLE' and self.mode == 'NORMAL' and not self.fallen and self.home_phase is None
              and self.had_goal and self.done_watch.done(sim_now(self.node))):
            self.start_homing(f'탐색 완료: 갈 수 있는 자기 구역 프런티어 0 개가 {self.p["done_after"]:.0f} s')
        elif (self.active and self.state == 'IDLE' and self.mode == 'NORMAL' and not self.fallen and self.home_phase is None
              and self.had_goal and sim_now(self.node) - max(self.follow_t, self.follow_end, self.resume_t) > self.p['idle_done']):
            self.start_homing(f'갈 수 있는 목표 없이 {self.p["idle_done"]:.0f} s')
        elif self.state == 'IDLE' and self.pending is not None and self.mode == 'NORMAL':
            local, shared = self.pending
            self.start_goal(local, shared)

    def go_home(self):
        """이번 세트 탐색 시작 위치(self.home, map)로 Nav2. home_retries 번 넘게 실패하면 DONE(홈 실패)."""
        if self.home is None:                      # 아직 못 받음 (husky_mission 이 1 s 마다 보냄)
            self.home_next = sim_now(self.node) + 1.0
            return
        self.home_tries += 1
        if self.home_tries > self.p['home_retries']:
            self.home_phase = 'DONE'
            self.pub_event.publish(String(data='홈 실패'))
            self.pub_done.publish(Int32(data=self.set_k))
            self.set_state('DONE', f'원위치 복귀 {self.p["home_retries"]} 번 실패 -> 그 자리에서 멈춤')
            return
        self.home_next = sim_now(self.node) + 5.0
        if self.home_tries > self.p['home_crumb_after'] and len(self.crumbs) >= 2 and self.follow_ac.server_is_ready():
            self.home_by_crumbs()
            return
        self.home.header.stamp = self.node.get_clock().now().to_msg()
        self.log.info(f'[{self.ns}] 탐색 시작 위치 복귀 시도 {self.home_tries}/{self.p["home_retries"]}')
        self.start_goal(self.home, None)

    def path_hits_closed(self, path):
        """경로가 갇혔던 자리(closed) 반경 closed_radius 안을 지나면 True. 로봇이 이미 그 근처면(나가는 길) 검사 안 함."""
        if not self.closed or self.start_map is None:
            return False
        r = self.p['closed_radius']
        for cx, cy in self.closed:
            if math.hypot(self.start_map[0] - cx, self.start_map[1] - cy) < r + 0.3:
                continue
            for q in path.poses[::3]:
                if math.hypot(q.pose.position.x - cx, q.pose.position.y - cy) < r:
                    return True
        return False

    def note_fail_spot(self):
        """추종 실패 자리 기록. 같은 자리(반경 0.5 m)에서 2 번째면 갇힌 자리로 닫고(closed) 빵부스러기로 back_dist 물러나기 시작 -> True."""
        if self.map_off is None or self.odom_xy is None or self.home_phase is not None or self.fallen or self.mode != 'NORMAL':
            return False
        x, y = self.odom_xy[0] + self.map_off[0], self.odom_xy[1] + self.map_off[1]    # 실패한 지금 로봇 자리 (map). 경로 시작이 아님
        n = sum(1 for f in self.fail_spots if math.hypot(x - f[0], y - f[1]) < 0.5)
        self.fail_spots.append((x, y))
        if n < 1:
            return False
        if not any(math.hypot(x - c[0], y - c[1]) < 0.5 for c in self.closed):
            self.closed.append((x, y))
        if len(self.crumbs) < 3 or not self.follow_ac.server_is_ready():
            return False
        pts, acc = [self.crumbs[-1]], 0.0
        for a in reversed(self.crumbs[:-1]):
            acc += math.hypot(a[0] - pts[-1][0], a[1] - pts[-1][1])
            pts.append(a)
            if acc >= self.p['back_dist']:
                break
        if acc < 0.3:
            return False
        path = Path()
        path.header.frame_id = 'odom'
        path.header.stamp = self.node.get_clock().now().to_msg()
        for px, py in pts:
            ps = PoseStamped()
            ps.header = path.header
            ps.pose.position.x, ps.pose.position.y, ps.pose.orientation.w = px, py, 1.0
            path.poses.append(ps)
        self.gen += 1
        self.backing = True
        self.log.warn(f'[{self.ns}] 같은 자리 추종 실패 {n + 1} 번 -> 폐경로로 닫고 지나온 길 {acc:.1f} m 물러남')
        self.pub_path.publish(path)
        self.follow_turn_first(path, self.gen)
        return True

    def home_by_crumbs(self):
        """원위치 계획이 home_crumb_after 번 넘게 실패: 지나온 길(odom)을 거꾸로 FollowPath (갇힌 구역에서 들어온 길로 나옴)."""
        path = Path()
        path.header.frame_id = 'odom'
        path.header.stamp = self.node.get_clock().now().to_msg()
        for x, y in reversed(self.crumbs):
            ps = PoseStamped()
            ps.header = path.header
            ps.pose.position.x, ps.pose.position.y, ps.pose.orientation.w = x, y, 1.0
            path.poses.append(ps)
        self.gen += 1
        self.goal = (self.home, None, sim_now(self.node))
        self.log.warn(f'[{self.ns}] 원위치 계획 {self.home_tries - 1} 번 실패 -> 지나온 길 {len(path.poses)}점 거꾸로 추종')
        self.pub_path.publish(path)
        self.follow_turn_first(path, self.gen)

    def path_zone_violation(self, path):
        """경로가 다른 로봇 구역으로 들어가면 그 월드 (x, y), 아니면 None. 공유 목표가 없거나 탐색 시작 위치 복귀 중이면 검사 안 함."""
        if not self.others or self.home_phase is not None or self.goal is None or self.goal[1] is None or self.p['path_zone_margin'] < 0:
            return None
        local, shared = self.goal[0], self.goal[1]
        dx = shared.pose.position.x - local.pose.position.x      # 로봇 map -> 공유 지도(월드): 평행 이동 (shared_map_merger)
        dy = shared.pose.position.y - local.pose.position.y
        pts = [(ps.pose.position.x + dx, ps.pose.position.y + dy) for ps in path.poses]
        for r in self.others:
            hit = path_enters_rect(pts, r, self.p['path_zone_margin'])
            if hit is not None:
                return hit
        return None


class NavExecutorNode(Node):
    def __init__(self):
        super().__init__('nav_executor')
        robots = list(self.declare_parameter('robots', ['a1_1', 'a1_2']).value)
        defaults = dict(same_goal_tol=0.5, min_dwell=8.0, goal_timeout=90.0, goal_margin=0.2,
                        planner_id='GridBased', reached_ttl=60.0, reached_grace=8.0, fall_tilt=45.0,
                        controller_id='FollowPath', goal_checker_id='general_goal_checker',
                        turn_first=0.785, turn_done=0.35, turn_speed=0.5, turn_sample=0.5, turn_timeout=30.0,
                        turn_max_tilt=6.0, commit=True, commit_radius=1.0,
                        escape_radius=0.1, escape_speed=0.08, escape_time=10.0,
                        nudge_after=3, nudge_dist=0.3, nudge_cost=128, nudge_time=20.0, zero_cmd_time=8.0,
                        done_after=30.0, home_timeout=300.0, home_retries=8, path_zone_margin=0.15,
                        tilt_guard=25.0, tilt_guard_time=1.0, crumb_spacing=0.2, closed_radius=0.4, back_dist=1.0, idle_done=90.0, fail_area_radius=1.0, fail_area_n=3, fail_area_ttl=900.0, home_crumb_after=3, goal_refine=True, goal_max_cost=127, goal_refine_radius=0.5)
        p = {k: type(v)(self.declare_parameter(k, v).value) for k, v in defaults.items()}
        self.frontiers = None      # ([(x, y)], 받은 시각) 공유 지도 = 월드
        self.create_subscription(PoseArray, '/frontier_candidates_comm', lambda m: setattr(
            self, 'frontiers', ([(q.position.x, q.position.y) for q in m.poses], sim_now(self))), 5)
        self.execs = [Executor(self, ns, p) for ns in robots]
        self.create_subscription(String, '/mission/state', self.on_mission, LATCHED)
        self.create_timer(0.5, lambda: [e.tick() for e in self.execs])
        self.create_timer(0.1, lambda: [(e.turn_step(), e.escape_step()) for e in self.execs])
        self.get_logger().info(f'nav_executor: robots={robots}, params={p}')

    def on_mission(self, msg):
        d = json.loads(msg.data)
        for e in self.execs:
            e.on_mission(d['phase'], d['set'], d.get('zones', {}))


def main(args=None):
    rclpy.init(args=args)
    node = NavExecutorNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except Exception:
        if rclpy.ok():
            raise                      # 실행 중 오류는 그대로 보여 준다
        # 종료 신호로 컨텍스트가 닫힌 뒤 타이머가 발행하다 난 RCLError 는 무시
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
