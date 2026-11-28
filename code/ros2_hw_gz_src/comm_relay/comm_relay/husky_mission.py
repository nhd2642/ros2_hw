"""Husky 임무 노드 (사용자가 여는 노드): 시작 명령·통신 0/1, 단계 진행, 구역 지정, 사람 집계.

사용자 요청 (mission_interfaces):
  action  /husky/run_mission  (RunMission)   : 임무 시작. 피드백 = 단계·세트·찾은 수, 결과 = 생존자 목록. 취소하면 그 자리에서 DONE.
    ros2 action send_goal --feedback /husky/run_mission mission_interfaces/action/RunMission "{num_sets: 0}"
  service /husky/set_comm     (SetComm)      : 통신 켜기/끄기. 끄면 A1 은 이번 세트 탐색 시작 위치로 복귀, Husky 는 정지
    ros2 service call /husky/set_comm mission_interfaces/srv/SetComm "{enabled: false}"
  service /husky/get_survivors (GetSurvivors): 지금까지 확인한 생존자 (세트 이름을 주면 그 세트 구역 안만)
  이 노드를 띄운 터미널에서 start / 0 / 1 을 쳐도 된다 (start = 목표 없는 임무 시작, 끝까지).

단계 (세트 = config 의 sets 순서: set1, set2, set3, set4, common):
  IDLE --start--> TRANSIT k : Husky 가 세트 k 주차점으로 주행 (husky_driver), A1 두 대는 뒤따름 (a1_follower).
                              A1 이 자기 자리에서 lag_stop 넘게 처지면 Husky 가 기다린다.
  TRANSIT --Husky 도착 + A1 둘 다 자리 도착--> EXPLORE k : 그때 A1 위치를 '탐색 시작 위치(홈)'로 기록, 구역(사각형)을 내림.
                              각 A1 은 자기 구역을 다 그리면 스스로 홈으로 돌아와 /<ns>/set_done = k 를 낸다 (nav_executor).
  EXPLORE --둘 다 set_done--> 다음 세트 TRANSIT (요청한 마지막 세트였으면 DONE -> 액션 결과). 사람 수가 설정(people)보다 적으면 '미발견'으로 남기고 진행
                              (지도는 끝났으므로 더 볼 곳이 없음).
  EXPLORE --explore_timeout--> REGROUP k : 남은 A1 도 홈으로 부름. 둘 다 오거나 regroup_timeout 이면 다음 세트.
출력: /mission/state (MissionState, latched)
      /husky/goal, /husky/go -> husky_driver      /<ns>/comm_status 하트비트 (통신 1 일 때만) -> return_relay, survivor_detector
      /<ns>/home_shared (탐색 시작 위치, EXPLORE 2 s 뒤 A1 위치) -> nav_goal_bridge -> nav_executor      /husky/survivors (SurvivorList, latched) 사람 목록(월드)
입력: /husky/survivor_report (SurvivorReport, 각 A1 의 survivor_detector)
      /<ns>/map_anchor (Pose2D, latched, nav_goal_bridge): 그 로봇 map -> 월드 기준 (사람 보고 좌표 변환)
"""
import math
import os
import sys
import threading

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Point, Pose2D, PoseStamped
from mission_interfaces.action import RunMission
from mission_interfaces.msg import MissionState, SurvivorList, SurvivorReport, Zone
from mission_interfaces.srv import GetSurvivors, SetComm
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.task import Future
from std_msgs.msg import Bool, Float32, Int32, String

from comm_relay.mission_logic import SurvivorTally, in_rect, start_poses

LATCHED = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)


def xy(m):
    return m.pose.position.x, m.pose.position.y


class HuskyMission(Node):
    def __init__(self):
        super().__init__('husky_mission')
        config = self.declare_parameter('config', '').value or os.path.join(
            get_package_share_directory('disaster_sim'), 'config', 'mission.yaml')
        with open(config) as f:
            cfg = yaml.safe_load(f)
        self.k = int(self.declare_parameter('start_set', 0).value)
        self.sets, self.slots, self.spawn = cfg['sets'], cfg['slots'], start_poses(cfg, self.k)
        self.anchor = {}            # 로봇 -> (x, y, theta) /<ns>/map_anchor. 받기 전에는 스폰 평행 이동만
        self.robots = list(self.slots)
        self.p = {k: float(self.declare_parameter(k, v).value) for k, v in dict(
            explore_timeout=1500.0, regroup_timeout=300.0, arrive_tol=0.6, arrive_wait=60.0, lag_stop=2.5, lag_go=1.5).items()}
        self.phase, self.t_phase, self.comm, self.go = 'IDLE', 0.0, True, True
        self.driver, self.t_arrived, self.husky = None, None, None
        self.pose, self.done, self.home, self.seen_err = {}, {}, {}, {}
        self.tally = SurvivorTally()
        self.k_end = len(self.sets)              # 이번 임무가 끝낼 세트 (RunMission.num_sets)
        self.goal, self.result, self.t_start = None, None, None   # 진행 중인 액션 목표, 끝나면 채우는 Future, 시작 시각

        self.pub_state = self.create_publisher(MissionState, '/mission/state', LATCHED)
        self.pub_goal = self.create_publisher(PoseStamped, '/husky/goal', 10)
        self.pub_go = self.create_publisher(Bool, '/husky/go', 10)
        self.pub_people = self.create_publisher(SurvivorList, '/husky/survivors', LATCHED)
        self.pub_hb = {ns: self.create_publisher(Bool, f'/{ns}/comm_status', 10) for ns in self.robots}
        self.pub_home = {ns: self.create_publisher(PoseStamped, f'/{ns}/home_shared', 10) for ns in self.robots}
        self.create_subscription(String, '/husky/driver_state', lambda m: setattr(self, 'driver', m.data), LATCHED)
        self.create_subscription(PoseStamped, '/husky/pose', lambda m: setattr(self, 'husky', m), 10)
        self.create_subscription(SurvivorReport, '/husky/survivor_report', self.on_report, 10)
        for ns in self.robots:
            self.create_subscription(PoseStamped, f'/{ns}/pose_shared', lambda m, n=ns: self.pose.__setitem__(n, xy(m)), 10)
            self.create_subscription(Int32, f'/{ns}/set_done', lambda m, n=ns: self.done.__setitem__(n, m.data), LATCHED)
            # 사람 좌표 변환용 기준 (nav_goal_bridge 가 내보냄: 월드 = R(theta)·map + (x, y), 평행 이동·방향 보정 포함, R1 2026-11-28)
            self.create_subscription(Pose2D, f'/{ns}/map_anchor', lambda m, n=ns: self.anchor.__setitem__(n, (m.x, m.y, m.theta)), LATCHED)
            self.create_subscription(Float32, f'/{ns}/slot_error', lambda m, n=ns: self.seen_err.__setitem__(n, (m.data, self.now())), 10)
        ActionServer(self, RunMission, '/husky/run_mission', self.execute, goal_callback=self.on_goal,
                     cancel_callback=lambda _: CancelResponse.ACCEPT)
        self.create_service(SetComm, '/husky/set_comm', self.on_set_comm)
        self.create_service(GetSurvivors, '/husky/get_survivors', self.on_get_survivors)
        self.create_timer(0.2, self.tick)
        self.create_timer(1.0, self.publish_state)
        self.publish_state()
        self.get_logger().info(f"Husky 임무 대기: 세트 {[s['name'] for s in self.sets]}, 시작 세트 {self.k}. "
                               f"요청: 액션 /husky/run_mission, 서비스 /husky/set_comm·/husky/get_survivors (터미널: start / 0 / 1)")

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    # ---------- 사용자 요청 ----------
    def start(self, num_sets):
        """IDLE 일 때만: 지금 세트부터 num_sets 개 (0 = 끝까지) 임무 시작."""
        if self.phase != 'IDLE':
            return False
        self.k_end = len(self.sets) if num_sets == 0 else min(len(self.sets), self.k + num_sets)
        self.t_start = self.now()
        self.set_phase('TRANSIT')
        return True

    def set_comm(self, on):
        self.comm = on
        self.get_logger().warn(f"[COMM] 통신 {'켬' if self.comm else '끔 -> A1 탐색 시작 위치로 복귀, Husky 정지'}")
        self.publish_state()

    def command(self, text):
        """터미널 입력 (start / 0 / 1)."""
        c = text.strip().lower()
        if c == 'start' and self.goal is None and self.start(0):
            return
        if c in ('0', '1'):
            self.set_comm(c == '1')
        else:
            self.get_logger().warn(f'명령 무시 {text!r} (start / 0 / 1, 지금 단계 {self.phase})')

    def on_goal(self, req):
        ok = self.phase == 'IDLE' and self.goal is None
        if not ok:
            self.get_logger().warn(f'임무 요청 거절: 이미 진행 중 또는 끝남 (단계 {self.phase})')
        return GoalResponse.ACCEPT if ok else GoalResponse.REJECT

    async def execute(self, gh):
        """RunMission: 임무를 시작하고 끝날 때(DONE)까지 기다렸다가 결과를 돌려준다. 피드백은 publish_state 가 1 s 마다."""
        self.goal, self.result = gh, Future()
        if not self.start(gh.request.num_sets):           # on_goal 과 사이에 터미널 start 가 끼어든 경우
            self.result.set_result((False, f'시작 못 함 (단계 {self.phase})'))
        ok, why = await self.result
        res = RunMission.Result(success=ok, message=why, survivors=self.survivor_list(), found=len(self.tally.people),
                                expected=self.expected(self.sets[self.k0():self.k]),
                                duration=float(self.now() - (self.t_start or self.now())))
        if gh.is_cancel_requested:
            gh.canceled()
        elif ok:
            gh.succeed()
        else:
            gh.abort()
        self.goal = None
        return res

    def k0(self):
        return int(self.get_parameter('start_set').value)

    def expected(self, sets):
        return sum(int(s['people']) for s in sets)

    def survivor_list(self):
        m = SurvivorList(people=[Point(x=float(x), y=float(y)) for x, y, _ in self.tally.people])
        m.header.frame_id, m.header.stamp = 'world', self.get_clock().now().to_msg()
        return m

    def on_set_comm(self, req, res):
        self.set_comm(req.enabled)
        res.success, res.message = True, f"통신 {'켬' if req.enabled else '끔'}"
        return res

    def on_get_survivors(self, req, res):
        res.survivors, res.success = self.survivor_list(), True
        if not req.set_name:
            res.expected = self.expected(self.sets)
            res.message = f'전체 {len(res.survivors.people)} 명 확인 (설정 {res.expected} 명)'
            return res
        s = next((s for s in self.sets if s['name'] == req.set_name), None)
        if s is None:
            res.success, res.message, res.survivors.people = False, f"세트 이름 모름 {req.set_name!r} ({[s['name'] for s in self.sets]})", []
            return res
        rects = list(s['zones'].values())
        res.survivors.people = [p for p in res.survivors.people if any(in_rect(r, p.x, p.y, 0.5) for r in rects)]   # tally.count 와 같은 여유
        res.expected = int(s['people'])
        res.message = f"{req.set_name}: {len(res.survivors.people)} 명 확인 (설정 {res.expected} 명)"
        return res

    # ---------- 단계 ----------
    def set_phase(self, phase):
        s = self.sets[self.k] if self.k < len(self.sets) else None
        self.get_logger().warn(f"[MISSION] {self.phase} -> {phase}" + (f" ({s['name']})" if s and phase != 'DONE' else ''))
        self.phase, self.t_phase, self.t_arrived = phase, self.now(), None
        if phase != 'REGROUP':
            self.home = {}
        self.publish_state()
        if phase == 'DONE' and self.result is not None and not self.result.done():
            self.result.set_result((self.k >= self.k_end, '요청한 세트 모두 마침' if self.k >= self.k_end else '취소로 중단'))

    def publish_state(self):
        s = self.sets[min(self.k, len(self.sets) - 1)]
        zones = s['zones'] if self.phase in ('EXPLORE', 'REGROUP') else {}
        st = MissionState(phase=self.phase, set_index=self.k, set_name=s['name'], comm=self.comm, zones=[
            Zone(robot=ns, x_min=float(r[0]), x_max=float(r[1]), y_min=float(r[2]), y_max=float(r[3])) for ns, r in zones.items()])
        self.pub_state.publish(st)
        if self.goal is not None:
            self.goal.publish_feedback(RunMission.Feedback(state=st, found=len(self.tally.people),
                                                           expected=self.expected(self.sets[self.k0():min(self.k + 1, self.k_end)])))
        if self.phase == 'TRANSIT':
            g = PoseStamped()
            g.header.frame_id, g.pose.position.x, g.pose.position.y = 'world', float(s['park'][0]), float(s['park'][1])
            self.pub_goal.publish(g)
        for ns, (x, y) in (self.home.items() if self.phase in ('EXPLORE', 'REGROUP') else ()):
            h = PoseStamped()
            h.header.frame_id, h.pose.position.x, h.pose.position.y, h.pose.orientation.w = 'shared_map', x, y, 1.0
            self.pub_home[ns].publish(h)

    def slot_error(self, ns):
        """A1 과 자기 추종 자리 사이 거리 (모르면 None). a1_follower 가 라이다로 Husky 를 보고 잰 값이 있으면 그것 (SLAM 자세 오차와 무관)."""
        e, t = self.seen_err.get(ns, (None, -9.0))
        if self.now() - t < 1.0:
            return e
        if self.husky is None or ns not in self.pose:
            return None
        q = self.husky.pose.orientation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        hx, hy = xy(self.husky)
        dx, dy = self.slots[ns]
        sx, sy = hx + math.cos(yaw) * dx - math.sin(yaw) * dy, hy + math.sin(yaw) * dx + math.cos(yaw) * dy
        return math.hypot(self.pose[ns][0] - sx, self.pose[ns][1] - sy)

    def tick(self):
        now = self.now()
        if self.goal is not None and self.goal.is_cancel_requested and self.phase != 'DONE':
            self.get_logger().warn('[MISSION] 임무 취소 요청')
            self.set_phase('DONE')
        if self.comm:
            for p in self.pub_hb.values():
                p.publish(Bool(data=True))
        if self.phase == 'TRANSIT':
            lag = [e for e in map(self.slot_error, self.robots) if e is not None]
            worst = max(lag, default=0.0)
            go = self.comm and worst < (self.p['lag_stop'] if self.go else self.p['lag_go'])
            if go != self.go:
                self.get_logger().info(f"Husky {'출발' if go else '대기'} (통신 {int(self.comm)}, A1 최대 처짐 {worst:.1f} m)")
            self.go = go
            if self.driver == 'ARRIVED' and now - self.t_phase > 1.0:
                self.t_arrived = self.t_arrived or now
                if (len(lag) == len(self.robots) and worst < self.p['arrive_tol']) or now - self.t_arrived > self.p['arrive_wait']:
                    self.set_phase('EXPLORE')
        elif self.phase in ('EXPLORE', 'REGROUP'):
            if not self.home and now - self.t_phase > 2.0:      # 탐색 시작 위치 = 지금 A1 위치 (자세 보정이 반영된 뒤에 기록)
                self.home = dict(self.pose)
            limit = self.p['explore_timeout'] if self.phase == 'EXPLORE' else self.p['regroup_timeout']
            if all(self.done.get(ns) == self.k for ns in self.robots) or (self.phase == 'REGROUP' and now - self.t_phase > limit):
                self.finish_set()
            elif now - self.t_phase > limit:
                self.set_phase('REGROUP')
        self.pub_go.publish(Bool(data=self.phase == 'TRANSIT' and self.go))

    def finish_set(self):
        s = self.sets[self.k]
        found = self.tally.count(s['zones'].values())
        back = [ns for ns in self.robots if self.done.get(ns) == self.k]
        self.get_logger().warn(f"[MISSION] {s['name']} 완료: 사람 {found}/{s['people']} 명"
                               + ('' if found >= s['people'] else f" (미발견 {s['people'] - found})")
                               + f", 복귀한 A1 {back}, {self.now() - self.t_phase:.0f} s")
        self.k += 1
        self.set_phase('TRANSIT' if self.k < self.k_end else 'DONE')

    # ---------- 사람 보고 ----------
    def on_report(self, msg):
        if msg.robot not in self.spawn:
            self.get_logger().warn(f'사람 보고 무시 (모르는 로봇 {msg.robot!r})')
            return
        ox, oy, th = self.anchor.get(msg.robot, (self.spawn[msg.robot][0], self.spawn[msg.robot][1], 0.0))   # 보고 좌표 = 그 로봇 map
        c, si = math.cos(th), math.sin(th)
        x, y = c * msg.position.x - si * msg.position.y + ox, si * msg.position.x + c * msg.position.y + oy
        if self.tally.add(x, y):
            self.get_logger().warn(f"[HUSKY] 사람 발견 보고: {msg.robot} 월드 ({x:.1f}, {y:.1f}), 지금까지 {len(self.tally.people)} 명")
        self.pub_people.publish(self.survivor_list())


def main(args=None):
    rclpy.init(args=args)
    node = HuskyMission()
    if sys.stdin and sys.stdin.isatty():              # 이 노드를 띄운 터미널에서 바로 명령 입력
        threading.Thread(target=lambda: [node.command(line) for line in sys.stdin if line.strip()], daemon=True).start()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
