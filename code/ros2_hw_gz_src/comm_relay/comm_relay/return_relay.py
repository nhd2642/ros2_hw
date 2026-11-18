"""복귀 노드 (로봇 쪽 로직). 상태/규칙은 return_logic.py 참고.

입력 : /<ns>/comm_status (Bool 하트비트), /<ns>/odom (EKF, odom 프레임),
       /<ns>/scan (LaserScan, 길 위 장애물 검사. 최근 2 스캔 모두 되짚을 길 띠 안 점 min_points 개 이상이면 막힘)
출력 : /<ns>/cmd_vel_return  Twist  — 직접 복귀 주행/정지 명령 (cmd_vel_arbiter 가 우선순위로 CHAMP 에 넘김)
       /<ns>/comm_mode       String — WAIT/NORMAL/RETURNING/HOLD/STOPPING (바뀔 때 + 1 Hz, transient local)
       /<ns>/return_events   String — 상태 전이·장애물 정지/재개 등 사건 (로그와 같은 문장, 시험·모니터용)
시간은 벽시계(monotonic) 기준: los_checker 와 같게 둬서 시뮬 실시간 비율과 무관하게 timeout 이 일정.
"""
import json
import math

import rclpy
from rclpy.executors import ExternalShutdownException
from geometry_msgs.msg import Twist
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, String

from comm_relay.return_logic import RETURNING, ReturnLogic, path_hits, scan_points

def sim_now(node):
    """노드 시계(초). use_sim_time 이면 시뮬 시간 — 2026-11-10 벽시계(time.monotonic) 대신 (RTF 0.1~0.3 에서 한도가 시뮬 기준 3~7 배 엄격했음)"""
    return node.get_clock().now().nanoseconds * 1e-9


LATCHED = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                     durability=DurabilityPolicy.TRANSIENT_LOCAL)


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class Robot:
    def __init__(self, node, ns, p, obs):
        self.node, self.ns = node, ns
        self.logic = ReturnLogic(**p)
        self.obs = obs                     # 장애물 검사 설정 dict, None 이면 검사 안 함
        self.scans = []                    # 최근 스캔 (수신 시각, msg) 2 개
        self.scan_warned = False
        self.last_mode_pub = 0.0
        out = node.get_parameter('output_topic').value
        node.create_subscription(Bool, f'/{ns}/comm_status', self.on_comm, 10)
        node.create_subscription(Odometry, f'/{ns}/odom', self.on_odom, 20)
        if obs is not None:
            node.create_subscription(LaserScan, f'/{ns}/{obs["scan_topic"]}', self.on_scan, qos_profile_sensor_data)
        self.pub_cmd = node.create_publisher(Twist, f'/{ns}/{out}', 10)
        self.pub_mode = node.create_publisher(String, f'/{ns}/comm_mode', LATCHED)
        self.pub_event = node.create_publisher(String, f'/{ns}/return_events', 10)
        self.prev_state = None

    def on_comm(self, msg):
        if msg.data:
            self.logic.on_heartbeat(sim_now(self.node))

    def on_odom(self, msg):
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        self.logic.on_pose(p.x, p.y, yaw_of(q), sim_now(self.node))

    def on_scan(self, msg):
        self.scans = (self.scans + [(sim_now(self.node), msg)])[-2:]

    def blocked(self):
        """최근 2 스캔 모두, 앞으로 되짚을 길(지나온 breadcrumb 꺾은선, 앞 look_ahead m) 띠 안에
        점이 min_points 개 이상이면 True. 스캔이 없거나 1 s 넘게 끊기면 False (경고 1 회, 검사 없이 복귀)."""
        o = self.obs
        now = sim_now(self.node)
        if len(self.scans) < 2 or now - self.scans[-1][0] > 1.0:
            if not self.scan_warned:
                self.node.get_logger().warn(f'[{self.ns}] 스캔 없음 -> 길 위 장애물 검사 없이 복귀')
                self.scan_warned = True
            return False
        path = self.logic.upcoming(o['look_ahead'])
        x, y, yaw = self.logic.pose
        reach = o['look_ahead'] + o['half_width'] + 0.05
        hits = 0
        for _, m in self.scans:
            if o['ignore_self']:
                pts = scan_points(m.ranges, m.angle_min, m.angle_increment, max(m.range_min, 0.12), m.range_max,
                                  x, y, yaw, reach)
            else:                               # 시험 대조용: 몸 상자·range_min 값 거르지 않음 (이전 동작)
                pts = scan_points(m.ranges, m.angle_min, m.angle_increment, m.range_min - 0.02, m.range_max,
                                  x, y, yaw, reach, self_box=None)
            hits = path_hits(pts, path, o['half_width'], collect=True)
            if len(hits) < o['min_points']:
                return False
        # 사건 문장용: 로봇 odom 위치, 잡힌 점 수, 가장 가까운 점의 거리·방위(로봇 앞 0°, 왼쪽 +)
        r, b = min((math.hypot(px - x, py - y), math.degrees(math.atan2(py - y, px - x) - yaw)) for px, py in hits)
        b = (b + 180) % 360 - 180
        self.logic.block_note = (f', 로봇 odom ({x:.2f}, {y:.2f}) yaw {math.degrees(yaw):.0f}°, 띠 안 {len(hits)}점, '
                                 f'가장 가까운 점 {r:.2f} m 방위 {b:.0f}°')
        return True

    def tick(self):
        t = sim_now(self.node)
        cmd = self.logic.step(t, blocked=self.blocked if self.obs is not None else None)
        for _, s in self.logic.events:
            self.node.get_logger().warn(f'[{self.ns}] {s}')
            self.pub_event.publish(String(data=s))
        self.logic.events.clear()

        if cmd is not None:
            tw = Twist()
            tw.linear.x, tw.angular.z = float(cmd[0]), float(cmd[1])
            self.pub_cmd.publish(tw)
        st = self.logic.state
        if st != self.prev_state or t - self.last_mode_pub >= 1.0:
            self.pub_mode.publish(String(data=st))
            self.prev_state, self.last_mode_pub = st, t
        if st == RETURNING and cmd is not None and cmd != (0.0, 0.0):
            tx, ty = self.logic.target()
            x, y, _ = self.logic.pose
            self.node.get_logger().info(
                f'[{self.ns}] RETURN ACTIVE target ({tx:.2f}, {ty:.2f}) '
                f'dist {math.hypot(tx - x, ty - y):.2f} m, crumbs {len(self.logic.crumbs)}',
                throttle_duration_sec=2.0)


class ReturnRelayNode(Node):
    def __init__(self):
        super().__init__('return_relay')
        robots = list(self.declare_parameter('robots', ['a1_1', 'a1_2']).value)
        self.declare_parameter('output_topic', 'cmd_vel_return')
        defaults = dict(comm_timeout=1.0, odom_timeout=0.5, spacing=0.10, reach_tol=0.25,
                        max_breadcrumbs=5000, vx=0.15, wz_turn=0.4, wz_max=0.4, k_heading=1.0,
                        turn_threshold=0.35, stop_duration=1.0, jump_threshold=0.5, path_points=300)
        p = {k: type(v)(self.declare_parameter(k, v).value) for k, v in defaults.items()}
        obs_defaults = dict(obstacle_check=True, scan_topic='scan', half_width=0.15, look_ahead=0.5, min_points=3,
                            ignore_self=True)
        obs = {k: type(v)(self.declare_parameter(k, v).value) for k, v in obs_defaults.items()}
        self.robots = [Robot(self, ns, p, obs if obs['obstacle_check'] else None) for ns in robots]
        self.create_subscription(String, '/mission/state', self.on_mission, LATCHED)
        self.create_timer(0.1, self.tick)
        self.get_logger().info(f'return_relay: robots={robots}, params={p}, obstacle={obs}')

    def on_mission(self, msg):
        explore = json.loads(msg.data).get('phase') in ('EXPLORE', 'REGROUP')
        for r in self.robots:
            r.logic.record = explore

    def tick(self):
        for r in self.robots:
            r.tick()


def main(args=None):
    rclpy.init(args=args)
    node = ReturnRelayNode()
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
