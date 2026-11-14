"""A1 의 Husky 추종 (이동 단계 TRANSIT 전용): Husky 뒤 자기 자리(slot)를 향해 걷고, 2D 라이다로 장애물·건물·Husky 를 피한다.

조향은 Husky 주행과 같은 drive_logic.Steer. Nav2 는 쓰지 않는다 (움직이는 목표를 다시 계획하면 가다 서다 함).
Husky 위치는 통신으로 받은 자세로 어림잡고, 그 근처 라이다 점 묶음(drive_logic.find_body)으로 고쳐 쓴다: 자기 SLAM 자세가 어긋나 있어도
  눈앞의 Husky 기준으로 자리를 잡는다. 못 찾으면(가려짐·너무 멂) 받은 자세 그대로.
  /<ns>/slot_error (Float32) = 그렇게 본 자기 자리까지 거리 -> husky_mission 이 기다림·도착 판정에 쓴다.
  /<ns>/pose_fix (PointStamped, 월드 dx dy) = 이동이 끝나 탐색 단계로 넘어갈 때 한 번: 'Husky 기준으로 보면 내 공유 자세가 이만큼 틀렸다'.
    nav_goal_bridge(지도 병합)가 그만큼 정렬을 옮겨, 세트마다 구역 사각형(월드)과 A1 지도가 다시 맞는다 (Husky 자세 추정은 40 m 에 0.15 m 안).
입력 : /mission/state (phase 가 TRANSIT 일 때만 동작), /husky/pose (월드), /<ns>/pose_shared (SLAM 자세, 월드 정렬), /<ns>/scan
출력 : /<ns>/cmd_vel_follow -> cmd_vel_arbiter (우선순위 최하: 조종·사람 발견 정지·통신 복귀가 먼저)
Husky 자세가 1 s 넘게 안 오거나 자기 자세·스캔이 끊기면 선다.
"""
import json
import math

import rclpy
from geometry_msgs.msg import PointStamped, PoseStamped, Twist
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32, String

from comm_relay.drive_logic import Steer, find_body, scan_xy

LATCHED = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
BODY = (0.25, 0.30, 0.17)      # 라이다 기준 A1 몸 상자 (앞, 뒤, 반폭) — 자기 다리·몸통 점 제외 (return_logic.SELF_BOX 와 같음)


def pose2d(m):
    q = m.pose.orientation
    return m.pose.position.x, m.pose.position.y, math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


class Follower:
    def __init__(self, node, ns, slot, v_max):
        self.node, self.ns, self.slot = node, ns, slot
        # 띠 반폭 = 몸 반폭 0.17 + 여유 0.2, 앞 = 0.25 + 여유 0.2. v_min: CHAMP 는 0.1 m/s 아래 명령에서 거의 안 나감
        self.steer = Steer(v_max=v_max, w_max=0.5, half=0.37, front=0.45, look=1.2, tol=0.25, v_min=0.1)
        self.pose, self.pts, self.t = None, None, {}
        node.create_subscription(PoseStamped, f'/{ns}/pose_shared', lambda m: self.put('pose', pose2d(m)), 10)
        node.create_subscription(LaserScan, f'/{ns}/scan', lambda m: self.put('pts', scan_xy(
            m.ranges, m.angle_min, m.angle_increment, m.range_min, m.range_max, body=BODY)), qos_profile_sensor_data)
        self.pub = node.create_publisher(Twist, f'/{ns}/cmd_vel_follow', 10)
        self.pub_err = node.create_publisher(Float32, f'/{ns}/slot_error', 10)
        self.pub_fix = node.create_publisher(PointStamped, f'/{ns}/pose_fix', 10)
        self.seen_t = -9.0             # Husky 를 라이다로 마지막으로 본 시각
        self.state = None
        self.fix = (0.0, 0.0)          # 라이다로 본 Husky - 자세로 어림한 Husky (몸 기준, 천천히 따라감)

    def put(self, key, value):
        setattr(self, key, value)
        self.t[key] = self.node.now()

    def send_fix(self):
        """이동 끝: 방금 본 Husky 기준으로 내 공유 자세 오차(월드)를 알린다. 0.1 m 미만·1.5 m 초과(잘못 본 것)는 안 보냄."""
        fx, fy = self.fix
        if self.pose is None or self.node.now() - self.seen_t > 3.0 or not 0.1 < math.hypot(fx, fy) < 1.5:
            return
        c, s = math.cos(self.pose[2]), math.sin(self.pose[2])
        m = PointStamped()
        m.header.frame_id, m.point.x, m.point.y = 'world', -(c * fx - s * fy), -(s * fx + c * fy)
        self.pub_fix.publish(m)
        self.node.get_logger().warn(f'[{self.ns}] Husky 기준 자세 보정 ({m.point.x:+.2f}, {m.point.y:+.2f}) m')

    def tick(self, husky):
        now, tw, s = self.node.now(), Twist(), 'WAIT'
        if husky is not None and all(now - self.t.get(k, -9.0) < 1.0 for k in ('pose', 'pts')):
            hx, hy, hyaw = husky
            x, y, yaw = self.pose
            c, si = math.cos(yaw), math.sin(yaw)

            def body(wx, wy):          # 월드 점 -> 내 몸 기준
                return c * (wx - x) + si * (wy - y), -si * (wx - x) + c * (wy - y)
            guess = body(hx, hy)
            seen = find_body(self.pts, (guess[0] + self.fix[0], guess[1] + self.fix[1]), yaw=hyaw - yaw)
            if seen is not None:
                self.fix = tuple(0.7 * f + 0.3 * (m - g) for f, m, g in zip(self.fix, seen, guess))
                self.seen_t = now
            ch, sh = math.cos(hyaw), math.sin(hyaw)
            t = body(hx + ch * self.slot[0] - sh * self.slot[1], hy + sh * self.slot[0] + ch * self.slot[1])
            tx, ty = t[0] + self.fix[0], t[1] + self.fix[1]
            dist = math.hypot(tx, ty)
            tw.linear.x, tw.angular.z, s = self.steer.cmd(self.pts, math.atan2(ty, tx), dist)
            self.pub_err.publish(Float32(data=float(dist)))
        if s != self.state:
            self.node.get_logger().info(f'[{self.ns}] 추종 {self.state} -> {s}')
            self.state = s
        self.pub.publish(tw)


class A1Follower(Node):
    def __init__(self):
        super().__init__('a1_follower')
        robots = list(self.declare_parameter('robots', ['a1_1', 'a1_2']).value)
        v_max = float(self.declare_parameter('v_max', 0.35).value)   # gait_sweep: 평지 직진 0.5 m/s 까지 명령의 95~108 %, 기울기 6° 아래
        slots = {ns: list(self.declare_parameter(f'slot.{ns}', [-1.6, 0.8 * (1 if i else -1)]).value) for i, ns in enumerate(robots)}
        self.active, self.husky, self.husky_t = False, None, -9.0
        self.create_subscription(String, '/mission/state', self.on_state, LATCHED)
        self.create_subscription(PoseStamped, '/husky/pose', self.on_husky, 10)
        self.followers = [Follower(self, ns, slots[ns], v_max) for ns in robots]
        self.create_timer(0.1, self.tick)

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def on_state(self, m):
        phase = json.loads(m.data).get('phase')
        if self.active and phase == 'EXPLORE':
            for f in self.followers:
                f.send_fix()
        self.active = phase == 'TRANSIT'

    def on_husky(self, m):
        self.husky, self.husky_t = pose2d(m), self.now()

    def tick(self):
        if not self.active:
            for f in self.followers:
                f.state, f.fix = None, (0.0, 0.0)
            return                                  # 발행 안 함 -> arbiter 가 다른 출처를 고름
        husky = self.husky if self.now() - self.husky_t < 1.0 else None
        for f in self.followers:
            f.tick(husky)


def main(args=None):
    rclpy.init(args=args)
    node = A1Follower()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
