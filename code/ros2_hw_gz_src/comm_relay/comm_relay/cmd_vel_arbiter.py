"""로봇별 cmd_vel 중재 노드: CHAMP 의 /<ns>/cmd_vel 에 쓰는 쪽을 하나로 만든다.

입력(우선순위 높은 순, 기본값):
  /<ns>/cmd_vel_teleop     4  사람 조종 (항상 최우선)
  /<ns>/cmd_vel_hold       3  survivor_detector: 사람 발견하면 1 초 0 명령 (record15)
  /<ns>/cmd_vel_return     2  return_relay 직접 복귀/정지
  /<ns>/cmd_vel_nav_test   1  Nav2 controller_server 출력 (팀원 nav2_controllers.launch.py 의 리맵 그대로)
  /<ns>/cmd_vel_follow     0  a1_follower: Husky 추종 (이동 단계에만 나옴)
출력: /<ns>/cmd_vel 을 rate Hz 로 계속 발행. timeout 안에 들어온 입력이 없으면 0 명령.
  -> CHAMP 에 cmd_vel 타임아웃이 없어 마지막 명령으로 계속 걷던 문제를 여기서 막는다.
주의: 이 노드가 떠 있으면 /<ns>/cmd_vel 에 직접 쓰는 도구(teleop 기본 설정, gait_eval, wp.py)는 0 명령과 섞인다.
      조종은 cmd_vel_teleop 으로 보낸다. 다른 발행자가 보이면 경고를 낸다.
"""

import rclpy
from rclpy.executors import ExternalShutdownException
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

from comm_relay.arbiter_logic import Arbiter


def sim_now(node):
    """노드 시계(초). use_sim_time 이면 시뮬 시간 — 2026-11-10 벽시계(time.monotonic) 대신 (RTF 0.1~0.3 에서 한도가 시뮬 기준 3~7 배 엄격했음)"""
    return node.get_clock().now().nanoseconds * 1e-9


class RobotMux:
    def __init__(self, node, ns, sources, timeout, output):
        self.node, self.ns = node, ns
        self.arb = Arbiter([(name, prio) for name, _, prio in sources], timeout)
        for name, topic, _ in sources:
            node.create_subscription(Twist, f'/{ns}/{topic}', lambda m, n=name: self.on(n, m), 10)
        self.out_topic = f'/{ns}/{output}'
        self.pub = node.create_publisher(Twist, self.out_topic, 10)
        # 바뀔 때만 보내므로 늦게 붙은 구독자(모니터)도 지금 출처를 받게 transient local
        self.pub_src = node.create_publisher(String, f'/{ns}/cmd_vel_source', QoSProfile(
            depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.src = None
        self.warned = False

    def on(self, name, m):
        self.arb.on_cmd(name, sim_now(self.node), (m.linear.x, m.linear.y, m.angular.z))

    def tick(self):
        src, (vx, vy, wz) = self.arb.select(sim_now(self.node))
        tw = Twist()
        tw.linear.x, tw.linear.y, tw.angular.z = float(vx), float(vy), float(wz)
        self.pub.publish(tw)
        if src != self.src:
            self.node.get_logger().info(f'[{self.ns}] cmd_vel source: {self.src} -> {src}')
            self.src = src
            self.pub_src.publish(String(data=src))

    def check_publishers(self):
        others = [i.node_name for i in self.node.get_publishers_info_by_topic(self.out_topic)
                  if i.node_name != self.node.get_name()]
        if others and not self.warned:
            self.node.get_logger().warn(
                f'{self.out_topic} 에 다른 발행자 {others} — 0 명령과 섞입니다. cmd_vel_teleop 등으로 보내세요')
        self.warned = bool(others)


class CmdVelArbiterNode(Node):
    def __init__(self):
        super().__init__('cmd_vel_arbiter')
        robots = list(self.declare_parameter('robots', ['a1_1', 'a1_2']).value)
        names = list(self.declare_parameter('source_names', ['teleop', 'hold', 'return', 'nav', 'follow']).value)
        topics = list(self.declare_parameter(
            'source_topics', ['cmd_vel_teleop', 'cmd_vel_hold', 'cmd_vel_return', 'cmd_vel_nav_test', 'cmd_vel_follow']).value)
        prios = list(self.declare_parameter('source_priorities', [4, 3, 2, 1, 0]).value)
        timeout = float(self.declare_parameter('timeout', 0.5).value)
        rate = float(self.declare_parameter('rate', 20.0).value)
        output = self.declare_parameter('output_topic', 'cmd_vel').value
        sources = list(zip(names, topics, prios))
        self.muxes = [RobotMux(self, ns, sources, timeout, output) for ns in robots]
        self.create_timer(1.0 / rate, lambda: [m.tick() for m in self.muxes])
        self.create_timer(5.0, lambda: [m.check_publishers() for m in self.muxes])
        self.get_logger().info(f'cmd_vel_arbiter: robots={robots}, sources={sources}, timeout={timeout}')


def main(args=None):
    rclpy.init(args=args)
    node = CmdVelArbiterNode()
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
