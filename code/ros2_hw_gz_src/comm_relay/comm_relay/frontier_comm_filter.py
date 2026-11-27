"""프런티어 후보에서 지금 탐색 구역 밖(+여유)과 실패한 목표 주변을 뺀다.

/frontier_candidates (PoseArray, shared_map = 월드 정렬) -> /frontier_candidates_comm (goal_assigner 입력으로 리맵).
구역 = /mission/state 의 zones (로봇별 사각형). 구역 가장자리 margin 안쪽만 남겨서 문 밖(도로) 프런티어를 뺀다.
탐색 단계가 아니면(zones 없음) 전부 뺀다. 실패 목표는 /exploration_blacklist (nav_executor 가 발행).
"""
import math

import rclpy
from rclpy.executors import ExternalShutdownException
from geometry_msgs.msg import PoseArray, PoseStamped
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from mission_interfaces.msg import MissionState

from comm_relay.mission_logic import in_rect, zone_dict


def sim_now(node):
    """노드 시계(초). use_sim_time 이면 시뮬 시간 — 2026-11-10 벽시계(time.monotonic) 대신 (RTF 0.1~0.3 에서 한도가 시뮬 기준 3~7 배 엄격했음)"""
    return node.get_clock().now().nanoseconds * 1e-9


class FrontierCommFilter(Node):
    def __init__(self):
        super().__init__('frontier_comm_filter')
        self.margin = float(self.declare_parameter('margin', 0.3).value)
        self.bl_radius = float(self.declare_parameter('blacklist_radius', 0.9).value)
        self.bl_ttl = float(self.declare_parameter('blacklist_ttl', 120.0).value)
        self.zones = []              # 지금 탐색 구역 사각형들
        self.create_subscription(MissionState, '/mission/state', lambda m: setattr(
            self, 'zones', list(zone_dict(m.zones).values())), QoSProfile(
                depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.blacklist = []          # (x, y, 등록 시각)
        self.create_subscription(PoseArray, '/frontier_candidates', self.on_candidates, 10)
        self.create_subscription(PoseStamped, '/exploration_blacklist', self.on_blacklist, 10)
        self.pub = self.create_publisher(PoseArray, '/frontier_candidates_comm', 10)
        self.last_report = None
        self.get_logger().info(f'frontier_comm_filter: margin={self.margin}')

    def on_blacklist(self, msg):
        p = msg.pose.position
        self.blacklist.append((p.x, p.y, sim_now(self)))
        self.get_logger().warn(f'블랙리스트 추가 ({p.x:.2f}, {p.y:.2f}) [{msg.header.frame_id}]')

    def keep(self, x, y):
        if not any(in_rect(r, x, y, -self.margin) for r in self.zones):
            return False
        return all(math.hypot(x - bx, y - by) > self.bl_radius for bx, by, _ in self.blacklist)

    def on_candidates(self, msg):
        now = sim_now(self)
        self.blacklist = [b for b in self.blacklist if now - b[2] <= self.bl_ttl]
        out = PoseArray(header=msg.header)
        out.poses = [p for p in msg.poses if self.keep(p.position.x, p.position.y)]
        self.pub.publish(out)
        report = (len(msg.poses), len(out.poses))
        if report != self.last_report:
            self.get_logger().info(f'frontier {report[0]} -> {report[1]} (구역 밖/블랙리스트 제외)')
            self.last_report = report


def main(args=None):
    rclpy.init(args=args)
    node = FrontierCommFilter()
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
