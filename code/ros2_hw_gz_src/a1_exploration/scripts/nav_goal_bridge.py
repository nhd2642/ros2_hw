import math
from collections import deque

import rclpy
from geometry_msgs.msg import PointStamped, PoseStamped, Pose2D
from mission_interfaces.msg import MissionState
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time

from shared_map_with_robots import SharedMapWithRobots
from shared_map_merger import POSES, anchor_rotated, heading_angle, rotate, yaw

LATCHED = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
HEADING_WINDOW = 5.0                # s: 탐색 시작 직전 이만큼의 map->odom 방향을 평균
HEADING_MAX_STEP = math.radians(10.0)   # 한 번에 이보다 크게 돌리라는 값은 잘못 잰 것으로 보고 버림


class NavGoalBridge(SharedMapWithRobots):

    def __init__(self):
        super().__init__()

        self.local_goal_pubs = {}

        for ns in POSES:

            self.local_goal_pubs[ns] = self.create_publisher(
                PoseStamped,
                f'/{ns}/nav_goal_local',
                10
            )

            self.create_subscription(
                PoseStamped,
                f'/{ns}/exploration_goal',
                lambda msg, robot=ns: self.convert_goal(robot, msg),
                10
            )
            # Husky 기준 자세 보정 (a1_follower 가 이동이 끝날 때 보냄): 이 로봇 지도의 정렬을 그만큼 옮긴다
            self.create_subscription(
                PointStamped,
                f'/{ns}/pose_fix',
                lambda msg, robot=ns: self.shift_anchor(robot, msg),
                10
            )
            # 탐색 시작 위치(홈): husky_mission 의 공유 지도 좌표 -> 로봇 map (nav_executor 가 복귀 목표로 씀)
            home_pub = self.create_publisher(
                PoseStamped, f'/{ns}/home_local', 10
            )
            self.create_subscription(
                PoseStamped,
                f'/{ns}/home_shared',
                lambda msg, robot=ns, pub=home_pub: self.convert_goal(robot, msg, pub),
                10
            )

        # 방향 보정 (R1, 2026-11-28): SLAM 지도가 월드에 비해 돌면 평행 이동 보정점에서 멀수록 공유 자세가 틀어진다
        # (공동 건물 오차가 탐색 시작점 거리에 비례, 0.6~2.5°). 탐색 단계로 넘어갈 때마다 공유 자세 방향을 odom(IMU) 방향에 맞춘다.
        # 기준 좌표는 /<ns>/map_anchor (x, y, theta: 공유 = R(theta)·map + (x, y)) 로 내보내 husky_mission 이 사람 위치 변환에 같은 값을 쓴다.
        self.heading = {ns: deque(maxlen=60) for ns in POSES}      # (t, map->odom yaw, odom->base yaw), 5 Hz
        self.anchor_pubs = {ns: self.create_publisher(Pose2D, f'/{ns}/map_anchor', LATCHED) for ns in POSES}
        self.sent_anchor = {}
        self.phase = None
        self.create_subscription(MissionState, '/mission/state', self.on_mission, LATCHED)
        self.create_timer(0.2, self.sample_heading)

        self.get_logger().info(
            'Nav2 Goal Bridge Started'
        )

    def sample_heading(self):
        t = self.get_clock().now().nanoseconds * 1e-9
        for ns in POSES:
            try:
                mo = self.buffers[ns].lookup_transform('map', 'odom', Time())
                ob = self.buffers[ns].lookup_transform('odom', 'base_footprint', Time())
                self.heading[ns].append((t, yaw(mo.transform.rotation), yaw(ob.transform.rotation)))
            except Exception:
                pass
            a = self.anchors.get(ns)
            if a is not None and a != self.sent_anchor.get(ns):
                self.sent_anchor[ns] = a
                self.anchor_pubs[ns].publish(Pose2D(x=float(a[0]), y=float(a[1]), theta=float(a[2])))

    def on_mission(self, m):
        key = (m.phase, m.set_index)
        if key != self.phase and m.phase == 'EXPLORE':
            for ns in POSES:
                self.align_heading(ns)
        self.phase = key

    def align_heading(self, ns):
        if ns not in self.anchors or not self.heading[ns]:
            return
        t_end = self.heading[ns][-1][0]
        target = heading_angle([s for s in self.heading[ns] if s[0] >= t_end - HEADING_WINDOW])
        if target is None:
            self.get_logger().warn(f'{ns}: 방향 보정 안 함 (회전 안 하는 표본 부족)')
            return
        angle = self.anchors[ns][2]
        step = math.atan2(math.sin(target - angle), math.cos(target - angle))
        if abs(step) > HEADING_MAX_STEP:
            self.get_logger().warn(f'{ns}: 방향 보정 버림 ({math.degrees(step):+.1f}° 너무 큼)')
            return
        try:
            p = self.buffers[ns].lookup_transform('map', 'base_footprint', Time()).transform.translation
        except Exception:
            return
        self.anchors[ns] = anchor_rotated(self.anchors[ns], (p.x, p.y), angle + step)
        self.get_logger().warn(f'{ns}: 방향 보정 {math.degrees(step):+.2f}° (기준 회전 {math.degrees(angle + step):+.2f}°)')

    def shift_anchor(self, ns, msg):
        if ns in self.anchors:
            ax, ay, angle = self.anchors[ns]
            self.anchors[ns] = (ax + msg.point.x, ay + msg.point.y, angle)
            self.get_logger().info(f'{ns}: 정렬 보정 ({msg.point.x:+.2f}, {msg.point.y:+.2f}) m')

    def convert_goal(self, ns, msg, pub=None):

        if ns not in self.anchors:
            self.get_logger().warn(
                f'{ns}: Waiting for map alignment'
            )
            return

        if msg.header.frame_id != 'shared_map':
            self.get_logger().warn(
                f'{ns}: Expected shared_map frame'
            )
            return

        # 공동 지도 정렬 정보
        ax, ay, angle = self.anchors[ns]

        # 공동 지도에서 목표 위치
        gx = msg.pose.position.x
        gy = msg.pose.position.y

        # 공동 지도 -> 로컬 map 변환
        dx = gx - ax
        dy = gy - ay

        lx, ly = rotate(
            dx, dy, -angle
        )

        # 목표 방향 변환
        shared_yaw = yaw(msg.pose.orientation)
        local_yaw = shared_yaw - angle

        # Nav2에 전달할 로컬 목표
        local_goal = PoseStamped()

        local_goal.header.frame_id = 'map'
        local_goal.header.stamp = (
            self.get_clock().now().to_msg()
        )

        local_goal.pose.position.x = lx
        local_goal.pose.position.y = ly
        local_goal.pose.position.z = 0.0

        local_goal.pose.orientation.z = math.sin(
            local_yaw / 2.0
        )

        local_goal.pose.orientation.w = math.cos(
            local_yaw / 2.0
        )

        if pub is not None:
            pub.publish(local_goal)
            return

        self.local_goal_pubs[ns].publish(local_goal)

        self.get_logger().info(
            f'{ns}: '
            f'Shared ({gx:.2f}, {gy:.2f}) '
            f'-> Local ({lx:.2f}, {ly:.2f})'
        )


def main():

    rclpy.init()

    node = NavGoalBridge()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
