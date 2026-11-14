import math

import rclpy
from geometry_msgs.msg import PointStamped, PoseStamped

from shared_map_with_robots import SharedMapWithRobots
from shared_map_merger import POSES, rotate, yaw


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

        self.get_logger().info(
            'Nav2 Goal Bridge Started'
        )

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
