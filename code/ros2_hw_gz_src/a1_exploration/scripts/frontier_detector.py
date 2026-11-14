import json
from collections import deque
import math

import numpy as np
import rclpy

from rclpy.node import Node
from rclpy.qos import (
    QoSProfile,
    ReliabilityPolicy,
    DurabilityPolicy
)

from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import String
from geometry_msgs.msg import Point, Pose, PoseArray
from visualization_msgs.msg import Marker, MarkerArray


def get_frontiers(grid, min_cells=5):
    """
    Frontier = 미탐색 영역(-1)에 인접한 빈 공간(0~20)
    """

    free = (grid >= 0) & (grid <= 20)
    unknown = grid == -1

    adjacent = np.zeros_like(unknown, dtype=bool)

    adjacent[1:, :] |= unknown[:-1, :]
    adjacent[:-1, :] |= unknown[1:, :]
    adjacent[:, 1:] |= unknown[:, :-1]
    adjacent[:, :-1] |= unknown[:, 1:]

    frontier = free & adjacent

    # 연결된 Frontier 셀들을 그룹화
    visited = np.zeros_like(frontier, dtype=bool)

    height, width = frontier.shape
    clusters = []

    for row, col in zip(*np.where(frontier)):

        if visited[row, col]:
            continue

        queue = deque([(int(row), int(col))])
        visited[row, col] = True

        cluster = []

        while queue:

            r, c = queue.popleft()
            cluster.append((r, c))

            # 8방향 연결 검사
            for dr in (-1, 0, 1):
                for dc in (-1, 0, 1):

                    nr = r + dr
                    nc = c + dc

                    if (
                        0 <= nr < height
                        and 0 <= nc < width
                        and frontier[nr, nc]
                        and not visited[nr, nc]
                    ):
                        visited[nr, nc] = True
                        queue.append((nr, nc))

        # 너무 작은 경계는 제외
        if len(cluster) >= min_cells:
            clusters.append(cluster)

    # 큰 Frontier부터 정렬
    return sorted(clusters, key=len, reverse=True)


class FrontierDetector(Node):

    def __init__(self):

        super().__init__('frontier_detector')

        qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL
        )

        self.map_msg = None

        self.create_subscription(
            OccupancyGrid,
            '/shared_map',
            self.map_callback,
            qos
        )

        self.marker_pub = self.create_publisher(
            MarkerArray,
            '/frontier_markers',
            10
        )

        self.goal_pub = self.create_publisher(
            PoseArray,
            '/frontier_candidates',
            10
        )

        # 지금 탐색 구역 (husky_mission 의 /mission/state, 월드 사각형 [x0, x1, y0, y1]). 구역 안에서만 프런티어를 찾는다:
        # 도로·다른 세트의 프런티어가 큰 묶음 상위 15 개를 차지해 건물 안 후보가 빠졌음 (2026-11-12 세트 1: 묶음 73 개 중 구역 안 8 개만 통과)
        self.zones = []
        self.create_subscription(
            String, '/mission/state',
            lambda m: setattr(self, 'zones', list(json.loads(m.data).get('zones', {}).values())),
            qos
        )

        self.create_timer(2.0, self.detect)

        self.get_logger().info(
            'Frontier Detector Started'
        )

    def map_callback(self, msg):
        self.map_msg = msg

    def grid_to_world(self, row, col, msg):

        info = msg.info

        local_x = (col + 0.5) * info.resolution
        local_y = (row + 0.5) * info.resolution

        q = info.origin.orientation

        yaw = math.atan2(
            2 * (q.w * q.z + q.x * q.y),
            1 - 2 * (q.y * q.y + q.z * q.z)
        )

        x = (
            info.origin.position.x
            + math.cos(yaw) * local_x
            - math.sin(yaw) * local_y
        )

        y = (
            info.origin.position.y
            + math.sin(yaw) * local_x
            + math.cos(yaw) * local_y
        )

        return x, y

    def make_marker(self, marker_type, name, marker_id, msg):

        marker = Marker()

        marker.header.frame_id = (
            msg.header.frame_id or 'shared_map'
        )
        marker.header.stamp = self.get_clock().now().to_msg()

        marker.ns = name
        marker.id = marker_id
        marker.type = marker_type
        marker.action = Marker.ADD

        marker.pose.orientation.w = 1.0

        return marker

    def detect(self):

        if self.map_msg is None:
            self.get_logger().warn(
                'Waiting for /shared_map'
            )
            return

        msg = self.map_msg

        width = msg.info.width
        height = msg.info.height

        if (
            width == 0
            or height == 0
            or len(msg.data) != width * height
        ):
            self.get_logger().warn('Invalid map')
            return

        grid = np.asarray(
            msg.data, dtype=np.int16
        ).reshape((height, width))

        # 구역 밖은 장애물(100)로 바꿔 프런티어가 안 생기게 (구역이 없으면 = 탐색 단계 아님 -> 전부 밖)
        res = msg.info.resolution
        ox, oy = msg.info.origin.position.x, msg.info.origin.position.y
        inside = np.zeros(grid.shape, dtype=bool)
        for x0, x1, y0, y1 in self.zones:
            c0, c1 = max(0, int((x0 - ox) / res)), max(0, int((x1 - ox) / res) + 1)
            r0, r1 = max(0, int((y0 - oy) / res)), max(0, int((y1 - oy) / res) + 1)
            inside[r0:r1, c0:c1] = True
        grid = np.where(inside, grid, 100)

        clusters = get_frontiers(grid)

        # 모든 Frontier 셀 표시
        frontier_marker = self.make_marker(
            Marker.POINTS,
            'frontier_cells',
            0,
            msg
        )

        frontier_marker.scale.x = 0.07
        frontier_marker.scale.y = 0.07

        frontier_marker.color.r = 1.0
        frontier_marker.color.g = 0.75
        frontier_marker.color.b = 0.1
        frontier_marker.color.a = 1.0

        # 탐색 후보 위치 표시
        goal_marker = self.make_marker(
            Marker.SPHERE_LIST,
            'frontier_centers',
            1,
            msg
        )

        goal_marker.scale.x = 0.22
        goal_marker.scale.y = 0.22
        goal_marker.scale.z = 0.22

        goal_marker.color.r = 0.15
        goal_marker.color.g = 0.9
        goal_marker.color.b = 0.25
        goal_marker.color.a = 1.0

        goals = PoseArray()
        goals.header.frame_id = (
            msg.header.frame_id or 'shared_map'
        )
        goals.header.stamp = self.get_clock().now().to_msg()

        # 노란 점: Frontier 영역
        for cluster in clusters:

            for row, col in cluster:

                x, y = self.grid_to_world(
                    row, col, msg
                )

                frontier_marker.points.append(
                    Point(x=x, y=y, z=0.08)
                )

        # 초록 구: Frontier 후보 위치
        for cluster in clusters[:15]:

            mean_row = sum(
                p[0] for p in cluster
            ) / len(cluster)

            mean_col = sum(
                p[1] for p in cluster
            ) / len(cluster)

            row, col = min(
                cluster,
                key=lambda p: (
                    (p[0] - mean_row) ** 2
                    + (p[1] - mean_col) ** 2
                )
            )

            x, y = self.grid_to_world(
                row, col, msg
            )

            goal_marker.points.append(
                Point(x=x, y=y, z=0.15)
            )

            pose = Pose()

            pose.position.x = x
            pose.position.y = y
            pose.orientation.w = 1.0

            goals.poses.append(pose)

        self.marker_pub.publish(
            MarkerArray(
                markers=[frontier_marker, goal_marker]
            )
        )

        self.goal_pub.publish(goals)

        self.get_logger().info(
            f'Frontier clusters: {len(clusters)}, '
            f'Candidates: {len(goals.poses)}'
        )


def main():

    rclpy.init()

    node = FrontierDetector()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
