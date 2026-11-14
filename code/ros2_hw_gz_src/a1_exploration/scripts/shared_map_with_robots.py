import math

import rclpy
from rclpy.time import Time
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker, MarkerArray

from shared_map_merger import SharedMapMerger, POSES, rotate, yaw


class SharedMapWithRobots(SharedMapMerger):

    def __init__(self):
        super().__init__()

        # RViz 로봇 위치 표시용
        self.marker_pub = self.create_publisher(
            MarkerArray,
            '/robot_position_markers',
            10
        )

        # 로봇별 공동 지도 좌표계 위치
        self.pose_pubs = {
            ns: self.create_publisher(
                PoseStamped,
                f'/{ns}/pose_shared',
                10
            )
            for ns in POSES
        }

        self.create_timer(0.1, self.publish_robot_poses)   # a1_follower 가 조향에 씀

        self.get_logger().info(
            'Robot Position Tracker Started'
        )

    def publish_robot_poses(self):

        # 기존 지도 병합 노드가 계산한 좌표 정렬 정보 사용
        if not all(ns in self.anchors for ns in POSES):
            return

        markers = []
        stamp = self.get_clock().now().to_msg()

        for index, ns in enumerate(POSES):

            try:
                tf = self.buffers[ns].lookup_transform(
                    'map',
                    'base_footprint',
                    Time()
                )

            except Exception:
                continue

            # 1. 로봇의 로컬 지도 좌표
            local = tf.transform.translation

            # 2. 공동 지도 정렬 정보
            ax, ay, angle = self.anchors[ns]

            # 3. 공동 지도 좌표로 변환
            rx, ry = rotate(
                local.x,
                local.y,
                angle
            )

            x = ax + rx
            y = ay + ry

            heading = yaw(
                tf.transform.rotation
            ) + angle

            qz = math.sin(heading / 2)
            qw = math.cos(heading / 2)

            # 4. 로봇 위치 발행
            pose = PoseStamped()

            pose.header.frame_id = 'shared_map'
            pose.header.stamp = stamp

            pose.pose.position.x = x
            pose.pose.position.y = y

            pose.pose.orientation.z = qz
            pose.pose.orientation.w = qw

            self.pose_pubs[ns].publish(pose)

            # A1_1 파란색 / A1_2 빨간색
            if ns == 'a1_1':
                color = (0.1, 0.4, 1.0)
            else:
                color = (1.0, 0.2, 0.2)

            # 5. 로봇 방향 화살표
            arrow = Marker()

            arrow.header = pose.header
            arrow.ns = 'robot_pose'
            arrow.id = index

            arrow.type = Marker.ARROW
            arrow.action = Marker.ADD

            arrow.pose.position.x = x
            arrow.pose.position.y = y
            arrow.pose.position.z = 0.15

            arrow.pose.orientation.z = qz
            arrow.pose.orientation.w = qw

            arrow.scale.x = 0.55
            arrow.scale.y = 0.20
            arrow.scale.z = 0.12

            arrow.color.r = color[0]
            arrow.color.g = color[1]
            arrow.color.b = color[2]
            arrow.color.a = 1.0

            markers.append(arrow)

            # 6. 로봇 이름 표시
            label = Marker()

            label.header = pose.header
            label.ns = 'robot_label'
            label.id = index

            label.type = Marker.TEXT_VIEW_FACING
            label.action = Marker.ADD

            label.pose.position.x = x
            label.pose.position.y = y
            label.pose.position.z = 0.6

            label.pose.orientation.w = 1.0
            label.scale.z = 0.35

            label.color.r = color[0]
            label.color.g = color[1]
            label.color.b = color[2]
            label.color.a = 1.0

            label.text = ns.upper()

            markers.append(label)

        if markers:
            self.marker_pub.publish(
                MarkerArray(markers=markers)
            )


def main():

    rclpy.init()

    node = SharedMapWithRobots()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
