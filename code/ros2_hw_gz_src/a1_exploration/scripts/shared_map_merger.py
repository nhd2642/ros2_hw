import math
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.duration import Duration
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from nav_msgs.msg import OccupancyGrid
from tf2_msgs.msg import TFMessage
from tf2_ros import Buffer

POSES = {
    'a1_1': (0.0, -1.1, -math.pi / 2),
    'a1_2': (0.0, 1.1, math.pi / 2)
}

def yaw(q):
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    )

def rotate(x, y, angle):
    c, s = math.cos(angle), math.sin(angle)
    return c * x - s * y, s * x + c * y


def anchor_rotated(anchor, local_xy, new_angle):
    """기준 (ax, ay, angle) 의 회전을 new_angle 로 바꾼다. 중심 = 로봇의 map 위치 local_xy (그 점의 공유 좌표는 그대로)."""
    ax, ay, angle = anchor
    ox, oy = rotate(local_xy[0], local_xy[1], angle)
    nx, ny = rotate(local_xy[0], local_xy[1], new_angle)
    return ax + ox - nx, ay + oy - ny, new_angle


def heading_angle(samples, max_rate=math.radians(5.0), min_n=5):
    """R1 (2026-11-28): 공유 자세 방향을 odom(IMU, 월드 방향) 에 맞추는 기준 회전 = -(map->odom yaw) 의 원형 평균.
    samples = [(t, map->odom yaw, odom->base yaw)] 시각 순. 몸이 max_rate (rad/s) 보다 빨리 도는 동안의 표본은 뺀다
    (G1: 회전 중 SLAM 방향 오차가 커짐). 남은 표본이 min_n 개 미만이면 None."""
    keep = [mo for (t0, _, b0), (t1, mo, b1) in zip(samples, samples[1:])
            if t1 > t0 and abs(math.atan2(math.sin(b1 - b0), math.cos(b1 - b0))) / (t1 - t0) < max_rate]
    if len(keep) < min_n:
        return None
    return -math.atan2(sum(math.sin(a) for a in keep), sum(math.cos(a) for a in keep))

class SharedMapMerger(Node):
    def __init__(self):
        super().__init__('shared_map_merger')
        for ns in POSES:      # 스폰 자세 (월드 x, y, yaw) — launch 가 mission.yaml 값을 넘김
            POSES[ns] = tuple(self.declare_parameter(f'spawn.{ns}', list(POSES[ns])).value)
        self.maps = {}
        self.anchors = {}
        self.buffers = {
            ns: Buffer(cache_time=Duration(seconds=30))
            for ns in POSES
        }

        map_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL
        )
        static_qos = QoSProfile(
            depth=100,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL
        )

        for ns in POSES:
            self.create_subscription(
                OccupancyGrid,
                f'/{ns}/map',
                self.map_callback(ns),
                map_qos
            )
            self.create_subscription(
                TFMessage,
                f'/{ns}/tf',
                self.tf_callback(ns, False),
                100
            )
            self.create_subscription(
                TFMessage,
                f'/{ns}/tf_static',
                self.tf_callback(ns, True),
                static_qos
            )

        self.pub = self.create_publisher(
            OccupancyGrid, '/shared_map', map_qos
        )
        self.create_timer(1.0, self.publish_map)
        self.get_logger().info(
            'Waiting for both maps and TF...'
        )

    def map_callback(self, ns):
        def cb(msg):
            self.maps[ns] = msg
        return cb

    def tf_callback(self, ns, static):
        def cb(msg):
            for t in msg.transforms:
                # 쓰는 것은 map -> odom -> base_footprint 뿐. 다리 관절 TF(로봇당 초당 수백 개)까지 넣으면 이 노드가 CPU 50 % 를 씀
                if t.child_frame_id not in ('odom', 'base_footprint'):
                    continue
                try:
                    if static:
                        self.buffers[ns].set_transform_static(
                            t, 'shared_map_merger'
                        )
                    else:
                        self.buffers[ns].set_transform(
                            t, 'shared_map_merger'
                        )
                except Exception:
                    pass
        return cb

    def get_anchor(self, ns):
        if ns in self.anchors:
            return True
        try:
            transform = self.buffers[ns].lookup_transform(
                'map', 'base_footprint', Time()
            )
        except Exception:
            return False

        p = transform.transform.translation
        wx, wy, _ = POSES[ns]
        # map(= odom) 방향은 IMU(ENU)라 이미 월드와 같다. 스폰 yaw 를 믿고 맞추면 기립하며 몸이 돈 만큼 공유 좌표가 틀어짐
        # (2026-11-12: a1_2 가 기립 때 10° 돌아 pose_shared 가 10° 틀어지고 5 m 가서 1.1 m 벗어남)
        angle = 0.0
        dx, dy = rotate(p.x, p.y, angle)

        self.anchors[ns] = (
            wx - dx,
            wy - dy,
            angle
        )
        self.get_logger().info(f'{ns} aligned')
        return True

    def points(self, ns, xs, ys, msg):
        meta = msg.info
        theta = yaw(meta.origin.orientation)

        x, y = rotate(
            xs * meta.resolution,
            ys * meta.resolution,
            theta
        )
        x += meta.origin.position.x
        y += meta.origin.position.y

        ax, ay, angle = self.anchors[ns]
        x, y = rotate(x, y, angle)
        return x + ax, y + ay

    def publish_map(self):
        if not all(ns in self.maps for ns in POSES):
            return
        if not all(self.get_anchor(ns) for ns in POSES):
            return

        resolution = 0.05
        bounds = []

        for ns, msg in self.maps.items():
            w, h = msg.info.width, msg.info.height
            x, y = self.points(
                ns,
                np.array([0, w, w, 0]),
                np.array([0, 0, h, h]),
                msg
            )
            bounds.append((
                x.min(), y.min(), x.max(), y.max()
            ))

        xmin = math.floor(
            min(b[0] for b in bounds) / resolution
        ) * resolution
        ymin = math.floor(
            min(b[1] for b in bounds) / resolution
        ) * resolution

        width = math.ceil(
            (max(b[2] for b in bounds) - xmin) / resolution
        ) + 1
        height = math.ceil(
            (max(b[3] for b in bounds) - ymin) / resolution
        ) + 1

        if width * height > 2000000:
            self.get_logger().warn('Map too large')
            return

        merged = np.full(
            width * height, -1, dtype=np.int8
        )

        for ns, msg in self.maps.items():
            values = np.asarray(
                msg.data, dtype=np.int16
            )
            valid = np.flatnonzero(values >= 0)

            if not valid.size:
                continue

            xs = valid % msg.info.width + 0.5
            ys = valid // msg.info.width + 0.5

            world_x, world_y = self.points(
                ns, xs, ys, msg
            )

            cols = np.floor(
                (world_x - xmin) / resolution
            ).astype(np.int64)
            rows = np.floor(
                (world_y - ymin) / resolution
            ).astype(np.int64)

            inside = (
                (cols >= 0) & (cols < width) &
                (rows >= 0) & (rows < height)
            )

            indexes = (
                rows[inside] * width + cols[inside]
            )

            np.maximum.at(
                merged,
                indexes,
                values[valid[inside]].astype(np.int8)
            )

        result = OccupancyGrid()
        result.header.stamp = (
            self.get_clock().now().to_msg()
        )
        result.header.frame_id = 'shared_map'
        result.info.resolution = resolution
        result.info.width = width
        result.info.height = height
        result.info.origin.position.x = xmin
        result.info.origin.position.y = ymin
        result.info.origin.orientation.w = 1.0
        result.data = merged.tolist()

        self.pub.publish(result)

def main():
    rclpy.init()
    node = SharedMapMerger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
